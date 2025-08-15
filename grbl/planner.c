/*
  planner.c - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Jens Geisler

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

#define SOME_LARGE_VALUE 1.0E+38 // Used by rapids and acceleration maximization calculations. Just needs
                                 // to be larger than any feasible (mm/min)^2 or mm/sec^2 value.

static plan_block_t block_buffer[BLOCK_BUFFER_SIZE]; // A ring buffer for motion instructions
static uint8_t block_buffer_tail;                    // Index of the block to process now       // 当前被执行的 block
static uint8_t block_buffer_head;                    // Index of the next block to be pushed    // 下一个待写入的 block（缓冲区末尾）
static uint8_t next_buffer_head;                     // Index of the next buffer head           // 下一个待写入的 block（缓冲区末尾的下一个位置）
static uint8_t block_buffer_planned;                 // Index of the optimally planned block    // 最后一个被“规划”过的 block（用于前瞻优化）

// Define planner variables
typedef struct
{
  int32_t position[N_AXIS];         // The planner position of the tool in absolute steps. Kept separate
                                    // from g-code position for movements requiring multiple line motions,
                                    // i.e. arcs, canned cycles, and backlash compensation.
  float previous_unit_vec[N_AXIS];  // Unit vector of previous path line segment
  float previous_nominal_speed_sqr; // Nominal speed of previous path line segment
} planner_t;
static planner_t pl;

// Returns the index of the next block in the ring buffer. Also called by stepper segment buffer.
uint8_t plan_next_block_index(uint8_t block_index)
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE)
  {
    block_index = 0;
  }
  return (block_index);
}

// Returns the index of the previous block in the ring buffer
static uint8_t plan_prev_block_index(uint8_t block_index)
{
  if (block_index == 0)
  {
    block_index = BLOCK_BUFFER_SIZE;
  }
  block_index--;
  return (block_index);
}

/*                            PLANNER SPEED DEFINITION
                                     +--------+   <- current->nominal_speed
                                    /          \
         current->entry_speed ->   +            \
                                   |             + <- next->entry_speed (aka exit speed)
                                   +-------------+
                                       time -->

  Recalculates the motion plan according to the following basic guidelines:

    1. Go over every feasible block sequentially in reverse order and calculate the junction speeds
        (i.e. current->entry_speed) such that:
      a. No junction speed exceeds the pre-computed maximum junction speed limit or nominal speeds of
         neighboring blocks.
      b. A block entry speed cannot exceed one reverse-computed from its exit speed (next->entry_speed)
         with a maximum allowable deceleration over the block travel distance.
      c. The last (or newest appended) block is planned from a complete stop (an exit speed of zero).
    2. Go over every block in chronological (forward) order and dial down junction speed values if
      a. The exit speed exceeds the one forward-computed from its entry speed with the maximum allowable
         acceleration over the block travel distance.

  When these stages are complete, the planner will have maximized the velocity profiles throughout the all
  of the planner blocks, where every block is operating at its maximum allowable acceleration limits. In
  other words, for all of the blocks in the planner, the plan is optimal and no further speed improvements
  are possible. If a new block is added to the buffer, the plan is recomputed according to the said
  guidelines for a new optimal plan.

  To increase computational efficiency of these guidelines, a set of planner block pointers have been
  created to indicate stop-compute points for when the planner guidelines cannot logically make any further
  changes or improvements to the plan when in normal operation and new blocks are streamed and added to the
  planner buffer. For example, if a subset of sequential blocks in the planner have been planned and are
  bracketed by junction velocities at their maximums (or by the first planner block as well), no new block
  added to the planner buffer will alter the velocity profiles within them. So we no longer have to compute
  them. Or, if a set of sequential blocks from the first block in the planner (or a optimal stop-compute
  point) are all accelerating, they are all optimal and can not be altered by a new block added to the
  planner buffer, as this will only further increase the plan speed to chronological blocks until a maximum
  junction velocity is reached. However, if the operational conditions of the plan changes from infrequently
  used feed holds or feedrate overrides, the stop-compute pointers will be reset and the entire plan is
  recomputed as stated in the general guidelines.

  Planner buffer index mapping:
  - block_buffer_tail: Points to the beginning of the planner buffer. First to be executed or being executed.
  - block_buffer_head: Points to the buffer block after the last block in the buffer. Used to indicate whether
      the buffer is full or empty. As described for standard ring buffers, this block is always empty.
  - next_buffer_head: Points to next planner buffer block after the buffer head block. When equal to the
      buffer tail, this indicates the buffer is full.
  - block_buffer_planned: Points to the first buffer block after the last optimally planned block for normal
      streaming operating conditions. Use for planning optimizations by avoiding recomputing parts of the
      planner buffer that don't change with the addition of a new block, as describe above. In addition,
      this block can never be less than block_buffer_tail and will always be pushed forward and maintain
      this requirement when encountered by the plan_discard_current_block() routine during a cycle.

  NOTE: Since the planner only computes on what's in the planner buffer, some motions with lots of short
  line segments, like G2/3 arcs or complex curves, may seem to move slow. This is because there simply isn't
  enough combined distance traveled in the entire buffer to accelerate up to the nominal speed and then
  decelerate to a complete stop at the end of the buffer, as stated by the guidelines. If this happens and
  becomes an annoyance, there are a few simple solutions: (1) Maximize the machine acceleration. The planner
  will be able to compute higher velocity profiles within the same combined distance. (2) Maximize line
  motion(s) distance per block to a desired tolerance. The more combined distance the planner has to use,
  the faster it can go. (3) Maximize the planner buffer size. This also will increase the combined distance
  for the planner to compute over. It also increases the number of computations the planner has to perform
  to compute an optimal plan, so select carefully. The Arduino 328p memory is already maxed out, but future
  ARM versions should have enough memory and speed for look-ahead blocks numbering up to a hundred or more.

*/
static void planner_recalculate()
{
  // Initialize block index to the last block in the planner buffer.
  // 指向最后一个block（buffer_head所处的数据是最后的数据）
  uint8_t block_index = plan_prev_block_index(block_buffer_head);

  // 缓冲区中只有一个 block 未进行过规划，直接返回
  if (block_index == block_buffer_planned)
  {
    return;
  }

  // Reverse Pass: Coarsely maximize all possible deceleration curves back-planning from the last
  // block in buffer. Cease planning when the last optimal planned or tail pointer is reached.
  // NOTE: Forward pass will later refine and correct the reverse pass to create an optimal plan.
  float entry_speed_sqr;
  plan_block_t *next;
  plan_block_t *current = &block_buffer[block_index];

  // Calculate maximum entry speed for last block in buffer, where the exit speed is always zero.
  current->entry_speed_sqr = min(current->max_entry_speed_sqr, 2 * current->acceleration * current->millimeters);

  block_index = plan_prev_block_index(block_index);
  if (block_index == block_buffer_planned)
  { // Only two plannable blocks in buffer. Reverse pass complete.
    // Check if the first block is the tail. If so, notify stepper to update its current parameters.
    if (block_index == block_buffer_tail)
    {
      st_update_plan_block_parameters();  //把
    }
  }
  else
  { // Three or more plan-able blocks
    while (block_index != block_buffer_planned)
    {
      next = current;
      current = &block_buffer[block_index];
      block_index = plan_prev_block_index(block_index);

      // Check if next block is the tail block(=planned block). If so, update current stepper parameters.
      if (block_index == block_buffer_tail)
      {
        st_update_plan_block_parameters();
      }

      // Compute maximum entry speed decelerating over the current block from its exit speed.
      if (current->entry_speed_sqr != current->max_entry_speed_sqr)
      {
        entry_speed_sqr = next->entry_speed_sqr + 2 * current->acceleration * current->millimeters;
        if (entry_speed_sqr < current->max_entry_speed_sqr)
        {
          current->entry_speed_sqr = entry_speed_sqr;
        }
        else
        {
          current->entry_speed_sqr = current->max_entry_speed_sqr;
        }
      }
    }
  }

  // Forward Pass: Forward plan the acceleration curve from the planned pointer onward.
  // Also scans for optimal plan breakpoints and appropriately updates the planned pointer.
  next = &block_buffer[block_buffer_planned]; // Begin at buffer planned pointer
  block_index = plan_next_block_index(block_buffer_planned);
  while (block_index != block_buffer_head)
  {
    current = next;
    next = &block_buffer[block_index];

    // Any acceleration detected in the forward pass automatically moves the optimal planned
    // pointer forward, since everything before this is all optimal. In other words, nothing
    // can improve the plan from the buffer tail to the planned pointer by logic.
    if (current->entry_speed_sqr < next->entry_speed_sqr)
    {
      entry_speed_sqr = current->entry_speed_sqr + 2 * current->acceleration * current->millimeters;
      // If true, current block is full-acceleration and we can move the planned pointer forward.
      if (entry_speed_sqr < next->entry_speed_sqr)
      {
        next->entry_speed_sqr = entry_speed_sqr; // Always <= max_entry_speed_sqr. Backward pass sets this.
        block_buffer_planned = block_index;      // Set optimal plan pointer.
      }
    }

    // Any block set at its maximum entry speed also creates an optimal plan up to this
    // point in the buffer. When the plan is bracketed by either the beginning of the
    // buffer and a maximum entry speed or two maximum entry speeds, every block in between
    // cannot logically be further improved. Hence, we don't have to recompute them anymore.
    if (next->entry_speed_sqr == next->max_entry_speed_sqr)
    {
      block_buffer_planned = block_index;
    }
    block_index = plan_next_block_index(block_index);
  }
}

void plan_reset()
{
  memset(&pl, 0, sizeof(planner_t)); // Clear planner struct
  block_buffer_tail = 0;
  block_buffer_head = 0;    // Empty = tail
  next_buffer_head = 1;     // plan_next_block_index(block_buffer_head)
  block_buffer_planned = 0; // = block_buffer_tail;
}

void plan_discard_current_block()
{
  if (block_buffer_head != block_buffer_tail)
  { // Discard non-empty buffer.
    uint8_t block_index = plan_next_block_index(block_buffer_tail);
    // Push block_buffer_planned pointer, if encountered.
    if (block_buffer_tail == block_buffer_planned)
    {
      block_buffer_planned = block_index;
    }
    block_buffer_tail = block_index;
  }
}

plan_block_t *plan_get_current_block()
{
  if (block_buffer_head == block_buffer_tail)
  {
    return (NULL);
  } // Buffer empty
  return (&block_buffer[block_buffer_tail]);
}

float plan_get_exec_block_exit_speed()
{
  uint8_t block_index = plan_next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head)
  {
    return (0.0);
  }
  return (sqrt(block_buffer[block_index].entry_speed_sqr));
}

// Returns the availability status of the block ring buffer. True, if full.
uint8_t plan_check_full_buffer()
{
  if (block_buffer_tail == next_buffer_head)
  {
    return (true);
  }
  return (false);
}

/* 向缓冲区添加直线运动指令
   target[N_AXIS]：各轴目标位置（毫米制，带符号的绝对坐标）
   feed_rate：运动速度（若启用反时进给模式，该值表示频率，即完成运动需1/feed_rate分钟）
   传入规划器的位置数据必须基于机床坐标系（与工件坐标系解耦，由G代码解析器处理坐标变换）
   注意：默认缓冲区可用（状态校验由上层motion_control模块处理），缓冲区头尾指针永不重合
   进给率参数有三种解析模式：
     1) 常规模式(invert_feed_rate=false)：直接作为进给速度(mm/min)
     2) 反时模式(invert_feed_rate=true)：转换为时间约束(分钟/次)
     3) 快速定位模式(feed_rate<0 且 invert_feed_rate=false)：触发G00级快速运动 */
#ifdef USE_LINE_NUMBERS
void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number)
#else
void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate)
#endif
{
  // ================= 运动块初始化 =================
  plan_block_t *block = &block_buffer[block_buffer_head]; // 获取环形缓冲区头部块
  block->step_event_count = 0;                            // 步进事件计数器清零
  block->millimeters = 0;                                 // 运动距离清零
  block->direction_bits = 0;                              // 方向位集清零（每位代表轴运动方向）
  block->acceleration = SOME_LARGE_VALUE;                 // 初始化为极大值，后续按轴限制缩放
#ifdef USE_LINE_NUMBERS
  block->line_number = line_number; // 关联G代码行号（用于异常追踪）
#endif

  // ============== 运动学参数计算 ==============
  int32_t target_steps[N_AXIS];     // 目标位置步数
  float unit_vec[N_AXIS], delta_mm; // 单位向量 & 轴位移
  uint8_t idx;

// --- COREXY并联机构特殊处理 ---
#ifdef COREXY
  // A/B电机步数计算：ΔA = |ΔX + ΔY|, ΔB = |ΔX - ΔY|
  target_steps[A_MOTOR] = lround(target[A_MOTOR] * settings.steps_per_mm[A_MOTOR]);
  target_steps[B_MOTOR] = lround(target[B_MOTOR] * settings.steps_per_mm[B_MOTOR]);
  block->steps[A_MOTOR] = labs((target_steps[X_AXIS] - pl.position[X_AXIS]) + (target_steps[Y_AXIS] - pl.position[Y_AXIS]));
  block->steps[B_MOTOR] = labs((target_steps[X_AXIS] - pl.position[X_AXIS]) - (target_steps[Y_AXIS] - pl.position[Y_AXIS]));
#endif

  // --- 各轴步进计算与运动方向判定 ---
  for (idx = 0; idx < N_AXIS; idx++)
  {
// 笛卡尔机构标准计算
#ifndef COREXY
    target_steps[idx] = lround(target[idx] * settings.steps_per_mm[idx]);           // 目标步数(码值)=输入位置(mm)×步距当量
    block->steps[idx] = labs(target_steps[idx] - pl.position[idx]);                 // 轴步数=目标步数-当前位置步数（码值）
    block->step_event_count = max(block->step_event_count, block->steps[idx]);      // 取最大轴步数为基准步数
    delta_mm = (target_steps[idx] - pl.position[idx]) / settings.steps_per_mm[idx]; // 轴位移(mm)
#else
    // COREXY模式下XYZ轴独立计算（AB轴已单独处理）
    if (!(idx == A_MOTOR) && !(idx == B_MOTOR))
    {
      target_steps[idx] = lround(target[idx] * settings.steps_per_mm[idx]);
      block->steps[idx] = labs(target_steps[idx] - pl.position[idx]);
    }
    block->step_event_count = max(block->step_event_count, block->steps[idx]);
    // COREXY位移重构（非欧氏距离）
    if (idx == A_MOTOR)
    {
      delta_mm = (target_steps[X_AXIS] - pl.position[X_AXIS] + target_steps[Y_AXIS] - pl.position[Y_AXIS]) / settings.steps_per_mm[idx];
    }
    else if (idx == B_MOTOR)
    {
      delta_mm = (target_steps[X_AXIS] - pl.position[X_AXIS] - target_steps[Y_AXIS] + pl.position[Y_AXIS]) / settings.steps_per_mm[idx];
    }
    else
    {
      delta_mm = (target_steps[idx] - pl.position[idx]) / settings.steps_per_mm[idx];
    }
#endif

    unit_vec[idx] = delta_mm; // 暂存轴位移，用于后续计算
    // 设置方向位：位移为负时置位方向位
    if (delta_mm < 0)
    {
      block->direction_bits |= get_direction_pin_mask(idx);
    }
    // 累加位移平方（用于欧氏距离计算）
    block->millimeters += delta_mm * delta_mm;  //三个轴正交，三个循环计算得到线段在空间里移动的真实距离是s*s=x*x+y*y+z*z
  }


  block->millimeters = sqrt(block->millimeters); // 实际运动距离=√(ΣΔ²)

  // ============== 运动参数校验 ==============
  if (block->step_event_count == 0)
  {
    return;
  } // 零长度指令拦截
  // 进给率模式转换
  if (feed_rate < 0)
  {
    feed_rate = SOME_LARGE_VALUE;
  } // 负值触发快速定位
  else if (invert_feed_rate)
  {
    feed_rate *= block->millimeters;
  } // 反时模式：速度=距离×频率
  if (feed_rate < MINIMUM_FEED_RATE)
  {
    feed_rate = MINIMUM_FEED_RATE;
  } // 防数值下溢

  // ============== 速度前瞻控制 ==============
  float inverse_unit_vec_value;
  float inverse_millimeters = 1.0 / block->millimeters;   // 单位向量归一化因子
  float junction_cos_theta = 0; // 路径转角余弦（用于速度平滑）
  // 单位向量归一化 & 动力学约束
  for (idx = 0; idx < N_AXIS; idx++)
  {
    if (unit_vec[idx] != 0)
    {
      unit_vec[idx] *= inverse_millimeters; // 单位向量归一化
      inverse_unit_vec_value = fabs(1.0 / unit_vec[idx]);
      // 已知轴的最大速度限制倒推实际进给速度限制
      feed_rate = min(feed_rate, settings.max_rate[idx] * inverse_unit_vec_value);
      block->acceleration = min(block->acceleration, settings.acceleration[idx] * inverse_unit_vec_value);
      // 计算路径转角余弦（点积公式）
      junction_cos_theta -= pl.previous_unit_vec[idx] * unit_vec[idx];
    }
  }

  // --- 路径衔接速度计算 ---
  if (block_buffer_head == block_buffer_tail)
  {                                      // 缓冲区起始块
    block->entry_speed_sqr = 0.0;        // 入口速度=0（从静止启动）
    block->max_junction_speed_sqr = 0.0; // 衔接速度=0
  }
  else
  {
    if (junction_cos_theta > 0.999999)
    { // 直线路径（转角≈0°）
      block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED;
    }
    else
    {                                                              // 曲线路径（转角>0°）
      junction_cos_theta = max(junction_cos_theta, -0.999999);     // 防除零保护
      float sin_theta_d2 = sqrt(0.5 * (1.0 - junction_cos_theta)); // 半角公式 sin(θ/2)=√[(1-cosθ)/2]
      // a = v²/r
      block->max_junction_speed_sqr = max(MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED,
                                          (block->acceleration * settings.junction_deviation * sin_theta_d2) / (1.0 - sin_theta_d2));
    }
  }

  // ============== 运动块最终处理 ==============
  block->nominal_speed_sqr = feed_rate * feed_rate; // 标称速度平方
  // 计算最大入口速度(取转角速度、当前速度和前一个速度的最小值)
  block->max_entry_speed_sqr = min(block->max_junction_speed_sqr,
                                   min(block->nominal_speed_sqr, pl.previous_nominal_speed_sqr));
  // 更新全局状态
  memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // 保存单位向量
  pl.previous_nominal_speed_sqr = block->nominal_speed_sqr; // 保存标称速度
  memcpy(pl.position, target_steps, sizeof(target_steps));  // 更新规划器位置
  // 缓冲区指针更新
  block_buffer_head = next_buffer_head;
  next_buffer_head = plan_next_block_index(block_buffer_head);
  planner_recalculate(); // 触发全局规划重计算
}

// Reset the planner position vectors. Called by the system abort/initialization routine.
void plan_sync_position()
{
  // TODO: For motor configurations not in the same coordinate frame as the machine position,
  // this function needs to be updated to accomodate the difference.
  uint8_t idx;
  for (idx = 0; idx < N_AXIS; idx++)
  {
#ifdef COREXY
    if (idx == X_AXIS)
    {
      pl.position[X_AXIS] = system_convert_corexy_to_x_axis_steps(sys.position);
    }
    else if (idx == Y_AXIS)
    {
      pl.position[Y_AXIS] = system_convert_corexy_to_y_axis_steps(sys.position);
    }
    else
    {
      pl.position[idx] = sys.position[idx];
    }
#else
    pl.position[idx] = sys.position[idx];
#endif
  }
}

// Returns the number of active blocks are in the planner buffer.
uint8_t plan_get_block_buffer_count()
{
  if (block_buffer_head >= block_buffer_tail)
  {
    return (block_buffer_head - block_buffer_tail);
  }
  return (BLOCK_BUFFER_SIZE - (block_buffer_tail - block_buffer_head));
}

// Re-initialize buffer plan with a partially completed block, assumed to exist at the buffer tail.
// Called after a steppers have come to a complete stop for a feed hold and the cycle is stopped.
void plan_cycle_reinitialize()
{
  // Re-plan from a complete stop. Reset planner entry speeds and buffer planned pointer.
  st_update_plan_block_parameters();
  block_buffer_planned = block_buffer_tail;
  planner_recalculate();
}
