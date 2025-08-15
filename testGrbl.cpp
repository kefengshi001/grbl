#include <cstdint>

#define N_AXIS 3
#define BLOCK_BUFFER_SIZE 18
#define CARTESIAN_EPSILON 1e-6
#define MINIMUM_JUNCTION_SPEED 0.0
#define JUNCTION_DEVIATION 0.01

typedef struct
{
    // Bresenham算法追踪直线所需的字段（步进电机执行核心）
    // 注意：步进电机算法依赖以下字段，严禁修改
    Direction direction_bits;  // 运动方向位集（对应config.h中的*_DIRECTION_BIT宏）
    uint32_t steps[N_AXIS];    // 各轴步进计数
    uint32_t step_event_count; // 本区块执行所需的最大轴步数（步进事件总数）

    // 运动规划器管理加速度的字段（速度前瞻控制核心）
    float entry_speed_sqr;        // 当前规划的交界入口速度（单位：(mm/min)²）
    float max_entry_speed_sqr;    // 基于交界限速与邻接标称速度计算的最大允许入口速度（单位：(mm/min)²）
    float max_junction_speed_sqr; // 基于运动方向向量计算的交界入口限速（单位：(mm/min)²）
    float nominal_speed_sqr;      // 轴速限制调整后的本区块标称速度（单位：(mm/min)²）
    float acceleration;           // 轴加速度限制调整后的线性加速度（单位：mm/min²）
    float millimeters;            // 本区块待执行的剩余位移（单位：mm）
                                  // uint8_t max_override;       // 基于轴速限制的最大速度倍率值（可选）
} plan_block_t;

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

enum class Direction : int
{ // 显式指定底层类型为 int
    NEGATIVE = -1,
    STOPPED = 0,
    POSITIVE = 1
};

#define SOME_LARGE_VALUE 1.0E+38 // Used by rapids and acceleration maximization calculations. Just needs
                                 // to be larger than any feasible (mm/min)^2 or mm/sec^2 value.

static plan_block_t block_buffer[BLOCK_BUFFER_SIZE]; // A ring buffer for motion instructions
static uint8_t block_buffer_tail;                    // Index of the block to process now       // 当前被执行的 block
static uint8_t block_buffer_head;                    // Index of the next block to be pushed    // 下一个待写入的 block（缓冲区末尾）
static uint8_t next_buffer_head;                     // Index of the next buffer head           // 下一个待写入的 block（缓冲区末尾的下一个位置）
static uint8_t block_buffer_planned;                 // Index of the optimally planned block    // 最后一个被“规划”过的 block（用于前瞻优化）


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


void plan_buffer_line(float *target, float feed_rate)
{
    plan_block_t *block = &block_buffer[block_buffer_head]; // 获取环形缓冲区头部块
    block->millimeters = 0;                                 // 运动距离清零
    block->direction_ = 0;                                  // 方向位集清零（每位代表轴运动方向）
    block->acceleration = SOME_LARGE_VALUE;

    float unit_vec[N_AXIS], delta_mm; // 单位向量 & 轴位移
    uint8_t idx;

    // x/y/z轴步进计算与运动方向判定
    for (idx = 0; idx < N_AXIS; idx++)
    {
        delta_mm = target[idx] - pl.position[idx]; // 计算笛卡尔空间位移
        block->millimeters += delta_mm * delta_mm; // 计算运动距离
        unit_vec[idx] = delta_mm;                  // 暂存轴位移，用于后续计算

        // 设置方向位：位移为正时置位方向位
        block->direction_bits =
            (delta_mm > CARTESIAN_EPSILON) ? Direction::POSITIVE : (delta_mm < -CARTESIAN_EPSILON) ? Direction::NEGATIVE
                                                                                                   : Direction::STOPPED;
    }

    block->millimeters = sqrt(block->millimeters); // 实际运动距离=√(ΣΔ²)

    // 运动参数校验
    float inverse_unit_vec_value;
    float inverse_millimeters = 1.0 / block->millimeters; // 单位向量归一化因子
    float junction_cos_theta = 0;                         // 路径转角余弦（用于速度平滑）

    // 计算路径转角余弦
    for (idx = 0; idx < N_AXIS; idx++)
    {
        if (unit_vec[idx] != 0)
        {

            // // 已知轴的最大速度限制倒推实际进给速度限制<暂不考虑>
            // unit_vec[idx] *= inverse_millimeters; // 单位向量归一化
            // inverse_unit_vec_value = fabs(1.0 / unit_vec[idx]);
            // feed_rate = min(feed_rate, settings.max_rate[idx] * inverse_unit_vec_value);
            // block->acceleration = min(block->acceleration, settings.acceleration[idx] * inverse_unit_vec_value);

            // 计算路径转角余弦（点积公式）
            junction_cos_theta -= pl.previous_unit_vec[idx] * unit_vec[idx];
        }
    }

    // --- 路径衔接速度计算 ---
    if (block_buffer_head == block_buffer_tail)
    {                                        // 缓冲区起始块
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
        {                                                                // 曲线路径（转角>0°）
            junction_cos_theta = max(junction_cos_theta, -0.999999);     // 防除零保护
            float sin_theta_d2 = sqrt(0.5 * (1.0 - junction_cos_theta)); // 半角公式 sin(θ/2)=√[(1-cosθ)/2]
            // a = v²/r
            block->max_junction_speed_sqr = max(MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED,
                                                (block->acceleration * JUNCTION_DEVIATION * sin_theta_d2) / (1.0 - sin_theta_d2));
        }
    }

    //运动块最终处理
    block->nominal_speed_sqr = feed_rate * feed_rate; // 标称速度平方
    // 计算最大入口速度(取转角速度、当前速度和前一个速度的最小值)
    block->max_entry_speed_sqr = min(block->max_junction_speed_sqr,
                                     min(block->nominal_speed_sqr, pl.previous_nominal_speed_sqr));
    // 更新全局状态
    memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // 保存单位向量
    pl.previous_nominal_speed_sqr = block->nominal_speed_sqr; // 保存标称速度平方
    memcpy(pl.position, target, sizeof(int32_t) * N_AXIS);    // 更新规划器位置
    
    // 更新环形缓冲区头指针
    block_buffer_head = next_buffer_head;
    next_buffer_head = plan_next_block_index(next_buffer_head); // 更新下一个待写入的 block 位置

    planner_recalculate(); // 触发全局规划重计算
}