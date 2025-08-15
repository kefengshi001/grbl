/*
  planner.h - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2011-2015 Sungeun K. Jeon 
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef planner_h
#define planner_h


// The number of linear motions that can be in the plan at any give time
#ifndef BLOCK_BUFFER_SIZE
  #ifdef USE_LINE_NUMBERS
    #define BLOCK_BUFFER_SIZE 16
  #else
    #define BLOCK_BUFFER_SIZE 18
  #endif
#endif

// 此结构体存储G代码块运动的线性位移数据，其关键“标称值”严格遵循原始G代码设定
typedef struct {
  // Bresenham算法追踪直线所需的字段（步进电机执行核心）
  // 注意：步进电机算法依赖以下字段，严禁修改
  uint8_t direction_bits;    // 运动方向位集（对应config.h中的*_DIRECTION_BIT宏）
  uint32_t steps[N_AXIS];    // 各轴步进计数
  uint32_t step_event_count; // 本区块执行所需的最大轴步数（步进事件总数）

  // 运动规划器管理加速度的字段（速度前瞻控制核心）
  float entry_speed_sqr;         // 当前规划的交界入口速度（单位：(mm/min)²）--在planner_recalculate()中计算
  float max_entry_speed_sqr;     // 基于交界限速与邻接标称速度计算的最大允许入口速度（单位：(mm/min)²）
  float max_junction_speed_sqr;  // 基于运动方向向量计算的交界入口限速（单位：(mm/min)²）
  float nominal_speed_sqr;       // 轴速限制调整后的本区块标称速度（单位：(mm/min)²）
  float acceleration;            // 轴加速度限制调整后的线性加速度（单位：mm/min²）
  float millimeters;             // 本区块待执行的剩余位移（单位：mm）
  // uint8_t max_override;       // 基于轴速限制的最大速度倍率值（可选）

  #ifdef USE_LINE_NUMBERS
    int32_t line_number;        // 关联的G代码行号（调试追踪用）
  #endif
} plan_block_t;

      
// Initialize and reset the motion plan subsystem
void plan_reset();

// Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position 
// in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
#ifdef USE_LINE_NUMBERS
  void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number);
#else
  void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate);
#endif

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the current block. Returns NULL if buffer empty
plan_block_t *plan_get_current_block();

// Called periodically by step segment buffer. Mostly used internally by planner.
uint8_t plan_next_block_index(uint8_t block_index);

// Called by step segment buffer when computing executing block velocity profile.
float plan_get_exec_block_exit_speed();

// Reset the planner position vector (in steps)
void plan_sync_position();

// Reinitialize plan with a partially completed block
void plan_cycle_reinitialize();

// Returns the number of active blocks are in the planner buffer.
uint8_t plan_get_block_buffer_count();

// Returns the status of the block ring buffer. True, if buffer is full.
uint8_t plan_check_full_buffer();

#endif
