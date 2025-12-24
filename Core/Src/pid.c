/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
 
#include "pid.h"

/**
  * @brief  init pid parameter
  * @param  pid struct
    @param  parameter
  * @retval None
  */
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
  pid->i_out   = 0.0f;
  pid->err[0]  = 0.0f;
  pid->err[1]  = 0.0f;
}

/**
  * @brief  pid calculation
  * @param  pid struct
    @param  reference value
    @param  feedback value
  * @retval calculation result
  */
float pid_calc(pid_struct_t *pid, float ref, float fdb)
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}
/**
 * @brief 更新累计角度（处理 GM6020 编码器 0~8191 溢出）
 * @param total: 累计角度指针（int32_t）
 * @param current_enc: 当前编码器值（0~8191）
 * @param last_enc: 上一次编码器值指针
 * @return 最新累计角度
 */
int32_t angle_update_total(int32_t *total, uint16_t current_enc, uint16_t *last_enc)
{
  int16_t diff = (int16_t)(current_enc - *last_enc);

  // 处理 13 位编码器溢出（半圈=4096）
  if (diff > 4096) {
    diff -= 8192;
  } else if (diff < -4096) {
    diff += 8192;
  }

  *total += diff;
  *last_enc = current_enc;
  return *total;
}