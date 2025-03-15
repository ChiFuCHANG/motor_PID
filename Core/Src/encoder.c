#ifndef SRC_Encoder_C_
#define SRC_Encoder_C_

#include "encoder.h"

// 取得當前位置 (單位：弧度)
float ENC_pos(Encoder *e, uint16_t cnt)
{
	float pos = e->pre_pos;
    float rad_per_cnt = 2 * M_PI / (float)e->MTR_PPR;  // 每個計數對應的弧度
    int16_t delta = cnt - e->pre_cnt;

    // 處理計數器溢出
    if (delta > (e->MTR_PPR / 2))
    {
        delta -= e->MTR_PPR;
    }
    else if (delta < -(e->MTR_PPR / 2))
    {
        delta += e->MTR_PPR;
    }

    // 更新計數值
    e->pre_cnt = cnt;

    // 計算並更新位置
    pos += delta * rad_per_cnt;
    return pos;
}

// 取得當前速度 (單位：弧度/秒)
float ENC_vel(Encoder *e, float pos)
{
    float velocity = (pos - e->pre_pos) / (float)SAMP_TIME;
    e->pre_pos=  pos;
    return velocity;
}

#endif /* SRC_Encoder_C_ */

