#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct{
	float pre_pos;
	uint16_t MTR_PPR;
	int16_t pre_cnt;
} Encoder;

float ENC_pos(Encoder *e, uint16_t cnt);
float ENC_vel(Encoder *e, float pos);

#ifdef __cplusplus
}
#endif
#endif /* INC_ENCODER_H_ */
