#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void motor_init();
void motor_pwm(float);

#ifdef __cplusplus
}
#endif
#endif /* INC_MOTOR_H_ */
