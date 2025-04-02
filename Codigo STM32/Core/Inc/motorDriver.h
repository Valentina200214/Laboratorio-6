/*
 * motorDriver.h
 *
 *  Created on: Mar 6, 2025
 *      Author: Andres
 */

#ifndef INC_MOTORDRIVER_H_
#define INC_MOTORDRIVER_H_

#include "main.h"

void motoresInit(TIM_HandleTypeDef *timmotor, uint32_t canal,uint32_t canal1);
void motores(int8_t m1, int8_t m2);



#endif /* INC_MOTORDRIVER_H_ */
