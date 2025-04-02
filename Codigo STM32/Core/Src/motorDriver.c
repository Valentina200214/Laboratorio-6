/*
 * motorDriver.c
 *
 *  Created on: Mar 6, 2025
 *      Author: Andres
 */

#include "motorDriver.h"
TIM_HandleTypeDef *timmotorGlobal;



void motoresInit(TIM_HandleTypeDef *timmotor, uint32_t canal,uint32_t canal1){

	timmotorGlobal = timmotor;
	HAL_TIM_PWM_Start(timmotorGlobal, canal);
	HAL_TIM_PWM_Start(timmotorGlobal, canal1);
	timmotorGlobal->Instance->CCR1 = 0;
	timmotorGlobal->Instance->CCR2 = 0;
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);

}
void motores(int8_t m1, int8_t m2){

	//Control m1
	if(m1 > 0){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,1);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);

		if(m1 > 100)m1 = 100;
		timmotorGlobal->Instance->CCR1 = m1;
	}
	else if(m1 < 0){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,0);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
		m1 *= -1;

		if(m1 > 100)m1 = 100;
		timmotorGlobal->Instance->CCR1 = m1;
	}
	else{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,0);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
		timmotorGlobal->Instance->CCR1 = 0;
	}


	//Control m2
		if(m2 > 0){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);

			if(m2 > 100)m2 = 100;
			timmotorGlobal->Instance->CCR2 = m2;
		}
		else if(m2 < 0){
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
			m2 *= -1;

			if(m2 > 100)m2 = 100;
			timmotorGlobal->Instance->CCR2 = m2;
		}
		else{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
			timmotorGlobal->Instance->CCR2 = 0;
		}




}
