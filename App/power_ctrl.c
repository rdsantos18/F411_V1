/*
 * power_ctrl.c
 *
 *  Created on: Aug 15, 2023
 *      Author: rinaldo.santos
 */

#include "main.h"
#include "tim.h"
#include "power_ctrl.h"

extern float target_iron, temp_iron;
extern uint16_t pwm_iron;

uint32_t timer_power_iron = 0;
uint8_t flag_power_iron = 0;

void power_iron(void)
{
	if( (HAL_GetTick() - timer_power_iron > 500) && flag_power_iron ) {
		timer_power_iron = HAL_GetTick();
		// Temperatura Menor que TARGET
		if( (float)target_iron != 0.0f && (float)temp_iron < (float)target_iron ) {
			if(pwm_iron <= 4095) pwm_iron++;
			__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_iron);
		}
		// Temperatura Maior que Target
		if( (float)target_iron != 0.0f && (float)temp_iron > (float)target_iron ) {
			if(pwm_iron >= 1) pwm_iron--;
			__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_iron);
		}
	}
}
