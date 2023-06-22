/*
 * File: SERVO_cfg.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "SERVO.h"

const SERVO_CfgType SERVO_CfgParam[SERVO_NUM] =
{
	// Servo Motor 1 Configurations s_1
    {
	    GPIOF,
		GPIO_PIN_7,
		TIM11,
		&TIM11->CCR1,
		TIM_CHANNEL_1,
		72000000,
		0.65,
		2.3
	},
    // Servo Motor 2 Configurations h_1
	{
		GPIOF,
		GPIO_PIN_8,
		TIM13,
		&TIM13->CCR1,
		TIM_CHANNEL_1,
		72000000,
		0.4,
		2.4
	},
	// Servo Motor 3 Configurations k_1
	{
		GPIOE,
		GPIO_PIN_5,
		TIM9,
		&TIM9->CCR1,
		TIM_CHANNEL_1,
		72000000,
		0.4,
				2.4
	},
    // Servo Motor 4 Configurations s_2
	{
		GPIOE,
		GPIO_PIN_6,
		TIM9,
		&TIM9->CCR2,
		TIM_CHANNEL_2,
		72000000,
		0.65,
		2.3
	},
	// Servo Motor 5 Configurations h_2
	{
		GPIOA,
		GPIO_PIN_0,
		TIM2,
		&TIM2->CCR1,
		TIM_CHANNEL_1,
		72000000,
		0.4,
				2.4
	},
	// Servo Motor 6 Configurations K_2
	{
		GPIOB,
		GPIO_PIN_10,
		TIM2,
		&TIM2->CCR3,
		TIM_CHANNEL_3,
		72000000,
		0.4,
				2.4
	},
	// Servo Motor 7 Configurations s_3
	{
		GPIOA,
		GPIO_PIN_3,
		TIM2,
		&TIM2->CCR4,
		TIM_CHANNEL_4,
		72000000,
		0.65,
		2.3
	},
	// Servo Motor 8 Configurations h_3
	{
		GPIOA,
		GPIO_PIN_6,
		TIM3,
		&TIM3->CCR1,
		TIM_CHANNEL_1,
		72000000,
		0.4,
				2.4
	},
	// Servo Motor 9 Configurations k_3
	{
		GPIOC,
		GPIO_PIN_7,
		TIM3,
		&TIM3->CCR2,
		TIM_CHANNEL_2,
		72000000,
		0.4,
				2.4
	},
// Servo Motor 10 Configurations s_4
	{
		GPIOC,
		GPIO_PIN_8,
		TIM3,
		&TIM3->CCR3,
		TIM_CHANNEL_3,
		72000000,
		0.65,
		2.3
	},
	// Servo Motor 11 Configurations h_4
	{
		GPIOB,
		GPIO_PIN_1,
		TIM3,
		&TIM3->CCR4,
		TIM_CHANNEL_4,
		72000000,
		0.4,
				2.4
	},
// Servo Motor 12 Configurations k_4
	{
		GPIOD,
		GPIO_PIN_12,
		TIM4,
		&TIM4->CCR1,
		TIM_CHANNEL_1,
		72000000,
		0.4,
				2.4
	}
};
