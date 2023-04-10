/*
 * stepper.c
 *
 *  Created on: Dec 8, 2022
 *      Author: artem
 */


#include "stepper.h"

void light_on(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, SET);
}

void light_off(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, RESET);
}

void stat_leds_init(Stepper *stepper, GPIO_TypeDef* stat_GN_port, uint16_t stat_GN_pin,
		GPIO_TypeDef* stat_RD_port, uint16_t stat_RD_pin)
{
	stepper->stat_GN_port = stat_GN_port;
	stepper->stat_GN_pin = stat_GN_pin;
	stepper->stat_RD_port = stat_RD_port;
	stepper->stat_RD_pin = stat_RD_pin;

	light_off(stepper->stat_RD_port, stepper->stat_RD_pin);
	light_on(stepper->stat_GN_port, stepper->stat_GN_pin);

}

void stepper_setPositionZero(Stepper *stepper)
{
	stepper->curPosition = 0;
	stepper->curPositionMM = 0;
	stepper->targetPosition = 0;
}

void stepper_moveForward(Stepper *stepper)
{
	stepper->currentCommand = MoveForward;
	stepper->state = Run_Forward;
}

void stepper_moveBack(Stepper *stepper)
{
	stepper->currentCommand = MoveBack;
	stepper->state = Run_Backward;
}

void stepper_moveTo(Stepper *stepper, int target)
{
	stepper->targetPosition = target;

	if (stepper->targetPosition > stepper->curPosition)
	{
		stepper->currentCommand = MoveTo;
		stepper->state = Run_Forward;
	}
	else if (stepper->targetPosition < stepper->curPosition)
	{
		stepper->currentCommand = MoveTo;
		stepper->state = Run_Backward;
	}
	else
	{
		stepper->currentCommand = Stop;
		stepper->state = Wait;

	}
}

void stepper_stop(Stepper *stepper)
{
	stepper->currentCommand = Stop;
	stepper->state = Wait;
	stepper->targetPosition = 0;
}

void stepper_checkCyclic(Stepper *stepper)
{
	if (stepper->cycle.is_active)
	{
		if(stepper->state == Wait)
		{
			if (stepper->curPosition == stepper->cycle.commands[stepper->cycle.currentCommand])
			{
				if(stepper->cycle.currentCommand == stepper->cycle.commandsCount-1)
				{
					stepper->cycle.currentCommand = 0;
				}
				else stepper->cycle.currentCommand++;
			}
			else stepper_moveTo(stepper, stepper->cycle.commands[stepper->cycle.currentCommand]);
		}
	}
}

void stepper_init(Stepper *stepper,
		GPIO_TypeDef* enable_port, uint16_t enable_pin,
		GPIO_TypeDef* direction_port, uint16_t direction_pin,
		GPIO_TypeDef* step_port, uint16_t step_pin,
		GPIO_TypeDef* reset_port, uint16_t reset_pin )
{
	stepper->enable_port = enable_port;
	stepper->enable_pin = enable_pin;
	stepper->direction_port = direction_port;
	stepper->direction_pin = direction_pin;
	stepper->step_port = step_port;
	stepper->step_pin = step_pin;


	stepper->state = Wait;
	stepper->curPosition = 0;
	//stepper->stepDelay = 40;

	stepper->forward.block = 0;
	stepper->backward.block = 0;

}

void buttons_init(Stepper *stepper,
		GPIO_TypeDef* btn_fw_port, uint16_t btn_fw_pin,
		GPIO_TypeDef* btn_bw_port, uint16_t btn_bw_pin   )
{
	stepper->forward.btn_port = btn_fw_port;
	stepper->forward.btn_pin = btn_fw_pin;
	stepper->backward.btn_port = btn_bw_port;
	stepper->backward.btn_pin = btn_bw_pin;
	stepper->timerPeriod = 29999;
}

void enders_init(Stepper *stepper,
		GPIO_TypeDef* end_fw_port, uint16_t end_fw_pin,
		GPIO_TypeDef* end_bw_port, uint16_t end_bw_pin  )
{
	stepper->forward.end_port = end_fw_port;
	stepper->forward.end_pin = end_fw_pin;
	stepper->backward.end_port = end_bw_port;
	stepper->backward.end_pin = end_bw_pin;

	stepper->curPosition = 0;



	//проверка на блокировки движения при включении
	if (HAL_GPIO_ReadPin(stepper->forward.end_port, stepper->forward.end_pin))
		stepper->forward.block = 1;
	else stepper->forward.block = 0;
	if (HAL_GPIO_ReadPin(stepper->backward.end_port, stepper->backward.end_pin))
		stepper->backward.block = 1;
	else stepper->backward.block = 0;
}

uint8_t check_button(Direction *dir)
{
	uint8_t result = 0;
	if (HAL_GPIO_ReadPin(dir->btn_port, dir->btn_pin) != dir->btn_oldstate)
	{
		if((HAL_GPIO_ReadPin(dir->btn_port, dir->btn_pin)) == GPIO_PIN_SET)
		{
			// нажатие кнопки
 		   result = 1;
		}
		else
		{
			// отпускание кнопки
			result = 2;
		}

		dir->btn_oldstate = HAL_GPIO_ReadPin(dir->btn_port, dir->btn_pin);
	}
	return result;

}

uint8_t check_ender(Direction *dir)
{
	uint8_t result = 0;
	if (HAL_GPIO_ReadPin(dir->end_port, dir->end_pin) != dir->end_oldstate)
	{
		if (dir->invert_ender == 0)
		{
			if (!HAL_GPIO_ReadPin(dir->end_port, dir->end_pin))
			{
				// пришло нажатие на концевик
				dir->block = 1;
				result = 1;
			}
			else
			{
				// отпускание концевика
				dir->block = 0;
				result = 2;
			}
		}

		else if (dir->invert_ender == 1)
		{
			if (HAL_GPIO_ReadPin(dir->end_port, dir->end_pin))
			{
				// пришло нажатие на концевик
				dir->block = 1;
				result = 1;
			}
			else
			{
				// отпускание концевика
				dir->block = 0;
				result = 2;
			}
		}
		dir->end_oldstate = HAL_GPIO_ReadPin(dir->end_port, dir->end_pin);
	}
	return result;
}

void new_command(Stepper *stepper, Command cmd)
{
	switch (cmd)
	{
		case Stop:
			stepper_stop(stepper);
			break;
		case MoveForward:
			stepper_moveForward(stepper);
			break;
		case MoveBack:
			stepper_moveBack(stepper);
			break;
	}

}

void stop_cycle(Stepper *stepper)
{
	stepper->cycle.is_active = 0;
	for (uint8_t i = 0; i < stepper->cycle.commandsCount; i++) stepper->cycle.commands[i] = 0;
	stepper->cycle.commandsCount = 0;
}

void end_leds_init(Stepper *stepper,
		GPIO_TypeDef* end_led_fw_port, uint16_t end_led_fw_pin,
		GPIO_TypeDef* end_led_bw_port, uint16_t end_led_bw_pin)
{
	stepper->forward.led_port = end_led_fw_port;
	stepper->forward.led_pin = end_led_fw_pin;
	stepper->backward.led_port = end_led_bw_port;
	stepper->backward.led_pin = end_led_bw_pin;
}

void end_leds_check(Stepper *stepper)
{
	if(stepper->forward.block == 1)
		light_on(stepper->forward.led_port, stepper->forward.led_pin);
	else
		light_off(stepper->forward.led_port, stepper->forward.led_pin);

	if(stepper->backward.block == 1)
		light_on(stepper->backward.led_port, stepper->backward.led_pin);
	else
		light_off(stepper->backward.led_port, stepper->backward.led_pin);

}

void mstep_init(Stepper *stepper,
		GPIO_TypeDef* ms1_port, uint16_t ms1_pin,
		GPIO_TypeDef* ms2_port, uint16_t ms2_pin,
		GPIO_TypeDef* ms3_port, uint16_t ms3_pin)
{
	stepper->ms1_port = ms1_port;
	stepper->ms1_pin = ms1_pin;
	stepper->ms2_port = ms2_port;
	stepper->ms2_pin = ms2_pin;
	stepper->ms3_port = ms2_port;
	stepper->ms3_pin = ms2_pin;
}

void stepper_change_ms(Stepper *stepper, uint8_t ms)
{
	if (ms == 0) return;
	if (ms == 16) stepper->cur_ms = 8;
	stepper->cur_ms = 8;
	switch(ms)
	{
		case whole:
			HAL_GPIO_WritePin(stepper->ms1_port, stepper->ms1_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(stepper->ms2_port, stepper->ms2_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(stepper->ms3_port, stepper->ms3_pin, GPIO_PIN_RESET);
			break;
		case half:
			HAL_GPIO_WritePin(stepper->ms1_port, stepper->ms1_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->ms2_port, stepper->ms2_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(stepper->ms3_port, stepper->ms3_pin, GPIO_PIN_RESET);
			break;
		case quarter: // 1
			HAL_GPIO_WritePin(stepper->ms1_port, stepper->ms1_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(stepper->ms2_port, stepper->ms2_pin, GPIO_PIN_SET);  //// не работает ms2
			HAL_GPIO_WritePin(stepper->ms3_port, stepper->ms3_pin, GPIO_PIN_RESET);
			break;
		case eighth: //  1/2
			HAL_GPIO_WritePin(stepper->ms1_port, stepper->ms1_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->ms2_port, stepper->ms2_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->ms3_port, stepper->ms3_pin, GPIO_PIN_RESET);
			break;
		case sixteenth: // 1/8
			HAL_GPIO_WritePin(stepper->ms1_port, stepper->ms1_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->ms2_port, stepper->ms2_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->ms3_port, stepper->ms3_pin, GPIO_PIN_SET);
			break;
	}

}

void stepper_boost(Stepper *stepper, TIM_HandleTypeDef *timer) //ускорение двигателя
{
	stepper->countTimer++;
	if (stepper->countTimer>30)
	{
		if (stepper->timerPeriod > 2999) // 3999) //7999
		{
			stepper->timerPeriod -= 500; // 700
			__HAL_TIM_SET_AUTORELOAD(timer, stepper->timerPeriod);
		}

		stepper->countTimer = 0;

	}
}

void stepper_slowing(Stepper *stepper, TIM_HandleTypeDef *timer)
{
	stepper->countTimer++;
	if (stepper->countTimer>10)
	{
		if (stepper->timerPeriod < 59999) // 3999) //7999
		{
			stepper->timerPeriod += 1000; // 700
			__HAL_TIM_SET_AUTORELOAD(timer, stepper->timerPeriod);
		}

		stepper->countTimer = 0;
	}
}

void stepper_stop_boosting(Stepper *stepper, TIM_HandleTypeDef *timer)
{
	stepper->timerPeriod = 19999; //19999; //29999
	__HAL_TIM_SET_AUTORELOAD(timer, stepper->timerPeriod);

}

void stepper_step(Stepper *stepper, TIM_HandleTypeDef *htim)
{
	if (stepper->state != Wait)
	{
		HAL_GPIO_TogglePin(stepper->step_port, stepper->step_pin);
		if (HAL_GPIO_ReadPin(stepper->step_port, stepper->step_pin) == SET)
		{
			if (stepper->state == MoveForward) stepper->curPosition++;
			else if (stepper->state == MoveBack) stepper->curPosition--;
			stepper_save_current_position_mkm(stepper, stepper->curPosition);
		}
		if (stepper->currentCommand == MoveTo)
		{
			if (abs(stepper->targetPosition - stepper->curPosition) > 200) stepper_boost(stepper, htim); // 200
			else if (abs(stepper->targetPosition - stepper->curPosition) < 200) // 400
			{
				stepper_slowing(stepper, htim);
//				if (abs(stepper->targetPosition - stepper->curPosition) < 5) stepper_stop_boosting(stepper, htim);
				if (stepper->targetPosition == stepper->curPosition)
				{
					stepper_stop_boosting(stepper, htim);
					stepper_stop(stepper);
				}
			}
		}
		else stepper_boost(stepper, htim);
	}
	else
	{
		stepper_stop_boosting(stepper, htim);
	}
}

void stepper_save_current_position_mkm(Stepper *stepper, int cur_position)
{
	// 100 (400)  шагов на мм в режиме полного шага; 1/400 = мм на один шаг; 1/400/n (n - режим микрошага); 1/400/n * 10 000 000  чтобы не использовать плавающую точку
	stepper->curPositionMM = (int64_t)((int64_t)cur_position * 25000 / stepper->cur_ms); // 25000
}

void stepper_tick(Stepper *stepper)
{
	switch(stepper->currentCommand)
	{
		case(Wait):
			HAL_GPIO_WritePin(stepper->enable_port, stepper->enable_pin, GPIO_PIN_SET);
			break;
		case(MoveForward):
			HAL_GPIO_WritePin(stepper->direction_port, stepper->direction_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(stepper->enable_port, stepper->enable_pin, GPIO_PIN_RESET);
			break;
		case(MoveBack):
			HAL_GPIO_WritePin(stepper->direction_port, stepper->direction_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->enable_port, stepper->enable_pin, GPIO_PIN_RESET);
			break;
		case(MoveTo):
			if (stepper->state == Run_Forward)
			{
				HAL_GPIO_WritePin(stepper->direction_port, stepper->direction_pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(stepper->enable_port, stepper->enable_pin, GPIO_PIN_RESET);
				if (stepper->curPosition >= stepper->targetPosition) stepper_stop(stepper);
			}
			if (stepper->state == Run_Backward)
			{
				HAL_GPIO_WritePin(stepper->direction_port, stepper->direction_pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(stepper->enable_port, stepper->enable_pin, GPIO_PIN_RESET);
				if (stepper->curPosition <= stepper->targetPosition) stepper_stop(stepper);
			}
			break;
	}
}
