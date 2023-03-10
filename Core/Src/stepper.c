/*
 * stepper.c
 *
 *  Created on: Dec 8, 2022
 *      Author: artem
 */


#include "stepper.h"

void stepper_setPositionZero(Stepper *stepper)
{
	stepper->curPosition = 0;
	stepper->curPositionMM = 0;
	stepper->targetPosition = 0;
}

void stepper_moveForward(Stepper *stepper)
{
	stepper->direction = forward;
	stepper->state = MoveForward;
}

void stepper_moveBack(Stepper *stepper)
{
	stepper->direction = backward;
	stepper->state = MoveBack;
}

void stepper_moveTo(Stepper *stepper, int target)
{
	stepper->targetPosition = target;

	if (stepper->targetPosition > stepper->curPosition)
	{
		stepper->state = MoveTo_Forward;
		stepper->direction = forward;
	}
	else if (stepper->targetPosition < stepper->curPosition)
	{
		stepper->state = MoveTo_Backward;
		stepper->direction = backward;
	}
	else stepper->state = Wait;
}

void stepper_stop(Stepper *stepper)
{
	stepper->direction = null;
	stepper->state = Wait;
	stepper->targetPosition = 0;
}


void stepper_cyclic(Stepper *stepper)
{
	if (stepper->cycle.is_active)
	{
		stepper_moveTo(stepper, stepper->cycle.commands[stepper->cycle.currentCommand]);
		if (stepper->curPosition == stepper->cycle.commands[stepper->cycle.currentCommand])
		{
			if(stepper->cycle.currentCommand == stepper->cycle.commandsCount-1)
			{
				stepper->cycle.currentCommand = 0;
			}
			else stepper->cycle.currentCommand++;
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

	stepper->block_fw = 0;
	stepper->block_bw = 0;

}

void buttons_init(Stepper *stepper,
		GPIO_TypeDef* btn_fw_port, uint16_t btn_fw_pin,
		GPIO_TypeDef* btn_bw_port, uint16_t btn_bw_pin   )
{
	stepper->btn_fw_port = btn_fw_port;
	stepper->btn_fw_pin = btn_fw_pin;
	stepper->btn_bw_port = btn_bw_port;
	stepper->btn_bw_pin = btn_bw_pin;
	stepper->timerPeriod = 29999;
}

void enders_init(Stepper *stepper,
		GPIO_TypeDef* end_fw_port, uint16_t end_fw_pin,
		GPIO_TypeDef* end_bw_port, uint16_t end_bw_pin  )
{
	stepper->end_fw_port = end_fw_port;
	stepper->end_fw_pin = end_fw_pin;
	stepper->end_bw_port = end_bw_port;
	stepper->end_bw_pin = end_bw_pin;

	stepper->curPosition = 0;


	//???????????????? ???? ???????????????????? ???????????????? ?????? ??????????????????
	if (HAL_GPIO_ReadPin(stepper->end_fw_port, stepper->end_fw_pin))
		stepper->block_fw = 1;
	else stepper->block_fw = 0;
	if (HAL_GPIO_ReadPin(stepper->end_bw_port, stepper->end_bw_pin))
		stepper->block_bw = 1;
	else stepper->block_bw = 0;
}

void check_buttons(Stepper *stepper)
{
	if (HAL_GPIO_ReadPin(stepper->btn_fw_port, stepper->btn_fw_pin) != stepper->btn_fw_oldstate)
	{
		if((HAL_GPIO_ReadPin(stepper->btn_fw_port, stepper->btn_fw_pin)) == GPIO_PIN_SET)
		{
		 //???????????????? ???????????? ???????? ??
			if (stepper->block_fw == 0)
			{
				if ((stepper->state == MoveForward)
						|| (stepper->state == MoveBack)
						|| (stepper->state == MoveTo_Backward))
					stepper_stop(stepper);
				else stepper_moveForward(stepper);
			}
		}
		else stepper_stop(stepper);

		stepper->btn_fw_oldstate = HAL_GPIO_ReadPin(stepper->btn_fw_port, stepper->btn_fw_pin);
	}



	else if (HAL_GPIO_ReadPin(stepper->btn_bw_port, stepper->btn_bw_pin) != stepper->btn_bw_oldstate)
	{
		if((HAL_GPIO_ReadPin(stepper->btn_bw_port, stepper->btn_bw_pin)) == GPIO_PIN_SET)
		{
		 //???????????????? ???????????? ???????? ??
			if (stepper->block_bw == 0)
			{
				if ((stepper->state == MoveForward)
						|| (stepper->state == MoveBack)
						|| (stepper->state == MoveTo_Forward))
					stepper_stop(stepper);
				else stepper_moveBack(stepper);
			}
		}
		else stepper_stop(stepper);

		stepper->btn_bw_oldstate = HAL_GPIO_ReadPin(stepper->btn_bw_port, stepper->btn_bw_pin);
	}
}

void check_enders(Stepper *stepper)
{
	if (HAL_GPIO_ReadPin(stepper->end_fw_port, stepper->end_fw_pin) != stepper->end_fw_oldstate)
	{
		if (!HAL_GPIO_ReadPin(stepper->end_fw_port, stepper->end_fw_pin))
		{
			stepper_stop(stepper);
			stepper->block_fw = 1;
			HAL_GPIO_WritePin(stepper->end_led_fw_port, stepper->end_led_fw_pin, GPIO_PIN_SET);
		}
		else
		{
			stepper->block_fw = 0;
			HAL_GPIO_WritePin(stepper->end_led_fw_port, stepper->end_led_fw_pin, GPIO_PIN_RESET);
		}

		stepper->end_fw_oldstate = HAL_GPIO_ReadPin(stepper->end_fw_port, stepper->end_fw_pin);
	}
	if (HAL_GPIO_ReadPin(stepper->end_bw_port, stepper->end_bw_pin) != stepper->end_bw_oldstate)
	{
		if (!HAL_GPIO_ReadPin(stepper->end_bw_port, stepper->end_bw_pin))
		{
			stepper_stop(stepper);
			stepper->block_bw = 1;
			HAL_GPIO_WritePin(stepper->end_led_bw_port, stepper->end_led_bw_pin, GPIO_PIN_SET);
		}
		else
		{
			stepper->block_bw = 0;
			HAL_GPIO_WritePin(stepper->end_led_bw_port, stepper->end_led_bw_pin, GPIO_PIN_RESET);
		}
		stepper->end_bw_oldstate = HAL_GPIO_ReadPin(stepper->end_bw_port, stepper->end_bw_pin);
	}
}

void end_leds_init(Stepper *stepper,
		GPIO_TypeDef* end_led_fw_port, uint16_t end_led_fw_pin,
		GPIO_TypeDef* end_led_bw_port, uint16_t end_led_bw_pin)
{
	stepper->end_led_fw_port = end_led_fw_port;
	stepper->end_led_fw_pin = end_led_fw_pin;
	stepper->end_led_bw_port = end_led_bw_port;
	stepper->end_led_bw_pin = end_led_bw_pin;
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

	stepper->cur_ms = ms;
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
		case quarter:
			HAL_GPIO_WritePin(stepper->ms1_port, stepper->ms1_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(stepper->ms2_port, stepper->ms2_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->ms3_port, stepper->ms3_pin, GPIO_PIN_RESET);
			break;
		case eighth:
			HAL_GPIO_WritePin(stepper->ms1_port, stepper->ms1_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->ms2_port, stepper->ms2_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->ms3_port, stepper->ms3_pin, GPIO_PIN_RESET);
			break;
		case sixteenth:
			HAL_GPIO_WritePin(stepper->ms1_port, stepper->ms1_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->ms2_port, stepper->ms2_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->ms3_port, stepper->ms3_pin, GPIO_PIN_SET);
			break;
	}

}

void stepper_boost(Stepper *stepper, TIM_HandleTypeDef *timer) //?????????????????? ??????????????????
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
			if (stepper->direction == forward) stepper->curPosition++;
			else if (stepper->direction == backward) stepper->curPosition--;
			stepper_save_current_position_mkm(stepper, stepper->curPosition);
		}
		if (stepper->state == MoveTo_Forward || stepper->state == MoveTo_Backward)
		{
			if (abs(stepper->targetPosition - stepper->curPosition) > 200) stepper_boost(stepper, htim);
			else if (abs(stepper->targetPosition - stepper->curPosition) < 400) stepper_slowing(stepper, htim);
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
	// 100 ?????????? ???? ???? ?? ???????????? ?????????????? ????????; 1/200 = ???? ???? ???????? ??????; 1/200/n (n - ?????????? ??????????????????); 1/200/n * 10 000 000  ?????????? ???? ???????????????????????? ?????????????????? ??????????
	stepper->curPositionMM = (int64_t)((int64_t)cur_position * 50000 / stepper->cur_ms);
}

void stepper_tick(Stepper *stepper)
{
	switch(stepper->state)
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
		case(MoveTo_Forward):
			HAL_GPIO_WritePin(stepper->direction_port, stepper->direction_pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(stepper->enable_port, stepper->enable_pin, GPIO_PIN_RESET);
			if (stepper->curPosition >= stepper->targetPosition) stepper_stop(stepper);
			break;
		case(MoveTo_Backward):
			HAL_GPIO_WritePin(stepper->direction_port, stepper->direction_pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(stepper->enable_port, stepper->enable_pin, GPIO_PIN_RESET);
			if (stepper->curPosition <= stepper->targetPosition) stepper_stop(stepper);
			break;
	}
}
