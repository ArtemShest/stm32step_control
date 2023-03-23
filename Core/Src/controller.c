/*
 * controller.c
 *
 *  Created on: Mar 10, 2023
 *      Author: artem
 */
#include "controller.h"

#include "stepper.h"
#include "w5100s.h"

Stepper stepperA;
Stepper stepperB;
W5100s w5100s;

Message cmd;
Global_Command command;

void controller()
{
	check_buttons(&stepperA);
	check_buttons(&stepperB);

	stepper_tick(&stepperA);
	stepper_tick(&stepperB);


	uint8_t comm = check_commandFromEth();
	if (comm)
	{
		do_command(check_targetStepper(cmd.target), comm); //and send answer
	}
	stepper_checkCyclic(&stepperA);
	stepper_checkCyclic(&stepperB);

	end_leds_check(&stepperA);
	end_leds_check(&stepperB);
	/*
	stepper_cyclic(&stepperA);
	stepper_cyclic(&stepperB); */
}


void steppers_init()
{
	stepper_init(&stepperA,
		 ENABLE_A_GPIO_Port, ENABLE_A_Pin,
		 DIR_A_GPIO_Port, DIR_A_Pin,
		 STEP_A_GPIO_Port, STEP_A_Pin,
		 RESET_A_GPIO_Port, RESET_A_Pin);
	 buttons_init(&stepperA,
			 BTN_FW_A_GPIO_Port, BTN_FW_A_Pin,
			 BTN_BW_A_GPIO_Port, BTN_BW_A_Pin);
	 enders_init(&stepperA,
			 LS1_A_GPIO_Port, LS1_A_Pin,
			 LS2_A_GPIO_Port, LS2_A_Pin);
	 end_leds_init(&stepperA,
			 LED_FW_A_GPIO_Port, LED_FW_A_Pin,
			 LED_BW_A_GPIO_Port, LED_BW_A_Pin);
	 mstep_init(&stepperA,
			 MS1_A_GPIO_Port, MS1_A_Pin,
			 MS2_A_GPIO_Port, MS2_A_Pin,
			 MS3_A_GPIO_Port, MS3_A_Pin);
	 stat_leds_init(&stepperA,
			 STAT_GN_A_GPIO_Port, STAT_GN_A_Pin,
			 STAT_RD_A_GPIO_Port, STAT_RD_A_Pin);


// ------------- init stepper B ----------------
 stepper_init(&stepperB,
		 ENABLE_B_GPIO_Port, ENABLE_B_Pin,
		 DIR_B_GPIO_Port, DIR_B_Pin,
		 STEP_B_GPIO_Port, STEP_B_Pin,
		 RESET_B_GPIO_Port, RESET_B_Pin);
	 buttons_init(&stepperB,
			 BTN_FW_B_GPIO_Port, BTN_FW_B_Pin,
			 BTN_BW_B_GPIO_Port, BTN_BW_B_Pin);
	 enders_init(&stepperB,
			 LS1_B_GPIO_Port, LS1_B_Pin,
			 LS2_B_GPIO_Port, LS2_B_Pin);
	 end_leds_init(&stepperB,
			 LED_FW_B_GPIO_Port, LED_FW_B_Pin,
			 LED_BW_B_GPIO_Port, LED_BW_B_Pin);
	 mstep_init(&stepperB,
			 MS1_B_GPIO_Port, MS1_B_Pin,
			 MS2_B_GPIO_Port, MS2_B_Pin,
			 MS3_B_GPIO_Port, MS3_B_Pin);
	 stat_leds_init(&stepperB,
			 STAT_GN_B_GPIO_Port, STAT_GN_B_Pin,
			 STAT_RD_B_GPIO_Port, STAT_RD_B_Pin);

		stepper_change_ms(&stepperA, sixteenth);
		stepper_change_ms(&stepperB, sixteenth);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //sleep A
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //sleep B


		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); //reset A
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //reset B

		HAL_Delay(1);

		HAL_GPIO_WritePin(stepperA.enable_port, stepperA.enable_pin, GPIO_PIN_SET); //enable A
		HAL_Delay(1);

		HAL_GPIO_WritePin(stepperB.enable_port, stepperB.enable_pin, GPIO_PIN_SET); //enable B
		HAL_Delay(1);
}


uint8_t check_commandFromEth()
{
	uint8_t commm = 0;
	if (w5100s_check() != Continue)
	{
		if (w5100s.is_new_command)
		{
			w5100s.is_new_command = 0;
			commm = parse_message();
		}
	}
	return commm;
}

Stepper* check_targetStepper(uint8_t target)
{
	Stepper *stp;
	if (target == 0)	stp = &stepperA;
	else if (target == 1)  stp = &stepperB;
	return stp;
}

void do_command(Stepper *stp, uint8_t command)
{
	if (cmd.comm != get_cur_pos)
	{
		stp->cycle.is_active = 0;
		for (uint8_t i = 0; i < stp->cycle.commandsCount; i++)
			stp->cycle.commands[i] = 0;
		stp->cycle.commandsCount = 0;
	}

	switch(cmd.comm)
	{
	case stop:
		stepper_stop(stp);
		break;
	case run_forward:
		if (stp->forward.block == 0)
		{
			if ((stp->currentCommand == MoveForward)
					|| (stp->currentCommand == MoveBack))
				stepper_stop(stp);
			else stepper_moveForward(stp);
		}
		break;
	case run_backward:
		if (stp->backward.block == 0)
		{
			if ((stp->currentCommand == MoveForward)
					|| (stp->currentCommand == MoveBack))
				stepper_stop(stp);
			else stepper_moveBack(stp);
		}
		break;
	case run_cycle:
		CyclicDataParseFromArr(stp);
		stp->cycle.currentCommand = 0;
		stp->cycle.is_active = 1;
		break;
	case set_position_zero:
		stepper_setPositionZero(stp);
		break;
	case run_zero:
		stepper_moveTo(stp, 0);
		break;
	case run_to:
		stepper_moveTo(stp, dataParseFromArr());
		break;
	case shift_on:
		if (stp->currentCommand == MoveTo)
			stepper_moveTo(stp, dataParseFromArr() + stp->targetPosition);
		else if((stp->currentCommand == MoveForward) || (stp->currentCommand == MoveBack))
			stepper_moveTo(stp, dataParseFromArr() + stp->curPosition);
		else stepper_moveTo(stp, dataParseFromArr() + stp->curPosition);
		break;
	case get_cur_pos:
		break;
	case change_ms:
		stepper_change_ms(stp, data_toMS());
		break;
	}

	w5100s_sendAns(stepperA.curPosition, stepperB.curPosition, stepperA.curPositionMM, stepperB.curPositionMM);
}


uint8_t data_toMS()
{
	if (cmd.comm == change_ms)
	{
		switch(cmd.data[0])
		{
			case whole:
			case half:
			case quarter:
			case eighth:
			case sixteenth:
				return cmd.data[0];
			default:
				return 0;
		}
	}
}

void CyclicDataParseFromArr(Stepper *stp)
{
	if (cmd.comm == run_cycle)
	{
		stp->cycle.commandsCount = cmd.data[0];
		for (uint8_t i=0; i< stp->cycle.commandsCount; i++)
		{
			for (uint8_t j=1; j<5; j++)
			{
				stp->cycle.commands[i] += cmd.data[j+(i*4)] << (8*(4-j));
			}
		}
	}
}

int dataParseFromArr()
{
	if ((cmd.comm == run_to) || (cmd.comm == shift_on))
	{
		int dataInt = 0;
		uint8_t unitMeasure = cmd.data[0];
		for(uint8_t i = 1; i < 5; i++)
		{
			dataInt += (cmd.data[i] << (8*(4-i)));
		}
		if (unitMeasure == 2) //требуется перевод из мкм в шаги
		{
			// dataInt =      5 микрон на 16 шагов
			int8_t remains = dataInt % 5;
			dataInt = (dataInt - remains) / 5 * 16;
			// округлить до числа, которое делится на 5, умножить на 16
		}
		return dataInt;
	}
	else return 0;
}


uint8_t parse_message()
{
	cmd.comm = w5100s.recieve_msg[4]; //get command nomber

	cmd.target = w5100s.recieve_msg[5]; //target stepper

	cmd.data_size = w5100s.recieve_msg_size - 6;

	for(uint8_t i = 0; i < cmd.data_size; i++)
	{
		cmd.data[i] =  w5100s.recieve_msg[i+6];
	}

	return cmd.comm;
}


void check_buttons(Stepper *stepper)
{
	switch (check_button(&(stepper->forward))) //проверка нажатия на кнопку движения вперед
	{
		case 1:
			if (stepper->cycle.is_active) stop_cycle(stepper);
			if (stepper->forward.block == 0)
				new_command(stepper, MoveForward);
			break;
		case 2:
			new_command(stepper, Stop);
			break;
	}
	switch (check_button(&(stepper->backward))) //проверка нажатия на кнопку движения назад
	{
		case 1:
			if (stepper->cycle.is_active) stop_cycle(stepper);
			if (stepper->backward.block == 0)
				new_command(stepper, MoveBack);
			break;
		case 2:
			new_command(stepper, Stop);
			break;
	}
	switch (check_ender(&(stepper->forward))) //проверка нажатия на концевик движения вперед
	{
		case 1:
			if (stepper->cycle.is_active) stop_cycle(stepper);
			new_command(stepper, Stop);
			break;
		case 2:
			break;
	}
	switch (check_ender(&(stepper->backward))) //проверка нажатия на концевик движения назад
	{
		case 1:
			if (stepper->cycle.is_active) stop_cycle(stepper);
			new_command(stepper, Stop);
			break;
		case 2:
			break;
	}
}
