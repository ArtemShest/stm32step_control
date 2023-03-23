/*
 * controller.h
 *
 *  Created on: Mar 10, 2023
 *      Author: artem
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_


#include "main.h"
#include "stepper.h"

typedef struct
{
	uint8_t from[4];
	uint8_t comm;
	uint8_t target;  // stepA or stepB [ 0 or 1]
	uint8_t data[100];
	uint8_t data_size;
	uint8_t unitMeasure; // 1 - steps;;;; 2 - mkm
} Message;

typedef enum
{
	stop = 1,
	run_forward = 2,
	run_backward = 3,
	run_cycle = 4,
	set_position_zero = 5,
	run_zero = 6,
	run_to = 7,
	shift_on = 8,
	get_cur_pos = 9,
	change_ip = 10,
	change_ms = 11
} Global_Command;

uint8_t data_toMS();
void steppers_init();
Stepper* check_targetStepper(uint8_t target);
void CyclicDataParseFromArr(Stepper *stp);
int dataParseFromArr();
void do_command(Stepper *stp, uint8_t command);
uint8_t parse_message();
void check_buttons(Stepper *stepper);
void controller();
uint8_t check_commandFromEth();

#endif /* INC_CONTROLLER_H_ */
