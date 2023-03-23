/*
 * stepper.h
 *
 *  Created on: Dec 8, 2022
 *      Author: artem
 */

#include "main.h"


#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

typedef struct
{
	GPIO_TypeDef* btn_port;
		uint16_t btn_pin;
		uint8_t btn_oldstate;
	GPIO_TypeDef* end_port;
		uint16_t end_pin;
		uint8_t end_oldstate;
	GPIO_TypeDef* led_port;
		uint16_t led_pin;

	uint8_t block;
} Direction;

typedef enum
{
	Stop = 0,
	MoveForward = 1,
	MoveBack = 2,
	MoveTo = 3,
	MoveCyclic = 4,
} Command;

typedef enum
{
	Wait = 0,
	Run_Forward = 1,
	Run_Backward = 2,
} State;

typedef struct {

	Command currentCommand;
	State state;
	enum microstep
	{
		whole = 1,
		half = 2,
		quarter = 4,
		eighth = 8,
		sixteenth = 16,
	} ms;
	struct Cycle
	{
		int commands[24];
		uint8_t commandsCount;
		uint8_t currentCommand;
		uint8_t is_active;
	} cycle;

	uint8_t cur_ms;
	int curPosition;
	int64_t curPositionMM;
	int targetPosition;
	uint16_t timerPeriod; //  5 999 ---> 49 999
	uint16_t countTimer;
	int speed;

	GPIO_TypeDef* enable_port;
		uint16_t enable_pin;
	GPIO_TypeDef* direction_port;
		uint16_t direction_pin;
	GPIO_TypeDef* step_port;
		uint16_t step_pin;
	GPIO_TypeDef* reset_port;
		uint16_t reset_pin;

	GPIO_TypeDef* ms1_port;
		uint16_t ms1_pin;
	GPIO_TypeDef* ms2_port;
		uint16_t ms2_pin;
	GPIO_TypeDef* ms3_port;
		uint16_t ms3_pin;

	GPIO_TypeDef* stat_GN_port;
		uint16_t stat_GN_pin;
	GPIO_TypeDef* stat_RD_port;
		uint16_t stat_RD_pin;

	Direction forward;
	Direction backward;
} Stepper;


void stepper_tick(Stepper *stepper);
void light_on(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void light_off(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void stat_leds_init(Stepper *stepper,
		GPIO_TypeDef* stat_GN_port, uint16_t stat_GN_pin,
		GPIO_TypeDef* stat_RD_port, uint16_t stat_RD_pin);
uint8_t check_ender(Direction *dir);
void stepper_setPositionZero(Stepper *stepper);
void stepper_moveForward(Stepper *stepper);
void stepper_moveBack(Stepper *stepper);
void stepper_moveTo(Stepper *stepper, int target);
void stepper_checkCyclic(Stepper *stepper);
void stop_cycle(Stepper *stepper);
void stepper_stop(Stepper *stepper);
void stepper_init(Stepper *stepper,
		GPIO_TypeDef* enable_port, uint16_t enable_pin,
		GPIO_TypeDef* direction_port, uint16_t direction_pin,
		GPIO_TypeDef* step_port, uint16_t step_pin,
		GPIO_TypeDef* reset_port, uint16_t reset_pin );

void buttons_init(Stepper *stepper,
		GPIO_TypeDef* btn_fw_port, uint16_t btn_fw_pin,
		GPIO_TypeDef* btn_bw_port, uint16_t btn_bw_pin   );

void enders_init(Stepper *stepper,
		GPIO_TypeDef* end_fw_port, uint16_t end_fw_pin,
		GPIO_TypeDef* end_bw_port, uint16_t end_bw_pin  );

void check_enders(Stepper *stepper);
void check_buttons(Stepper *stepper);
uint8_t check_button(Direction *dir);

void end_leds_init(Stepper *stepper,
		GPIO_TypeDef* end_led_fw_port, uint16_t end_led_fw_pin,
		GPIO_TypeDef* end_led_bw_port, uint16_t end_led_bw_pin);

void end_leds_check(Stepper *stepper);

void mstep_init(Stepper *stepper,
		GPIO_TypeDef* ms1_port, uint16_t ms1_pin,
		GPIO_TypeDef* ms2_port, uint16_t ms2_pin,
		GPIO_TypeDef* ms3_port, uint16_t ms3_pin);

void stepper_change_ms(Stepper *stepper, uint8_t ms);
void stepper_boost(Stepper *stepper, TIM_HandleTypeDef *timer);
void stepper_stop_boosting(Stepper *stepper, TIM_HandleTypeDef *timer);
void stepper_save_current_position_mkm(Stepper *stepper, int cur_position);

#endif /* SRC_STEPPER_H_ */
