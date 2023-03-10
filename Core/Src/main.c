/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stepper.h"
#include "w5100s.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc = 0;
Stepper stepperA;
Stepper stepperB;
W5100s w5100s;

uint8_t ipaddr[4]=IP_ADDR;
uint8_t ipgate[4]=IP_GATE;
uint8_t ipmask[4]=IP_MASK;
uint16_t local_port = 10;

struct msg
{
	uint8_t from[4];
	uint8_t comm;
	uint8_t target;  // stepA or stepB [ 0 or 1]
	uint8_t data[100];
	uint8_t data_size;
	uint8_t unitMeasure; // 1 - steps;;;; 2 - mkm
} cmd;

enum comm
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
	change_ip = 10
} command;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void check_server_state();
void do_command();
void parse_message();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
        if(htim->Instance == TIM1)
        	stepper_step(&stepperA, htim);
        else if(htim->Instance == TIM8)
        	stepper_step(&stepperB, htim);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	// ------------- init stepper A ----------------

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF); // очищаем флаг
   HAL_TIM_Base_Start_IT(&htim1);

   __HAL_TIM_CLEAR_FLAG(&htim8, TIM_SR_UIF); // очищаем флаг
   HAL_TIM_Base_Start_IT(&htim8);

   set_dac(1200);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

     check_enders(&stepperA);
     check_enders(&stepperB);

  w5100s_init();

  printf ("%d ", EEPROM_readState(&hspi3));
  printf ("\r\n");


  for(uint8_t i; i < 20; i++)
  {

	  EEPROM_write(&hspi3, i + 2000, i*2);
	  HAL_Delay(100);
	  printf ("%d ", EEPROM_read(&hspi3, i + 2000));
	  printf ("\r\n");

  }

  printf ("%d ", EEPROM_readState(&hspi3));
  printf ("\r\n");



  while (1)
  {
	  check_buttons(&stepperA);
	  check_buttons(&stepperB);

	  check_enders(&stepperA);
	  check_enders(&stepperB);

	  stepper_tick(&stepperA);
	  stepper_tick(&stepperB);

	  //HAL_Delay(10);

	  check_server_state();
	  stepper_cyclic(&stepperA);
	  stepper_cyclic(&stepperB);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void check_server_state()
{
	if (w5100s_check() != Continue)
	{
		if (w5100s.is_new_command)
		{
			//printf ("new command!!!!\r\n");
			w5100s.is_new_command = 0;
			parse_message();
			do_command(); //and send answer
		}
	}
}

void do_command()
{
	Stepper *stp;
	if (cmd.target == 0)
		stp = &stepperA;
	else if (cmd.target == 1)
		stp = &stepperB;

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
		if (stp->block_fw == 0)
		{
			if ((stp->state == MoveForward)
					|| (stp->state == MoveBack)
					|| (stp->state == MoveTo_Backward))
				stepper_stop(stp);
			else stepper_moveForward(stp);
		}
		break;
	case run_backward:
		if (stp->block_bw == 0)
		{
			if ((stp->state == MoveForward)
					|| (stp->state == MoveBack)
					|| (stp->state == MoveTo_Forward))
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
		if ((stp->state == MoveTo_Forward) || (stp->state == MoveTo_Backward))
			stepper_moveTo(stp, dataParseFromArr() + stp->targetPosition);
		else if((stp->state == MoveForward) || (stp->state == MoveBack))
			stepper_moveTo(stp, dataParseFromArr() + stp->curPosition);
		else stepper_moveTo(stp, dataParseFromArr() + stp->curPosition);
		break;
	case get_cur_pos:
		break;
	}

	w5100s_sendAns(stepperA.curPosition, stepperB.curPosition, stepperA.curPositionMM, stepperB.curPositionMM);
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

void parse_message()
{
	cmd.comm = w5100s.recieve_msg[4]; //get command nomber

	cmd.target = w5100s.recieve_msg[5]; //target stepper

	cmd.data_size = w5100s.recieve_msg_size - 6;

	for(uint8_t i = 0; i < cmd.data_size; i++)
	{
		cmd.data[i] =  w5100s.recieve_msg[i+6];
	    //printf ("%d ", cmd.data[i]);
	}
	//printf ("\r\n");
}

int __io_putchar(int ch)
{
	ITM_SendChar(ch);
	return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
