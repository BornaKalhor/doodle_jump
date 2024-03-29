/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define boardRows 20
#define boardColumns 4
#define jumpOnBlock 7
#define jumpOnCoil 20
#define maxObjects 14
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern char menuState;
char board[boardColumns][boardRows + 1];
char boardTemp[boardColumns][boardRows + 1];
int playerRow, playerCol;
char playerOn;
int jumpCount;

int playerHeight;
int playerHeightInScreen;
int score;

int BProbBase;
int SProbBase;
int VProbBase;
int LProbBase;
int MProbBase;

int lastBlockHeightInScreen;

int voidCount;
int blockCount;
int boosterCount;
int looseCount;

int playerFalling;

int pauseGame;

int bulletRow, bulletCol;

int monsterCount;
int monsterLoc[50][2];
char monsterState[50];

int turn = 0;
int D0, D1, D2, D3;

int v, vol;
int degree;

int buzzLen;

RTC_TimeTypeDef mytime;



char timeStr[20] = {' '};
char dateStr[20] = {' '};

unsigned char showarr[30] = {' '};

GPIO_TypeDef *const Row_ports[] = {GPIOD, GPIOD, GPIOD, GPIOD};
const uint16_t Row_pins[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};
GPIO_TypeDef *const Column_ports[] = {GPIOD, GPIOD, GPIOD, GPIOD};
const uint16_t Column_pins[] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0};
volatile uint32_t last_gpio_exti;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void keypadCallback(int8_t column_number);
//char* getStr(char m);
void printGame();
void initGameState();
void processTurn();
void movePlayerTo(int toCol, int toRow);
int getRandom(int lower, int upper);
char chooseWhichObject(int j);
void setRowObjects(int j);
void fireBullet();
void set_seg_value(int D);
void setScoreSeven();

void show(int i);

void buzz(int len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(OSC_IN_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */
  keypadCallback(3);
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_12);
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(OSC_OUT_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */
  keypadCallback(2);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1);
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 and Touch Sense controller.
  */
void EXTI2_TSC_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_TSC_IRQn 0 */

  /* USER CODE END EXTI2_TSC_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_TSC_IRQn 1 */
  keypadCallback(1);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);
  /* USER CODE END EXTI2_TSC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(CS_I2C_SPI_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, 1);
  keypadCallback(0);
  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9);
	unsigned char buff[512] = {' '};
//  // Update State variables
//  print("test");
//  if (menuState != 'g') { // this is game state
//	  initGameState();
//	  menuState = 'g';
//
//  }

  // Upload on LCD
	unsigned char hello[64] = "\n\n\nTurn started \n";
/*	HAL_UART_Transmit(&huart2, hello, sizeof(hello), 500);							*/
  switch (menuState) {
  	  case 'z':
  		  clear();
  		  setCursor(0, 0);
  		  print("Doodle Jump");
  		  break;
	  case 'd':
	  	clear();
		  setCursor(0, 0);
		  print("You Lost ");
		  setCursor(0, 1);
		  sprintf(buff, " %d ", score);
		  print(buff);
	   	break;
  	  case 'm':
  		  clear();
  		  setCursor(0, 0);
  		  print("1 - Start   2 - About us");
  		  break;
  	  case 'g':

		sprintf(buff, "DEBUG: \n ph:%d, phscreen:%d sc:%d \n", playerHeight, playerHeightInScreen, score);
/*		HAL_UART_Transmit(&huart2, buff, sizeof(buff), 500);							*/
		sprintf(buff, "DEBUG: \n bl:%d, ls:%d, vd:%d, ms:%d, bs:%d \n",
			blockCount, looseCount, voidCount, monsterCount, boosterCount);
/*		HAL_UART_Transmit(&huart2, buff, sizeof(buff), 500);							*/
		if (!pauseGame)
			processTurn();
		printGame();
//  		  DEBUG scores
//  		  char buff[20];
//  		  sprintf(buff, "%d %d %d", score, playerHeight, playerHeightInScreen);
//  		  sprintf(buff, "%d", getRandom(0, 9));
//  		  print(buff);
  		  int pc = 0;
  		  int i, j;
  		  for (i = 0; i < boardColumns; i ++) {
  			  for (j = 0;j < boardRows; j ++ ) {
  				  if (board[i][j] == 'p')
  					  pc ++;
  			  }
  		  }
  		  if (pc > 1) {
				unsigned char hello[64] = "******* HOLY SHIT \n";
/*  			  HAL_UART_Transmit(&huart2, hello, sizeof(hello), 500);					*/
  		  }
  		  break;
  	  case 'a':
  		  clear();
  		  setCursor(0, 0);
  		  print(" Yosef and Borna");
  		  break;
  }

  // Delay for better visual
  HAL_Delay(1);

  v=HAL_ADC_GetValue(&hadc4);
  degree = ((((((v)*200)/(4095))*50) +50) / 100) / 11;
//  degree = (((v)*30)/(4095))*50+500;
  //int step = degree / 50;

  //if(degree < 0) degree = 0;
  //if(degree > 9) degree = 9;
  show(degree);

//  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);

//  HAL_Delay(200);
  HAL_ADC_Start_IT(&hadc4);





  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

//  v=HAL_ADC_GetValue(&hadc4);
//  degree = (((v)*10)/(4095))/**50+500*/;
//
//  show(degree);
//
//  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
//
////  HAL_Delay(200);
//  HAL_ADC_Start_IT(&hadc4);
//

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles Timer 6 interrupt and DAC underrun interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  	  if (buzzLen > 0)
  		  	 buzzLen --;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, buzzLen % 100);
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */
//  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_14);
//  turn = 2;
  setScoreSeven();

  if (turn % 4 == 0) {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
	  set_seg_value(D0);
  } else if (turn % 4 == 1) {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	  set_seg_value(D1);
  } else if (turn % 4 == 2) {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
//	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
	  set_seg_value(D2);
  } else if (turn % 4 == 3) {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);
	  set_seg_value(D3);
  }

  turn = (turn + 1) % 4;
  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles ADC4 interrupt.
  */
void ADC4_IRQHandler(void)
{
  /* USER CODE BEGIN ADC4_IRQn 0 */

  /* USER CODE END ADC4_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc4);
  /* USER CODE BEGIN ADC4_IRQn 1 */
//  HAL_ADC_Start_IT(&hadc4);
  /* USER CODE END ADC4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

int getRandom(int lower, int upper)
{

	int num = (rand() %
	   (upper - lower + 1)) + lower;
    return num;
}

void fireBullet() {
	if (playerFalling || board[playerCol][playerRow + 1] != 'e' || bulletCol != -1)
		return;
	buzz(50);
	bulletCol = playerCol;
	bulletRow = playerRow + 1;
	board[bulletCol][bulletRow] = '*';
}

void processTurn()
{
	int i, j;
	/*
	 * board
	 * 	'e': empty
	 * 	'b': simple block
	 * 	's': spoil or coil block
	 * 	'v': void
	 *  'l': loose block
	 *  'm': monster
	 *  '!': null and not valid
	 */
	// Shift screen to down
	if (playerHeightInScreen > 12) {
	    // unsigned char hello[64] = "SHIFT DOWN \n";
		// HAL_UART_Transmit(&huart2, hello, sizeof(hello), 500);
		playerHeightInScreen --;
		playerRow --;
		lastBlockHeightInScreen --;
		bulletRow --;

		// delete old monsters states
		// unsigned char hello3[64] = "SHIFTING MONSTERS \n";
		// HAL_UART_Transmit(&huart2, hello3, sizeof(hello3), 500);
		for (i = 0; i < boardColumns; i ++) {
			if (board[i][0] == 'm') {
				// shift monster array to left
				// unsigned char hello5[64] = "SHIFTING MONSTERS \n";
				// HAL_UART_Transmit(&huart2, hello5, sizeof(hello5), 500);
				for (j = 0; j < monsterCount - 1; j ++ ) {
					monsterLoc[j][0] = monsterLoc[j + 1][0];
					monsterLoc[j][1] = monsterLoc[j + 1][1];
				}
				for (j = 0; j < monsterCount - 1; j ++ ) {
					monsterState[j] = monsterState[j + 1];
				}
				monsterCount --;
			} else if (board[i][0] == 'v') {
				voidCount --;
			} else if (board[i][0] == 'b') {
				blockCount --;
			} else if (board[i][0] == 's') {
				boosterCount --;
			} else if (board[i][0] == 'l') {
				looseCount --;
			}
		}
		for (i = 0; i < monsterCount; i ++ ) {
			monsterLoc[i][1] --;
		}
		// 		unsigned char hello4[64] = "SHIFTING BOARD DOWN \n";
		// HAL_UART_Transmit(&huart2, hello4, sizeof(hello4), 500);
		for (j = 0; j < boardRows - 1; j ++ ) {
			// replace row[j] with row[j + 1]
			for (i = 0; i < boardColumns; i ++ ) {
				board[i][j] = board[i][j + 1];
			}
		}
		for (i = 0; i < boardColumns; i ++ ) {
			board[i][boardRows - 1] = 'e';
		}
		// unsigned char hello2[64] = "Selecting new objects \n";
		// HAL_UART_Transmit(&huart2, hello2, sizeof(hello2), 500);
		setRowObjects(boardRows - 1);
		// unsigned char hello1[64] = "Selected objects for new row \n";
		// HAL_UART_Transmit(&huart2, hello1, sizeof(hello1), 500);
	}

	// Void
	if (playerRow > 0 && playerOn == 'v') {
		playerFalling = 1;
	}
	// Loose Block
	if (playerRow > 0 && board[playerCol][playerRow - 1] == 'l' && jumpCount < 1) {
		board[playerCol][playerRow - 1] = 'e';
	}

	// Monster
	if (playerRow > 0 && playerOn == 'm') {
		playerFalling = 1;
	}

	// Move bullet
	if (bulletCol != -1 && board[bulletCol][bulletRow + 1] == 'e') {
		board[bulletCol][bulletRow] = 'e';
		bulletRow ++;
		board[bulletCol][bulletRow] = '*';
	} else if (bulletCol != -1 && board[bulletCol][bulletRow + 1] != 'e') {
		board[bulletCol][bulletRow] = 'e';
		if (board[bulletCol][bulletRow + 1] == 'm') {
			for (i = 0; i < monsterCount; i ++) {
				if (monsterLoc[i][0] == bulletCol && monsterLoc[i][1] == bulletRow + 1) {
					for (j = i; j < monsterCount - 1; j ++ ) {
						monsterLoc[j][0] = monsterLoc[j + 1][0];
						monsterLoc[j][1] = monsterLoc[j + 1][1];
					}
					for (j = i; j < monsterCount - 1; j ++ ) {
						monsterState[j] = monsterState[j + 1];
					}
					monsterCount --;
					break;
				}
			}
			buzz(250);
			board[bulletCol][bulletRow + 1] = 'e';
		}
		bulletCol = -1;
		bulletRow = -1;
	}

	// Move monsters
	for (i = 0; i < monsterCount; i ++ ) {
		if (monsterState[i] == 'l') {
			if (monsterLoc[i][0] > 0 && board[monsterLoc[i][0] - 1][monsterLoc[i][1]] == 'e') {
				board[monsterLoc[i][0] - 1][monsterLoc[i][1]] = 'm';
				board[monsterLoc[i][0]][monsterLoc[i][1]] = 'e';
				monsterLoc[i][0] --;
			} else{
				monsterState[i] = 'r';
			}
		} else {
			if (monsterLoc[i][0] < boardColumns - 1 && board[monsterLoc[i][0] + 1][monsterLoc[i][1]] == 'e') {
				board[monsterLoc[i][0] + 1][monsterLoc[i][1]] = 'm';
				board[monsterLoc[i][0]][monsterLoc[i][1]] = 'e';
				monsterLoc[i][0] ++;
			} else {
				monsterState[i] = 'l';
			}

		}
	}

    //	Jump and Gravity
	if (playerRow == 0) {
		// Die
		menuState = 'd';
	} else if (playerFalling) {
		buzz(500);
		movePlayerTo(playerCol, playerRow - 1);
	} else if (jumpCount == 0 && playerRow > 0) { // Jump rule
		// Gravity rule
		if (board[playerCol][playerRow - 1] == 'e' ||
			board[playerCol][playerRow - 1] == 'm' ||
			board[playerCol][playerRow - 1] == 'v') {
			movePlayerTo(playerCol, playerRow - 1);
		}


		if (board[playerCol][playerRow - 1] == 'b') { // Jump on simple block
			jumpCount = jumpOnBlock;
			buzz(100);
		}

		if (board[playerCol][playerRow - 1] == 's') { // Jump on coil block
			jumpCount = jumpOnCoil;
			buzz(140);
		}

	} else if (jumpCount > 0) { // Go up rule
		movePlayerTo(playerCol, playerRow + 1);
		jumpCount --;
	}


}

void movePlayerTo(int toCol, int toRow)
{
	if (playerRow < toRow) {
		playerHeight ++;
		playerHeightInScreen ++;
		if (playerHeight > score)
			score = playerHeight;
	} else if (playerRow > toRow) {
		playerHeight --;
		playerHeightInScreen --;
	}
	if (playerOn != 'm')
		board[playerCol][playerRow] = playerOn;
	playerCol = toCol;
	playerRow = toRow;
	playerOn = board[playerCol][playerRow];
	board[playerCol][playerRow] = 'p';
}

void printGame()
{
	setCursor(0,  0);
	char tmp[2];
	int c, i, j;
	for (c = 0; c < boardColumns; c ++ ) {

		i = c;
		if (c == 1)
			i = 2;
		else if (c == 2)
			i = 1;

		for (j = 0; j < boardRows; j ++ ) {
			if (board[i][j] != boardTemp[i][j]) {
				setCursor(j, i);
				tmp[0] = board[i][j];
				tmp[1] = '\0';
				  switch (board[i][j])
					{
					case 'p':
						write(0);
						break;
					case 'e':
						print(" ");
				       	break;
					case 'b':
						write(1);
						break;
					case 'm':
						write(2);
						break;
					case 's':
						write(3);
						break;
					case 'v':
						write(4);
						break;
					case 'l':
						write(5);
						break;
				     default:
				    	 print(tmp);
				       break;
				     }
				boardTemp[i][j] = board[i][j];
			}
		}
	}
}

char chooseWhichObject(int j)
{
	/*
	 * board
	 * 	'e': empty
	 * 	'b': simple block
	 * 	's': spoil or coil block
	 * 	'v': void
	 *  'l': loose block
	 *  'm': monster
	 *  '!': null and not valid
	 */

	if (j - lastBlockHeightInScreen > 4) {
		lastBlockHeightInScreen = j;
		return 'b';		
	}

	if (blockCount + looseCount + voidCount + monsterCount + boosterCount > maxObjects + degree)
		return 'e';

	int BProb = BProbBase + BProbBase / (sqrt(score)); // as score goes high it will be so hard
	if (getRandom(0, 100) < BProb) {
		lastBlockHeightInScreen = j;
		return 'b';
	}

//	return 'e';

	int SProb = SProbBase + SProbBase / (sqrt(score)); // as score goes high it will be so hard
	if (getRandom(0, 100) < SProb) {
		return 's';
	}

	int LProb = LProbBase + LProbBase / (sqrt(score)); // as score goes high it will be so hard
	if (getRandom(0, 100) < LProb) {
		return 'l';
	}

	if (score > 20 && monsterCount < 4) {
		int MProb = MProbBase + MProbBase * (sqrt(score)); // as score goes high it will be so hard
		if (MProb > 2 * MProbBase)
			MProb = 2 * MProbBase;
		int t = getRandom(0, 100);

		if (t < MProb + degree) {
//			  char buff[20];
//			  sprintf(buff, "%d", t);
//			  print(buff);
			return 'm';
		}
	}


	if (score > 20 && voidCount < 3) {
		int VProb = VProbBase + VProbBase * (sqrt(score)); // as score goes high it will be so hard
		if (VProb > 2 * VProbBase)
			VProb = 2 * VProb;
		if (getRandom(0, 100) < VProb + degree) {
			return 'v';
		}
	}

	return 'e';
}

void setRowObjects(int j)
{
	/*
	 * how to generage a row in this game?
	 * choose witch character should be choosed for this row
	 * choose witch col to place it
	 */
	int i;
	int maxObjectsOnRow = 2;
	for (i = 0; i < boardColumns; i ++ ) {
		char chosen = chooseWhichObject(j);
		if (chosen != 'e') {
			if (chosen == 'm') {
				monsterLoc[monsterCount][0] = i;
				monsterLoc[monsterCount][1] = j;
				int r = getRandom(0, 1);
				if (r == 0)
					monsterState[monsterCount] = 'l'; // go to left
				else
					monsterState[monsterCount] = 'r'; // go to right
				monsterCount ++;
			} else if (chosen == 'v') {
				voidCount ++;
			} else if (chosen == 'b') {
				blockCount ++;
			} else if (chosen == 's') {
				boosterCount ++;
			} else if (chosen == 'l') {
				looseCount ++;
			}
			maxObjectsOnRow --;
			board[i][j] = chosen;
		}
		if (maxObjectsOnRow < 1) break;
	}
}

void initGameState()
{
	/*
	 * board
	 * 	'e': empty
	 * 	'b': simple block
	 * 	's': spoil or coil block
	 * 	'v': void
	 *  'l': loose block
	 *  'm': monster
	 *  '!': null and not valid
	 */
	int i, j;
	for (i = 0; i < boardColumns; i ++) {
		for (j = 0; j < boardRows; j ++) {
			board[i][j] = 'e';
			boardTemp[i][j] = '!'; // this means it is the first turn and no value is there
		}
		board[i][boardRows] = '\0';
	}
	board[1][0] = 'b';
	lastBlockHeightInScreen = 0;
	board[1][1] = 'p';
	playerRow = 1;
	playerCol = 1;
	playerOn = 'e';
	jumpCount = 0;

	bulletRow = -1;
	bulletCol = -1;

	pauseGame = 0;

	BProbBase = 10;
	SProbBase = 1;
	VProbBase = 1;
	LProbBase = 2;
	MProbBase = 1;

	buzzLen = 0;

	monsterCount = 0;
	voidCount = 0;
	blockCount = 1;
	boosterCount = 0;
	looseCount = 0;

	playerFalling = 0;

	score = 1;
	playerHeight = score;
	playerHeightInScreen = playerHeight;

	srand(time(0));

	for (j = 2; j < boardRows; j ++ ) {
		setRowObjects(j);
	}
}

void keypadCallback(int8_t column_number)
{
  if (last_gpio_exti + 250 > HAL_GetTick()) // Simple button debouncing
  {
    return;
  }
  last_gpio_exti = HAL_GetTick();

  int8_t row_number = -1;

  HAL_GPIO_WritePin(Row_ports[0], Row_pins[0], 0);
  HAL_GPIO_WritePin(Row_ports[1], Row_pins[1], 0);
  HAL_GPIO_WritePin(Row_ports[2], Row_pins[2], 0);
  HAL_GPIO_WritePin(Row_ports[3], Row_pins[3], 0);

  for (uint8_t row = 0; row < 4; row++)
  {
    HAL_GPIO_WritePin(Row_ports[row], Row_pins[row], 1);
    if (HAL_GPIO_ReadPin(Column_ports[column_number], Column_pins[column_number]))
    {
      row_number = row;
      break;
    }
    HAL_GPIO_WritePin(Row_ports[row], Row_pins[row], 0);
  }

  HAL_GPIO_WritePin(Row_ports[0], Row_pins[0], 1);
  HAL_GPIO_WritePin(Row_ports[1], Row_pins[1], 1);
  HAL_GPIO_WritePin(Row_ports[2], Row_pins[2], 1);
  HAL_GPIO_WritePin(Row_ports[3], Row_pins[3], 1);

  if (row_number == -1 || column_number == -1)
  {
    return; // Reject invalid scan
  }
  //   C0   C1   C2   C3
  // +----+----+----+----+
  // | 1  | 2  | 3  | 4  |  R0
  // +----+----+----+----+
  // | 5  | 6  | 7  | 8  |  R1
  // +----+----+----+----+
  // | 9  | 10 | 11 | 12 |  R2
  // +----+----+----+----+
  // | 13 | 14 | 15 | 16 |  R3
  // +----+----+----+----+
  const uint8_t button_number = row_number * 4 + column_number + 1;

//  setCursor(curser_column, 0);
  switch (button_number)
     {
     case 1:
//    	  print("1");
       break;
     case 2:
//    	  print("2");
       break;
     case 3:
//    	  print("3");
       break;
     case 4:
//    	  print("4");
       break;
     case 5:
//    	  print("5");
       break;
     case 6:
//    	  print("6");
       break;
     case 7:
//    	  print("7");
       break;
     case 8:
//    	  print("8");
       break;
     case 9:
	 	if (pauseGame) 
		 	pauseGame = 0;
		else 
			pauseGame = 1;
//    	  print("9");
       break;
     case 10:
     	 if (menuState == 'z')
     		 menuState = 'm';
       break;
     case 11:
//    	  print("11");
		fireBullet();
       break;
     case 12:
//    	  print("12");
       break;
     case 13:
//    	 print("13");
       break;
     case 14:
    	 if (menuState == 'm')
    		 menuState = 'a';
    	 else if (menuState == 'g' && !playerFalling) {
    		 //    		 Player move right
			 movePlayerTo((playerCol + 1) % boardColumns, playerRow);
    	 }
       break;
     case 15:
    	 if (menuState == 'm') {
    		 initGameState();
    		 menuState = 'g';
    	 } else if (menuState == 'g' && !playerFalling) {
    		 //    		 Move player left
			 if (playerCol == 0) {
				 movePlayerTo(boardColumns - 1, playerRow);
			 } else {
				 movePlayerTo(playerCol - 1, playerRow);
			 }
    	 }
       break;
     case 16:
    	  if (menuState == 'a')
    		  menuState = 'm';
			else if (menuState == 'd')
				menuState = 'm';
       break;

     default:
       break;
     }
}


void set_seg_value(int D) {
	if (D == 0) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	} else if (D == 1) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	} else if (D == 2) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	} else if (D == 3) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	} else if(D == 4) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	} else if(D == 5) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	} else if(D == 6) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	} else if(D == 7) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	} else if(D == 8) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
	} else if(D == 9) {
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
	}
}

void setScoreSeven() {
	D3 = score % 10;
	D2 = (score / 10) % 10;
	D1 = (score / 100) % 10;
}


void show(int i){
	int show = sprintf(showarr, "difficulty %d \n", i);
	HAL_UART_Transmit(&huart2, showarr, sizeof(showarr), 1000);
	D0 = i;


}


void buzz(int len){
//	int i;
	buzzLen = len;

//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	return;
}
/* USER CODE END 1 */

