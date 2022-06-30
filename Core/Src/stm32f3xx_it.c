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

int voidCount;

int monsterCount;
int monsterLoc[50][2];
char monsterState[50];

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
char chooseWhichObject();
void setRowObjects(int j);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
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

//  // Update State variables
//  print("test");
  if (menuState != 'g') { // this is game state
	  initGameState();
	  menuState = 'g';

  }

  // Upload on LCD
  switch (menuState) {
  	  case 'z':
  		  clear();
  		  setCursor(0, 0);
  		  print("Doodle Jump");
  		  break;
  	  case 'm':
  		  clear();
  		  setCursor(0, 0);
  		  print("1 - Start   2 - About us");
  		  break;
  	  case 'g':
  		  processTurn();
  		  printGame();
//  		  DEBUG scores
//  		  char buff[20];
//  		  sprintf(buff, "%d %d %d", score, playerHeight, playerHeightInScreen);
//  		  sprintf(buff, "%d", getRandom(0, 9));
//  		  print(buff);
  		  break;
  	  case 'a':
  		  clear();
  		  setCursor(0, 0);
  		  print(" Yosef and Borna");
  		  break;
  }

  // Delay for better visual
  HAL_Delay(1);
  /* USER CODE END TIM2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

int getRandom(int lower, int upper)
{

	int num = (rand() %
	   (upper - lower + 1)) + lower;
    return num;
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
		playerHeightInScreen --;
		playerRow --;

		// delete old monsters states
		for (i = 0; i < boardColumns; i ++) {
			if (board[i][0] == 'm') {
				// shift monster array to left
				for (i = 0; i < monsterCount - 1; i ++ ) {
					monsterLoc[i][0] = monsterLoc[i + 1][0];
					monsterLoc[i][1] = monsterLoc[i + 1][1];
				}
				for (i = 0; i < monsterCount - 1; i ++ ) {
					monsterState[i] = monsterState[i + 1];
				}
				monsterCount --;
			} else if (board[i][0] == 'v') {
				voidCount --;
			}
		}
		for (i = 0; i < monsterCount; i ++ ) {
			monsterLoc[i][1] --;
		}

		for (j = 0; j < boardRows - 1; j ++ ) {
			// replace row[j] with row[j + 1]
			for (i = 0; i < boardColumns; i ++ ) {
				board[i][j] = board[i][j + 1];
			}
		}
		for (i = 0; i < boardColumns; i ++ ) {
			board[i][boardRows - 1] = 'e';
		}
		setRowObjects(boardRows - 1);
	}
	// Void
	if (board[playerCol][playerRow] == 'v') {
		// Die
	}
	// Loose Block
	if (playerRow > 0 && board[playerCol][playerRow - 1] == 'l') {
		board[playerCol][playerRow - 1] = 'e';
	}

	// Monster
	if (playerRow > 0 && board[playerCol][playerRow] == 'm') {
		// Die
	}

    // Move monsters
	for (i = 0; i < monsterCount; i ++ ) {
		if (i ==0)
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
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
	} else if (jumpCount == 0 && playerRow > 0) { // Jump rule
		// Gravity rule
		if (board[playerCol][playerRow - 1] == 'e') {
			movePlayerTo(playerCol, playerRow - 1);
		}


		if (board[playerCol][playerRow - 1] == 'b') { // Jump on simple block
			jumpCount = jumpOnBlock;
		}

		if (board[playerCol][playerRow - 1] == 's') { // Jump on simple block
			jumpCount = jumpOnCoil;
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
				     case 'e':
				    	print(" ");
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

char chooseWhichObject()
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

	int BProb = BProbBase + BProbBase / (sqrt(score)); // as score goes high it will be so hard
	if (getRandom(0, 100) < BProb) {
		return 'b';
	}

	int SProb = SProbBase + SProbBase / (sqrt(score)); // as score goes high it will be so hard
	if (getRandom(0, 100) < SProb) {
		return 's';
	}

	if (score > 20 && monsterCount < 5) {
		int MProb = MProbBase + MProbBase * (sqrt(score)); // as score goes high it will be so hard
		if (MProb > 2 * MProbBase)
			MProb = 2 * MProbBase;
		int t = getRandom(0, 100);

		if (t < MProb) {
			  char buff[20];
			  sprintf(buff, "%d", t);
			  print(buff);
			return 'm';
		}
	}

	int LProb = LProbBase + LProbBase / (sqrt(score)); // as score goes high it will be so hard
	if (getRandom(0, 100) < LProb) {
		return 'l';
	}

	if (score > 20 && voidCount < 5) {
		int VProb = VProbBase + VProbBase * (sqrt(score)); // as score goes high it will be so hard
		if (VProb > 2 * VProbBase)
			VProb = 2 * VProb;
		if (getRandom(0, 100) < VProb) {
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
		char chosen = chooseWhichObject();
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
		}
		board[i][boardRows] = '\0';
		boardTemp[i][j] = '!'; // this means it is the first turn and no value is there
	}
	board[1][0] = 'b';
	board[1][1] = 'p';
	playerRow = 1;
	playerCol = 1;
	playerOn = 'e';
	jumpCount = 0;

	BProbBase = 10;
	SProbBase = 1;
	VProbBase = 1;
	LProbBase = 2;
	MProbBase = 1;

	monsterCount = 0;
	voidCount = 0;

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
//    	  print("9");
       break;
     case 10:
     	 if (menuState == 'z')
     		 menuState = 'm';
       break;
     case 11:
//    	  print("11");
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
    	 else if (menuState == 'g') {
    		 //    		 Player move right
			 movePlayerTo((playerCol + 1) % boardColumns, playerRow);
    	 }
       break;
     case 15:
    	 if (menuState == 'm') {
    		 initGameState();
    		 menuState = 'g';
    	 } else if (menuState == 'g') {
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
       break;

     default:
       break;
     }
}

/* USER CODE END 1 */

