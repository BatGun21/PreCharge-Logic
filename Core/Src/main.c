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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R_PRECHARGE 1000000
// Precharge resistance in ohms
#define C_PRECHARGE 10e-6 // Precharge capacitance in farads
#define V_SUPPLY 3.0 // Supply voltage in Volts
#define V_THRESHOLD 0.5 * V_SUPPLY // Voltage threshold for error condition
#define RC_TIME_CONSTANT (R_PRECHARGE * C_PRECHARGE) // RC time constant

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t counter = 0;
uint16_t adc_data;
char errMsg[100];
char succMsg[100];
int switchon = 0;
char msg2 [50];
char msg [20];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void DelayMS(unsigned int time);
void SysTick_Init(uint32_t ticks);
int adc_check (void);
uint16_t adc_rx(void);
void adc_init(void);
void SysTick_Handler(void);
uint16_t read_adc(uint8_t channel);
void Precharge(void);
void ConfigureVoltageSourcePin();
float adcValtoVolts (uint16_t adcVal);
void LED_init(void);
void switchpressed(void);
void Voltage_Measurement(void);
void printTimestamp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ConfigureVoltageSourcePin();
  adc_init();
  LED_init();
  GPIOC->ODR |= 0x02;  // Turn off the precharge relay (pnp transistor)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

//	  switchpressed();
//	  if (switchon){
//	      // Start the precharge process
//		  Precharge();
//	  }
	  Precharge();
	  Voltage_Measurement();
	  DelayMS(100);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

float adcValtoVolts (uint16_t adcVal){
	float Vin = (adcVal/4096.0)*2.9;
	return Vin;
}

void LED_init(void){

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Enabling Clock for GPIOD
	GPIOD->MODER |= GPIO_MODER_MODER14_0; //Set bit 0 to 1 Red
	GPIOD->MODER |= GPIO_MODER_MODER15_0; //Set bit 0 to 1 Blue
	GPIOD->MODER |= GPIO_MODER_MODER13_0; //Set bit 0 to 1 Orange
	GPIOD->MODER |= GPIO_MODER_MODER12_0; //Set bit 0 to 1 Green
}

void switchpressed(void){
	if((GPIOA->IDR & GPIO_IDR_IDR_0) == GPIO_IDR_IDR_0){
		 if (switchon){
			 switchon = 0;
			 GPIOD->ODR &= 0xefff; //Turns off Green LED
		 }else{
			 switchon = 1;
			 GPIOD->ODR |= 0x1000; //Turns on Green LED
		 }
	}
}

//void EXTI0_1_IRQHandler(void) {
//    if (EXTI->PR & EXTI_PR_PR0) { // Check if EXTI Line 0 triggered the interrupt
//
//        EXTI->PR = EXTI_PR_PR0; // Clear the interrupt pending bit by writing '1' to it
//    }
//}




void Switch_init (void){
	  /*----- USER Switch INIT -----*/

    // Enable the GPIO port clock for user switch (assuming it's on GPIOA)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure the user switch pin (PA0) as input
    GPIOA->MODER &= ~GPIO_MODER_MODER0; // Clear bits
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0; // No pull-up, no pull-down (floating)


//    // Configure EXTI0 to trigger on rising edge
//    EXTI->RTSR |= EXTI_RTSR_TR0; // Trigger on rising edge
//    EXTI->IMR |= EXTI_IMR_MR0; // Enable interrupt on EXTI0
//
//
//    // Enable EXTI0 interrupt in NVIC
//    NVIC_SetPriority(EXTI0_IRQn, 0); // Set priority to 0 (adjust as needed)
//    NVIC_EnableIRQ(EXTI0_IRQn);
}

void Precharge(void) {

	uint16_t adcVal = read_adc(9);
	float Vin = adcValtoVolts(adcVal);
    // Wait for 3 RC time constants
    DelayMS(3 * RC_TIME_CONSTANT);

    // Check if the voltage across the capacitor is above 60% of the supply voltage
    if (Vin < V_THRESHOLD) {
    	GPIOC->ODR |= GPIO_ODR_ODR_1;  // Turn off the precharge relay (pnp transistor)
        sprintf(errMsg, "Error: Check Connection Vol = %.3f V ", Vin);
        HAL_UART_Transmit(&huart2, (uint8_t*)errMsg, strlen(errMsg), 200);
        GPIOD->ODR ^= 0x8000;
        DelayMS(200);
    }else {
        GPIOC->ODR &= ~(GPIO_ODR_ODR_1); // Turn on the relay
        sprintf(succMsg, "Success: Good Connection Vol = %.3f V ", Vin);
        HAL_UART_Transmit(&huart2, (uint8_t*)succMsg, strlen(succMsg), 200);
        GPIOD->ODR &= 0xffff7fff;
    }
}

void ConfigureVoltageSourcePin() {
    // Enable the GPIO port clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Configure PC1 as general purpose output
    GPIOC->MODER |= GPIO_MODER_MODER1_0;

    // Configure PC1 as push-pull
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_1;

    // Configure PC1 to high speed
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;

    // Configure PC1 as pull-up
    GPIOC->PUPDR &= 0x04;

}

void adc_init(void) {
    // Enable the ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Enable the GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Configure PB1 as analog input
    GPIOB->MODER |= GPIO_MODER_MODER1; // Analog mode
    GPIOB->OTYPER |= GPIO_OTYPER_OT1; // Open Drain PB1

    // Configure ADC settings
    ADC1->CR1 &= ~ADC_CR1_RES; // Clear the RES bits for 12-bit resolution
    ADC1->CR2 &= ~ADC_CR2_ALIGN; // Data right-aligned
    ADC1->CR2 |= ADC_CR2_CONT; // Continuous conversion mode
    ADC1->SQR3 &= ~ADC_SQR3_SQ1; // Clear the SQ1 bits
    ADC1->SQR3 |= 9 << ADC_SQR3_SQ1_Pos; // Set the channel number in SQ1 bits (Channel 9 for PB1)
    ADC1->SQR3 |= 8 << ADC_SQR3_SQ1_Pos; // Set the channel number in SQ1 bits (Channel 8 for PB0)
    // Enable the ADC
    ADC1->CR2 |= ADC_CR2_ADON;

    // Wait for ADC to be ready
    DelayMS(100);

    // Start the conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint16_t adc_rx(void){

	uint16_t adcValue = 0;
	adcValue = ADC1->DR;

	return adcValue;
}

uint16_t read_adc(uint8_t channel) {
    // Set the channel in the sequence register
    ADC1->SQR3 = (channel & 0x1F);  // Assuming channel is less than 16

    // Start the conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;  // Start conversion

    // Wait for the end of conversion
    while (!((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC)) {}

    // Read the converted value
    uint16_t result = ADC1->DR;

    return result;
}

int adc_check (void){
	int check = 0;
	if ((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC){
		check = 1;
	}
	return check;
}

void DelayMS(unsigned int time){

	for(int i=0; i<=time; i++){
		while ((SysTick->CTRL & 0x00010000) == 0){
				//Wait for 1 millisec.
		}
	}
}

void SysTick_Init(uint32_t ticks){

	SysTick->CTRL = 0; // Disable SysTick

	SysTick->LOAD = ticks-1; // Set Reload Register

	// Setting Interrupt Priority to the highest
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1);

	SysTick->VAL = 0; // Reset the SysTick counter value

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // Selecting internal clock source
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enabling SysTick exception Request when 0


	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable SysTick

}

void SysTick_Handler(void) {

	if (counter == 0xffffffff) {
        counter = 0; // Reset the counter if the maximum value is reached
    } else {
        counter++; // Increment the counter
    }
}
void Voltage_Measurement(void){
	  adc_data = read_adc(9);
	  // Voltage Measurement 30V
	  float Vin = (adc_data*(2.9)/4095.0);
//			  Vin = Vin*(30.0/2.72); //Correction for Voltage divider
	  Vin += (0.6/30.0)*Vin; //Correction using observation
	  sprintf(msg2, " Vol = %.3f V ", Vin);
	  printTimestamp();
	  HAL_UART_Transmit(&huart2, (uint8_t*)(msg2), strlen(msg2), 200);
}


void printTimestamp(void) {
	sprintf(msg, "   Time = %lu ms:", counter);
	HAL_UART_Transmit(&huart2, (uint8_t*)(msg), strlen(msg), 200);
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
