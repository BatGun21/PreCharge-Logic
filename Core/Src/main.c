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
#define R_PRECHARGE 250 // in ohms
#define C_PRECHARGE 1000e-6 // in farads
#define RC_TIME_CONSTANT (R_PRECHARGE * C_PRECHARGE)// in ms
#define DEBOUNCE_DELAY 200 // in ms
#define MAX_SUPPLY_VOLTAGE 61 // in Volts
#define MIN_SUPPLY_VOLTAGE 30 // in Volts
#define MAX_SENSE_VOLTAGE 61 // in Volts
#define MIN_SENSE_VOLTAGE 30 // in Volts
#define Number_of_Samples 5
#define Variance 2 // in Volts
#define Clock_Frequency 16000 //KHz
#define ADC_CHANNEL 9
#define GPIO_PORT_PCHG GPIOC
#define GPIO_PORT_LEDS GPIOD
#define GPIO_PORT_ADC GPIOB
#define GPIO_PORT_SWITCH GPIOA
#define GPIO_PIN_PCHG_RELAY GPIO_ODR_ODR_4
#define GPIO_PIN_CONTACTOR_RELAY GPIO_ODR_ODR_1
#define GPIO_PIN_SWITCH GPIO_IDR_ID0
#define ON 1
#define OFF 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t adc_data;

char errMsg[100];
char succMsg[100];
int switchon = 0;
char msg2 [50];
char msg [20];
int counter = 0;
int delayActiveFlag = 0;
int current = 0;
int V_threshold = 0;
float avg_v_supply = 0;
float avg_V_in = 0;

float V_supply_arr [Number_of_Samples] = {0,0,0,0,0};
float V_in_arr [Number_of_Samples] = {0,0,0,0,0};

char supplyError [50] = "Critical: Supply is out of Range ";
char noiseError [50] = "Noise error / Check Connection ";
char timeOut[50] = "7RC timeout has reached";
char killSwitch[50] =  "kill switch was pressed";
volatile uint32_t debounceTimer = 0;
int killSwitchFlagRE = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void SysTick_Init(uint32_t ticks);
uint16_t adc_rx(void);
void adc_init(void);
void SysTick_Handler(void);
uint16_t read_adc(uint8_t channel);
void Precharge(void);
void ConfigureVoltageSourcePin();
float adcValtoVolts (uint16_t adcVal);
void LED_init(void);
void switchpressed(void);
void Voltage_Print(void);
void printTimestamp(void);
void DelayMSW(unsigned int time);
float Average(float array[], int size);
void sense_V_supply(void);
void sense_V_in(void);
float Avg_V_in (void);
float Avg_V_Supply (void);
void EXTI_Init(void);
void supplySenseLoop (void);
int time_expired (int delayTime, int currentTime);
int debounceSwitch(int pin);
void PreChargeRelayCTRL(int state);
void ContactorRelayCTRL(int state);
int PreChargeRelayIsOff(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Wait {
	int currentTime;
	int delayTime ;
	int activeFlag;

};

struct Wait Three_RC = {0,3*RC_TIME_CONSTANT,0};
struct Wait Seven_RC = {0,7*RC_TIME_CONSTANT,0};
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
  SysTick_Init(Clock_Frequency);
  EXTI_Init();
  ContactorRelayCTRL(OFF);
  PreChargeRelayCTRL(OFF);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */


	  if (killSwitchFlagRE){
		  int pin = 0;
		  pin  = debounceSwitch(GPIO_PORT_SWITCH->IDR & GPIO_PIN_SWITCH);
		  if (pin){
			  PreChargeRelayCTRL(OFF);
			  ContactorRelayCTRL(OFF);
			  while(1){
				//Halt Operation
			  }
		  }

	  }else{
		  if(PreChargeRelayIsOff()) {
			  supplySenseLoop();
		  }else{
			  Precharge();
		  }
	  }
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

int debounceSwitch(int pin){
	int currPin = 0;
	int temp = 0;
	temp = pin;
	DelayMSW(1);
	if (pin==temp){

		DelayMSW(1);
		if (pin==temp){
		  	currPin = temp;
		}
	}else{
		currPin = pin;
	}
	return currPin;
}


void EXTI0_IRQHandler(void) {
    // Check if the interrupt was triggered by PA0
    if (EXTI->PR & EXTI_PR_PR0) {
        killSwitchFlagRE = 1;
     }

        // Clear the EXTI0 pending flag
        EXTI->PR = EXTI_PR_PR0;
}

void EXTI_Init(void) {
    // Enable the GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Enable SYSCFG clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure PA0 as input
    GPIO_PORT_SWITCH->MODER &= ~GPIO_MODER_MODER0; // Clear bits
    GPIO_PORT_SWITCH->PUPDR &= ~GPIO_PUPDR_PUPDR0; // No pull-up, no pull-down

    // Connect EXTI0 to PA0
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0_Msk;

    // Configure EXTI0 to trigger on rising edge
    EXTI->IMR |= EXTI_IMR_MR0; // Enable interrupt on EXTI0
    EXTI->RTSR |= EXTI_RTSR_TR0; // Trigger on rising edge

    // Enable EXTI0 interrupt in NVIC
    NVIC_SetPriority(EXTI0_IRQn, 0); // Set priority to 0
    NVIC_EnableIRQ(EXTI0_IRQn);
}

float Avg_and_remove_outliers_V_Supply (void){

	float V_supply = 0;
	float V_mean = 0;

	V_mean = Average(V_supply_arr, Number_of_Samples);

	for (int i = 0; i < Number_of_Samples; i++){
		if (!(abs(V_mean - V_supply_arr[i]) < Variance)){
			V_supply_arr[i] = 0.0;
		}
	}

	V_supply = Average(V_supply_arr, Number_of_Samples);
	return V_supply;
}

float Avg_and_remove_outliers_V_in (void){

	float V_in = 0;
	float V_mean = 0;

	V_mean = Average(V_in_arr, Number_of_Samples);

	for (int i = 0; i < Number_of_Samples; i++){
		if (!(abs(V_mean - V_in_arr[i]) < Variance)){
			V_in_arr[i] = 0.0;
		}
	}

	V_in = Average(V_in_arr, Number_of_Samples);
	return V_in;
}

void sense_V_supply(void){

	int i = 0;

	while (i<Number_of_Samples){

		uint16_t adcVal = read_adc(ADC_CHANNEL);
		float V_supply = adcValtoVolts(adcVal);

		if ((V_supply >= MIN_SUPPLY_VOLTAGE) && (V_supply <= MAX_SUPPLY_VOLTAGE)){
			V_supply_arr[i] = V_supply;
			i++;
		}else{
			HAL_UART_Transmit(&huart2, (uint8_t*)supplyError, strlen(supplyError), 100); //supply out of range
		}
	}
}

void sense_V_in(void){

	int i = 0;

	while (i<Number_of_Samples){

		uint16_t adcVal = read_adc(ADC_CHANNEL);
		float V_in = adcValtoVolts(adcVal);

		if ((V_in >= MIN_SENSE_VOLTAGE) && (V_in <= MAX_SENSE_VOLTAGE)){
			V_in_arr[i] = V_in;

		}else{
			V_in_arr[i] = V_in;
			HAL_UART_Transmit(&huart2, (uint8_t*)noiseError, strlen(noiseError), 100); //For Noise
		}
		i++;

	}
}

float adcValtoVolts (uint16_t adcVal){
	float Vin = (adcVal/4096.0)*2.9;
	Vin = Vin*(48.0/2.70); //Correction for Voltage divider for 48V
	Vin += (0.6/30.0)*Vin; //Correction using observation
	return Vin;
}

void LED_init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Enabling Clock for GPIOD
	GPIO_PORT_LEDS->MODER |= GPIO_MODER_MODER14_0; //Set bit 0 to 1 Red
	GPIO_PORT_LEDS->MODER |= GPIO_MODER_MODER15_0; //Set bit 0 to 1 Blue
	GPIO_PORT_LEDS->MODER |= GPIO_MODER_MODER13_0; //Set bit 0 to 1 Orange
	GPIO_PORT_LEDS->MODER |= GPIO_MODER_MODER12_0; //Set bit 0 to 1 Green
}

int time_expired (int delayTime, int currentTime){
	int timeExpiredFlag = 0;
	if (counter> currentTime+delayTime){
		timeExpiredFlag = 1;
	}else{
		timeExpiredFlag = 0;
	}
	return timeExpiredFlag;
}

void Precharge(void) {

	if (!Three_RC.activeFlag){
		Three_RC.currentTime = counter;
		Three_RC.activeFlag = 1;
	}
	if (!Seven_RC.activeFlag){
		Seven_RC.currentTime = counter;
		Seven_RC.activeFlag = 1;
	}
	if (time_expired(Three_RC.delayTime, Three_RC.currentTime)){

		sense_V_in();
		avg_V_in = Avg_and_remove_outliers_V_in();

	    if (avg_V_in <= V_threshold) {

	    	ContactorRelayCTRL(OFF); // Turn off the contactor relay // This turn off is essential to turn of when the supply is cut off
	        GPIO_PORT_LEDS->ODR &= 0xffff7fff;//State LED OFF

	        if (time_expired(Seven_RC.delayTime, Seven_RC.currentTime)){

	        	Seven_RC.activeFlag = 0;
	        	PreChargeRelayCTRL(OFF);

	        }

	    }else {

	        ContactorRelayCTRL(ON);
	        GPIO_PORT_LEDS->ODR |= 0x8000;//State LED ON

	    }
	    Three_RC.activeFlag = 0;
	}
}

void ConfigureVoltageSourcePin(void) {
    // Enable the GPIO port clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Configure PC1 as general purpose output
    GPIO_PORT_PCHG->MODER |= GPIO_MODER_MODER1_0;

    // Configure PC1 as open-drain for using a Pull Up Resistor
    GPIO_PORT_PCHG->OTYPER |= GPIO_OTYPER_OT_1;

    // Configure PC1 to high speed
    GPIO_PORT_PCHG->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;

    // Configure PC4 as general purpose output
    GPIO_PORT_PCHG->MODER |= GPIO_MODER_MODER4_0;

    // Configure PC4 as open-drain
    GPIO_PORT_PCHG->OTYPER |= GPIO_OTYPER_OT_4;
    // Using a Pull Up Resistor

    // Configure PC4 to high speed
    GPIO_PORT_PCHG->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;
}

void adc_init(void) {
    // Enable the ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Enable the GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Configure PB1 as analog input
    GPIO_PORT_ADC->MODER |= GPIO_MODER_MODER1; // Analog mode
    GPIO_PORT_ADC->OTYPER |= GPIO_OTYPER_OT1; // Open Drain PB1

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
    DelayMSW(100);

    // Start the conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint16_t read_adc(uint8_t channel) {
    // Set the channel in the sequence register
    ADC1->SQR3 = (channel & 0x1F);  // Assuming channel is less than 16

    // Start the conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for the end of conversion
    while (!((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC)) {}

    // Read the converted value
    uint16_t result = ADC1->DR;

    return result;
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

void DelayMSW(unsigned int time){
	for(int i=0; i<=time; i++){
		while ((SysTick->CTRL & 0x00010000) == 0){
				//Wait for 1 millisec.
		}
	}
}

void SysTick_Handler(void) {

	if (counter == 0xffffffff) {
        counter = 0; // Reset the counter if the maximum value is reached
    } else {
        counter++; // Increment the counter
    }

    if (debounceTimer > 0) {
        debounceTimer--;
    }

}

void Voltage_Print(void){ // Debug
	  uint16_t adcVal = read_adc(ADC_CHANNEL);
	  float Vin = adcValtoVolts(adcVal);
	  sprintf(msg2, " Vol = %.3f V ", Vin);
	  printTimestamp();
	  HAL_UART_Transmit(&huart2, (uint8_t*)(msg2), strlen(msg2), 200);
}

void printTimestamp(void) { // Debug
	sprintf(msg, " Time = %d ms:", counter);
	HAL_UART_Transmit(&huart2, (uint8_t*)(msg), strlen(msg), 200);
}

float Average(float array[], int size) {
    float sum = 0.0;
    float count = 0;
    for (int i = 0; i < size; i++) {
        if (array[i] != 0) {
            sum += array[i];
            count++;
        }
    }
    // Avoid division by zero
    if (count == 0) {
        return 0.0;
    }
    return sum / count;
}


void supplySenseLoop (void){
	  do{
		  sense_V_supply();
		  avg_v_supply = Avg_and_remove_outliers_V_Supply();

		  V_threshold = 0.9 * avg_v_supply;

		  PreChargeRelayCTRL(ON);
		  DelayMSW(50); // Wait for connection to stable

	  }while (!((avg_v_supply >= MIN_SUPPLY_VOLTAGE) && (avg_v_supply <= MAX_SUPPLY_VOLTAGE)));
}

void PreChargeRelayCTRL(int state){
	if (state){
		GPIO_PORT_PCHG->ODR &= ~(GPIO_PIN_PCHG_RELAY); // PreCharge Relay is ON
	}else{
		GPIO_PORT_PCHG->ODR |= GPIO_PIN_PCHG_RELAY;  // Turn off the precharge relay (pnp transistor)
	}
}

void ContactorRelayCTRL(int state){
	if (state){
		GPIO_PORT_PCHG->ODR &= ~(GPIO_PIN_CONTACTOR_RELAY); // Turn on the contactor relay
	}else{
		GPIO_PORT_PCHG->ODR |= GPIO_PIN_CONTACTOR_RELAY;  // Turn off the Contactor relay (pnp transistor)
	}
}

int PreChargeRelayIsOff(void){
	int state = 0;
	if ((GPIO_PORT_PCHG->ODR & GPIO_PIN_PCHG_RELAY)== GPIO_PIN_PCHG_RELAY){
		state = 1;
	}
	return state;
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
