/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "motorDriver.h"
#include <math.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buffer[63];
int16_t rpm = 0;
uint8_t set= 1;
float pulsos = 0;
uint32_t tiempo = 0;
int32_t dir_mm;
int16_t pwm_mm = 0;
int32_t val_pwm = 0;
float pulsos_ant = 0;
float pulsos_act = 0;
int32_t contador = 0;
int16_t rpm_f = 0;
float diametro_rueda = 38.2;
int16_t velocidad_mm_s = 0;
int16_t velocidad_rad_s = 0;
int32_t valor;
int32_t voltaje;
int16_t vel_mm_s;
int8_t velocidad_motor;

#define FILTER_LENGTH 10
float fir_coeffs[FILTER_LENGTH] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
float rpm_buffer[FILTER_LENGTH];
int buffer_index = 0;
int32_t contador_actual = 0;
float voltaje_float;

volatile uint8_t indexTX = 0;
volatile uint8_t datosTX [68];
volatile uint8_t indexRX = 0;
volatile uint8_t datosRX [15];


//6 lab

uint16_t adcValue[5] = {0,0,0,0,0};
uint16_t adcMM[5] = {0,0,0,0,0};
float adcVoltaje[5] = {0,0,0,0,0};
char texto[64];

#define VOLTAGE_AVG_LENGTH 10
float voltage_buffer[5][VOLTAGE_AVG_LENGTH];
int voltage_buffer_index = 0;
float voltage_avg[5] = {0, 0, 0, 0, 0};



typedef struct {
	uint8_t inicio;
	uint8_t tamano;
	uint8_t *datos;
	uint8_t crc;
	uint8_t fin;

}PAQUETE;

PAQUETE pk1;

int16_t aplicarFiltroFIR(int16_t RPM_actual);
int8_t serializarPaquete(const PAQUETE* paquete, uint8_t *buffer);

uint8_t calcularCRC(uint8_t *datos, uint8_t tam){
	uint8_t crc =0;
	for(int i = 0; i < tam; i++ ){
		crc ^= datos[i];
	}
	return crc;
}


void datos_enviar(int16_t rev, int32_t contador, uint32_t tiempo, int16_t mm_s, int16_t rad_s,
                  uint16_t adc1, uint16_t adc2, uint16_t adc3, uint16_t adc4,
                  float volt1, float volt2, float volt3, float volt4,
                  uint16_t dist1, uint16_t dist2, uint16_t dist3, uint16_t dist4) {

    int idxx = 0;

    buffer[idxx++] = 1;
    buffer[idxx++] = (rev >> 8) & 0xFF;
    buffer[idxx++] = rev & 0xFF;

    buffer[idxx++] = 2;
    buffer[idxx++] = (contador >> 24) & 0xFF;
    buffer[idxx++] = (contador >> 16) & 0xFF;
    buffer[idxx++] = (contador >> 8) & 0xFF;
    buffer[idxx++] = contador & 0xFF;


    buffer[idxx++] = 3;
    buffer[idxx++] = (tiempo >> 24) & 0xFF;
    buffer[idxx++] = (tiempo >> 16) & 0xFF;
    buffer[idxx++] = (tiempo >> 8) & 0xFF;
    buffer[idxx++] = tiempo & 0xFF;


    buffer[idxx++] = 4;
    buffer[idxx++] = (mm_s >> 8) & 0xFF;
    buffer[idxx++] = mm_s & 0xFF;


    buffer[idxx++] = 5;
    buffer[idxx++] = (rad_s >> 8) & 0xFF;
    buffer[idxx++] = rad_s & 0xFF;


    buffer[idxx++] = 6;
    buffer[idxx++] = (adc1 >> 8) & 0xFF;
    buffer[idxx++] = adc1 & 0xFF;

    buffer[idxx++] = 7;
    buffer[idxx++] = (adc2 >> 8) & 0xFF;
    buffer[idxx++] = adc2 & 0xFF;

    buffer[idxx++] = 8;
    buffer[idxx++] = (adc3 >> 8) & 0xFF;
    buffer[idxx++] = adc3 & 0xFF;

    buffer[idxx++] = 9;
    buffer[idxx++] = (adc4 >> 8) & 0xFF;
    buffer[idxx++] = adc4 & 0xFF;


    buffer[idxx++] = 10;
    uint8_t* volt1_bytes = (uint8_t*)&volt1;
    buffer[idxx++] = volt1_bytes[0];
    buffer[idxx++] = volt1_bytes[1];
    buffer[idxx++] = volt1_bytes[2];
    buffer[idxx++] = volt1_bytes[3];


    buffer[idxx++] = 11;
    uint8_t* volt2_bytes = (uint8_t*)&volt2;
    buffer[idxx++] = volt2_bytes[0];
    buffer[idxx++] = volt2_bytes[1];
    buffer[idxx++] = volt2_bytes[2];
    buffer[idxx++] = volt2_bytes[3];


    buffer[idxx++] = 12;
    uint8_t* volt3_bytes = (uint8_t*)&volt3;
    buffer[idxx++] = volt3_bytes[0];
    buffer[idxx++] = volt3_bytes[1];
    buffer[idxx++] = volt3_bytes[2];
    buffer[idxx++] = volt3_bytes[3];


    buffer[idxx++] = 13;
    uint8_t* volt4_bytes = (uint8_t*)&volt4;
    buffer[idxx++] = volt4_bytes[0];
    buffer[idxx++] = volt4_bytes[1];
    buffer[idxx++] = volt4_bytes[2];
    buffer[idxx++] = volt4_bytes[3];


    buffer[idxx++] = 14;
    buffer[idxx++] = (dist1 >> 8) & 0xFF;
    buffer[idxx++] = dist1 & 0xFF;


    buffer[idxx++] = 15;
    buffer[idxx++] = (dist2 >> 8) & 0xFF;
    buffer[idxx++] = dist2 & 0xFF;

    buffer[idxx++] = 16;
    buffer[idxx++] = (dist3 >> 8) & 0xFF;
    buffer[idxx++] = dist3 & 0xFF;

    buffer[idxx++] = 17;
    buffer[idxx++] = (dist4 >> 8) & 0xFF;
    buffer[idxx++] = dist4 & 0xFF;

    EnviarPaquete(buffer, idxx);
}

void EnviarPaquete(uint8_t *dat, uint8_t tam){
	pk1.inicio = 0x09;
	pk1.tamano = tam + 4; //(OverHead)
	//memcpy (&pk1.datos[0], dat, tam+4);
	pk1.datos = dat;
	pk1.crc = 0x00;
	pk1.fin = 0x07;

	int numDatos = serializarPaquete(&pk1, datosTX);
	CDC_Transmit_FS(datosTX, numDatos);
}

int8_t serializarPaquete(const PAQUETE* paquete, uint8_t *buffer){

	int idx=0;

	if(!paquete || !buffer)return -1;

	buffer [idx++]= paquete -> inicio;
	buffer [idx++]= paquete -> tamano;
	if(paquete -> datos && paquete -> tamano > 3){
		memcpy( & buffer[idx], paquete -> datos, paquete -> tamano - 3);
		idx = idx + paquete -> tamano - 3;
	}
	uint8_t ss = idx;
	buffer[idx++] = calcularCRC(buffer, ss);
	buffer[idx++] = paquete -> fin;
	return idx;
}

void CDC_ReceiveCallBack(uint8_t* Buf, uint32_t Len){
	memcpy(datosRX,Buf, Len);
	indexRX = Len;
}

void vel(int16_t rpm_s) {
    velocidad_mm_s = (3.1416 * diametro_rueda * rpm_s) / 60.0;
    velocidad_rad_s = (2 * 3.1416 * rpm_s) / 60.0;
}

void calculo_rpm() {
    if (set == 1) {
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        set = 0;
    }

    if (__HAL_TIM_GET_COUNTER(&htim3) >= 200) {
        __HAL_TIM_SET_COUNTER(&htim3, 0);

        contador_actual = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        pulsos_act = abs(contador_actual) / 1431.0;
        pulsos = pulsos_act - pulsos_ant;
        pulsos_ant = pulsos_act;

        rpm = abs(pulsos * 600);
        if (rpm < 0) rpm = 0;
        rpm_f = aplicarFiltroFIR(rpm);
        vel(rpm_f);
    }
}

void conversor(float volt){

    if (volt < 0){
    	volt = 0;
    }
    else if (volt > 7.5){
    	volt = 7.5;
    }
    else{
     val_pwm = (int32_t)((volt * (-100)) / (7.5));
    }

}

void conversor_mm_s(int32_t velocidad) {

    if (velocidad > 744){
    	velocidad = 744;
    }

    pwm_mm = ((velocidad*100) / 744) * (-1);
}

void instruction(){

	if(indexRX != 0){
		  if(datosRX[0] == 0x0A){
			  velocidad_motor = 100;
			  motores(0, velocidad_motor);
		  }
		  else if(datosRX[0] == 0x0B){
			  velocidad_motor = -100;
			  motores(0, velocidad_motor);
		  }
		  else if(datosRX[0] == 0x0C){
			  velocidad_motor = 0;
			  motores(0, velocidad_motor);
		  }
		  else if(datosRX[0] == 0x09){
			  valor = (datosRX[1]);

			  velocidad_motor = valor * (-1);
			  motores(0, velocidad_motor);
		  }
		  else if(datosRX[0] == 0x07){

			  memcpy(&voltaje_float, &datosRX[1], sizeof(float));

			  if (voltaje_float < 0) {
				  voltaje_float = 0;
			  }
			  else if (voltaje_float > 7.5) {
				  voltaje_float = 7.5;
			  }

			  conversor(voltaje_float);
			  velocidad_motor = val_pwm;
			  motores(0, velocidad_motor);
		  }
		  else if(datosRX[0] == 0x08){
			  vel_mm_s =(datosRX[1] << 8) | (datosRX[2]);

			  conversor_mm_s(vel_mm_s);
			  velocidad_motor = pwm_mm;
			  motores(0, velocidad_motor);
		  }

	}
}

int16_t aplicarFiltroFIR(int16_t rpm_actual){
	rpm_buffer[buffer_index] = rpm_actual;
	buffer_index = (buffer_index + 1) % FILTER_LENGTH;

	float filtered_rpm = 0.0;
	for (int i = 0; i < FILTER_LENGTH; i++) {
		int idx = (buffer_index - i + FILTER_LENGTH) % FILTER_LENGTH;
		filtered_rpm += fir_coeffs[i] * rpm_buffer[idx];
	}
	return filtered_rpm;
}


int abc;
void calibracion(){
	int vrb = 0;
	__HAL_TIM_SET_COUNTER(&htim4 , 0);
	abc=__HAL_TIM_GET_COUNTER(&htim4);
	while (abc >= 0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);

		while ((abc=__HAL_TIM_GET_COUNTER(&htim4)) >= 2) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);

			while ((abc=__HAL_TIM_GET_COUNTER(&htim4)) >= 4) {
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);

				while ((abc=__HAL_TIM_GET_COUNTER(&htim4)) >= 6) {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);

					if ((abc=__HAL_TIM_GET_COUNTER(&htim4)) >= 8) {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
						vrb = 1;
						break;

					}
				}

				if(vrb == 1){
					break;
				}

			}

			if(vrb == 1){
				break;
			}
		}


		if(vrb == 1){
			break;
		}
	}

}

void calculo_adc() {

    for (int i = 0; i < 5; i++) {
        adcVoltaje[i] = (adcValue[i] * 3.3) / 4095.0;

    }
}



void calculo_mm() {

    adcMM[0] = (uint16_t)((-62.045 * adcVoltaje[0]) + 200);

    adcMM[1] = (uint16_t)((-14.32 * adcVoltaje[1]) + 46);

    adcMM[2] = (uint16_t)((-73.07 * adcVoltaje[2]) + 234);

    adcMM[3] = (uint16_t)((-52.79 * adcVoltaje[3]) + 140);

}



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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  motoresInit(&htim9, TIM_CHANNEL_1, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  HAL_ADC_Start_DMA(&hadc1, adcValue, 5);

  for (int i = 0; i < 5; i++) {
      for (int j = 0; j < VOLTAGE_AVG_LENGTH; j++) {
          voltage_buffer[i][j] = 0.0;
      }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



	  instruction();

	  calculo_rpm();

	  contador = ((int32_t)__HAL_TIM_GET_COUNTER(&htim5)) /1431;
	  tiempo = (__HAL_TIM_GET_COUNTER(&htim2)/2);


	  calibracion();
	  calculo_adc();
	  calculo_mm();

	  datos_enviar(rpm_f, contador, tiempo, velocidad_mm_s, velocidad_rad_s,
	                   adcValue[0], adcValue[1], adcValue[2], adcValue[3],
	                   adcVoltaje[0], adcVoltaje[1], adcVoltaje[2], adcVoltaje[3],
	                   adcMM[0], adcMM[1], adcMM[2], adcMM[3]);




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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 48000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 28-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 100-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Start_DMA(&hadc1, adcValue, 5);
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
