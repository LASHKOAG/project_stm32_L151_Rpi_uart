/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ina260.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_PORT 				   GPIOC
#define LED_PIN 				   GPIO_PIN_1

#define BTN_IRQ_PORT    		   GPIOC
#define BTN_IRQ_PIN     		   GPIO_PIN_13

#define BTN_USER_PORT	 		   GPIOD
#define BTN_USER_PIN 	 		   GPIO_PIN_2

#define PIN_GLOBAL_POWER_PORT  	   GPIOC
#define PIN_GLOBAL_POWER_PIN	   GPIO_PIN_0

#define PIN_STATUS_CHARGER_PORT	   GPIOB
#define PIN_STATUS_CHARGER_PIN	   GPIO_PIN_0

//size_rx_uart  12 if Windows x64
//size_rx_uart  10 if Ubuntu x64
//size_rx_uart  6 if Raspberry Pi
#define size_rx_uart  6

#pragma pack(push, 1)
typedef struct
{
	uint8_t command;
	uint8_t length;
	int8_t* buff;
}tcp_packet_t;
#pragma pack(pop)

#define SLAVE_SUCCESS_ANSWER        0
#define SLAVE_BAD_ANSWER           -1

#define CMD_ANSWER  		        1
#define CMD_GET_INAINFO			    2
#define CMD_SHUTDOWN     		    3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void get_sleep();
void blink(uint8_t, uint32_t);//для отладки
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
//void send_message (tcp_packet_t*);
void get_volt_and_chargerStatus(ina260_t *);


int8_t flag_btn_irq = 0;
int8_t flag_btn_user_pressed = 0;
int8_t flag_get_UART=0;
int8_t flag_need_sleep = 1;

uint8_t rx_uart[size_rx_uart]={0, };
uint8_t tx_uart[10]={10, 11, 12, 13, 14, 15, 16, 17, 18, 19}; //для отладки

uint8_t tx_uart_2[1]={11}; //для отладки
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
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	//для отладки
	blink(6, 40);


	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	//	HAL_UART_Transmit_IT (&huart1, (uint8_t*)tx_uart, 10);//для отладки
	HAL_UART_Transmit(&huart1, tx_uart_2, 1, 100);//для отладки
	uint8_t addr_ina = 0x45;
	ina260_t *m_pow = NULL;
	m_pow = ina260_new (&hi2c2, (uint8_t*)addr_ina); //

	ina260_set_config(m_pow,
			iotPower,
			iomContinuous,
			ictConvert1p1ms,
			ictConvert1p1ms,
			issSample16);

//	uint8_t readyStatus = ina260_ready(m_pow);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		//отправляем в сон
		if(flag_need_sleep){
			get_sleep();
			//выход после режима сна
			flag_need_sleep=0;
		}
		//произошло короткое нажатие кнопки - вызывает прерывание
		//если произошло прерывание по кнопке BTN_IRQ
		if(flag_btn_irq){
			blink(6, 50);
			//необходимо проверить состояние кнопки на удержание BTN_USER
			//(BTN_USER и BTN_IRQ - физически это одна и та же кнопка, выводы дублируются на ещё один пин, так как на ней и режим прерывания и обычный )
			HAL_Delay(1900);
			if(HAL_GPIO_ReadPin(BTN_USER_PORT, BTN_USER_PIN) == GPIO_PIN_RESET){
				//кнопка удерживается какой то период времени
				flag_btn_user_pressed=1;
			}

			//кнопку удерживали, необходимо вкл прибор
			if(flag_btn_user_pressed){
				//включаем пин, отвечающий за глобальное управление питанием
				HAL_GPIO_WritePin(PIN_GLOBAL_POWER_PORT, PIN_GLOBAL_POWER_PIN, GPIO_PIN_SET);
				blink(6,50);  //для отладки
				//LED сигнализирующий, что питание подано
				HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
				flag_btn_user_pressed = 0;

				//разрешаем работу по UART`у
				flag_get_UART=1;

				while(flag_get_UART){
					HAL_UART_Receive_IT (&huart1, (uint8_t*)rx_uart, size_rx_uart);
					//данный буффер собирается в обработчике прерываний,
					//т.е когда буффер rx_uart размером (size_rx_uart) получает все байты,
					//тогда происходит прерывание и срабатывает обработчик прерываний HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
					int8_t action=rx_uart[0];

					switch (action)
					{
					//команда получения данных с INA и пина, который показывает источник зарядки
					case 0x02:
						blink(6,50);
						get_volt_and_chargerStatus(m_pow);
						break;
					//команда выключения
					case 0x03:
						blink(6, 1000);

						flag_get_UART = 0;
						flag_btn_user_pressed = 0;
						flag_btn_irq=0;
						flag_need_sleep=1;

						HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); // для отладки - удалить

						//перед выключением питания необходим таймаут
						//для корректного выключения master`a
						// HAL_Delay(60000);
						HAL_Delay(6000);

						//пин управления питанием в LOW
						HAL_GPIO_WritePin(PIN_GLOBAL_POWER_PORT, PIN_GLOBAL_POWER_PIN, GPIO_PIN_RESET);
						//светодиод индикации питания выкл
						HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
						break;

					default:
						//другие команды игнорируются
						break;
					}

					//обнуление массива rx_uart
					memset(&rx_uart[0], 0x00, size_rx_uart);
				}

			}else{
				flag_need_sleep=1;
				flag_btn_irq=0;
			}

		}

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC0 PC1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin== BTN_IRQ_PIN) {
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		flag_btn_irq=1;
		flag_need_sleep = 0;
	}else{
		__NOP();
	}
}

void get_sleep()
{
	HAL_SuspendTick();
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	HAL_ResumeTick();
}

void blink(uint8_t counter_blink, uint32_t t_ms)
{
	for(int8_t i=0; i<counter_blink; ++i){
		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); //Toggle LED
		HAL_Delay (t_ms);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1){

	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	__NOP();
}
//
//void send_message (tcp_packet_t* ans_struct){
//	uint16_t len_send=sizeof(ans_struct->command)+sizeof(ans_struct->length) + ans_struct->length;
//
//	const uint16_t sizeTempMSV = len_send;
//	uint8_t TempMSV[sizeTempMSV];
//	memset(&TempMSV[0], 0x00, len_send);
//	//собираем сообщение
//	uint8_t pos=0;
//	memcpy(&TempMSV[pos], &ans_struct->command, sizeof(ans_struct->command));
//	pos+=sizeof(ans_struct->command);
//	memcpy(&TempMSV[pos], &ans_struct->length, sizeof(ans_struct->length));
//	pos+=sizeof(ans_struct->length);
//	memcpy(&TempMSV[pos], &ans_struct->buff[0], ans_struct->length);
//
//	//send message
//	//	HAL_UART_Transmit_IT (&huart1, (uint8_t*)TempMSV, len_send-13);
//	HAL_UART_Transmit_IT (&huart1, (uint8_t*)&TempMSV[0], len_send);
//	//	HAL_UART_Transmit(&huart1, (uint8_t*)&TempMSV[0], len_send-13, 100);
//}

void get_volt_and_chargerStatus(ina260_t *_pow)
{
	float voltage = 0.0f;
	int8_t flag_status_charger = 0;
	int8_t res = SLAVE_SUCCESS_ANSWER;

	//get measure INA260
	if(ina260_get_voltage(_pow, &voltage) !=HAL_OK){
		res = SLAVE_BAD_ANSWER;
	}

	//get flag - charging status pin from charger or battery
	if(HAL_GPIO_ReadPin(PIN_STATUS_CHARGER_PORT, PIN_STATUS_CHARGER_PIN)==0){
		//charging from battery
		flag_status_charger = 0;
	}else{
		//charging from power supply
		flag_status_charger = 1;
	}

	tcp_packet_t ans;
	memset(&ans, 0x00, sizeof(tcp_packet_t));

	//собираем сообщение------------------------------------------
	ans.command = CMD_ANSWER;
	uint8_t length_data = sizeof(voltage) + sizeof(flag_status_charger);
	ans.length = sizeof(ans.command) + sizeof(res) + sizeof(length_data) + sizeof(voltage) + sizeof(flag_status_charger);
	uint8_t pos = 0;
	int8_t Buffer[ans.length];
	memset(&Buffer[0], 0x00, ans.length);

	memcpy(&Buffer[pos], (int8_t*)&ans.command, sizeof(ans.command));
	pos+= sizeof(ans.command);
	memcpy(&Buffer[pos], (int8_t*)&res, sizeof(res));
	pos+=sizeof(res);
	memcpy(&Buffer[pos], (int8_t*)&length_data, sizeof(length_data));
	pos+=sizeof(length_data);
	memcpy(&Buffer[pos], (int8_t*)&voltage, sizeof(voltage));
	pos+=sizeof(voltage);
	memcpy(&Buffer[pos], (int8_t*)&flag_status_charger, sizeof(flag_status_charger));
	pos+=sizeof(flag_status_charger);
	//------------------------------------------------------------

	//send message in COM-PORT (UART to master)
	HAL_UART_Transmit(&huart1, (uint8_t*)&Buffer[0], ans.length, 100);
//	HAL_UART_Transmit_IT (&huart1, (uint8_t*)&Buffer[0], ans.length);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
