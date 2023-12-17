#include "main.h"

// ADC_HandleTypeDef hadc1;
// DMA_HandleTypeDef hdma_adc1;

// DAC_HandleTypeDef hdac1;
// DMA_HandleTypeDef hdma_dac1_ch1;

// UART_HandleTypeDef hlpuart1;

// TIM_HandleTypeDef htim2;
// TIM_HandleTypeDef htim3;
// TIM_HandleTypeDef htim8;

// /* USER CODE BEGIN PV */

// /* USER CODE END PV */

// /* Private function prototypes -----------------------------------------------*/
// void SystemClock_Config(void);
// static void MX_GPIO_Init(void);
// static void MX_DMA_Init(void);
// static void MX_LPUART1_UART_Init(void);
// static void MX_TIM2_Init(void);
// static void MX_DAC1_Init(void);
// static void MX_ADC1_Init(void);
// static void MX_TIM3_Init(void);
// static void MX_TIM8_Init(void);
// void StartDefaultTask(void const *argument);

// /* USER CODE BEGIN PFP */

// uint16_t adcBuffer[256];
// float ReIm[256 * 2];
// float mod[256];

// uint16_t sin_wave[256];
// uint16_t sin_wave_3rd_harmonic[256];
// static void prvTaskDACCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
// 				    const char *pcCommandString)
// {
// 	// BaseType_t parameter_lenght;
// 	// const char *parameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &parameter_lenght);

// 	// pegar os parametros do shell
// 	char parameter[] = {"s,i,n,e"};

// 	if (!strcmp(parameter, "sine")) {
// 		HAL_TIM_Base_Stop(&htim2);
// 		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
// 		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)sin_wave, 256,
// 				  DAC_ALIGN_12B_R);
// 		HAL_TIM_Base_Start(&htim2);
// 		strcpy(pcWriteBuffer, "Sine set for the DAC signal\n\r");
// 	} else if (!strcmp(parameter, "sine3rd")) {
// 		HAL_TIM_Base_Stop(&htim2);
// 		HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
// 		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)sin_wave_3rd_harmonic, 256,
// 				  DAC_ALIGN_12B_R);
// 		HAL_TIM_Base_Start(&htim2);
// 		strcpy(pcWriteBuffer, "Sine 3rd harmonic set for the DAC signal\n\r");
// 	} else {
// 		strcpy(pcWriteBuffer, "Not a valid DAC signal!\n\r");
// 	}
// 	return false;
// }

// static void prvTaskFFTCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
// 				    const char *pcCommandString)
// {
// 	// BaseType_t parameter_lenght;
// 	// const char *parameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &parameter_lenght);
// 	// const char *parameter2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &parameter_lenght);

// 	// pegar os argumentos do shell

// 	// int first_harm = atoi(parameter1);
// 	// int number_harm = atoi(parameter2);

// 	// if ((first_harm >= 0) && (number_harm > 0)) {
// 	// 	int len = sprintf(pcWriteBuffer,
// 	// 			  "FFT result for the current DAC signal (%d, %d): ", first_harm,
// 	// 			  number_harm);
// 	// 	for (int i = first_harm; i < (number_harm + first_harm); i++) {
// 	// 		if (i == 0) {
// 	// 			len += sprintf(&pcWriteBuffer[len], "%f ", mod[i] / 2.0);
// 	// 		} else {
// 	// 			len += sprintf(&pcWriteBuffer[len], "%f ", mod[i]);
// 	// 		}
// 	// 	}
// 	// 	sprintf(&pcWriteBuffer[len], "\n\r");
// 	// } else {
// 	// 	strcpy(pcWriteBuffer, "Invalid parameters!\n\r");
// 	// }

// 	return false;
// }

// #define cmdPARAMTER_NOT_USED ((void *)0)
// #define MAX_INPUT_LENGTH     50
// #define MAX_OUTPUT_LENGTH    512

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
// {
// 	// portBASE_TYPE pxHigherPriorityTaskWoken = false;
// 	// xSemaphoreGiveFromISR(sem_adc, &pxHigherPriorityTaskWoken);
// 	// portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
// }

// void adc_task(void *param)
// {
// 	// HAL_ADC_Start_DMA(&h1, (uint32_t *)adcBuffer, 256);
// 	// HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)sin_wave_3rd_harmonic, 256,
// 	// 		  DAC_ALIGN_12B_R);

// 	// HAL_TIM_Base_Start(&htim8);
// 	// HAL_TIM_Base_Start(&htim3);

// 	// while (1) {
// 	// 	// xSemaphoreTake(sem_adc, portMAX_DELAY);

// 	// 	int k = 0;
// 	// 	for (int i = 0; i < 256; i++) {
// 	// 		ReIm[k] = (float)adcBuffer[i] * 0.0008056640625;
// 	// 		ReIm[k + 1] = 0.0;
// 	// 		k += 2;
// 	// 	}

// 	// 	arm_cfft_f32(&arm_cfft_sR_f32_len256, ReIm, 0, 1);
// 	// 	arm_cmplx_mag_f32(ReIm, mod, 256);
// 	// 	arm_scale_f32(mod, 0.0078125, mod, 128);

// 	// 	volatile float fund_phase =
// 	// 		atan2f(ReIm[3], ReIm[2]) * 180 / M_PI; // Fase R da harmonica fundamental
// 	// 	(void)fund_phase;
// 	// }
// }

// void SystemClock_Config(void)
// {
// 	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
// 	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
// 	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
// 	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
// 	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
// 	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
// 	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
// 	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
// 	RCC_OscInitStruct.PLL.PLLN = 85;
// 	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
// 	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
// 	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
// 	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
// 		Error_Handler();
// 	}

// 	/** Initializes the CPU, AHB and APB buses clocks
// 	 */
// 	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
// 				      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
// 	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
// 	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
// 	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
// 	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

// 	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
// 		Error_Handler();
// 	}
// }

// static void MX_ADC1_Init(void)
// {
// 	ADC_MultiModeTypeDef multimode = {0};
// 	ADC_ChannelConfTypeDef sConfig = {0};
// 	hadc1.Instance = ADC1;
// 	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
// 	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
// 	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
// 	hadc1.Init.GainCompensation = 0;
// 	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
// 	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
// 	hadc1.Init.LowPowerAutoWait = DISABLE;
// 	hadc1.Init.ContinuousConvMode = DISABLE;
// 	hadc1.Init.NbrOfConversion = 1;
// 	hadc1.Init.DiscontinuousConvMode = DISABLE;
// 	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
// 	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
// 	hadc1.Init.DMAContinuousRequests = ENABLE;
// 	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
// 	hadc1.Init.OversamplingMode = DISABLE;
// 	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
// 		Error_Handler();
// 	}

// 	multimode.Mode = ADC_MODE_INDEPENDENT;
// 	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
// 		Error_Handler();
// 	}
// 	sConfig.Channel = ADC_CHANNEL_1;
// 	sConfig.Rank = ADC_REGULAR_RANK_1;
// 	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
// 	sConfig.SingleDiff = ADC_SINGLE_ENDED;
// 	sConfig.OffsetNumber = ADC_OFFSET_NONE;
// 	sConfig.Offset = 0;
// 	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
// 		Error_Handler();
// 	}
// }

// static void MX_TIM2_Init(void)
// {
// 	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
// 	TIM_MasterConfigTypeDef sMasterConfig = {0};
// 	htim2.Instance = TIM2;
// 	htim2.Init.Prescaler = 0;
// 	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
// 	htim2.Init.Period = 1699;
// 	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
// 	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
// 	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
// 		Error_Handler();
// 	}
// 	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
// 	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
// 		Error_Handler();
// 	}
// 	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
// 	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
// 	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
// 		Error_Handler();
// 	}
// }

// static void MX_TIM3_Init(void)
// {
// 	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
// 	TIM_MasterConfigTypeDef sMasterConfig = {0};
// 	htim3.Instance = TIM3;
// 	htim3.Init.Prescaler = 0;
// 	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
// 	htim3.Init.Period = 11067;
// 	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
// 	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
// 	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
// 		Error_Handler();
// 	}
// 	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
// 	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
// 		Error_Handler();
// 	}
// 	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
// 	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
// 	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
// 		Error_Handler();
// 	}
// }

// static void MX_TIM8_Init(void)
// {

// 	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
// 	TIM_MasterConfigTypeDef sMasterConfig = {0};

// 	htim8.Instance = TIM8;
// 	htim8.Init.Prescaler = 0;
// 	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
// 	htim8.Init.Period = 11067;
// 	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
// 	htim8.Init.RepetitionCounter = 0;
// 	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
// 	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
// 		Error_Handler();
// 	}
// 	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
// 	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
// 		Error_Handler();
// 	}
// 	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
// 	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
// 	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
// 	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
// 		Error_Handler();
// 	}
// }

// static void MX_DMA_Init(void)
// {

// 	/* DMA controller clock enable */
// 	__HAL_RCC_DMAMUX1_CLK_ENABLE();
// 	__HAL_RCC_DMA1_CLK_ENABLE();

// 	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
// 	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
// 	/* DMA1_Channel2_IRQn interrupt configuration */
// 	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
// 	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
// }

// static void MX_GPIO_Init(void)
// {
// 	GPIO_InitTypeDef GPIO_InitStruct = {0};

// 	__HAL_RCC_GPIOC_CLK_ENABLE();
// 	__HAL_RCC_GPIOF_CLK_ENABLE();
// 	__HAL_RCC_GPIOA_CLK_ENABLE();

// 	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
// 	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
// }

// extern unsigned int ulHighFrequencyTimerTicks;

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
// 	/* USER CODE BEGIN Callback 0 */

// 	/* USER CODE END Callback 0 */
// 	if (htim->Instance == TIM1) {
// 		HAL_IncTick();
// 	}
// 	/* USER CODE BEGIN Callback 1 */
// 	if (htim->Instance == TIM2) {
// 		ulHighFrequencyTimerTicks++;
// 	}
// 	/* USER CODE END Callback 1 */
// }