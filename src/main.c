#include "main.h"
#include <math.h>
#include <arm_const_structs.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

DMA_HandleTypeDef hdma_dac1_ch1;

uint16_t adcBuffer[256];
float32_t ReIm[256 * 2];
float32_t mod[256];

#define M_PI 3.1415926;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_UART4_Init(void);

// ************* ADC ****************

uint16_t sin_wave[256] = {
	2048, 2098, 2148, 2199, 2249, 2299, 2349, 2399, 2448, 2498, 2547, 2596, 2644, 2692, 2740,
	2787, 2834, 2880, 2926, 2971, 3016, 3060, 3104, 3147, 3189, 3230, 3271, 3311, 3351, 3389,
	3427, 3464, 3500, 3535, 3569, 3602, 3635, 3666, 3697, 3726, 3754, 3782, 3808, 3833, 3857,
	3880, 3902, 3923, 3943, 3961, 3979, 3995, 4010, 4024, 4036, 4048, 4058, 4067, 4074, 4081,
	4086, 4090, 4093, 4095, 4095, 4094, 4092, 4088, 4084, 4078, 4071, 4062, 4053, 4042, 4030,
	4017, 4002, 3987, 3970, 3952, 3933, 3913, 3891, 3869, 3845, 3821, 3795, 3768, 3740, 3711,
	3681, 3651, 3619, 3586, 3552, 3517, 3482, 3445, 3408, 3370, 3331, 3291, 3251, 3210, 3168,
	3125, 3082, 3038, 2994, 2949, 2903, 2857, 2811, 2764, 2716, 2668, 2620, 2571, 2522, 2473,
	2424, 2374, 2324, 2274, 2224, 2174, 2123, 2073, 2022, 1972, 1921, 1871, 1821, 1771, 1721,
	1671, 1622, 1573, 1524, 1475, 1427, 1379, 1331, 1284, 1238, 1192, 1146, 1101, 1057, 1013,
	970,  927,  885,  844,  804,  764,  725,  687,  650,  613,  578,  543,  509,  476,  444,
	414,  384,  355,  327,  300,  274,  250,  226,  204,  182,  162,  143,  125,  108,  93,
	78,   65,   53,   42,   33,   24,   17,   11,   7,    3,    1,    0,    0,    2,    5,
	9,    14,   21,   28,   37,   47,   59,   71,   85,   100,  116,  134,  152,  172,  193,
	215,  238,  262,  287,  313,  341,  369,  398,  429,  460,  493,  526,  560,  595,  631,
	668,  706,  744,  784,  824,  865,  906,  948,  991,  1035, 1079, 1124, 1169, 1215, 1261,
	1308, 1355, 1403, 1451, 1499, 1548, 1597, 1647, 1696, 1746, 1796, 1846, 1896, 1947, 1997,
	2047};

uint16_t sin_wave_3rd_harmonic[256] = {
	2048, 2136, 2224, 2311, 2398, 2484, 2569, 2652, 2734, 2814, 2892, 2968, 3041, 3112, 3180,
	3245, 3308, 3367, 3423, 3476, 3526, 3572, 3615, 3654, 3690, 3723, 3752, 3778, 3800, 3819,
	3835, 3848, 3858, 3866, 3870, 3872, 3871, 3869, 3864, 3857, 3848, 3838, 3827, 3814, 3801,
	3786, 3771, 3756, 3740, 3725, 3709, 3694, 3679, 3665, 3652, 3639, 3628, 3617, 3608, 3600,
	3594, 3589, 3585, 3584, 3583, 3584, 3587, 3591, 3597, 3604, 3613, 3622, 3633, 3645, 3658,
	3672, 3686, 3701, 3717, 3732, 3748, 3764, 3779, 3794, 3808, 3821, 3833, 3844, 3853, 3860,
	3866, 3870, 3872, 3871, 3868, 3862, 3854, 3842, 3828, 3810, 3789, 3765, 3738, 3707, 3673,
	3635, 3594, 3549, 3501, 3450, 3396, 3338, 3277, 3213, 3146, 3077, 3005, 2930, 2853, 2774,
	2693, 2611, 2527, 2441, 2355, 2268, 2180, 2092, 2003, 1915, 1827, 1740, 1654, 1568, 1484,
	1402, 1321, 1242, 1165, 1090, 1018, 949,  882,  818,  757,  699,  645,  594,  546,  501,
	460,  422,  388,  357,  330,  306,  285,  267,  253,  241,  233,  227,  224,  223,  225,
	229,  235,  242,  251,  262,  274,  287,  301,  316,  331,  347,  363,  378,  394,  409,
	423,  437,  450,  462,  473,  482,  491,  498,  504,  508,  511,  512,  511,  510,  506,
	501,  495,  487,  478,  467,  456,  443,  430,  416,  401,  386,  370,  355,  339,  324,
	309,  294,  281,  268,  257,  247,  238,  231,  226,  224,  223,  225,  229,  237,  247,
	260,  276,  295,  317,  343,  372,  405,  441,  480,  523,  569,  619,  672,  728,  787,
	850,  915,  983,  1054, 1127, 1203, 1281, 1361, 1443, 1526, 1611, 1697, 1784, 1871, 1959,
	2047};

// static void prvTaskDACCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
// 			      const char *pcCommandString)
// {
// 	// pegar os parametros do shell
// 	char parameter[] = "sine";

// 	if (!strcmp(parameter, "sine")) {
// 		HAL_TIM_Base_Stop(&htim2);
// 		HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
// 		HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)sin_wave, 256, DAC_ALIGN_12B_R);
// 		HAL_TIM_Base_Start(&htim2);
// 		printk("Sine set for the DAC signal\n\r");
// 	} else if (!strcmp(parameter, "sine3rd")) {
// 		HAL_TIM_Base_Stop(&htim2);
// 		HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
// 		HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)sin_wave_3rd_harmonic, 256,
// 				  DAC_ALIGN_12B_R);
// 		HAL_TIM_Base_Start(&htim2);
// 		printk("Sine 3rd harmonic set for the DAC signal\n\r");
// 	} else {
// 		printk("Not a valid DAC signal!\n\r");
// 	}
// }

// static void prvTaskFFTCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
// 			      const char *pcCommandString)
// {
// 	const char *parameter1 = "10";
// 	const char *parameter2 = "10";

// 	// pegar os argumentos do shell

// 	int first_harm = atoi(parameter1);
// 	int number_harm = atoi(parameter2);

// 	if ((first_harm >= 0) && (number_harm > 0)) {
// 		int len = sprintf(pcWriteBuffer,
// 				  "FFT result for the current DAC signal (%d, %d): ", first_harm,
// 				  number_harm);
// 		for (int i = first_harm; i < (number_harm + first_harm); i++) {
// 			if (i == 0) {
// 				len += sprintf(&pcWriteBuffer[len], "%f ", mod[i] / 2.0);
// 			} else {
// 				len += sprintf(&pcWriteBuffer[len], "%f ", mod[i]);
// 			}
// 		}
// 		sprintf(&pcWriteBuffer[len], "\n\r");
// 	} else {
// 		printk("Invalid parameters!\n\r");
// 	}
// }

void adc_task(void *param)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 256);
	// TODO: Entender por que essa bosta nao funcionando
	HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL_1, (uint32_t *)sin_wave_3rd_harmonic, 256,
			  DAC_ALIGN_12B_R);
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_Base_Start(&htim3);

	while (1) {
		int k = 0;
		for (int i = 0; i < 256; i++) {
			ReIm[k] = (float)adcBuffer[i] * 0.0008056640625;
			ReIm[k + 1] = 0.0;
			k += 2;
		}

		arm_cfft_f32(&arm_cfft_sR_f32_len256, ReIm, 0, 1);
		arm_cmplx_mag_f32(ReIm, mod, 256);
		arm_scale_f32(mod, 0.0078125, mod, 128);

		volatile float fund_phase =
			atan2f(ReIm[3], ReIm[2]) * 180 / M_PI; // Fase R da harmonica fundamental
		(void)fund_phase;
		k_msleep(1000);
	}
}

// ************* ADC ****************

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_DAC_Init();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM8_Init();
	MX_UART4_Init();

	return 0;
}

K_THREAD_DEFINE(adc, STACKSIZE, adc_task, NULL, NULL, NULL, 7, 0, 0);

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
				      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	PeriphClkInit.PeriphClockSelection =
		RCC_PERIPHCLK_UART4 | RCC_PERIPHCLK_TIM8 | RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
	PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
}
static void MX_ADC1_Init(void)
{
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;

	multimode.Mode = ADC_MODE_INDEPENDENT;

	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
}

static void MX_DAC_Init(void)
{
	DAC_ChannelConfTypeDef sConfig = {0};
	hdac.Instance = DAC;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
}

static void MX_TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1699;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
}

static void MX_TIM3_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 11067;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
}

static void MX_TIM8_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 11067;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
}

static void MX_UART4_Init(void)
{
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
}

void DMA1_Channel1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_dac1_ch1);
}

void DMA1_Channel2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_adc1);
}

static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	IRQ_CONNECT(DMA2_Channel3_IRQn, 5, DMA1_Channel1_IRQHandler, 0, 0);
	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA2_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOE,
			  CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD7_Pin | LD9_Pin |
				  LD10_Pin | LD8_Pin | LD6_Pin,
			  GPIO_PIN_RESET);

	GPIO_InitStruct.Pin =
		DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin | MEMS_INT1_Pin | MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD7_Pin | LD9_Pin |
			      LD10_Pin | LD8_Pin | LD6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MISO_Pin | SPI1_MISOA7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = DM_Pin | DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF14_USB;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = I2C1_SCL_Pin | I2C1_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
