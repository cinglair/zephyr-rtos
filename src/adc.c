#include "main.h"
#define M_PI 3.1415926;

uint16_t adcBuffer[256];
float32_t ReIm[256 * 2];
float32_t mod[256];

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_dac_ch1;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart4;

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

void adc_task(void *param)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 256);
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
			atan2f(ReIm[3], ReIm[2]) * 180 / M_PI;
		(void)fund_phase;

		k_msleep(SLEEP_TIME_MS);
	}
}

K_THREAD_DEFINE(adc, STACKSIZE, adc_task, NULL, NULL, NULL, 7, 0, 0);

static int cmd_signal_generate(const struct shell *sh, size_t argc, char **argv, void *data)
{
	int signal;
    signal = (int)data;

	if (signal == 1) {
		HAL_TIM_Base_Stop(&htim2);
		HAL_DAC_Stop_DMA(&hdac, DAC1_CHANNEL_1);
		HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL_1, (uint32_t *)sin_wave, 256, DAC_ALIGN_12B_R);
		HAL_TIM_Base_Start(&htim2);
		printk("Sinal do DAC configurado para: SENO\n\r");
	} else if (signal == 2) {
		HAL_TIM_Base_Stop(&htim2);
		HAL_DAC_Stop_DMA(&hdac, DAC1_CHANNEL_1);
		HAL_DAC_Start_DMA(&hdac, DAC1_CHANNEL_1, (uint32_t *)sin_wave_3rd_harmonic, 256,
				  DAC_ALIGN_12B_R);
		HAL_TIM_Base_Start(&htim2);
		printk("Sinal do DAC configurado para: 3º Harmônico do Seno\n\r");
	} else {
		printk("Não é um sinal válido para o DAC!\n\r");
	}

    return 0;
}

SHELL_SUBCMD_DICT_SET_CREATE(
    sub_signal_generate, 
    cmd_signal_generate, 
    (sine, 1, "SENO"), 
    (sine_3rd_harmonic, 2, "3º Harmônico do Seno")
);
SHELL_CMD_REGISTER(signal_generate, &sub_signal_generate, "Gera um sinal pelo DAC", NULL);