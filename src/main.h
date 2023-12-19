
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/shell/shell.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>
#include <inttypes.h>
#include <string.h>
#include <zephyr/init.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <stm32f3xx_hal_dma.h>
#include <stm32f3xx_hal_dac.h>
#include <stm32f3xx_hal_uart.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
LOG_MODULE_DECLARE(zbus, CONFIG_ZBUS_LOG_LEVEL);

/* Tamanho da pilha usada por cada thread */
#define STACKSIZE 1024

/* Prioridade de cada thread no escalonador */
#define PRIORITY 7

#define SLEEP_TIME_MS 500

#define ARCH_STACK_PTR_ALIGN           8
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 1000
#define CONFIG_NUM_PREEMPT_PRIORITIES  15

extern K_THREAD_STACK_DEFINE(thread_stack, STACKSIZE);
extern struct k_thread thread_data;

#define DRDY_Pin              GPIO_PIN_2
#define DRDY_GPIO_Port        GPIOE
#define CS_I2C_SPI_Pin        GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port  GPIOE
#define MEMS_INT3_Pin         GPIO_PIN_4
#define MEMS_INT3_GPIO_Port   GPIOE
#define MEMS_INT4_Pin         GPIO_PIN_5
#define MEMS_INT4_GPIO_Port   GPIOE
#define OSC32_IN_Pin          GPIO_PIN_14
#define OSC32_IN_GPIO_Port    GPIOC
#define OSC32_OUT_Pin         GPIO_PIN_15
#define OSC32_OUT_GPIO_Port   GPIOC
#define OSC_IN_Pin            GPIO_PIN_0
#define OSC_IN_GPIO_Port      GPIOF
#define OSC_OUT_Pin           GPIO_PIN_1
#define OSC_OUT_GPIO_Port     GPIOF
#define B1_Pin                GPIO_PIN_0
#define B1_GPIO_Port          GPIOA
#define SPI1_SCK_Pin          GPIO_PIN_5
#define SPI1_SCK_GPIO_Port    GPIOA
#define SPI1_MISO_Pin         GPIO_PIN_6
#define SPI1_MISO_GPIO_Port   GPIOA
#define SPI1_MISOA7_Pin       GPIO_PIN_7
#define SPI1_MISOA7_GPIO_Port GPIOA
#define LD4_Pin               GPIO_PIN_8
#define LD4_GPIO_Port         GPIOE
#define LD3_Pin               GPIO_PIN_9
#define LD3_GPIO_Port         GPIOE
#define LD5_Pin               GPIO_PIN_10
#define LD5_GPIO_Port         GPIOE
#define LD7_Pin               GPIO_PIN_11
#define LD7_GPIO_Port         GPIOE
#define LD9_Pin               GPIO_PIN_12
#define LD9_GPIO_Port         GPIOE
#define LD10_Pin              GPIO_PIN_13
#define LD10_GPIO_Port        GPIOE
#define LD8_Pin               GPIO_PIN_14
#define LD8_GPIO_Port         GPIOE
#define LD6_Pin               GPIO_PIN_15
#define LD6_GPIO_Port         GPIOE
#define DM_Pin                GPIO_PIN_11
#define DM_GPIO_Port          GPIOA
#define DP_Pin                GPIO_PIN_12
#define DP_GPIO_Port          GPIOA
#define SWDIO_Pin             GPIO_PIN_13
#define SWDIO_GPIO_Port       GPIOA
#define SWCLK_Pin             GPIO_PIN_14
#define SWCLK_GPIO_Port       GPIOA
#define SWO_Pin               GPIO_PIN_3
#define SWO_GPIO_Port         GPIOB
#define I2C1_SCL_Pin          GPIO_PIN_6
#define I2C1_SCL_GPIO_Port    GPIOB
#define I2C1_SDA_Pin          GPIO_PIN_7
#define I2C1_SDA_GPIO_Port    GPIOB
#define MEMS_INT1_Pin         GPIO_PIN_0
#define MEMS_INT1_GPIO_Port   GPIOE
#define MEMS_INT2_Pin         GPIO_PIN_1
#define MEMS_INT2_GPIO_Port   GPIOE