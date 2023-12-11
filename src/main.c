/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/shell/shell.h>
#include <zephyr/zbus/zbus.h>
#include <inttypes.h>

#include <string.h>

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define SW0_NODE  DT_ALIAS(sw0)

#define SLEEP_TIME_MS 500

#define ARCH_STACK_PTR_ALIGN           8
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 1000
#define CONFIG_NUM_PREEMPT_PRIORITIES  15
#define CONFIG_MAIN_STACK_SIZE         256

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(LED1_NODE, okay)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

ZBUS_CHAN_DEFINE(led_chan, /* Name */
		 char,     /* Message type */

		 NULL,                                 /* Validator */
		 NULL,                                 /* User data */
		 ZBUS_OBSERVERS(listener, subscriber), /* observers */
		 ZBUS_MSG_INIT(0)                      /* Initial value */
);

static void listener_callback(const struct zbus_channel *chan)
{
	const char *led_msg = zbus_chan_const_msg(chan);
	printk("Mensagem do LED: %c\n", *led_msg);
}

ZBUS_LISTENER_DEFINE(listener, listener_callback);

ZBUS_SUBSCRIBER_DEFINE(subscriber, 4);

static void subscriber_task(void)
{
	const struct zbus_channel *chan;

	while (!zbus_sub_wait(&subscriber, &chan, K_FOREVER)) {
		char led_msg;

		if (&led_chan == chan) {
			zbus_chan_read(&led_chan, &led_msg, K_MSEC(500));
		}
	}
}

struct led {
	struct gpio_dt_spec spec;
	uint8_t num;
};

static const struct led led0 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}),
	.num = 0,
};

static const struct led led1 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0}),
	.num = 0,
};

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback buttonCbData;

static int buttonPressCount = 0;
static int zbusBlinkLed = 0;

static gpio_callback_handler_t button_pressed()
{
	buttonPressCount++;
	printk("Botão apertado em %" PRIu32 "\n", k_cycle_get_32());
	printk("Quantidade de vezes que o botão foi apertado %d\n", buttonPressCount);
}

void button_task()
{
	int ret;

	// Verificar se o botão está pronto para uso
	if (!gpio_is_ready_dt(&button)) {
		printk("Erro: Botão %s não está pronto\n", button.port->name);
		return 0;
	}

	// Configuração do pino
	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Erro %d: Falha ao configurar o Botão %s no pino %d\n", ret,
		       button.port->name, button.pin);
		return 0;
	}

	// Configuração da interrupção do botão
	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);

	if (ret != 0) {
		printk("Erro %d: Falha ao configurar a interrupção do Botão %s no pino %d\n", ret,
		       button.port->name, button.pin);
		return 0;
	}

	gpio_init_callback(&buttonCbData, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &buttonCbData);
	printk("Botão configurado na porta %s no pino %d\n", button.port->name, button.pin);

	k_msleep(SLEEP_TIME_MS);
}

void blink(const struct led *led, uint32_t sleep_ms, uint32_t id)
{
	const struct gpio_dt_spec *spec = &led->spec;
	int ret;
	int cnt = 0;

	if (!device_is_ready(spec->port)) {
		printk("Error: %s device is not ready\n", spec->port->name);
		return;
	}

	ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure pin %d (LED '%d')\n", ret, spec->pin,
		       led->num);
		return;
	}

	while (1) {
		gpio_pin_set(spec->port, spec->pin, cnt % 2);
		cnt++;

		k_msleep(sleep_ms);
	}
}

static int cmd_led(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "Ligar LED\n");
	k_msleep(200);
	char led_msg = '1';
	zbus_chan_pub(&led_chan, &led_msg, K_SECONDS(1));
	return 0;
}

SHELL_CMD_ARG_REGISTER(led, NULL, "Description: Pisca um Led", cmd_led, 1, 0);

void blink0(void)
{

	blink(&led0, 200, 0);
}

void blink1(void)
{
	char msg = " ";
	zbus_chan_read(&led_chan, &msg, K_MSEC(500));
	blink(&led1, 200, 0);
}

void button0(void)
{
	button_task();
}

K_THREAD_DEFINE(blink0_id, STACKSIZE, blink0, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(blink1_id, STACKSIZE, blink1, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(button0_thread, STACKSIZE, button0, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(subscriber_task_id, CONFIG_MAIN_STACK_SIZE, subscriber_task, NULL, NULL, NULL, 3, 0,
		0);

K_THREAD_DEFINE(subscriber_task_id1, CONFIG_MAIN_STACK_SIZE, subscriber_task, NULL, NULL, NULL, 3,
		0, 0);
int main(void)
{
	return 0;
}
