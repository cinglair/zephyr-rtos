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

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(LED1_NODE, okay)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

struct led {
	struct gpio_dt_spec spec;
	uint8_t num;
};

static const struct led led0 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}),
	.num = 0,
};

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(LED_NODE, gpios, {0});
static struct gpio_callback buttonCbData;

static int buttonPressCount = 0;

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

void blink0(void)
{
	blink(&led0, 200, 0);
}

void button0(void)
{
	button_task();
}

K_THREAD_DEFINE(blink0_id, STACKSIZE, blink0, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(button0_thread, STACKSIZE, button0, NULL, NULL, NULL, PRIORITY, 0, 0);

int main(void)
{
	return 0;
}
