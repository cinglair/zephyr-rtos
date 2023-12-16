#include "led.h"

static const struct led led0 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}),
	.num = 0,
};

struct k_timer status_led_timer;

static void blink_callback(struct k_timer *timer)
{
	gpio_pin_toggle(led0.spec.port, led0.spec.pin);
}

void blink(const struct led *led)
{
	const struct gpio_dt_spec *spec = &led->spec;
	int ret;

	if (!device_is_ready(spec->port)) {
		LOG_INF("Erro: LED %s não está pronto\n", spec->port->name);
		return;
	}

	ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT);
	if (ret != 0) {
		LOG_INF("Erro %d: falha ao configurar o pino %d para o LED '%d'\n", ret, spec->pin,
			led->num);
		return;
	}

	k_timer_init(&status_led_timer, blink_callback, NULL);
	k_timer_start(&status_led_timer, K_MSEC(SLEEP_TIME_MS), K_MSEC(SLEEP_TIME_MS));

	k_sleep(K_FOREVER);
}

void blink0(void)
{
	blink(&led0);
}

K_THREAD_DEFINE(status_led, STACKSIZE, blink0, NULL, NULL, NULL, PRIORITY, 0, 0);