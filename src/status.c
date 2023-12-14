#include "led.h"

static const struct led led0 = {
	.spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}),
	.num = 0,
};

void blink(const struct led *led)
{
	const struct gpio_dt_spec *spec = &led->spec;
	int ret;
	int cnt = 0;

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

	while (1) {
		gpio_pin_set(spec->port, spec->pin, cnt % 2);
		cnt++;

		k_msleep(SLEEP_TIME_MS);
	}
}

void blink0(void)
{
	blink(&led0);
}

K_THREAD_DEFINE(status_led, STACKSIZE, blink0, NULL, NULL, NULL, PRIORITY, 0, 0);