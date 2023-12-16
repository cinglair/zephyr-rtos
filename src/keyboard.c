#include "keyboard.h"

static const struct button button0 = {
	.spec = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0}),
	.num = 0,
};
static struct gpio_callback buttonCbData;
static struct k_work_delayable cooldown_work;

static void button_event_handler()
{
	printk("Botão Pressionado\n");
}

static void cooldown_expired(struct k_work *work)
{
    ARG_UNUSED(work);
    gpio_pin_get_dt(&button0.spec);
    button_event_handler();
}

static void button_press_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_work_reschedule(&cooldown_work, K_MSEC(100));
}

int keyboard(const struct button *button)
{
	const struct gpio_dt_spec *spec = &button->spec;
	int ret;

	// Verificar se o botão está pronto para uso
	if (!device_is_ready(spec->port)) {
		LOG_INF("Erro: Botão %s não está pronto\n", spec->port->name);
		return 0;
	}

	// Configuração do pino
	ret = gpio_pin_configure_dt(spec, GPIO_INPUT);
	if (ret != 0) {
		LOG_INF("Erro %d: Falha ao configurar o Botão %s no pino %d\n", ret,
			spec->port->name, spec->pin);
		return 0;
	}

	// Configuração da interrupção do botão
	ret = gpio_pin_interrupt_configure_dt(spec, GPIO_INT_EDGE_TO_ACTIVE);

	if (ret != 0) {
		LOG_INF("Erro %d: Falha ao configurar a interrupção do Botão %s no pino %d\n", ret,
			spec->port->name, spec->pin);
		return 0;
	}

	gpio_init_callback(&buttonCbData, button_press_callback, BIT(spec->pin));
	gpio_add_callback(spec->port, &buttonCbData);
	k_work_init_delayable(&cooldown_work, cooldown_expired);
	LOG_INF("Botão configurado na porta %s no pino %d\n", spec->port->name, spec->pin);

    while(1) {
		k_sleep(K_MSEC(SLEEP_TIME_MS));
	}
}

void keyboard0(void)
{
	keyboard(&button0);
}

K_THREAD_DEFINE(keyboard_task, STACKSIZE, keyboard0, NULL, NULL, NULL, PRIORITY, 0, 0);
