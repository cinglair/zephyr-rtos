#include "main.h"

static int cmd_led(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "Ligar LED\n");

	k_msleep(200);
	return 0;
}

SHELL_CMD_ARG_REGISTER(led, NULL, "Description: Pisca um Led", cmd_led, 1, 0);

// ************* COMANDOS DO ADC LIGA E DESLIGA *************

static void cmd_demo_on(const struct shell *shell, size_t argc, char **argv)
{
	printk("ADC ligado");
}
static void cmd_demo_off(const struct shell *shell, size_t argc, char **argv)
{
	printk("ADC desligado");
}
SHELL_STATIC_SUBCMD_SET_CREATE(sub_demo, SHELL_CMD(on, NULL, "Print params command.", cmd_demo_on),
			       SHELL_CMD(off, NULL, "Ping command.", cmd_demo_off),
			       SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(demo, &sub_demo, "Demo commands", NULL);

// ************* ------------------------------- *************
/// TESTE DO EXEMPLO DO ZBUS

ZBUS_CHAN_DEFINE(sensor_data_chan, char, NULL, NULL,
		 ZBUS_OBSERVERS(thread_handler1_sub), /* observers */
		 ZBUS_MSG_INIT(0)                     /* Initial value {0} */
);

void peripheral_thread(void)
{
	char sm = 't';

	while (1) {
		zbus_chan_pub(&sensor_data_chan, &sm, K_MSEC(200));
		k_msleep(100);
	}
}

K_THREAD_DEFINE(peripheral_thread_id, 1024, peripheral_thread, NULL, NULL, NULL, 5, 0, 0);
ZBUS_SUBSCRIBER_DEFINE(thread_handler1_sub, 4);

static void cmd_adc(const struct shell *shell)
{
	const struct zbus_channel *chan;
	if (!zbus_sub_wait(&thread_handler1_sub, &chan, K_MSEC(200))) {
		char msg;
		zbus_chan_read(chan, &msg, K_MSEC(200));
		shell_print(shell, "Valor: %c\n", msg);
	}
}

SHELL_CMD_ARG_REGISTER(adc, NULL, "Description: Gera sinal ADC", cmd_adc, 1, 0);