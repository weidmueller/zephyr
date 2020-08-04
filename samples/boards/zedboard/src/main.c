#include <zephyr.h>

#if defined(CONFIG_STDOUT_CONSOLE)
#include <stdio.h>
#else
#include <sys/printk.h>
#endif

#include <drivers/gpio.h>
#include <net/net_core.h>
#include <net/net_if.h>

struct k_thread myThread1;
struct k_thread myThread2;

#define STACK_SIZE 1024
K_THREAD_STACK_DEFINE(myThread1Stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(myThread2Stack, STACK_SIZE);

static struct device        *ledgpio = NULL;
static struct device        *swsgpio = NULL;
static struct gpio_callback mySwitchCallback;

void GPIOCallbackHandler(
	struct device *port,
	struct gpio_callback *cb,
	uint32_t pins) {
	printk("GPIO callback! pins = 0x%08X\n", pins);
}

void myThreadsCode(void *id, void *unused1, void *unused2)
{   
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);

	uint32_t myId     = (uint32_t)id;
	uint32_t myCC     = 0;
	uint32_t inPort   = 0;
	uint8_t  ticktock = 0;

	while (1)
	{
		if (myCC % 1000 == 0) {
			if (ticktock == 0)
			{
				printk("%04x: tick\n", myId);
			}
			else
			{
				printk("%04x: tock\n", myId);
			}
			ticktock ^= 1;

			if (myId == 0xDEAD) {
				gpio_port_set_masked(ledgpio, 0xFF, (ticktock == 1) ? 0xFF: 0x00);
			} else {
				gpio_port_get(swsgpio, &inPort);
			}
		}

		k_sleep(K_MSEC(1));
		++myCC;
	}

}

void main(void)
{
	uint32_t       id1    = 0xDEAD;
	uint32_t       id2    = 0xBEEF;
	struct net_if  *iface = NULL;
	struct in_addr addr;

	printk("Configure GPIO...\n");

	ledgpio = device_get_binding(DT_LABEL(DT_ALIAS(ledgpio)));
	swsgpio = device_get_binding(DT_LABEL(DT_ALIAS(swsgpio)));

	gpio_pin_configure(ledgpio, 0, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
	gpio_pin_configure(ledgpio, 1, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
	gpio_pin_configure(ledgpio, 2, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
	gpio_pin_configure(ledgpio, 3, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
	gpio_pin_configure(ledgpio, 4, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
	gpio_pin_configure(ledgpio, 5, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);
	gpio_pin_configure(ledgpio, 6, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
	gpio_pin_configure(ledgpio, 7, GPIO_OUTPUT | GPIO_ACTIVE_HIGH);

	gpio_init_callback(&mySwitchCallback, GPIOCallbackHandler, 0x11);
	gpio_add_callback(swsgpio, &mySwitchCallback);

	gpio_pin_configure(swsgpio, 0, GPIO_INPUT);
	gpio_pin_configure(swsgpio, 1, GPIO_INPUT);
	gpio_pin_configure(swsgpio, 2, GPIO_INPUT);
	gpio_pin_configure(swsgpio, 3, GPIO_INPUT);
	gpio_pin_configure(swsgpio, 4, GPIO_INPUT);
	gpio_pin_configure(swsgpio, 5, GPIO_INPUT);
	gpio_pin_configure(swsgpio, 6, GPIO_INPUT);
	gpio_pin_configure(swsgpio, 7, GPIO_INPUT);

	gpio_pin_interrupt_configure(swsgpio, 0, GPIO_INT_ENABLE | GPIO_INT_EDGE);

	printk("GPIO config done.\n");

	printk("Configure IPv4...\n");
	iface = net_if_get_by_index(1);
	if (iface != NULL) {
		if (net_addr_pton(
			AF_INET,
			"192.168.10.100",
			&addr) < 0)
		{
			printk("IPv4 config failed (net_addr_pton)!\n");
		} else {
			net_if_ipv4_addr_add(iface, &addr, NET_ADDR_MANUAL, 0);
			printk("IPv4 config done.\n");
		}
	} else {
		printk("IPv4 config failed (resolve iface)!\n");
	}

	k_thread_create(
		&myThread1,
		myThread1Stack,
		STACK_SIZE,
		myThreadsCode,
		(void*)id1,
		NULL,
		NULL,
		2,
		K_USER,
		K_FOREVER);

	k_thread_start(&myThread1);

	k_thread_create(
		&myThread2,
		myThread2Stack,
		STACK_SIZE,
		myThreadsCode,
		(void*)id2,
		NULL,
		NULL,
		2,
		K_USER,
		K_FOREVER);

	k_thread_start(&myThread2);
}

