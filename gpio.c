// gpio.c
/**
*	\file
*	\brief
*		This is the source file for the GPIO device driver for Kinetis K64.
*		It contains all the implementation for configuration functions and runtime functions.
*		i.e., this is the application programming interface (API) for the GPIO peripheral.
*	\date	28/09/2022
*	\
*	    Author: Laura García
*
*/
#include <gpio.h>
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "config.h"

volatile static gpio_interrupt_flags_t g_intr_status_flag = {0};

static void (*gpio_C_callback)(uint32_t flags) = 0;
static void (*gpio_B_callback)(uint32_t flags) = 0;
static void (*gpio_A_callback)(uint32_t flags) = 0;


void GPIO_callback_init(gpio_name_t gpio, void (*handler)(uint32_t flags))
{
	switch(gpio){
		case GPIO_A:
			gpio_A_callback = handler;
			break;

		case GPIO_B:
			gpio_B_callback = handler;
			break;

		case GPIO_C:
			gpio_C_callback = handler;
			break;

//		case GPIO_D:
//			gpio_D_callback = handler;
//			break;
//
		default: // gpio == GPIO_E
			gpio_C_callback = handler;
			// IGNORE
//			gpio_E_callback = handler;
	}
}


void PORTC_IRQHandler(void)
{

	uint32_t irq_status = 0;

	irq_status = GPIO_PortGetInterruptFlags(GPIOC);

	if(gpio_C_callback)
	{
		gpio_C_callback(irq_status);
	}

	GPIO_PortClearInterruptFlags(GPIOC, irq_status);


}

void PORTB_IRQHandler(void)
{
	uint32_t irq_status = 0;

	irq_status = GPIO_PortGetInterruptFlags(GPIOB);

	if(gpio_B_callback)
	{
		gpio_B_callback(irq_status);
	}

	GPIO_PortClearInterruptFlags(GPIOB, irq_status);
}


void PORTA_IRQHandler(void)
{
	uint32_t irq_status = 0;

	irq_status = GPIO_PortGetInterruptFlags(GPIOA);

	if(gpio_A_callback)
	{
		gpio_A_callback(irq_status);
	}

	GPIO_PortClearInterruptFlags(GPIOA, irq_status);
}

void GPIO_init(void)
{
	gpio_pin_config_t gpio_input_config = {
			        kGPIO_DigitalInput,
			        0,
			    };

	gpio_pin_config_t gpio_output_config = {
			        kGPIO_DigitalOutput,
			        1,
			    };

	const port_pin_config_t button_config = {
			kPORT_PullUp,                                            /* Internal pull-up resistor is enabled */
			kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
			kPORT_PassiveFilterEnable,                              /* Passive filter is disabled */
			kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
			kPORT_HighDriveStrength,                                 /* High drive strength is configured */
			kPORT_MuxAsGpio,                                         /* Pin is configured as PTA4 */
			kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
			//CHECAR LA RESISTENCIA
		  };


	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortD);
	CLOCK_EnableClock(kCLOCK_PortE);

	//SWITCHES
	GPIO_PinInit(SW2_GPIO, SW2_PIN, &gpio_input_config);
	PORT_SetPinConfig(SW2_PORT, SW2_PIN, &button_config);
	PORT_SetPinInterruptConfig(SW2_PORT, SW2_PIN, kPORT_InterruptFallingEdge);

	GPIO_PinInit(SW3_GPIO, SW3_PIN, &gpio_input_config);
	PORT_SetPinConfig(SW3_PORT, SW3_PIN, &button_config);
	PORT_SetPinInterruptConfig(SW3_PORT, SW3_PIN, kPORT_InterruptFallingEdge);

	//LEDS
	PORT_SetPinMux(RED_LED_PORT, RED_LED_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(RED_LED_GPIO, RED_LED_PIN, &gpio_output_config);

	PORT_SetPinMux(BLUE_LED_PORT, BLUE_LED_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(BLUE_LED_GPIO, BLUE_LED_PIN, &gpio_output_config);

	PORT_SetPinMux(GREEN_LED_PORT, GREEN_LED_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(GREEN_LED_GPIO, GREEN_LED_PIN, &gpio_output_config);

	//SEGMENTS
	PORT_SetPinMux(SEGMENT_A_PORT, SEGMENT_A_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(SEGMENT_A_GPIO, SEGMENT_A_PIN, &gpio_output_config);

	PORT_SetPinMux(SEGMENT_B_PORT, SEGMENT_B_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(SEGMENT_B_GPIO, SEGMENT_B_PIN, &gpio_output_config);

	PORT_SetPinMux(SEGMENT_C_PORT, SEGMENT_C_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(SEGMENT_C_GPIO, SEGMENT_C_PIN, &gpio_output_config);

	PORT_SetPinMux(SEGMENT_D_PORT, SEGMENT_D_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(SEGMENT_D_GPIO, SEGMENT_D_PIN, &gpio_output_config);

	PORT_SetPinMux(SEGMENT_E_PORT, SEGMENT_E_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(SEGMENT_E_GPIO, SEGMENT_E_PIN, &gpio_output_config);

	PORT_SetPinMux(SEGMENT_F_PORT, SEGMENT_F_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(SEGMENT_F_GPIO, SEGMENT_F_PIN, &gpio_output_config);

	PORT_SetPinMux(SEGMENT_G_PORT, SEGMENT_G_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(SEGMENT_G_GPIO, SEGMENT_G_PIN, &gpio_output_config);

	//DISPLAY
	PORT_SetPinMux(DISPLAY_5_PORT, DISPLAY_5_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(DISPLAY_5_GPIO, DISPLAY_5_PIN, &gpio_output_config);

	PORT_SetPinMux(DISPLAY_4_PORT, DISPLAY_4_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(DISPLAY_4_GPIO, DISPLAY_4_PIN, &gpio_output_config);

	PORT_SetPinMux(DISPLAY_3_PORT, DISPLAY_3_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(DISPLAY_3_GPIO, DISPLAY_3_PIN, &gpio_output_config);

	PORT_SetPinMux(DISPLAY_2_PORT, DISPLAY_2_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(DISPLAY_2_GPIO, DISPLAY_2_PIN, &gpio_output_config);

	PORT_SetPinMux(DISPLAY_1_PORT, DISPLAY_1_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(DISPLAY_1_GPIO, DISPLAY_1_PIN, &gpio_output_config);

	PORT_SetPinMux(DISPLAY_0_PORT, DISPLAY_0_PIN, kPORT_MuxAsGpio);
	GPIO_PinInit(DISPLAY_0_GPIO, DISPLAY_0_PIN, &gpio_output_config);

	//BUTTONS
	PORT_SetPinConfig(STOPWATCH_PORT, STOPWATCH_PIN, &button_config);
	GPIO_PinInit(STOPWATCH_GPIO, STOPWATCH_PIN, &gpio_input_config);
	PORT_SetPinInterruptConfig(STOPWATCH_PORT, STOPWATCH_PIN, kPORT_InterruptFallingEdge);

	PORT_SetPinConfig(ALARM_PORT, ALARM_PIN, &button_config);
	GPIO_PinInit(ALARM_GPIO, ALARM_PIN, &gpio_input_config);
	PORT_SetPinInterruptConfig(ALARM_PORT, ALARM_PIN, kPORT_InterruptFallingEdge);

	PORT_SetPinConfig(CONFIG_TIME_PORT, CONFIG_TIME_PIN, &button_config);
	GPIO_PinInit(CONFIG_TIME_GPIO, CONFIG_TIME_PIN, &gpio_input_config);
	PORT_SetPinInterruptConfig(CONFIG_TIME_PORT, CONFIG_TIME_PIN, kPORT_InterruptFallingEdge);

	PORT_SetPinConfig(HOUR_START_PORT, HOUR_START_PIN, &button_config);
	GPIO_PinInit(HOUR_START_GPIO, HOUR_START_PIN, &gpio_input_config);
	PORT_SetPinInterruptConfig(HOUR_START_PORT, HOUR_START_PIN, kPORT_InterruptFallingEdge);

	PORT_SetPinConfig(MIN_STOP_PORT, MIN_STOP_PIN, &button_config);
	GPIO_PinInit(MIN_STOP_GPIO, MIN_STOP_PIN, &gpio_input_config);
	PORT_SetPinInterruptConfig(MIN_STOP_PORT, MIN_STOP_PIN, kPORT_InterruptFallingEdge);

	PORT_SetPinConfig(CLEAR_PORT, CLEAR_PIN, &button_config);
	GPIO_PinInit(CLEAR_GPIO, CLEAR_PIN, &gpio_input_config);

	PORT_SetPinConfig(PORTA, 2U, &button_config);
	GPIO_PinInit(GPIOA, 2U, &gpio_input_config);
	PORT_SetPinInterruptConfig(PORTA, 2U, kPORT_InterruptFallingEdge);

}


void GPIO_clear_irq_status(gpio_name_t gpio)
{
	switch(gpio){
		case GPIO_A:
			g_intr_status_flag.flag_port_a = false;
			break;
		case GPIO_B:
			g_intr_status_flag.flag_port_b = false;
			break;
		case GPIO_C:
			g_intr_status_flag.flag_port_c = false;
			break;
		case GPIO_D:
			g_intr_status_flag.flag_port_d = false;
			break;
		default: // gpio == GPIO_E
			g_intr_status_flag.flag_port_e = false;
	}
}

uint8_t GPIO_get_irq_status(gpio_name_t gpio)
{
	uint8_t status = 0;

	switch(gpio){
		case GPIO_A:
			status = g_intr_status_flag.flag_port_a;
			break;

		case GPIO_B:
			status = g_intr_status_flag.flag_port_b;
			break;

		case GPIO_C:
			status = g_intr_status_flag.flag_port_c;
			break;

		case GPIO_D:
			status = g_intr_status_flag.flag_port_d;
			break;

		default: // gpio == GPIO_E
			status = g_intr_status_flag.flag_port_e;
	}
	return(status);
}
