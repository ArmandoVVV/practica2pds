// gpio.h
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

#ifndef GPIO_H_
#define GPIO_H_

#include "stdint.h"

/*! These constants are used to select an specific port in the different API functions*/
typedef enum{GPIO_A, /*!< Definition to select GPIO A */
			 GPIO_B, /*!< Definition to select GPIO B */
			 GPIO_C, /*!< Definition to select GPIO C */
			 GPIO_D, /*!< Definition to select GPIO D */
			 GPIO_E, /*!< Definition to select GPIO E */
			 GPIO_F  /*!< Definition to select GPIO F */
			} gpio_name_t;

typedef struct
{
	uint8_t flag_port_a : 1;
	uint8_t flag_port_b : 1;
	uint8_t flag_port_c : 1;
	uint8_t flag_port_d : 1;
	uint8_t flag_port_e : 1;
} gpio_interrupt_flags_t;


void GPIO_callback_init(gpio_name_t gpio, void (*handler)(uint32_t flags));

void GPIO_clear_irq_status(gpio_name_t gpio);

uint8_t GPIO_get_irq_status(gpio_name_t gpio);

void GPIO_init(void);

#endif /* GPIO_H_ */
