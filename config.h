/*
 * config.h
 *
 * 		\brief
 * 		This is a source file for the configuration for the practice 1.
 * 		Device driver for Kinetis K64.
 * 		it contains all the definitions of the macros.
 *
 *  	Created on: Sep 20, 2022
 *		Author: Laura García
 */

#ifndef CONFIG_H_
#define CONFIG_H_

typedef enum{FALSE, TRUE} BooleanType;
typedef enum{DISABLE,
			 ENABLE
}device_status_t;

//GPIO
#define SW2_GPIO 		  (GPIOC)  //SWITCHES
#define SW3_GPIO		  (GPIOA)
#define RED_LED_GPIO  	  (GPIOB)  //LEDS
#define BLUE_LED_GPIO 	  (GPIOB)
#define GREEN_LED_GPIO	  (GPIOE)
#define SEGMENT_A_GPIO    (GPIOD)  //SEGMENTS
#define SEGMENT_B_GPIO    (GPIOD)
#define SEGMENT_C_GPIO    (GPIOD)
#define SEGMENT_D_GPIO    (GPIOD)
#define SEGMENT_E_GPIO    (GPIOD)
#define SEGMENT_F_GPIO    (GPIOD)
#define SEGMENT_G_GPIO    (GPIOD)
#define DISPLAY_5_GPIO    (GPIOC)  // DISPLAY
#define DISPLAY_4_GPIO    (GPIOC)
#define DISPLAY_3_GPIO    (GPIOC)
#define DISPLAY_2_GPIO    (GPIOC)
#define DISPLAY_1_GPIO    (GPIOC)
#define DISPLAY_0_GPIO    (GPIOC)
#define STOPWATCH_GPIO    (GPIOC)  //BUTTONS
#define ALARM_GPIO        (GPIOC)
#define CONFIG_TIME_GPIO  (GPIOB)
#define HOUR_START_GPIO   (GPIOB)
#define MIN_STOP_GPIO     (GPIOB)
#define CLEAR_GPIO        (GPIOB)


//PORTS
#define SW2_PORT	      (PORTC)  //SWITCHES
#define SW3_PORT		  (PORTA)
#define RED_LED_PORT	  (PORTB)  //LEDS
#define BLUE_LED_PORT	  (PORTB)
#define GREEN_LED_PORT	  (PORTE)
#define SEGMENT_A_PORT    (PORTD)  //SEGMENTS
#define SEGMENT_B_PORT    (PORTD)
#define SEGMENT_C_PORT    (PORTD)
#define SEGMENT_D_PORT    (PORTD)
#define SEGMENT_E_PORT    (PORTD)
#define SEGMENT_F_PORT    (PORTD)
#define SEGMENT_G_PORT    (PORTD)
#define DISPLAY_5_PORT    (PORTC)  // DISPLAY
#define DISPLAY_4_PORT    (PORTC)
#define DISPLAY_3_PORT    (PORTC)
#define DISPLAY_2_PORT    (PORTC)
#define DISPLAY_1_PORT    (PORTC)
#define DISPLAY_0_PORT    (PORTC)
#define STOPWATCH_PORT    (PORTC)  //BUTTONS
#define ALARM_PORT        (PORTC)
#define CONFIG_TIME_PORT  (PORTB)
#define HOUR_START_PORT   (PORTB)
#define MIN_STOP_PORT     (PORTB)
#define CLEAR_PORT        (PORTB)


//PIN
#define SW2_PIN			(6u)   //SWITCHES
#define SW3_PIN			(4u)
#define RED_LED_PIN		(22u)  //LEDS
#define BLUE_LED_PIN	(21u)
#define GREEN_LED_PIN	(26u)
#define SEGMENT_A_PIN   (0u)  //SEGMENTS
#define SEGMENT_B_PIN   (1u)
#define SEGMENT_C_PIN   (2u)
#define SEGMENT_D_PIN   (3u)
#define SEGMENT_E_PIN   (4u)
#define SEGMENT_F_PIN   (5u)
#define SEGMENT_G_PIN   (7u)
#define DISPLAY_5_PIN   (0u)  // DISPLAY
#define DISPLAY_4_PIN   (1u)
// WARNING:
#define DISPLAY_3_PIN   (2u)	// PIN DEFECTUOSO EL PIN ORIGINAL ES 2, CAMBIAR
// END WARNING
#define DISPLAY_2_PIN   (3u)
#define DISPLAY_1_PIN   (4u)
#define DISPLAY_0_PIN   (5u)
#define STOPWATCH_PIN   (10u)  //BUTTONS
#define ALARM_PIN       (11u)
#define CONFIG_TIME_PIN (11u)
#define HOUR_START_PIN  (10u)
#define MIN_STOP_PIN    (3u)
#define CLEAR_PIN       (2u)


// MASKS
#define SW2_MASK	  	  (1 << SW2_PIN)		 // SWITCHES
#define SW3_MASK		  (1 << SW3_PIN)
#define RED_LED_MASK	  (1 << RED_LED_PIN)	 // LEDS
#define BLUE_LED_MASK	  (1 << BLUE_LED_PIN)
#define GREEN_LED_MASK	  (1 << GREEN_LED_PIN)
#define SEGMENT_A_MASK    (1 << SEGMENT_A_PIN) // SEGMENTS
#define SEGMENT_B_MASK    (1 << SEGMENT_B_PIN)
#define SEGMENT_C_MASK    (1 << SEGMENT_C_PIN)
#define SEGMENT_D_MASK    (1 << SEGMENT_D_PIN)
#define SEGMENT_E_MASK    (1 << SEGMENT_E_PIN)
#define SEGMENT_F_MASK    (1 << SEGMENT_F_PIN)
#define SEGMENT_G_MASK    (1 << SEGMENT_G_PIN)
#define DISPLAY_5_MASK    (1 << DISPLAY_5_PIN)  // DISPLAY
#define DISPLAY_4_MASK    (1 << DISPLAY_4_PIN)
#define DISPLAY_3_MASK    (1 << DISPLAY_3_PIN)
#define DISPLAY_2_MASK    (1 << DISPLAY_2_PIN)
#define DISPLAY_1_MASK    (1 << DISPLAY_1_PIN)
#define DISPLAY_0_MASK    (1 << DISPLAY_0_PIN)
#define STOPWATCH_MASK    (1 << STOPWATCH_PIN)  //BUTTONS
#define ALARM_MASK        (1 << ALARM_PIN)
#define CONFIG_TIME_MASK  (1 << CONFIG_TIME_PIN)
#define HOUR_START_MASK   (1 << HOUR_START_PIN)
#define MIN_STOP_MASK     (1 << MIN_STOP_PIN)
#define CLEAR_MASK        (1 << CLEAR_PIN)

#endif /* CONFIG_H_ */
