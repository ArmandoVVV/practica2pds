/*
* Programa basico para la digitalizacion de una senal manteniendo una frecuencia
* constante de muestreo.
*
* Se utiliza el timer PIT (Periodic Interrupt Timer) configurado con el periodo
* de muestreo deseado.
* Durante un interrupt se toma una muestra del ADC y se pasa al DAC.
*
*/

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_pit.h"
#include "config.h"
#include "gpio.h"
#include "fsl_gpio.h"
#include "nvic.h"
#include "math.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_PIT_BASEADDR PIT
#define DEMO_PIT_CHANNEL  kPIT_Chnl_0
#define PIT_LED_HANDLER   PIT0_IRQHandler
#define PIT_IRQ_ID        PIT0_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

uint16_t ADC0_value(void);
volatile bool pitIsrFlag = false;
volatile bool ToggleFlag = false;

static int16_t volume_control = 0;
uint8_t dip_1 = 0;
uint8_t dip_2 = 0;

// h1
float h1[7] = {-0.1050, -0.1448, -0.1721, 0.8182, -0.1721, -0.1448, -0.1050};

// h2
float h2[7] = {0.0302, 0.0360, 0.0397, 0.0909, 0.0897, 0.0860, 0.0802};

static uint32_t counter = 0;

// Temp array for h1
float arr1[7] = {0};

// Temp array for h2
float arr2[7] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/
void PIT_LED_HANDLER(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerFlag);
    pitIsrFlag = true;
    __DSB();
}


/*******************************************************************************
 * main
 ******************************************************************************/

void volume_up(uint32_t sw2){
	volume_control += 10;

    if (volume_control > 300){
   	 volume_control = 300;
    }
}

void volume_down(uint32_t sw3){
	volume_control -= 10;

	if (volume_control < -300){
		volume_control = -300;
	}
}

// send the value to an array
void send_value (float value){
    // If counter is even
    if (!(counter & 1)){
        // send from array1 to array2
        for (uint8_t i = 0; i < 6; i++){
            arr2[i] = arr1[i + 1];
        }
        arr2[6] = value;
    }
    else{
        // send from array2 to array 1
        for (uint8_t i = 0; i < 6; i++){
            arr1[i] = arr2[i + 1];
        }
        arr1[6] = value;
    }
}

uint32_t convolution (float value, float h[]){
    float conv_result = 0;
    float analog_value_conv = 0.0f;
    float digital_result = 0.0f;

    // case when convolution is not necessary
    if (h == NULL){
        // Return original value
        digital_result = ((float) 4095U / 3.3F) * value;
        return (uint32_t) digital_result;
    }

    send_value(value);

    // If dip switch 1 is on
    if (GPIO_PinRead(GPIOB, 10U)){
        // Use arr2 to calculate convolution
        for (uint8_t i = 0; i < 7; i++){
            analog_value_conv += h[i] * arr2[(6) - i];
        }
    }
    else{ // if dip switch 1 is off
        // Use arr1 to calculate convolution
        for (uint8_t i = 0; i < 7; i++){
            analog_value_conv += h[i] * arr1[(6) - i];
        }
    }

    counter++;

    // transform the analog value to digital
    digital_result = ((float) 4095U / 3.3F) * analog_value_conv;
    // float absolute value
    conv_result = fabsf(digital_result);
    return (uint32_t) conv_result;
}


int main(void) {

  	/* Init board hardware. */

    volatile uint16_t adc_value;
    pit_config_t pitConfig;

    // Variable to store the real value
    float analog_value = 0.0f;
    static uint32_t output_value = 0;

	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();


    /*******************************************************************************
     * PIT Configuration */

        PIT_GetDefaultConfig(&pitConfig);
        /* Init pit module */
        PIT_Init(DEMO_PIT_BASEADDR, &pitConfig);
        /* Set timer period for channel 0 */
        PIT_SetTimerPeriod(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, USEC_TO_COUNT(45U, PIT_SOURCE_CLOCK));

    /*******************************************************************************
     * Enable Timer Interrupts */
        /* Enable timer interrupts for channel 0 */
        PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerInterruptEnable);
        /* Enable at the NVIC */
        EnableIRQ(PIT_IRQ_ID);

    /* Start timer channel 0 */
        PIT_StartTimer(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL);

    /*******************************************************************************
     * ADC Configuration */

    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    ADC0->CFG1 = ADC_CFG1_ADIV(0)|ADC_CFG1_ADLSMP_MASK |ADC_CFG1_MODE(1)|ADC_CFG1_ADICLK(0);
    ADC0->CFG2 = 0;
    ADC0->SC2 = 0;

    /*******************************************************************************
     * DAC Configuration */

    SIM->SCGC2 = 0x1000;
    DAC0->C0 = 0xC0;
    DAC0->DAT[0].DATL = 0;
    DAC0->DAT[0].DATH = 0;

    GPIO_init();
    NVIC_set_basepri_threshold(PRIORITY_8);
    NVIC_enable_interrupt_and_priotity(PORTC_IRQ,PRIORITY_2);
    NVIC_enable_interrupt_and_priotity(PORTA_IRQ,PRIORITY_2);
    NVIC_global_enable_interrupts;

    GPIO_callback_init(GPIO_A, volume_down);
    GPIO_callback_init(GPIO_C, volume_up);


     while (true){
             /* Check whether occur interrupt */
    	 if (true == pitIsrFlag){
    		 // ADC READ
    		 ADC0->SC1[0] = ADC_SC1_ADCH(12);
    		 while( (ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0);
    		 adc_value = ADC0->R[0];

    		 analog_value = ((float) (3.3F * adc_value)) / ((float) (4095U));

    		 dip_1 = GPIO_PinRead(GPIOB, 11U);
    		 dip_2 = GPIO_PinRead(GPIOB, 10U);

    		 if ((dip_1 == 0)) {
    			 if(dip_2 == 0){
    				 // convolution with h1
    				 output_value = convolution(analog_value, h1);
    			 }
    			 else {
    				 // convolution with h2
    				 output_value = convolution(analog_value, h2);
    			 }
    		 }
    		 // no convolution
    		 else {
    			 output_value = convolution(analog_value, NULL);
    		 }

    		 adc_value = output_value + volume_control;

    		 // if volume is at its minimum
    		 if(volume_control == -300){
    			 // mute
    			 adc_value = 0;
    		 }

    		 // limit value to 4095
    		 adc_value %= 4095;

    		 // the ADC value is given to the DAC
             DAC0->DAT[0].DATL = (adc_value) & 0xFF;
             DAC0->DAT[0].DATH = (adc_value >> 8) & 0x0F;

             // PIT Flag down
             pitIsrFlag = false;
    	 }
     }
}

