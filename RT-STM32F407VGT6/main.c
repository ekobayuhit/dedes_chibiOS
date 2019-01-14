/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
/*=============================================================================*/
/*-----------------------------------------------------------*/
/* ChibiOS on 3P Controller PCB -STM32F407VGT6 */
/* Arok Gandring Lokajaya                      */
/*-----------------------------------------------------------*/
/*=============================================================================*/
/*---------------------------------   Header   --------------------------------*/
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include <stdlib.h>
/*=============================================================================*/

/*=============================================================================*/
/* Module constants.                                                         */
/*=============================================================================*/
#define LED1            PAL_LINE(GPIOD, 7)
#define LED2            PAL_LINE(GPIOB, 5)
#define LED_OFF              PAL_LOW
#define LED_ON               PAL_HIGH

#define BUTTON1          PAL_LINE(GPIOB, 7)
#define BUTTON_PRESSED       PAL_HIGH

#define ADC_GRP1_NUM_CHANNELS   2 /* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_BUF_DEPTH      4 /* Depth of the conversion buffer, channels are sampled ADC_GRP1_BUF_DEPTH/ADC_GRP1_NUM_CHANNELS times each.*/
/*=============================================================================*/
/*------------------------  Variable GLobal  --------------------------------*/
/*----ext interrupt---*/
int count_button1pressed=0;
int event_button1pressed=0;
/*---adc---*/
int flag_adc=1;
adcsample_t avg_adc1ch0, avg_adc1ch1; //average adc value
static adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];
/*--------------------------------------------------------------------------*/
/*=============================================================================*/


/*=============================================================================*/
//PWM
static PWMConfig pwmcfgTim1 = {
  1000,                                    /* 10kHz PWM clock frequency. Timer clock in Hz.   */
  10000,                                    /* Initial PWM period 1S.  PWM period in ticks.     */
  /* PWM period (in ticks) 1S (1/100kHz=0.01mS 0.01ms*10000 ticks=0.1S) */
  NULL,                                     /* Period callback.              */
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},          /* CH1 mode and callback.         */
   {PWM_OUTPUT_DISABLED, NULL},          /* CH2 mode and callback.         */
   {PWM_OUTPUT_DISABLED, NULL},             /* CH3 mode and callback.         */
   {PWM_OUTPUT_DISABLED, NULL},            /* CH4 mode and callback.         */
  },
  0,                                        /* Control Register 2.            */ 
  127,                                       /*TIM BDTR (break & dead-time) if use STM32_PWM_USE_ADVANCED.*/
  0                                         /* DMA/Interrupt Enable Register. TIM DIER register*/
};

/*=============================================================================*/
//ADC
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;
}

void adc_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
  (void) buffer; (void) n;
  /* Note, only in the ADC_COMPLETE state because the ADC driver fires an
     intermediate callback when the buffer is half full.*/
  if (adcp->state == ADC_COMPLETE) {  
    /* Calculates the average values from the ADC samples.*/
    avg_adc1ch0 = (samples1[0] + samples1[2] + samples1[4] + samples1[6]) / 4;
    avg_adc1ch1 = (samples1[1] + samples1[3] + samples1[5] + samples1[7]) / 4;
  }
}
/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 1 channel, SW triggered.
 * Channels:    IN11.
 */
static const ADCConversionGroup adc1_cfg = {
  FALSE,                                      /*Enable circular buffer mode*/ 
  //should the buffer start over if it becomes full before the conversion is over
  ADC_GRP1_NUM_CHANNELS,                      /*Number of analog channels belonging to the conversion group*/
  adc_callback,                                       /*ADC Complete Callback function*/ //Callback function for when the buffer is full, only used if we're doing asynchronous calls
  adcerrorcallback,                           /*ADC Error Callback function*/
  0,                                          /* CR1 */
  ADC_CR2_SWSTART,                            /* CR2 */
  0, //ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),      /* SMPR1 */ //Sample times for channel 10-18
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_480) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_480),          /* SMPR2 */ //Sample times for channel 0-9 - channel ADC
  //480 cycles sampling time
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),     /* SQR1 */ //Conversion group sequence 13-16 + sequence length e.g. PC15, SQR3 starts with SQ13
  0,                                          /* SQR2 */ //Conversion group sequence 7-12, SQR2 starts with SQ7
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)             /* SQR3 */ //Conversion group sequence 1-6 e.g PC1,PA1 belong to this
};

/*=============================================================================*/
//External Interrupt -> PAL_USE_CALLBACKS
static event_source_t button_pressed_event;
static event_source_t button_released_event;

static void button1_cb(void *arg) {
  (void)arg;
  chSysLockFromISR();
  if (palReadLine(BUTTON1) == BUTTON_PRESSED) {
    chEvtBroadcastI(&button_pressed_event);
  }
  else {
    chEvtBroadcastI(&button_released_event);
  }
  chSysUnlockFromISR();
}
/*=============================================================================*/

/*=============================================================================*/
/*----------------------------  BLINK  -------------------------------------*/
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    systime_t time = (flag_adc == 1) ? 300 : 1000;
    palToggleLine(LED2);
    chThdSleepMilliseconds(time);
  }
}
/*--------------------------------------------------------------------------*/
/*=============================================================================*/

/*=============================================================================*/
/*--------------  External Interrupt Using PAL CALLBACK  --------------------*/
static THD_WORKING_AREA(waThread2, 128);
static THD_FUNCTION(Thread2, arg) {
  (void)arg;
  chRegSetThreadName("INTERRUPT BUTTON1");
  event_listener_t el0, el1;
  /* Events initialization and registration.*/
  chEvtObjectInit(&button_pressed_event);
  chEvtObjectInit(&button_released_event);
  chEvtRegister(&button_pressed_event, &el0, 0);
  chEvtRegister(&button_released_event, &el1, 1);
  /* Enabling events on both edges of the button line.*/
  palEnableLineEvent(BUTTON1, PAL_EVENT_MODE_BOTH_EDGES);
  palSetLineCallback(BUTTON1, button1_cb, NULL);
  while (true) {
    eventmask_t events;
    events = chEvtWaitOne(EVENT_MASK(0) | EVENT_MASK(1));
    if (events & EVENT_MASK(0)) {
      palWriteLine(LED1, LED_ON);
      if(event_button1pressed==0){
        count_button1pressed++;
        event_button1pressed=1;
        chprintf((BaseSequentialStream *)&SD2,"Button Pressed : %d \r\n", count_button1pressed);
        if((count_button1pressed/2 >= 1) && (count_button1pressed%2==0)){
          flag_adc=1; //start read adc value
          // chprintf((BaseSequentialStream *)&SD2,"Start Reading ADC Value \r\n\r\n");
        }
        else{
          flag_adc=0; //stop read adc value
          // chprintf((BaseSequentialStream *)&SD2,"Stop Reading ADC Value \r\n\r\n");
        }
      }
    }
    if (events & EVENT_MASK(1)) {
      palWriteLine(LED1, LED_OFF);
      event_button1pressed=0;
    }
  }
}
/*--------------------------------------------------------------------------*/
/*=============================================================================*/

/*=============================================================================*/
/*--------------  ADC1  --------------------*/
static THD_WORKING_AREA(waThread3, 128);
static THD_FUNCTION(Thread3, arg) {
  (void)arg;
  chRegSetThreadName("Read-ADC");
  /* Setting up analog inputs */
  palSetGroupMode(GPIOA, PAL_PORT_BIT(0) | PAL_PORT_BIT(1), 0, PAL_MODE_INPUT_ANALOG); //PA0 - PA1
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTSVREFE();
  // adcStartConversion(&ADCD1, &adc1_cfg, samples1, ADC_GRP1_BUF_DEPTH);
  while (true) {
    // if(flag_adc!=0){
    //   adcStartConversion(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
    //   chprintf((BaseSequentialStream *)&SD2,"ADC1_CH0 : %d \r\n\r\n", avg_adc1ch0);
    //   chprintf((BaseSequentialStream *)&SD2,"ADC1_CH1 : %d \r\n\r\n", avg_adc1ch1);
    // }
    // else{
    //   adcStopConversion(&ADCD1);
    //   adcSTM32DisableTSVREFE();
    // }
    adcStartConversion(&ADCD1, &adc1_cfg, samples1, ADC_GRP1_BUF_DEPTH);
    // chprintf((BaseSequentialStream *)&SD2,"ADC1_CH0 : %d \r\n\r\n", avg_adc1ch0);
    // chprintf((BaseSequentialStream *)&SD2,"ADC1_CH1 : %d \r\n\r\n", avg_adc1ch1);
    chThdSleepMilliseconds(100);
  }
}
/*--------------------------------------------------------------------------*/
/*=============================================================================*/

/*=============================================================================*/
/*--------------  PWM  --------------------*/
static THD_WORKING_AREA(waThread4, 128);
static THD_FUNCTION(Thread4, arg) {
  (void)arg;
  chRegSetThreadName("PWM");
  //see alternate function mapping on datasheet stm32f407
  palSetGroupMode(GPIOE, PAL_PORT_BIT(9) | PAL_PORT_BIT(11), 0, PAL_MODE_ALTERNATE(1)); //TIM1_CH1 - TIM1_CH2
  // palSetPadMode(GPIOE, 9, PAL_MODE_ALTERNATE(1)); //TIM1_CH1
  // palSetPadMode(GPIOE, 11, PAL_MODE_ALTERNATE(1)); //TIM1_CH2
  // palSetPadMode(GPIOB, 4, PAL_MODE_ALTERNATE(2)); //TIM3_CH1
  // palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(2)); //TIM4_CH1
  /*Initializes the PWM driver 1*/
  pwmStart(&PWMD1, &pwmcfgTim1);
  // pwmStart(&PWMD3, &pwmcfg);
  // pwmStart(&PWMD4, &pwmcfg);
  /*
    Note : channel 1 di hardware -> di program ditulis channel 0
    PWM channel identifier (0...channels-1)
  */
  // pwmEnablePeriodicNotification(&PWMD1); //To enable callbacks we should enable this
  //pwmEnableChannelNotification(&PWMD1, 1);
  /* Starts the PWM channel 1 using 75% duty cycle. */
  // pwmDisableChannel(&PWMD1, 1);
  // pwmStop(&PWMD1);
  // /* Changes PWM period, implicitly.  */
  // pwmChangePeriod(&PWMD1, 2000);
  while (true) {
    pwmcfgTim1.frequency = avg_adc1ch1*2000;
    pwmStart(&PWMD1, &pwmcfgTim1);
    chprintf((BaseSequentialStream *)&SD2,"Freq [Hz] : %d \r\n\r\n", pwmcfgTim1.frequency);
    pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, avg_adc1ch0*2));
    int DutyCyle = (avg_adc1ch0*2)/100;
    chprintf((BaseSequentialStream *)&SD2,"DutyCyle [%] : %d \r\n\r\n", DutyCyle);
    // pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, avg_adc1ch1*2));
    chThdSleepMilliseconds(1000);  
  }
}
/*--------------------------------------------------------------------------*/
/*=============================================================================*/
/*=============================================================================*/
/*----------------------------  SPI2  -------------------------------------*/
/*
 * Low speed SPI configuration (328.125kHz, CPHA=0, CPOL=0, MSb first).
 */
const SPIConfig ls_spicfg_cs1 = {
  false,
  NULL,
  GPIOE,
  15,
  SPI_CR1_BR_2 | SPI_CR1_BR_1,
  0
};
const SPIConfig ls_spicfg_cs2 = {
  false,
  NULL,
  GPIOB,
  11,
  SPI_CR1_BR_2 | SPI_CR1_BR_1,
  0
};

uint16_t ReadThermocouple(SPIConfig spicfg){
  uint8_t rxbuf[2] = {0,0};
  spiAcquireBus(&SPID2); /* Acquire ownership of the bus.    */
  spiStart(&SPID2, &spicfg);
  spiSelect(&SPID2); /* Slave Select assertion.          */
  spiReceive(&SPID2, 2, rxbuf);
  spiUnselect(&SPID2); /* Slave Select de-assertion.       */
  spiReleaseBus(&SPID2); /* Ownership release.               */
  uint16_t i = (rxbuf[0] << 8)|(rxbuf[1]);
  i = (i >> 3 ) *0.25; // *0.25degC
  return i;
}

static THD_WORKING_AREA(waThread5, 256);
static THD_FUNCTION(Thread5, arg) {
  (void)arg;
  chRegSetThreadName("Thermocouple");
  /*
   * SPI2 I/O pins setup.
   */
  palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);       /* New SCK.     */
  palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);       /* New MISO.    */
  palSetPadMode(GPIOB, 11, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);       /* CS 1     */
  palSetPad(GPIOB, 11);
  palSetPadMode(GPIOE, 15, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);       /* CS 2.      */
  palSetPad(GPIOE, 15);
  while (true) {

    uint16_t temp1 = ReadThermocouple(ls_spicfg_cs1);
    uint16_t temp2 = ReadThermocouple(ls_spicfg_cs2);
    chprintf((BaseSequentialStream *)&SD2,"Temp 1 (C) : %.2d \r\n\r\n", temp1);
    chprintf((BaseSequentialStream *)&SD2,"Temp 2 (C) : %.2d \r\n\r\n", temp2);
    chThdSleepMilliseconds(500);
  }
}
/*--------------------------------------------------------------------------*/
/*=============================================================================*/
/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();
  /*===========================================================================*/
  /*----------------------------GPIO Init-------------------------------------*/
  palSetPadMode(GPIOD, 7, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  palSetPadMode(GPIOB, 7, PAL_MODE_INPUT_PULLDOWN);
  palClearLine(LED1);
  palClearLine(LED2);
  /*-------------------------------------------------------------------------*/
  /*===========================================================================*/
  /*------------------  Debug via Serial Using UART2  ----------------------*/
  static const SerialConfig serialcfg = {
    38400,          /* Bit rate */
    0,              /* CR1 register */ 
    0,              /* CR2 register */
    0               /* CR3 register */
  };
  // Default is 38400-8-N-1
  // bits :8, stopbits : 1, parity :none, flow control : none
  /* Activates the serial driver 2.
   * PD5(TX) and PD6(RX) are routed to USART2.*/
  sdStart(&SD2, &serialcfg);
  palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));
  chprintf((BaseSequentialStream *)&SD2,"Debug Via Serial Using UART2 \r\n");
  /*-------------------------------------------------------------------------*/
  /*===========================================================================*/
  /*
   * Creates thread.
   * Thread * 	chThdCreateStatic (void *wsp, size_t size, tprio_t prio, tfunc_t pf, void *arg)
 	  Creates a new thread into a static memory area. 
    Ordering thread by decreasing priority they are main, Thread1, ...., and idle.
  */
  // chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1, Thread1, NULL);
  // chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO+2, Thread2, NULL);
  // chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO+8, Thread3, NULL);
  // chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO+7, Thread4, NULL);
  chThdCreateStatic(waThread5, sizeof(waThread5), NORMALPRIO+5, Thread5, NULL);
  /*===========================================================================*/
  /*
   * Normal main() thread activity. NORMALPRIO
   */
  while (true) {
  }
}
