*****************************************************************************
** Dedes-Project-ChibiOS                                                   **
** ekobayu/Arok Gandring Lokajaya                                          **
*****************************************************************************

Target : This runs on an PCB 3Phase Controller (STM32F407VGT6)
chibiOS 18.2.0

30-11-2018
	-chibiOS/RT
    -Blink LED
    -Serial2 (UART2)
11-12-2018
    -External Interrupt PAL_CALLBACK (Interrupt using button)
    -MultiThread (Blink and Ext Interrupt)
13-12-2018
    -ADC PA0 -PA1
14-12-2018
    -PWM  
    (Note : dalam library channel dimulai dari 0, 
     e.g channel 1 pada hardware ditulis sebagai channel 0 dalam program )
