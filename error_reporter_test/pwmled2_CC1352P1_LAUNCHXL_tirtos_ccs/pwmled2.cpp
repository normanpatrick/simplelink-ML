/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== pwmled2.c ========
 */
/* For usleep() */
#include <unistd.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/PWM.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#include <ti/drivers/GPIO.h>

#include <features.h>

// wahoo - Let's do something with a GPIO / button press
//         toggle between pwm vs pause-pwm?
extern "C" int tflite_main_test(int argc, char** argv);


UART_Handle g_uart = NULL;

volatile uint8_t pause_pwm = 0;
void gpioButtonFxn(uint_least8_t index) {
    pause_pwm = newfeature::test1(pause_pwm);
    if(g_uart != NULL) {
        if(pause_pwm) {
            newfeature::print(g_uart, (const char *)"Pausing PWM cycle\r\n");
        } else {
            newfeature::print(g_uart, (const char *)"Un-pausing PWM cycle\r\n");
        }
    }
}

UART_Handle connect_button(void) {

    UART_Handle uart = newfeature::uart_init();

    /* Configure the button pin */
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* install Button callback and enable it */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    return uart;
}

/*
 *  ======== mainThread ========
 *  Task periodically increments the PWM duty for the on board LED.
 */
extern "C" void *mainThread(void *arg0)
{
    /* Period and duty in microseconds */
    uint16_t   pwmPeriod = 3000;
    uint16_t   duty = 0;
    uint16_t   dutyInc = 100;

    /* Sleep time in microseconds */
    uint32_t   time = 50000;
    PWM_Handle pwm1 = NULL;
    PWM_Handle pwm2 = NULL;
    PWM_Params params;

    uint8_t last_pwm_pause = 0;

    GPIO_init();
    UART_init();

    g_uart = connect_button();

    /* Call driver init functions. */
    PWM_init();

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;
    pwm1 = PWM_open(CONFIG_PWM_0, &params);
    if (pwm1 == NULL) {
        /* CONFIG_PWM_0 did not open */
        while (1);
    }

    PWM_start(pwm1);

    pwm2 = PWM_open(CONFIG_PWM_1, &params);
    if (pwm2 == NULL) {
        /* CONFIG_PWM_0 did not open */
        while (1);
    }

    PWM_start(pwm2);

    tflite_main_test(1, NULL);

    /* Loop forever incrementing the PWM duty */
    while (1) {
        if(pause_pwm != last_pwm_pause) {
            tflite_main_test(1, NULL);
        }
        if(!pause_pwm) {
            PWM_setDuty(pwm1, duty);

            PWM_setDuty(pwm2, duty);

            duty = (duty + dutyInc);

            if (duty == pwmPeriod || (!duty)) {
                dutyInc = - dutyInc;
            }
        }
        last_pwm_pause = pause_pwm;
        usleep(time);
    }
}
