/*
 * features.cc
 *
 *  Created on: Oct 29, 2019
 *      Author: Nirmalendu Patra
 */
#include <stdint.h>
#include <string.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

namespace newfeature {
    uint8_t test1(uint8_t enable) {
        return enable ^ 1;
    }

    UART_Handle uart_init(void) {
        UART_Params uartParams;

        UART_Params_init(&uartParams);
        uartParams.writeDataMode  = UART_DATA_BINARY;
        uartParams.readDataMode   = UART_DATA_BINARY;
        uartParams.readReturnMode = UART_RETURN_FULL;

        UART_Handle uart = UART_open(CONFIG_UART_0, &uartParams);
        if (uart == NULL) {
            while (1);
        }
        return uart;
    }

    void uart_shutdown(UART_Handle uart) {
        UART_close(uart);
    }

    void print(UART_Handle uart, const char* s) {
        if(uart) {
            UART_write(uart, s, strlen(s));
        } else {
            while(1);
        }
    }
}
