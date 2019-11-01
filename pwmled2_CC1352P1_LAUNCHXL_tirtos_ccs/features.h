/*
 * features.h
 *
 *  Created on: Oct 29, 2019
 *      Author:  Nirmalendu Patra
 */

#ifndef __features_h__
#define __features_h__

#ifdef __cplusplus
namespace newfeature {
    uint8_t test1(uint8_t);
    UART_Handle uart_init(void);
    void uart_shutdown(UART_Handle uart);
    void print(UART_Handle uart, const char* s);

}

#endif // __cplusplus

#endif // __features_h__
