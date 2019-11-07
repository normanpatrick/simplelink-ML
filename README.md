# simplelink-ML
Run machine learning (ML/DL) on TI SimpleLink family microcontrollers with
tensorflow, tf-lite.

* Current experiments use TI CC1352P1 launchpad, should be compatible with CC13x2,
  CC26x2 and with other members in TI SimpleLink family.
  * CC1352 is a multiband, multi protocol wireless SoC, with Cortex-M0 for
    modem controller and a Corex-M4 (48MHz) for user application code.
* More information on tflite can be found at TinyML community

* Current system is put together with ugly hacks - to get the system working first.

* `error_reporter_test`
  * uses 2 PWM LEDs, 1 GPIO (button) and UART_0 for console messages
  * uses TI compiler.
* `tflite_inference_test`
  * uses UART_0 for console messages as well as for keyboard input.
  * uses gcc
