# Geegrow_MPU9250

This library works with MPU9250 sensor. It is an I2C sensor, so we communicate
with it using I2CTransport library.

We recomend you to use Geegrow MPU9250 board, because this board is tolerant
to 5V. If you want to use this library with another MPU9250 board, please be
noted that most of all are not tolerant to 5V, and you need to use 3.3V
controller or TTL converter.
The board can be purchased in out store https://geegrow.ru .

For more information you can visit https://github.com/geegrow/Geegrow_MPU9250

--------------------------------------------------------------------

"___________________________________" <--   This is a place for very motivating
                                            and enthusiastic slogan!

Please visit our store https://geegrow.ru and check out for some cool stuff!



<!-- START COMPATIBILITY TABLE -->

## Compatibility

MCU                | Tested Works | Doesn't Work | Not Tested  | Notes
------------------ | :----------: | :----------: | :---------: | -----
Atmega328 @ 16MHz  |              |              |     X       |
Atmega328 @ 12MHz  |              |              |     X       |
Atmega32u4 @ 16MHz |      X       |              |             |  
ESP8266            |              |              |     X       |
Atmega2560 @ 16MHz |              |              |     X       |
ATSAM3X8E          |              |              |     X       |
ATSAM21D           |              |              |     X       |
Intel Curie @ 32MHz|              |              |     X       |
STM32F2            |              |              |     X       |

  * ATmega328 @ 16MHz : Arduino UNO
  * ATmega328 @ 12MHz : Adafruit Pro Trinket 3V
  * ATmega32u4 @ 16MHz : Arduino Leonardo, Arduino Micro, Arduino Yun, Geegrow DaVinci
  * ESP8266 :
  * ATmega2560 @ 16MHz : Arduino Mega
  * ATSAM3X8E : Arduino Due
  * ATSAM21D : Arduino Zero, M0 Pro

<!-- END COMPATIBILITY TABLE -->
