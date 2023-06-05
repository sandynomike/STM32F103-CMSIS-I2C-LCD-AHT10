# STM32F103-CMSIS-I2C-LCD-AHT10
Sample project using a Blue Pill (STM32F103) a 16x2 LCD display driven by an I2C LCD driver module, and an AHT10
temperature and humidity sensor module.
<br><br>
The I2C LCD Driver Module is hooked up to the I2C1 port (pins B6/B7) and the AHT10 sensor module is
hooked up to I2C2 (pins B10/B11).
<br>
Uses the following simple include libraries:
- STM32F103-pause-lib.c          (Contains us timing routine)
- STM32F103-CMSIS-I2C-lib.c      (Contiains root I2C routines)
- STM32F103-CMSIS-I2C-LCD-lib.c  (Contains I2C LCD Driver Module routines)
- STM32F103-CMSIS-I2C-LCD-AHT10-lib.c  (Contains AHT10 I2C routines to get temperature and humidity.)

See main.c for details.
