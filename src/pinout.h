#ifndef PINOUT_H
#define PINOUT_H

// mutually-exclusive hardware flags
// HARDWARE_VERSION => motionhexa
// MINI_VERSION => minihexa

#ifndef HARDWARE_VERSION
#define HARDWARE_VERSION 0
#endif
#ifndef MINI_VERSION
#define MINI_VERSION 0
#endif
#if (HARDWARE_VERSION > 0) == (MINI_VERSION > 0)
#error "define exactly one of HARDWARE_VERSION or MINI_VERSION"
#endif

#define SDA 8
#define SCL 9

#if MINI_VERSION >= 1

// mini v2: WS2812B pixels in row-major
#define LED_SERIAL_DATA 0
#define UNCONNECTED_PIN_1 26

#elif HARDWARE_VERSION >= 7

#define EN_CHARGE 4 // pulled up, drive low to disable

#define PDM_CLK 20
#define PDM_LRCLK (PDM_CLK+1)
#define PDM_DATA 22

#define UNCONNECTED_PIN_1 29 // FIXME: this adc pin is exported, but used as random seed. need another random seed strategy.

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 18

#define PHOTOSENSOR_POWER_PIN 15
#define PHOTOSENSOR_READ_PIN 26  // Q3, sensor 0
#define PHOTOSENSOR1_READ_PIN 27 // Q2, sensor 1
#define PHOTOSENSOR2_READ_PIN 28 // Q1, sensor 2
#define PHOTOSENSOR_COUNT 3

// BMI270 on spi0
#define IMU_MISO_PIN 0
#define IMU_CS_PIN 1
#define IMU_SCK_PIN 2
#define IMU_MOSI_PIN 3
#define IMU_SPI_HZ 8000000 // BMI270 max is 10MHz
// MMC5603NJ is on the 3.3V i2c bus (SDA/SCL above) at 0x30

#define BUTTON_0 24
#define BUTTON_PRESSED_STATE HIGH

#define MOTION_INT_PIN 5
#define LED_LINE_0_PWR_PIN 11

#define VBUS_SENSOR_PIN 10

#define GPOUT_PIN 12

#define EN_LDO_PIN 23

#elif HARDWARE_VERSION >= 5

#define PDM_CLK 20
#define PDM_LRCLK (PDM_CLK+1)
#define PDM_DATA 23

#define UNCONNECTED_PIN_1 26

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 18

#define PHOTOSENSOR_POWER_PIN 25
#define PHOTOSENSOR_READ_PIN 27
#if HARDWARE_VERSION >= 6
#define PHOTOSENSOR_COUNT 1 // v5's sensor was never brought up (too much capacitance)
#endif

#define BUTTON_0 24
#define BUTTON_PRESSED_STATE HIGH

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#define VBUS_SENSOR_PIN 5
#define BATTERY_VOLTAGE_PIN 29

#define CHRG_PIN 16
#define GPOUT_PIN 1

#define EN_LDO_PIN 22

#if HARDWARE_VERSION == 5
#define EN_BOOST_PIN 0
#endif
#define V6_DETECTOR_PIN 0

#define DISABLE_CHARGE_PIN 17


#elif HARDWARE_VERSION == 4

#define PDM_CLK 23
#define PDM_LRCLK (PDM_CLK+1)
#define PDM_DATA 25

#define UNCONNECTED_PIN_1 26

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 18

#define PHOTOSENSOR_POWER_PIN 20
#define PHOTOSENSOR_READ_PIN 27

#define BUTTON_0 21
#define BUTTON_PRESSED_STATE HIGH

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#define PWR_SWITCH_PIN 28
#define VBUS_SENSOR_PIN 5
#define BATTERY_VOLTAGE_PIN 29

#define CHRG_PIN 16
#define GPOUT_PIN 1

#define EN_LDO_PIN 22
#define EN_BOOST_PIN 0
#define DISABLE_CHARGE_PIN 17

#elif HARDWARE_VERSION == 3

#define PDM_CLK 23
#define PDM_LRCLK (PDM_CLK+1)
#define PDM_DATA 25

#define UNCONNECTED_PIN_1 29

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 22

#define PHOTOSENSOR_POWER_PIN 26
#define PHOTOSENSOR_READ_PIN 27

#define BUTTON_0 17
#define BUTTON_PRESSED_STATE LOW

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#define PWR_SWITCH_PIN 28
#define VBUS_SENSOR_PIN 5

#define CHRG_PIN 16
#define GPOUT_PIN 0

#elif HARDWARE_VERSION == 2 // first form-factor rev

#define PDM_CLK 23
#define PDM_LRCLK (PDM_CLK+1)
#define PDM_DATA 25

#define UNCONNECTED_PIN_1 26

#define LED_SPI0_TX 19
#define LED_SPI0_SCK 22

#define PHOTOSENSOR_POWER_PIN 27
#define PHOTOSENSOR_READ_PIN 28

#define BATTERY_VOLTAGE_PIN 29

#define BUTTON_0 16
#define BUTTON_PRESSED_STATE LOW

#define MOTION_INT_PIN 2
#define LED_LINE_0_PWR_PIN 3

#else // first hardware rev

#define I2S_BCLK 1
#define I2S_LRCLK (I2S_BCLK+1)
#define I2S_DATA 3

#define UNCONNECTED_PIN_1 27

// These SPI pins are swapped from spec - FML but does not prevent FastLED from working
#define LED_SPI0_SCK 19
#define LED_SPI0_TX 18

#define PHOTOSENSOR_POWER_PIN 28
#define PHOTOSENSOR_READ_PIN 29

#define BUTTON_0 25
#define BUTTON_PRESSED_STATE LOW

#endif // pinout.

#ifndef PHOTOSENSOR_COUNT
#define PHOTOSENSOR_COUNT 0 // usable photosensors; v4 and earlier have none, v5's is not brought up
#endif


#define HAS_MOTION (HARDWARE_VERSION >= 1)
#define HAS_MICROPHONE (HARDWARE_VERSION >= 1)
#define HAS_BUTTON (HARDWARE_VERSION >= 1)
#define HAS_BUTTON_BOOT (HARDWARE_VERSION >= 4)
#define HAS_BATTERY (HARDWARE_VERSION >= 1)
#define HAS_AUTO_BRIGHTNESS (HARDWARE_VERSION >= 6)

#endif
