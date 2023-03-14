# AS5047P Driver
[![GitHub](https://img.shields.io/github/license/ziteh/as5047p-driver)](./LICENSE)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/ziteh/as5047p-driver)](https://github.com/ziteh/as5047p-driver/releases)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/ziteh/library/as5047-driver.svg)](https://registry.platformio.org/libraries/ziteh/as5047-driver)

A platform independent library for [ams AS5047P](https://ams.com/as5047p) rotary position sensor/magnetic encoder.

- `src/`: Main code.
  - [`as5047p.c`](./src/as5047p.c)
  - [`as5047p.h`](./src/as5047p.h)
- `examples/`: Some examples.
- `LICENSE`
- `README.md` (*this file*)

## Usage

### Install
There are 3 ways to install:

1. Go to VS Code > PIO Home > Libraries and search "as5047-driver", press the "Add to Project" button.

2. Add `lib_deps = ziteh/as5047-driver@^2.0.0` in `platformio.ini` file, for example:
```ini
[env:nucleo_f446re]
platform = ststm32
board = nucleo_f446re
framework = stm32cube
lib_deps = ziteh/as5047-driver@^2.0.0
```

3. PIO CLI: `pio pkg install --library "ziteh/as5047-driver@^2.0.0"`.

### Writing the Code
Include 
```c
#include <as5047p.h>
```

The following functions that must be implemented in the user file (e.g. `main.c`), function names are free:
```c
void as5047p_spi_send(uint16_t data);
uint16_t as5047p_spi_read(void);
void as5047p_spi_select(void);
void as5047p_spi_deselect(void);
void as5047p_delay(void)
```

Take STM32 HAL for example:
```c
void as5047p_spi_send(uint16_t data)
{
  HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);
}

uint16_t as5047p_spi_read(void)
{
  uint16_t data = 0;
  HAL_SPI_Receive(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);
  return data;
}

void as5047p_spi_select(void)
{
  HAL_GPIO_WritePin(AS5047P_SS_GPIO_Port, AS5047P_SS_Pin, GPIO_PIN_RESET);
}

void as5047p_spi_deselect(void)
{
  HAL_GPIO_WritePin(AS5047P_SS_GPIO_Port, AS5047P_SS_Pin, GPIO_PIN_SET);
}

// For 't_CSn': High time of CSn between two transmissions, >350 ns.
void as5047p_delay(void)
{
  HAL_Delay(1);
}
```

Make handle and init:
```c
as5047p_handle_t as5047p;
as5047p_make_handle(&as5047p_spi_send,
                    &as5047p_spi_read,
                    &as5047p_spi_select,
                    &as5047p_spi_deselect,
                    &as5047p_delay,
                    &as5047p);

// Configure SETTINGS1 and SETTINGS2.
as5047p_config(&as5047p, 0b00100101, 0b00000000); 

// Set specify position as zero.
as5047p_set_zero(&as5047p, 0);
```

Read angle in degree:
```c
float angle_deg;
int8_t error = as5047p_get_angle(&as5047p, without_daec, &angle_deg);

if (error == 0)
{
    printf("Angle: %f\r\n", angle_deg);
}
```

Read raw position:
```c
uint16_t position;
int8_t error = as5047p_get_position(&as5047p, without_daec, &position);

if (error == 0)
{
    printf("Raw: %5i\r\n", position);
}
```

## Examples
- STM32 HAL: [`examples/stm32_hal/src/main.c`](./examples/stm32_hal/src/main.c)  
- STM32 LibOpenCM3: [`examples/stm32_libopencm3/src/main.c`](./examples/stm32_libopencm3/src/main.c)
> [My blog](https://ziteh.github.io/2022/04/learningstm32-as5047p/) (Chinese) has more detailed descriptions.

## AS5047P SPI Interface
- SPI Mode = 1.
    - CPOL = 0: Clock is low when idle.
    - CPHA = 1: Data is sampled on the second edge (i.e. falling edge).
- CS (chip select) pin active low.
- Data size (data frame format) is 16-bit.
- Bit order is MSB first.
- Max clock rates up to 10 MHz.
- Only supports slave operation mode.
