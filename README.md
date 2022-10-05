# AS5047P Driver
A platform independent library for [ams AS5047P](https://ams.com/as5047p) rotary position sensor/magnetic encoder.

- `src/`: Main code.
  - [`as5047p.c`](./src/as5047p.c)
  - [`as5047p.h`](./src/as5047p.h)
- `examples/`: Some examples of this library.
  - [`stm32_hal/`](./examples/stm32_hal/)
- `LICENSE`
- `README.md` (*this file*)

## Usage
The following functions that must be implemented in the user file (e.g. `main.c`):
```c
void as5047p_spi_send(uint16_t data);
uint16_t as5047p_spi_read(void);
void as5047p_spi_select(void);
void as5047p_spi_deselect(void);
void as5047p_delay(void)
```

take STM32 HAL for example:
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

as5047p_config(&as5047p, 0b00100101, 0b00000000);
as5047p_set_zero(&as5047p, 0);
```

Read angle in degree:
```c
float angle_deg;
as5047p_get_angle(&as5047p, without_daec, &angle_deg);
printf("Angle: %3i\r\n", (int)angle_deg);
```

> STM32 HAL example: [examples/stm32_hal/src/main.c](./examples/stm32_hal/src/main.c)  
> [My blog](https://ziteh.github.io/2022/04/learningstm32-as5047p/) (Chinese) has more detailed descriptions.

## AS5047P SPI Interface
- Mode = 1 (CPOL = 0, CPHA = 1).
    - Clock is low when idle.
    - Data is sampled on the second edge (i.e. falling edge).
- CSn(chip select) pin active low.
- Data size is 16-bit.
- Bit order is MSB first.
- Max clock rates up to 10 MHz.
- Only supports slave operation mode.
