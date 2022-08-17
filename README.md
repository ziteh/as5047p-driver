# AS5047P Driver

A library for [ams AS5047P](https://ams.com/as5047p) rotary position sensor/magnetic encoder.

- `as5047p.c`
- `as5047p.h`
- `LICENSE`
- `README.md` (*This file*)


## Usage

[STM32 Example](https://github.com/ziteh/as5047p-example)

There are some functions that must be implemented in the user file (e.g. `main.c`).

Take STM32 HAL for example:
```c
/* main.c */

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
```


## AS5047P SPI Interface

- Mode = 1 (CPOL = 0, CPHA = 1).
    - Clock is low when idle.
    - Data is sampled on the second edge (i.e. falling edge).
- CSn(chip select) active low.
- Data size is 16-bit.
- Bit order is MSB first.
- Max clock rates up to 10 MHz.
- Only supports slave operation mode.
