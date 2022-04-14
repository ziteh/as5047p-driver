# AS5047P Library

- `as5047p.c`
- `as5047p.h`

## AS5047P SPI Interface

- Mode = 1 (CPOL = 0, CPHA = 1).
    - Clock is low when idle.
    - Data is sampled on the second edge(i.e. falling edge).
- CSn(chip select) active low.
- Data size is 16-bit.
- Bit order is MSB first.
- Max clock rates up to 10 MHz.
- Only supports slave operation mode.

## Usage

There are some functions that must be implemented in the user file(e.g. `main.c`).

For example (STM32 HAL):

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