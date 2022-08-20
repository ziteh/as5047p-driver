# AS5047P Library

A library for AMS AS5047P rotary position sensor/magnetic encoder.

- `as5047p.c`
- `as5047p.h`
- `README.md` (*This file*)


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

## License

MIT License

Copyright (c) 2022 ZiTe (honmonoh@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
