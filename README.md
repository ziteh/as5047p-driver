# STM32 AS5047P Example

- AS5047P code: [lib/AS5047P/](/lib/AS5047P/)
- Example code: [src/main.c](/src/main.c)

## Usage
Take NUCLEO-F446RE Board for example.

### Pin map

- D13(PA5): SPI_SCK
- D12(PA6): SPI_MISO
- D11(PA7): SPI_MOSI
- D10(PB6): SPI_CS

### USART

- Baudrate: 115200
- Data bits: 8
- Stop bits: 1
- Parity: None
- Line break: `\r\n`(CRLF)

### IDE

PlatformIO

## AS5047P SPI Interface
- Mode=1(CPOL=0, CPHA=1).
    - CPOL=0 --> Clock is low when idle.
    - CPHA=1 --> Data is sampled on the second edge(falling edge).
- CSn(chip select) active low.
- Data size=16-bit.
- Bit order is MSB first.
- Max clock rates up to 10 MHz.
- Only supports slave operation mode.
