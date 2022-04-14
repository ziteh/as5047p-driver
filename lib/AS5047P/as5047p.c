/**
 * @file as5047p.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  A library for AMS AS5047P rotary position sensor/magnetic encoder.
 * @copyright MIT License, Copyright (c) 2022 ZiTe
 *
 */

#include "as5047p.h"

#define BIT_MODITY(src, n, val) ((src) ^= (-(val) ^ (src)) & (1UL << (n)))
#define BIT_READ(src, n) (((src) >> (n)&1U))
#define BIT_TOGGLE(src, n) ((src) ^= 1UL << (n))

bool is_even_parity(uint16_t data);
void as5047p_spi_transmit(uint16_t data);
uint16_t as5047p_spi_receive(void);

__attribute__((weak)) void as5047p_spi_send(uint16_t data)
{
  /* NOTE: This function should not be modified,
           it can be implemented in the user file.

     EXAMPLE:
     HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);
   */
}

__attribute__((weak)) uint16_t as5047p_spi_read(void)
{
  /* NOTE: This function should not be modified,
           it can be implemented in the user file.

     EXAMPLE:
     uint16_t data = 0;
     HAL_SPI_Receive(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);
    return data;
   */
}

__attribute__((weak)) void as5047p_spi_select(void)
{
  /* NOTE: This function should not be modified,
           it can be implemented in the user file.

     EXAMPLE:
     HAL_GPIO_WritePin(AS5047P_SS_GPIO_Port, AS5047P_SS_Pin, GPIO_PIN_RESET);
   */
}

__attribute__((weak)) void as5047p_spi_deselect(void)
{
  /* NOTE: This function should not be modified,
           it can be implemented in the user file.

     EXAMPLE:
     HAL_GPIO_WritePin(AS5047P_SS_GPIO_Port, AS5047P_SS_Pin, GPIO_PIN_SET);
   */
}

void as5047p_send_command(bool is_read_cmd, uint16_t address)
{
  uint16_t frame = address & 0x3FFF;

  /* R/W: 0 for write, 1 for read. */
  BIT_MODITY(frame, 14, is_read_cmd ? 1 : 0);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_even_parity(frame))
  {
    BIT_TOGGLE(frame, 15);
  }

  as5047p_spi_transmit(frame);
}

void as5047p_send_data(uint16_t address, uint16_t data)
{
  uint16_t frame = data & 0x3FFF;

  /* Data frame bit 14 always low(0). */
  BIT_MODITY(frame, 14, 0);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_even_parity(frame))
  {
    BIT_TOGGLE(frame, 15);
  }

  as5047p_send_command(false, address);
  as5047p_spi_transmit(frame);
}

uint16_t as5047p_read_data(uint16_t address)
{
  as5047p_send_command(true, address);
  uint16_t received_data = as5047p_spi_receive();
  return received_data;
}

int as5047p_init(void)
{
  as5047p_send_data(AS5047P_SETTINGS1, 0x0001);
  as5047p_send_data(AS5047P_SETTINGS2, 0x0000);
  if (as5047p_read_data(AS5047P_ERRFL) != 0)
  {
    return -1; /* Error occurred. */
  }
  return 0; /* No error occurred. */
}

int as5047p_get_angle(bool with_daec, float *angle_degree)
{
  uint16_t address;
  if (with_daec)
  {
    /* Measured angle WITH dynamic angle error compensation(DAEC). */
    address = AS5047P_ANGLECOM;
  }
  else
  {
    /* Measured angle WITHOUT dynamic angle error compensation(DAEC). */
    address = AS5047P_ANGLEUNC;
  }

  uint16_t data = as5047p_read_data(address);
  if (BIT_READ(data, 14) == 0)
  {
    *angle_degree = (data & 0x3FFF) * (360.0 / 0x3FFF);
    return 0; /* No error occurred. */
  }
  return -1; /* Error occurred. */
}

void as5047p_set_zero(void)
{
  uint16_t present_position = as5047p_read_data(AS5047P_ANGLEUNC);

  /*  8 most significant bits of the zero position. */
  as5047p_send_data(AS5047P_ZPOSM, ((present_position >> 6) & 0x00FF));

  /* 6 least significant bits of the zero position. */
  as5047p_send_data(AS5047P_ZPOSL, (present_position & 0x003F));
}

void as5047p_spi_transmit(uint16_t data)
{
  as5047p_spi_select();
  as5047p_spi_send(data);
  as5047p_spi_deselect();
}

uint16_t as5047p_spi_receive(void)
{
  as5047p_spi_select();
  uint16_t data = as5047p_spi_read();
  as5047p_spi_deselect();
  return data;
}

bool is_even_parity(uint16_t data)
{
  uint8_t shift = 1;
  while (shift < (sizeof(data) * 8))
  {
    data ^= (data >> shift);
    shift <<= 1;
  }
  return !(data & 0x1);
}