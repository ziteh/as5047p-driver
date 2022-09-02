/**
 * @file as5047p.c
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  A library for AMS AS5047P rotary position sensor/magnetic encoder.
 * @copyright MIT License, Copyright (c) 2022 ZiTe
 *
 */

#include "as5047p.h"

#define BIT_MODITY(src, i, val) ((src) ^= (-(val) ^ (src)) & (1UL << (i)))
#define BIT_READ(src, i) (((src) >> (i)&1U))
#define BIT_TOGGLE(src, i) ((src) ^= 1UL << (i))

/* Volatile register address. */
#define AS5047P_NOP ((uint16_t)0x0000)
#define AS5047P_ERRFL ((uint16_t)0x0001)
#define AS5047P_PROG ((uint16_t)0x0003)
#define AS5047P_DIAAGC ((uint16_t)0x3FFC)
#define AS5047P_MAG ((uint16_t)0x3FFD)
#define AS5047P_ANGLEUNC ((uint16_t)0x3FFE)
#define AS5047P_ANGLECOM ((uint16_t)0x3FFF)

/* Non-Volatile register address. */
#define AS5047P_ZPOSM ((uint16_t)0x0016)
#define AS5047P_ZPOSL ((uint16_t)0x0017)
#define AS5047P_SETTINGS1 ((uint16_t)0x0018)
#define AS5047P_SETTINGS2 ((uint16_t)0x0019)

#define AS5047P_STEEINGS1_DEFAULT ((uint8_t)0x01)
#define AS5047P_STEEINGS2_DEFAULT ((uint8_t)0x00)

#define OP_WRITE ((uint8_t)0)
#define OP_READ ((uint8_t)1)

void as5047p_send_command(const as5047p_handle_t *as5047p_handle, uint16_t address, uint8_t op_read_write);
void as5047p_send_data(const as5047p_handle_t *as5047p_handle, uint16_t address, uint16_t data);
uint16_t as5047p_read_data(const as5047p_handle_t *as5047p_handle, uint16_t address);

void as5047p_spi_transmit(const as5047p_handle_t *as5047p_handle, uint16_t data);
uint16_t as5047p_spi_receive(const as5047p_handle_t *as5047p_handle);

void as5047p_nop(const as5047p_handle_t *as5047p_handle);
void delay(volatile uint16_t t);
uint8_t is_even_parity(uint16_t data);

/**
 * @brief Make a AS5047P handle.
 *
 * @param[in] spi_send_func
 * @param[in] spi_read_func
 * @param[in] spi_select_func
 * @param[in] spi_deselect_func
 * @param[in] delay_func The function that delay 350ns for t_CSn.
 * @param[out] as5047p_handle AS5047P handle.
 * @return Status code.
 *         0: Success.
 */
int8_t as5047p_make_handle(as5047p_spi_send_t spi_send_func,
                           as5047p_spi_read_t spi_read_func,
                           as5047p_spi_deselect_t spi_select_func,
                           as5047p_spi_deselect_t spi_deselect_func,
                           as5047p_delay_t delay_func,
                           as5047p_handle_t *as5047p_handle)
{
  as5047p_handle->spi_send = spi_send_func;
  as5047p_handle->spi_read = spi_read_func;
  as5047p_handle->spi_select = spi_select_func;
  as5047p_handle->spi_deselect = spi_deselect_func;
  as5047p_handle->delay = delay_func;
  return 0; /* Success. */
}

/**
 * @brief Reset.
 *
 * @param as5047p_handle
 */
void as5047p_reset(const as5047p_handle_t *as5047p_handle)
{
  as5047p_config(as5047p_handle, AS5047P_STEEINGS1_DEFAULT, AS5047P_STEEINGS2_DEFAULT);
  as5047p_set_zero(as5047p_handle, 0);
}

/**
 * @brief Setup AS5047P.
 *
 * @param as5047p_handle AS5047P handle.
 * @param settings1 Config 1.
 * @param settings2 Config 2.
 */
void as5047p_config(const as5047p_handle_t *as5047p_handle,
                    uint8_t settings1,
                    uint8_t settings2)
{
  /* SETTINGS1 bit 0 --> Factory Setting: Pre-Programmed to 1. */
  BIT_MODITY(settings1, 0, 1);

  /* SETTINGS1 bit 1 --> Not Used: Pre-Programmed to 0, must not be overwritten. */
  BIT_MODITY(settings1, 1, 0);

  as5047p_send_data(as5047p_handle, AS5047P_SETTINGS1, (uint16_t)(settings1 & 0x00FF));
  as5047p_send_data(as5047p_handle, AS5047P_SETTINGS2, (uint16_t)(settings2 & 0x00FF));
}

/**
 * @brief Reading error flags.
 *
 * @param as5047p_handle AS5047P handle.
 * @return Error flags. 0 for no error occurred.
 */
uint16_t as5047p_get_error_status(const as5047p_handle_t *as5047p_handle)
{
  return as5047p_read_data(as5047p_handle, AS5047P_ERRFL);
}

/**
 * @brief Read current position.
 *
 * @param as5047p_handle AS5047P handle.
 * @param with_daec With or without dynamic angle error compensation (DAEC).
 * @param position Current position raw value.
 * @return Status code.
 *         0: Success.
 *         -1: Error occurred.
 */
int8_t as5047p_get_position(const as5047p_handle_t *as5047p_handle,
                            as5047p_daec_t with_daec,
                            uint16_t *position)
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

  uint16_t data = as5047p_read_data(as5047p_handle, address);
  if (BIT_READ(data, 14) == 0)
  {
    *position = data & 0x3FFF;
    return 0; /* No error occurred. */
  }
  return -1; /* Error occurred. */
}

/**
 * @brief Read current angle in degree.
 *
 * @param as5047p_handle AS5047P handle
 * @param with_daec With or without dynamic angle error compensation (DAEC).
 * @param angle_degree Current angle in degree.
 * @return Status code.
 *         0: Success.
 *         -1: Error occurred.
 */
int8_t as5047p_get_angle(const as5047p_handle_t *as5047p_handle, as5047p_daec_t with_daec, float *angle_degree)
{
  uint16_t raw_position;
  int8_t error = as5047p_get_position(as5047p_handle, with_daec, &raw_position);
  if (error == 0)
  {
    /* Angle in degree = value * ( 360 / 2^14). */
    *angle_degree = raw_position * (360.0 / 0x4000);
  }

  return error;
}

/**
 * @brief Set specify position as zero.
 *
 * @param as5047p_handle AS5047P handle.
 * @param position Position raw value.
 */
void as5047p_set_zero(const as5047p_handle_t *as5047p_handle, uint16_t position)
{
  /* 8 most significant bits of the zero position. */
  as5047p_send_data(as5047p_handle, AS5047P_ZPOSM, ((position >> 6) & 0x00FF));

  /* 6 least significant bits of the zero position. */
  as5047p_send_data(as5047p_handle, AS5047P_ZPOSL, (position & 0x003F));

  as5047p_nop(as5047p_handle);
}

/**
 * @brief No operation instruction.
 *
 * @param as5047p_handle AS5047P handle.
 */
inline void as5047p_nop(const as5047p_handle_t *as5047p_handle)
{
  /* Reading the NOP register is equivalent to a nop (no operation) instruction. */
  as5047p_send_command(as5047p_handle, AS5047P_NOP, OP_READ);
}

/**
 * @brief Sending read or write command to AS5047P.
 *
 * @param as5047p_handle AS5047P handle.
 * @param address Register address.
 * @param op_read_write Read of write opration.
 */
void as5047p_send_command(const as5047p_handle_t *as5047p_handle, uint16_t address, uint8_t op_read_write)
{
  uint16_t frame = address & 0x3FFF;

  /* R/W: 0 for write, 1 for read. */
  BIT_MODITY(frame, 14, op_read_write);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_even_parity(frame))
  {
    BIT_TOGGLE(frame, 15);
  }

  as5047p_spi_transmit(as5047p_handle, frame);
}

/**
 * @brief Sending data to register.
 *
 * @param as5047p_handle AS5047P handle.
 * @param address Register address.
 * @param data Data.
 */
void as5047p_send_data(const as5047p_handle_t *as5047p_handle, uint16_t address, uint16_t data)
{
  uint16_t frame = data & 0x3FFF;

  /* Data frame bit 14 always low(0). */
  BIT_MODITY(frame, 14, 0);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_even_parity(frame))
  {
    BIT_TOGGLE(frame, 15);
  }

  as5047p_send_command(as5047p_handle, address, OP_WRITE);
  as5047p_spi_transmit(as5047p_handle, frame);
}

/**
 * @brief Reading data from register.
 *
 * @param as5047p_handle AS5047P handle.
 * @param address Register address.
 * @return Data.
 */
uint16_t as5047p_read_data(const as5047p_handle_t *as5047p_handle, uint16_t address)
{
  as5047p_send_command(as5047p_handle, address, OP_READ);
  return as5047p_spi_receive(as5047p_handle);
}

/**
 * @brief Start SPI transmit.
 *
 * @param as5047p_handle AS5047P handle.
 * @param data Data.
 */
inline void as5047p_spi_transmit(const as5047p_handle_t *as5047p_handle, uint16_t data)
{
  as5047p_handle->delay();

  as5047p_handle->spi_select();
  as5047p_handle->spi_send(data);
  as5047p_handle->spi_deselect();
}

/**
 * @brief Start SPI receive.
 *
 * @param as5047p_handle AS5047P handle.
 * @return Received data.
 */
inline uint16_t as5047p_spi_receive(const as5047p_handle_t *as5047p_handle)
{
  as5047p_handle->delay();

  as5047p_handle->spi_select();
  uint16_t data = as5047p_handle->spi_read();
  as5047p_handle->spi_deselect();

  return data;
}

/**
 * @brief Check data even parity.
 */
uint8_t is_even_parity(uint16_t data)
{
  uint8_t shift = 1;
  while (shift < (sizeof(data) * 8))
  {
    data ^= (data >> shift);
    shift <<= 1;
  }
  return !(data & 0x1);
}