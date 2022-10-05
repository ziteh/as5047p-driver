/**
 * @file as5047p.h
 * @author ZiTe (honmonoh@gmail.com)
 * @brief  A library for AMS AS5047P rotary position sensor/magnetic encoder.
 * @copyright MIT License, Copyright (c) 2022 ZiTe
 * @remark AS5047P SPI Interface:
 *         - Mode=1(CPOL=0, CPHA=1).
 *             - CPOL=0 --> Clock is low when idle.
 *             - CPHA=1 --> Data is sampled on the second edge(falling edge).
 *         - CSn(chip select) active low.
 *         - Data size=16-bit.
 *         - Bit order is MSB first.
 *         - Max clock rates up to 10 MHz.
 *         - Only supports slave operation mode.
 *
 */

#ifndef AS5047P_H
#define AS5047P_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

  typedef void (*as5047p_spi_send_t)(uint16_t data);
  typedef uint16_t (*as5047p_spi_read_t)(void);
  typedef void (*as5047p_spi_select_t)(void);
  typedef void (*as5047p_spi_deselect_t)(void);

  /**
   * @brief t_CSn: High time of CSn between two transmissions, Min: 350 ns.
   */
  typedef void (*as5047p_delay_t)(void);

  /**
   * @brief Dynamic angle error compensation (DAEC).
   */
  typedef enum
  {
    without_daec = 0,
    with_daec = !without_daec
  } as5047p_daec_t;

  typedef struct
  {
    as5047p_spi_send_t spi_send;
    as5047p_spi_read_t spi_read;
    as5047p_spi_select_t spi_select;
    as5047p_spi_deselect_t spi_deselect;
    as5047p_delay_t delay;
  } as5047p_handle_t;

  int8_t as5047p_make_handle(as5047p_spi_send_t spi_send_func,
                             as5047p_spi_read_t spi_read_func,
                             as5047p_spi_deselect_t spi_select_func,
                             as5047p_spi_deselect_t spi_deselect_func,
                             as5047p_delay_t delay_func,
                             as5047p_handle_t *as5047p_handle);

  void as5047p_reset(const as5047p_handle_t *as5047p_handle);

  void as5047p_config(const as5047p_handle_t *as5047p_handle,
                      uint8_t settings1,
                      uint8_t settings2);

  void as5047p_set_zero(const as5047p_handle_t *as5047p_handle, uint16_t position);

  int8_t as5047p_get_position(const as5047p_handle_t *as5047p_handle,
                              as5047p_daec_t with_daec,
                              uint16_t *position);

  int8_t as5047p_get_angle(const as5047p_handle_t *as5047p_handle,
                           as5047p_daec_t with_daec,
                           float *angle_degree);

  uint16_t as5047p_get_error_status(const as5047p_handle_t *as5047p_handle);

#ifdef __cplusplus
}
#endif

#endif /* AS5047P_H */