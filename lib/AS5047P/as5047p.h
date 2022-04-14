/**
 * @file as5047p.h
 * @author ZiTe (honmonoh@gmail.com)
 * @remark AS5047P SPI Interface:
 *         - Mode=1(CPOL=0, CPHA=1).
 *             - CPOL=0 --> Clock is low when idle.
 *             - CPHA=1 --> Data is sampled on the second edge(falling edge).
 *         - CSn(chip select) active low.
 *         - Data size=16-bit.
 *         - Bit order is MSB first.
 *         - Max clock rates up to 10 MHz.
 *         - Only supports slave operation mode.
 */

#ifndef __AS5047P_H
#define __AS5047P_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

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

  void as5047p_spi_send(uint16_t data);
  uint16_t as5047p_spi_read(void);
  void as5047p_spi_select(void);
  void as5047p_spi_deselect(void);

  void as5047p_send_command(bool is_read_cmd, uint16_t address);
  void as5047p_send_data(uint16_t address, uint16_t data);
  uint16_t as5047p_read_data(uint16_t address);

  int as5047p_init(void);
  int as5047p_get_angle(bool with_daec, float *angle_degree);
  void as5047p_set_zero(void);

#ifdef __cplusplus
}
#endif

#endif /* __AS5047P_H */