/**
 * @file main.c
 * @brief AS5047P driver example.
 * @author ZiTe (honmonoh@gmail.com)
 * @note Library repo: https://github.com/ziteh/as5047p-driver
 */

#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include "as5047p.h"

#define BAUDRATE (115200)

/* SPI GPIO. */
#define GPIO_SPI_SCK_MISO_MOSI_PORT (GPIOA)
#define GPIO_SPI_SCK_PIN (GPIO5)  /* D13. */
#define GPIO_SPI_MISO_PIN (GPIO6) /* D12. */
#define GPIO_SPI_MOSI_PIN (GPIO7) /* D11. */
#define GPIO_SPI_CS_PORT (GPIOB)
#define GPIO_SPI_CS_PIN (GPIO6) /* D10. */
#define GPIO_SPI_AF (GPIO_AF5)  /* Ref: Table-11 in DS10693. */

/* USART GPIO. */
#define GPIO_USART_TXRX_PORT (GPIOA)
#define GPIO_USART_TX_PIN (GPIO2) /* ST-Link (D1). */
#define GPIO_USART_RX_PIN (GPIO3) /* ST-Link (D0). */
#define GPIO_USART_AF (GPIO_AF7)  /* Ref: Table-11 in DS10693. */

static void rcc_setup(void);
static void usart_setup(void);
static void spi_setup(void);
static void systick_setup(void);
static void delay_ms(uint32_t ms);

void as5047p_spi_send(uint16_t data);
uint16_t as5047p_spi_read(void);
void as5047p_spi_select(void);
void as5047p_spi_deselect(void);
void as5047p_delay(void);

static volatile uint32_t systick_delay = 0;
as5047p_handle_t as5047p;

int main(void)
{
  rcc_setup();
  systick_setup();
  usart_setup();
  spi_setup();

  /* AS5047P init. */
  as5047p_make_handle(&as5047p_spi_send,
                      &as5047p_spi_read,
                      &as5047p_spi_select,
                      &as5047p_spi_deselect,
                      &as5047p_delay,
                      &as5047p);

  as5047p_config(&as5047p, 0b00100101, 0b00000000);
  as5047p_set_zero(&as5047p, 0);

  printf("\r\nSTM32 AS5047P, Ready.\r\n");

  while (1)
  {
    float angle;
    as5047p_get_angle(&as5047p, without_daec, &angle);

    uint16_t position;
    as5047p_get_position(&as5047p, without_daec, &position);

    printf("Angle: %3i, Raw: %5i\r\n", (int)angle, position);
    delay_ms(200);
  }

  return 0;
}

static void rcc_setup(void)
{
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_SPI1);
}

static void usart_setup(void)
{
  /* Set USART-Tx & Rx pin to alternate function. */
  gpio_mode_setup(GPIO_USART_TXRX_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_USART_TX_PIN | GPIO_USART_RX_PIN);

  gpio_set_af(GPIO_USART_TXRX_PORT,
              GPIO_USART_AF,
              GPIO_USART_TX_PIN | GPIO_USART_RX_PIN);

  /* Config USART. */
  usart_set_baudrate(USART2, BAUDRATE);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART2, USART_MODE_TX_RX);

  /* Setup interrupt. */
  // usart_enable_rx_interrupt(USART2); /* Enable receive interrupt. */
  // nvic_enable_irq(NVIC_USART2_IRQ);

  usart_enable(USART2);
}

static void spi_setup(void)
{
  /*
   * Set SPI-SCK & MISO & MOSI pin to alternate function.
   * Set SPI-CS pin to output push-pull (control CS by manual).
   */
  gpio_mode_setup(GPIO_SPI_SCK_MISO_MOSI_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN);

  gpio_set_output_options(GPIO_SPI_SCK_MISO_MOSI_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_50MHZ,
                          GPIO_SPI_SCK_PIN | GPIO_SPI_MOSI_PIN);

  gpio_set_af(GPIO_SPI_SCK_MISO_MOSI_PORT,
              GPIO_SPI_AF,
              GPIO_SPI_SCK_PIN | GPIO_SPI_MISO_PIN | GPIO_SPI_MOSI_PIN);

  /* Control CS by user instead of AF. */
  gpio_mode_setup(GPIO_SPI_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPI_CS_PIN);
  gpio_set_output_options(GPIO_SPI_CS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_SPI_CS_PIN);

  spi_disable(SPI1);
  spi_reset(SPI1);

  /* Set up in master mode. */
  spi_init_master(SPI1,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_64,   /* Clock. */
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, /* CPOL = 0. */
                  SPI_CR1_CPHA_CLK_TRANSITION_2,   /* CPHA = 1. */
                  SPI_CR1_DFF_16BIT,               /* Data frame format = 16 bit. */
                  SPI_CR1_MSBFIRST);               /* Data frame bit order. */

  /*
   * CS pin is not used on master side at standard multi-slave config.
   * It has to be managed internally (SSM=1, SSI=1)
   * to prevent any MODF error.
   */
  spi_enable_software_slave_management(SPI1); /* SSM = 1. */
  spi_set_nss_high(SPI1);                     /* SSI = 1. */

  as5047p_spi_deselect();
  spi_enable(SPI1);
}

static void systick_setup(void)
{
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  systick_set_reload(rcc_ahb_frequency / 8 / 1000 - 1);

  systick_interrupt_enable();
  systick_counter_enable();
}

static void delay_ms(uint32_t ms)
{
  systick_delay = ms;
  while (systick_delay != 0)
  {
    __asm__("nop"); /* Do nothing and wait. */
  }
}

void as5047p_spi_send(uint16_t data)
{
  spi_send(SPI1, data);

  /*
   * Wait for SPI transmit complete.
   * Ref: https://controllerstech.com/spi-using-registers-in-stm32/.
   */
  while (!(SPI_SR(SPI1) & SPI_SR_TXE)) /* Wait for 'Transmit buffer empty' flag to set. */
  {
  }
  while ((SPI_SR(SPI1) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }
}

uint16_t as5047p_spi_read(void)
{
  spi_send(SPI1, 0);                  /* Just for beget clock signal. */
  while ((SPI_SR(SPI1) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }

  uint16_t data = spi_read(SPI1);
  while ((SPI_SR(SPI1) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }

  return data;
}

void as5047p_spi_select(void)
{
  gpio_clear(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN); /* Low to select. */
}

void as5047p_spi_deselect(void)
{
  gpio_set(GPIO_SPI_CS_PORT, GPIO_SPI_CS_PIN); /* High to deselect. */
}

/**
 * @brief For 't_CSn': High time of CSn between two transmissions, >350 ns.
 */
void as5047p_delay(void)
{
  delay_ms(1);
}

/**
 * @brief SysTick handler (ISR).
 */
void sys_tick_handler(void)
{
  if (systick_delay != 0)
  {
    systick_delay--;
  }
}

/**
 * @brief USART2 Interrupt service routine.
 */
void usart2_isr(void)
{
  // uint8_t indata = usart_recv(USART2);
  USART_SR(USART2) &= ~USART_SR_RXNE; /* Clear 'Read data reg not empty' flag. */
}

/**
 * @brief For 'printf()'.
 */
int _write(int file, char *ptr, int len)
{
  int i;

  if (file == 1)
  {
    for (i = 0; i < len; i++)
    {
      usart_send_blocking(USART2, ptr[i]);
    }
    return i;
  }

  errno = EIO;
  return -1;
}
