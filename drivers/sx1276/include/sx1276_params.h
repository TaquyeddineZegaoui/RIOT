
#ifndef SX1276_PARAM_DIO0
#define SX1276_PARAM_DIO0 GPIO_PIN(PORT_A, 8)
#endif

#ifndef SX1276_PARAM_DIO1 
#define SX1276_PARAM_DIO1 GPIO_PIN(PORT_A, 9)
#endif

#ifndef SX1276_PARAM_DIO2 
#define SX1276_PARAM_DIO2 GPIO_PIN(PORT_A, 10)
#endif

#ifndef SX1276_PARAM_DIO3 
#define SX1276_PARAM_DIO3 GPIO_PIN(PORT_A, 11)
#endif

#ifndef SX1276_PARAM_RESET 
#define SX1276_PARAM_RESET GPIO_PIN(PORT_C, 6)
#endif

#ifndef SX1276_PARAM_SPI
#define SX1276_PARAM_SPI (SPI_0)
#endif

#ifndef SX1276_PARAM_SPI_SPEED
#define SX1276_PARAM_SPI_SPEED (SPI_SPEED_1MHZ)
#endif

#ifndef SX1276_PARAM_SPI_NSS 
#define SX1276_PARAM_SPI_NSS GPIO_PIN(PORT_A, 4)
#endif

#ifndef SX1276_PARAM_SPI_MODE 
#define SX1276_PARAM_SPI_MODE SPI_CONF_FIRST_RISING
#endif

//TODO: add PARAMS_DEFAULT
#define SX1276_PARAMS_DEFAULT    {.spi = SX1276_PARAM_SPI, \
                                  .nss_pin = SX1276_PARAM_SPI_NSS, \
                                  .reset_pin = SX1276_PARAM_RESET, \
                                  .dio0_pin = SX1276_PARAM_DIO0, \
                                  .dio1_pin = SX1276_PARAM_DIO1, \
                                  .dio2_pin = SX1276_PARAM_DIO2, \
                                  .dio3_pin = SX1276_PARAM_DIO3}

static const sx1276_params_t sx1276_params[] =
{
#ifdef SX1276_PARAMS_BOARD
    SX1276_PARAMS_BOARD,
#else
    SX1276_PARAMS_DEFAULT,
#endif
}


