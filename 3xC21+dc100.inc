/******************************
 * 3 x (Core-C21-DX DC100)
 ******************************/

#pragma once

#include <xs1.h>
#include <platform.h>
#include <flash.h>

#define APP_TILE 0
#define IFM_TILE 1

//      _   __          __          __ __    ____ 
//     / | / /___  ____/ /__     __/ // /_  / __ \
//    /  |/ / __ \/ __  / _ \   /_  _  __/ / / / /
//   / /|  / /_/ / /_/ /  __/  /_  _  __/ / /_/ / 
//  /_/ |_/\____/\__,_/\___/    /_//_/    \____/  
//                                            

/* SOMANET Core C21-DX */

#define NODE_0_APP_TILE 0
#define NODE_0_IFM_TILE 1

/* SPI port definitions */ 
on tile[NODE_0_APP_TILE]: fl_SPIPorts p_spi_flash_0 = { PORT_SPI_MISO_0,
                                PORT_SPI_SS_0,
                                PORT_SPI_CLK_0,
                                PORT_SPI_MOSI_0,
                                XS1_CLKBLK_2 };
/* RGB LED */
on tile[NODE_0_APP_TILE]: out port p_core_leds_0[3] = { XS1_PORT_1M,   /* Red */
                       XS1_PORT_1L,   /* Green */
                       XS1_PORT_1K }; /* Blue */

                 
/* SOMANET IFM DC-100 */

/* Buttons */
on tile[NODE_0_IFM_TILE]: in port p_ifm_btn1_0 = XS1_PORT_1J; /* Button 1 */
/* Button 2 shares port with encoder (see below) */

/* A4935 control ports */
on tile[NODE_0_IFM_TILE]: port p_ifm_coastn_0 = XS1_PORT_1G;
on tile[NODE_0_IFM_TILE]: out port p_ifm_esf_rstn_pwml_pwmh_0 = XS1_PORT_4F;
on tile[NODE_0_IFM_TILE]: port p_ifm_ff1_0 = XS1_PORT_1H;
on tile[NODE_0_IFM_TILE]: port p_ifm_ff2_0 = XS1_PORT_1I;

/* ADC */
on tile[NODE_0_IFM_TILE]: out port p_ifm_adc_clk_0 = XS1_PORT_1B;
on tile[NODE_0_IFM_TILE]: buffered out port:32 p_ifm_adc_conv_0 = XS1_PORT_1C;
on tile[NODE_0_IFM_TILE]: in buffered  port:32 p_ifm_adc_miso_0 = XS1_PORT_1D;
on tile[NODE_0_IFM_TILE]: in port p_ifm_dummy_port_0 = XS1_PORT_16A; /* any unused port */

/* BLDC Motor */
on tile[NODE_0_IFM_TILE]: in port p_ifm_hall_0 = XS1_PORT_4E;
on tile[NODE_0_IFM_TILE]: out buffered port:32 p_ifm_motor_hi_0[3] = { XS1_PORT_1K,   /* Hi A */
                            XS1_PORT_1O,   /* Hi B */
                            XS1_PORT_1M }; /* Hi C */
on tile[NODE_0_IFM_TILE]: buffered out port:32 p_ifm_motor_lo_0[3] = { XS1_PORT_1L,   /* Lo A */
                              XS1_PORT_1P,   /* Lo B */
                              XS1_PORT_1N }; /* Lo C */
on tile[NODE_0_IFM_TILE]: in port p_ifm_encoder_btn2_0 = XS1_PORT_4A; /* Encoder and Button 2 */

/* LEDs, Watchdog */
on stdcore[NODE_0_IFM_TILE]: out port p_ifm_shared_leds_wden_0 = XS1_PORT_4B; /* BlueGreenRed_WDEN */
on stdcore[NODE_0_IFM_TILE]: out port p_ifm_wd_tick_0 = XS1_PORT_1F; /* WatchdogTick */

/* Expansion Port */
on tile[NODE_0_IFM_TILE]: out port p_ifm_scl_0 = XS1_PORT_1E; /* I2C SCL */
on tile[NODE_0_IFM_TILE]: port p_ifm_sda_0 = XS1_PORT_1A; /* I2C SDA */
                   

//      _   __          __          __ __   ___
//     / | / /___  ____/ /__     __/ // /  <  /
//    /  |/ / __ \/ __  / _ \   /_  _  __/ / / 
//   / /|  / /_/ / /_/ /  __/  /_  _  __/ / /  
//  /_/ |_/\____/\__,_/\___/    /_//_/ / /_/   
//

/* SOMANET Core C21-DX */
                                        
#define NODE_1_APP_TILE 2
#define NODE_1_IFM_TILE 3

/* SPI port definitions */ 
on tile[NODE_1_APP_TILE]: fl_SPIPorts p_spi_flash_1 = { PORT_SPI_MISO_1,
                                PORT_SPI_SS_1,
                                PORT_SPI_CLK_1,
                                PORT_SPI_MOSI_1,
                                XS1_CLKBLK_2};
/* RGB LED */
on tile[NODE_1_APP_TILE]: out port p_core_leds_1[3] = { XS1_PORT_1M,   /* Red */
                       XS1_PORT_1L,   /* Green */
                       XS1_PORT_1K }; /* Blue */

/* SOMANET IFM DC-100 */

/* Buttons */
on tile[NODE_1_IFM_TILE]: in port p_ifm_btn1_1 = XS1_PORT_1J; /* Button 1 */
/* Button 2 shares port with encoder (see below) */

/* A4935 control ports */
on tile[NODE_1_IFM_TILE]: port p_ifm_coastn_1 = XS1_PORT_1G;
on tile[NODE_1_IFM_TILE]: out port p_ifm_esf_rstn_pwml_pwmh_1 = XS1_PORT_4F;
on tile[NODE_1_IFM_TILE]: port p_ifm_ff1_1 = XS1_PORT_1H;
on tile[NODE_1_IFM_TILE]: port p_ifm_ff2_1 = XS1_PORT_1I;

/* ADC */
on tile[NODE_1_IFM_TILE]: out port p_ifm_adc_clk_1 = XS1_PORT_1B;
on tile[NODE_1_IFM_TILE]: buffered out port:32 p_ifm_adc_conv_1 = XS1_PORT_1C;
on tile[NODE_1_IFM_TILE]: in buffered  port:32 p_ifm_adc_miso_1 = XS1_PORT_1D;
on tile[NODE_1_IFM_TILE]: in port p_ifm_dummy_port_1 = XS1_PORT_16A; /* any unused port */

/* BLDC Motor */
on tile[NODE_1_IFM_TILE]: in port p_ifm_hall_1 = XS1_PORT_4E;
on tile[NODE_1_IFM_TILE]: out buffered port:32 p_ifm_motor_hi_1[3] = { XS1_PORT_1K,   /* Hi A */
                            XS1_PORT_1O,   /* Hi B */
                            XS1_PORT_1M }; /* Hi C */
on tile[NODE_1_IFM_TILE]: buffered out port:32 p_ifm_motor_lo_1[3] = { XS1_PORT_1L,   /* Lo A */
                              XS1_PORT_1P,   /* Lo B */
                              XS1_PORT_1N }; /* Lo C */
on tile[NODE_1_IFM_TILE]: in port p_ifm_encoder_btn2_1 = XS1_PORT_4A; /* Encoder and Button 2 */

/* LEDs, Watchdog */
on stdcore[NODE_1_IFM_TILE]: out port p_ifm_shared_leds_wden_1 = XS1_PORT_4B; /* BlueGreenRed_WDEN */
on stdcore[NODE_1_IFM_TILE]: out port p_ifm_wd_tick_1 = XS1_PORT_1F; /* WatchdogTick */

/* Expansion Port */
on tile[NODE_1_IFM_TILE]: out port p_ifm_scl_1 = XS1_PORT_1E; /* I2C SCL */
on tile[NODE_1_IFM_TILE]: port p_ifm_sda_1 = XS1_PORT_1A; /* I2C SDA */

//      _   __          __          __ __   ___ 
//     / | / /___  ____/ /__     __/ // /  |__ \
//    /  |/ / __ \/ __  / _ \   /_  _  __/  _/ /
//   / /|  / /_/ / /_/ /  __/  /_  _  __/ / __/ 
//  /_/ |_/\____/\__,_/\___/    /_//_/ / /____/ 
//  

/* SOMANET Core C21-DX */                                    

#define NODE_2_APP_TILE 4
#define NODE_2_IFM_TILE 5

/* SPI port definitions */ 
on tile[NODE_2_APP_TILE]: fl_SPIPorts p_spi_flash_2 = { PORT_SPI_MISO_2,
                                PORT_SPI_SS_2,
                                PORT_SPI_CLK_2,
                                PORT_SPI_MOSI_2,
                                XS1_CLKBLK_2};
/* RGB LED */
on tile[NODE_2_APP_TILE]: out port p_core_leds_2[3] = { XS1_PORT_1M,   /* Red */
                       XS1_PORT_1L,   /* Green */
                       XS1_PORT_1K }; /* Blue */


/* SOMANET IFM DC-100 */

/* Buttons */
on tile[NODE_2_IFM_TILE]: in port p_ifm_btn1_2 = XS1_PORT_1J; /* Button 1 */
/* Button 2 shares port with encoder (see below) */

/* A4935 control ports */
on tile[NODE_2_IFM_TILE]: port p_ifm_coastn_2 = XS1_PORT_1G;
on tile[NODE_2_IFM_TILE]: out port p_ifm_esf_rstn_pwml_pwmh_2 = XS1_PORT_4F;
on tile[NODE_2_IFM_TILE]: port p_ifm_ff1_2 = XS1_PORT_1H;
on tile[NODE_2_IFM_TILE]: port p_ifm_ff2_2 = XS1_PORT_1I;

/* ADC */
on tile[NODE_2_IFM_TILE]: out port p_ifm_adc_clk_2 = XS1_PORT_1B;
on tile[NODE_2_IFM_TILE]: buffered out port:32 p_ifm_adc_conv_2 = XS1_PORT_1C;
on tile[NODE_2_IFM_TILE]: in buffered  port:32 p_ifm_adc_miso_2 = XS1_PORT_1D;
on tile[NODE_2_IFM_TILE]: in port p_ifm_dummy_port_2 = XS1_PORT_16A; /* any unused port */

/* BLDC Motor */
on tile[NODE_2_IFM_TILE]: in port p_ifm_hall_2 = XS1_PORT_4E;
on tile[NODE_2_IFM_TILE]: out buffered port:32 p_ifm_motor_hi_2[3] = { XS1_PORT_1K,   /* Hi A */
                            XS1_PORT_1O,   /* Hi B */
                            XS1_PORT_1M }; /* Hi C */
on tile[NODE_2_IFM_TILE]: buffered out port:32 p_ifm_motor_lo_2[3] = { XS1_PORT_1L,   /* Lo A */
                              XS1_PORT_1P,   /* Lo B */
                              XS1_PORT_1N }; /* Lo C */
on tile[NODE_2_IFM_TILE]: in port p_ifm_encoder_btn2_2 = XS1_PORT_4A; /* Encoder and Button 2 */

/* LEDs, Watchdog */
on stdcore[NODE_2_IFM_TILE]: out port p_ifm_shared_leds_wden_2 = XS1_PORT_4B; /* BlueGreenRed_WDEN */
on stdcore[NODE_2_IFM_TILE]: out port p_ifm_wd_tick_2 = XS1_PORT_1F; /* WatchdogTick */

/* Expansion Port */
on tile[NODE_2_IFM_TILE]: out port p_ifm_scl_2 = XS1_PORT_1E; /* I2C SCL */
on tile[NODE_2_IFM_TILE]: port p_ifm_sda_2 = XS1_PORT_1A; /* I2C SDA */