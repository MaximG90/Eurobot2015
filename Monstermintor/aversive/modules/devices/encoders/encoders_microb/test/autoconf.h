/*
 * Automatically generated C config: don't edit
 */
#define AUTOCONF_INCLUDED

/*
 * Hardware
 */
#undef  CONFIG_MCU_AT90S2313
#undef  CONFIG_MCU_AT90S2323
#undef  CONFIG_MCU_AT90S3333
#undef  CONFIG_MCU_AT90S2343
#undef  CONFIG_MCU_ATTINY22
#undef  CONFIG_MCU_ATTINY26
#undef  CONFIG_MCU_ATTINY45
#undef  CONFIG_MCU_AT90S4414
#undef  CONFIG_MCU_AT90S4433
#undef  CONFIG_MCU_AT90S4434
#undef  CONFIG_MCU_AT90S8515
#undef  CONFIG_MCU_AT90S8534
#undef  CONFIG_MCU_AT90S8535
#undef  CONFIG_MCU_AT86RF401
#undef  CONFIG_MCU_ATMEGA103
#undef  CONFIG_MCU_ATMEGA603
#undef  CONFIG_MCU_AT43USB320
#undef  CONFIG_MCU_AT43USB355
#undef  CONFIG_MCU_AT76C711
#undef  CONFIG_MCU_ATMEGA8
#undef  CONFIG_MCU_ATMEGA48
#undef  CONFIG_MCU_ATMEGA88
#undef  CONFIG_MCU_ATMEGA8515
#undef  CONFIG_MCU_ATMEGA8535
#undef  CONFIG_MCU_ATTINY13
#undef  CONFIG_MCU_ATTINY2313
#undef  CONFIG_MCU_ATMEGA16
#undef  CONFIG_MCU_ATMEGA161
#undef  CONFIG_MCU_ATMEGA162
#undef  CONFIG_MCU_ATMEGA163
#undef  CONFIG_MCU_ATMEGA165
#undef  CONFIG_MCU_ATMEGA168
#undef  CONFIG_MCU_ATMEGA169
#undef  CONFIG_MCU_ATMEGA32
#undef  CONFIG_MCU_ATMEGA323
#undef  CONFIG_MCU_ATMEGA325
#undef  CONFIG_MCU_ATMEGA328P
#undef  CONFIG_MCU_ATMEGA3250
#undef  CONFIG_MCU_ATMEGA64
#undef  CONFIG_MCU_ATMEGA645
#undef  CONFIG_MCU_ATMEGA6450
#define CONFIG_MCU_ATMEGA128 1
#undef  CONFIG_MCU_ATMEGA1281
#undef  CONFIG_MCU_AT90CAN128
#undef  CONFIG_MCU_AT94K
#undef  CONFIG_MCU_AT90S1200
#undef  CONFIG_MCU_ATMEGA2560
#undef  CONFIG_MCU_ATMEGA256
#undef  CONFIG_MCU_ATXMEGA128A1
#undef  CONFIG_MCU_ATMEGA168P
#undef  CONFIG_MCU_ATMEGA1284P
#define CONFIG_QUARTZ (16000000)

/*
 * Generation options
 */
#undef  CONFIG_OPTM_0
#undef  CONFIG_OPTM_1
#undef  CONFIG_OPTM_2
#undef  CONFIG_OPTM_3
#define CONFIG_OPTM_S 1
#define CONFIG_MATH_LIB 1
#undef  CONFIG_FDEVOPEN_COMPAT
#undef  CONFIG_NO_PRINTF
#undef  CONFIG_MINIMAL_PRINTF
#define CONFIG_STANDARD_PRINTF 1
#undef  CONFIG_ADVANCED_PRINTF
#define CONFIG_FORMAT_IHEX 1
#undef  CONFIG_FORMAT_SREC
#undef  CONFIG_FORMAT_BINARY

/*
 * Base modules
 */

/*
 * Enable math library in generation options to see all modules
 */
#undef  CONFIG_MODULE_CIRBUF
#undef  CONFIG_MODULE_CIRBUF_LARGE
#define CONFIG_MODULE_FIXED_POINT 1
#undef  CONFIG_MODULE_VECT2
#undef  CONFIG_MODULE_GEOMETRY
#undef  CONFIG_MODULE_HOSTSIM
#define CONFIG_MODULE_SCHEDULER 1
#undef  CONFIG_MODULE_SCHEDULER_STATS
#define CONFIG_MODULE_SCHEDULER_CREATE_CONFIG 1
#undef  CONFIG_MODULE_SCHEDULER_USE_TIMERS
#define CONFIG_MODULE_SCHEDULER_TIMER0 1
#undef  CONFIG_MODULE_SCHEDULER_MANUAL
#undef  CONFIG_MODULE_CALLOUT
#undef  CONFIG_MODULE_TIME
#undef  CONFIG_MODULE_TIME_CREATE_CONFIG
#undef  CONFIG_MODULE_TIME_EXT
#undef  CONFIG_MODULE_TIME_EXT_CREATE_CONFIG

/*
 * Communication modules
 */

/*
 * uart needs circular buffer, mf2 client may need scheduler
 */
#undef  CONFIG_MODULE_UART
#undef  CONFIG_MODULE_UART_9BITS
#undef  CONFIG_MODULE_UART_CREATE_CONFIG
#undef  CONFIG_MODULE_SPI
#undef  CONFIG_MODULE_SPI_CREATE_CONFIG
#undef  CONFIG_MODULE_I2C
#undef  CONFIG_MODULE_I2C_MASTER
#undef  CONFIG_MODULE_I2C_MULTIMASTER
#undef  CONFIG_MODULE_I2C_CREATE_CONFIG
#undef  CONFIG_MODULE_MF2_CLIENT
#undef  CONFIG_MODULE_MF2_CLIENT_USE_SCHEDULER
#undef  CONFIG_MODULE_MF2_CLIENT_CREATE_CONFIG
#undef  CONFIG_MODULE_MF2_SERVER
#undef  CONFIG_MODULE_MF2_SERVER_CREATE_CONFIG

/*
 * Hardware modules
 */
#undef  CONFIG_MODULE_TIMER
#undef  CONFIG_MODULE_TIMER_CREATE_CONFIG
#undef  CONFIG_MODULE_TIMER_DYNAMIC
#undef  CONFIG_MODULE_PWM
#undef  CONFIG_MODULE_PWM_CREATE_CONFIG
#undef  CONFIG_MODULE_PWM_NG
#undef  CONFIG_MODULE_ADC
#undef  CONFIG_MODULE_ADC_CREATE_CONFIG

/*
 * IHM modules
 */
#undef  CONFIG_MODULE_MENU
#undef  CONFIG_MODULE_VT100
#undef  CONFIG_MODULE_RDLINE
#undef  CONFIG_MODULE_RDLINE_CREATE_CONFIG
#undef  CONFIG_MODULE_RDLINE_KILL_BUF
#undef  CONFIG_MODULE_RDLINE_HISTORY
#undef  CONFIG_MODULE_PARSE
#undef  CONFIG_MODULE_PARSE_NO_FLOAT

/*
 * External devices modules
 */
#undef  CONFIG_MODULE_LCD
#undef  CONFIG_MODULE_LCD_CREATE_CONFIG
#undef  CONFIG_MODULE_MULTISERVO
#undef  CONFIG_MODULE_MULTISERVO_CREATE_CONFIG
#undef  CONFIG_MODULE_AX12
#undef  CONFIG_MODULE_AX12_CREATE_CONFIG

/*
 * Brushless motor drivers (you should enable pwm modules to see all)
 */
#undef  CONFIG_MODULE_BRUSHLESS_3PHASE_DIGITAL_HALL
#undef  CONFIG_MODULE_BRUSHLESS_3PHASE_DIGITAL_HALL_CREATE_CONFIG
#undef  CONFIG_MODULE_BRUSHLESS_3PHASE_DIGITAL_HALL_DOUBLE
#undef  CONFIG_MODULE_BRUSHLESS_3PHASE_DIGITAL_HALL_DOUBLE_CREATE_CONFIG

/*
 * Encoders (you need comm/spi for encoders_spi)
 */
#define CONFIG_MODULE_ENCODERS_MICROB 1
#define CONFIG_MODULE_ENCODERS_MICROB_CREATE_CONFIG 1
#undef  CONFIG_MODULE_ENCODERS_EIRBOT
#undef  CONFIG_MODULE_ENCODERS_EIRBOT_CREATE_CONFIG
#undef  CONFIG_MODULE_ENCODERS_SPI
#undef  CONFIG_MODULE_ENCODERS_SPI_CREATE_CONFIG

/*
 * Robot specific modules (fixed point lib may be needed)
 */
#undef  CONFIG_MODULE_ROBOT_SYSTEM
#undef  CONFIG_MODULE_ROBOT_SYSTEM_USE_F64
#undef  CONFIG_MODULE_ROBOT_SYSTEM_MOT_AND_EXT
#undef  CONFIG_MODULE_POSITION_MANAGER
#undef  CONFIG_MODULE_COMPENSATE_CENTRIFUGAL_FORCE
#undef  CONFIG_MODULE_TRAJECTORY_MANAGER
#undef  CONFIG_MODULE_BLOCKING_DETECTION_MANAGER
#undef  CONFIG_MODULE_OBSTACLE_AVOIDANCE
#undef  CONFIG_MODULE_OBSTACLE_AVOIDANCE_CREATE_CONFIG

/*
 * Control system modules
 */
#undef  CONFIG_MODULE_CONTROL_SYSTEM_MANAGER

/*
 * Filters
 */
#undef  CONFIG_MODULE_PID
#undef  CONFIG_MODULE_PID_CREATE_CONFIG
#undef  CONFIG_MODULE_RAMP
#undef  CONFIG_MODULE_QUADRAMP
#undef  CONFIG_MODULE_QUADRAMP_DERIVATE
#undef  CONFIG_MODULE_BIQUAD

/*
 * Radio devices
 */

/*
 * Some radio devices require SPI to be activated
 */
#undef  CONFIG_MODULE_CC2420
#undef  CONFIG_MODULE_CC2420_CREATE_CONFIG
#undef  CONFIG_MODULE_XBEE
#undef  CONFIG_MODULE_XBEE_STATS
#undef  CONFIG_MODULE_XBEE_ATCMD_HELP

/*
 * Crypto modules
 */

/*
 * Crypto modules depend on utils module
 */
#undef  CONFIG_MODULE_AES
#undef  CONFIG_MODULE_AES_CTR
#undef  CONFIG_MODULE_MD5
#undef  CONFIG_MODULE_MD5_HMAC
#undef  CONFIG_MODULE_RC4

/*
 * Encodings modules
 */

/*
 * Encoding modules depend on utils module
 */
#undef  CONFIG_MODULE_BASE64
#undef  CONFIG_MODULE_HAMMING

/*
 * Debug modules
 */

/*
 * Debug modules depend on utils module
 */
#undef  CONFIG_MODULE_DIAGNOSTIC
#undef  CONFIG_MODULE_DIAGNOSTIC_CREATE_CONFIG
#define CONFIG_MODULE_ERROR 1
#define CONFIG_MODULE_ERROR_CREATE_CONFIG 1

/*
 * Programmer options
 */
#define CONFIG_AVRDUDE 1
#undef  CONFIG_AVARICE

/*
 * Avrdude
 */
#undef  CONFIG_AVRDUDE_PROG_2232HIO
#undef  CONFIG_AVRDUDE_PROG_4232H
#undef  CONFIG_AVRDUDE_PROG_89ISP
#undef  CONFIG_AVRDUDE_PROG_ABCMINI
#undef  CONFIG_AVRDUDE_PROG_ALF
#undef  CONFIG_AVRDUDE_PROG_ARDUINO
#undef  CONFIG_AVRDUDE_PROG_ARDUINO_FT232R
#undef  CONFIG_AVRDUDE_PROG_ATISP
#undef  CONFIG_AVRDUDE_PROG_ATMELICE
#undef  CONFIG_AVRDUDE_PROG_ATMELICE_DW
#undef  CONFIG_AVRDUDE_PROG_ATMELICE_ISP
#undef  CONFIG_AVRDUDE_PROG_ATMELICE_PDI
#undef  CONFIG_AVRDUDE_PROG_AVR109
#undef  CONFIG_AVRDUDE_PROG_AVR910
#undef  CONFIG_AVRDUDE_PROG_AVR911
#undef  CONFIG_AVRDUDE_PROG_AVRFTDI
#undef  CONFIG_AVRDUDE_PROG_AVRISP
#undef  CONFIG_AVRDUDE_PROG_AVRISP2
#undef  CONFIG_AVRDUDE_PROG_AVRISPMKII
#undef  CONFIG_AVRDUDE_PROG_AVRISPV2
#undef  CONFIG_AVRDUDE_PROG_BASCOM
#undef  CONFIG_AVRDUDE_PROG_BLASTER
#undef  CONFIG_AVRDUDE_PROG_BSD
#undef  CONFIG_AVRDUDE_PROG_BUSPIRATE
#undef  CONFIG_AVRDUDE_PROG_BUSPIRATE_BB
#undef  CONFIG_AVRDUDE_PROG_BUTTERFLY
#undef  CONFIG_AVRDUDE_PROG_BUTTERFLY_MK
#undef  CONFIG_AVRDUDE_PROG_BWMEGA
#undef  CONFIG_AVRDUDE_PROG_C232HM
#undef  CONFIG_AVRDUDE_PROG_C2N232I
#undef  CONFIG_AVRDUDE_PROG_DAPA
#undef  CONFIG_AVRDUDE_PROG_DASA
#undef  CONFIG_AVRDUDE_PROG_DASA3
#undef  CONFIG_AVRDUDE_PROG_DIECIMILA
#undef  CONFIG_AVRDUDE_PROG_DRAGON_DW
#undef  CONFIG_AVRDUDE_PROG_DRAGON_HVSP
#undef  CONFIG_AVRDUDE_PROG_DRAGON_ISP
#undef  CONFIG_AVRDUDE_PROG_DRAGON_JTAG
#undef  CONFIG_AVRDUDE_PROG_DRAGON_PDI
#undef  CONFIG_AVRDUDE_PROG_DRAGON_PP
#undef  CONFIG_AVRDUDE_PROG_DT006
#undef  CONFIG_AVRDUDE_PROG_ERE_ISP_AVR
#undef  CONFIG_AVRDUDE_PROG_FLIP1
#undef  CONFIG_AVRDUDE_PROG_FLIP2
#undef  CONFIG_AVRDUDE_PROG_FRANK_STK200
#undef  CONFIG_AVRDUDE_PROG_FT232R
#undef  CONFIG_AVRDUDE_PROG_FT245R
#undef  CONFIG_AVRDUDE_PROG_FUTURELEC
#undef  CONFIG_AVRDUDE_PROG_JTAG1
#undef  CONFIG_AVRDUDE_PROG_JTAG1SLOW
#undef  CONFIG_AVRDUDE_PROG_JTAG2
#undef  CONFIG_AVRDUDE_PROG_JTAG2AVR32
#undef  CONFIG_AVRDUDE_PROG_JTAG2DW
#undef  CONFIG_AVRDUDE_PROG_JTAG2FAST
#undef  CONFIG_AVRDUDE_PROG_JTAG2ISP
#undef  CONFIG_AVRDUDE_PROG_JTAG2PDI
#undef  CONFIG_AVRDUDE_PROG_JTAG2SLOW
#undef  CONFIG_AVRDUDE_PROG_JTAG3
#undef  CONFIG_AVRDUDE_PROG_JTAG3DW
#undef  CONFIG_AVRDUDE_PROG_JTAG3ISP
#undef  CONFIG_AVRDUDE_PROG_JTAG3PDI
#undef  CONFIG_AVRDUDE_PROG_JTAGKEY
#undef  CONFIG_AVRDUDE_PROG_JTAGMKI
#undef  CONFIG_AVRDUDE_PROG_JTAGMKII
#undef  CONFIG_AVRDUDE_PROG_JTAGMKII_AVR32
#undef  CONFIG_AVRDUDE_PROG_LM3S811
#undef  CONFIG_AVRDUDE_PROG_MIB510
#undef  CONFIG_AVRDUDE_PROG_MKBUTTERFLY
#undef  CONFIG_AVRDUDE_PROG_NIBOBEE
#undef  CONFIG_AVRDUDE_PROG_O_LINK
#undef  CONFIG_AVRDUDE_PROG_OPENMOKO
#undef  CONFIG_AVRDUDE_PROG_PAVR
#undef  CONFIG_AVRDUDE_PROG_PICKIT2
#undef  CONFIG_AVRDUDE_PROG_PICOWEB
#undef  CONFIG_AVRDUDE_PROG_PONY_STK200
#undef  CONFIG_AVRDUDE_PROG_PONYSER
#undef  CONFIG_AVRDUDE_PROG_SIPROG
#undef  CONFIG_AVRDUDE_PROG_SP12
#define CONFIG_AVRDUDE_PROG_STK200 1
#undef  CONFIG_AVRDUDE_PROG_STK500
#undef  CONFIG_AVRDUDE_PROG_STK500HVSP
#undef  CONFIG_AVRDUDE_PROG_STK500PP
#undef  CONFIG_AVRDUDE_PROG_STK500V1
#undef  CONFIG_AVRDUDE_PROG_STK500V2
#undef  CONFIG_AVRDUDE_PROG_STK600
#undef  CONFIG_AVRDUDE_PROG_STK600HVSP
#undef  CONFIG_AVRDUDE_PROG_STK600PP
#undef  CONFIG_AVRDUDE_PROG_UM232H
#undef  CONFIG_AVRDUDE_PROG_USBASP
#undef  CONFIG_AVRDUDE_PROG_USBASP_CLONE
#undef  CONFIG_AVRDUDE_PROG_USBTINY
#undef  CONFIG_AVRDUDE_PROG_WIRING
#undef  CONFIG_AVRDUDE_PROG_XIL
#undef  CONFIG_AVRDUDE_PROG_XPLAINEDPRO
#define CONFIG_AVRDUDE_PORT "/dev/parport0"
#define CONFIG_AVRDUDE_BAUDRATE (19200)

/*
 * Avarice
 */
#define CONFIG_AVARICE_PORT "/dev/ttyS0"
#define CONFIG_AVARICE_DEBUG_PORT (1234)
#define CONFIG_AVARICE_PROG_MKI 1
#undef  CONFIG_AVARICE_PROG_MKII
#undef  CONFIG_AVARICE_PROG_DRAGON
#undef  CONFIG_AVRDUDE_CHECK_SIGNATURE
