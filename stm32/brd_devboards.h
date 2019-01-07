// Copyright (C) 2016-2019 Semtech (International) AG. All rights reserved.
//
// This file is subject to the terms and conditions defined in file 'LICENSE',
// which is part of this source code package.

// to be included from board.h

// -------------------------------------------
#if defined(CFG_nucleo_board)

#define GPIO_RST	BRD_GPIO(PORT_A, 0)
#define GPIO_DIO0	BRD_GPIO(PORT_A, 10)
#define GPIO_DIO1	BRD_GPIO(PORT_B, 3)
#define GPIO_DIO2	BRD_GPIO(PORT_B, 5)

#if defined(CFG_sx1272mbed)
#define BRD_sx1272_radio
#elif defined(CFG_sx1276mbed)
#define BRD_sx1276_radio
#define GPIO_TX		BRD_GPIO(PORT_C, 1)
#else
#error "Missing radio configuration"
#endif

#define BRD_RADIO_SPI	1
#define GPIO_NSS	BRD_GPIO(PORT_B, 6)
#define GPIO_SCK	BRD_GPIO_AF(PORT_A, 5, 0)
#define GPIO_MISO	BRD_GPIO_AF(PORT_A, 6, 0)
#define GPIO_MOSI	BRD_GPIO_AF(PORT_A, 7, 0)

#define GPIO_BOOT_LED	BRD_GPIO(PORT_A, 5) // -- LED is shared with SCK!!
//#define GPIO_DBG_LED	BRD_GPIO(PORT_A, 5) // -- LED is shared with SCK!!
#define GPIO_DBG_TX	BRD_GPIO_AF(PORT_A, 2, 4)
#define BRD_DBG_UART	2

#define GPIO_PERSO_TX	BRD_GPIO_AF(PORT_A, 2, 4)
#define GPIO_PERSO_RX	BRD_GPIO_AF(PORT_A, 3, 4)
#define BRD_PERSO_UART	2

#define BRD_USART	BRD_LPUART(1)
#define GPIO_USART_TX	BRD_GPIO_AF(PORT_C, 4, 2)
#define GPIO_USART_RX	BRD_GPIO_AF(PORT_C, 5, 2)


// -------------------------------------------
#elif defined(CFG_b_l072Z_lrwan1_board)

#define GPIO_RST	BRD_GPIO(PORT_C, 0)
#define GPIO_DIO0	BRD_GPIO(PORT_B, 4)
#define GPIO_DIO1	BRD_GPIO(PORT_B, 1)
#define GPIO_DIO2	BRD_GPIO(PORT_B, 0)
#define GPIO_DIO3	BRD_GPIO(PORT_C, 13)
#define GPIO_DIO4	BRD_GPIO(PORT_A, 5)
#define GPIO_DIO5	BRD_GPIO(PORT_A, 4)

#define GPIO_TCXO_PWR	BRD_GPIO(PORT_A, 12)
#define GPIO_RX		BRD_GPIO(PORT_A, 1)
#define GPIO_TX		BRD_GPIO(PORT_C, 1)

#define GPIO_LED1	BRD_GPIO(PORT_B, 5) // grn
#define GPIO_LED2	BRD_GPIO(PORT_A, 5) // red -- used by bootloader
#define GPIO_LED3	BRD_GPIO(PORT_B, 6) // blu
#define GPIO_LED4	BRD_GPIO(PORT_B, 7) // red

// button PB2

#define BRD_sx1276_radio

#define BRD_RADIO_SPI	1
#define GPIO_NSS	BRD_GPIO(PORT_A, 15)
#define GPIO_SCK	BRD_GPIO_AF(PORT_B, 3, 0)
#define GPIO_MISO	BRD_GPIO_AF(PORT_A, 6, 0)
#define GPIO_MOSI	BRD_GPIO_AF(PORT_A, 7, 0)

#define GPIO_DBG_LED	GPIO_LED4
#define GPIO_DBG_TX	BRD_GPIO_AF(PORT_A, 2, 4)
#define BRD_DBG_UART	2

#define BRD_USART	1
#define GPIO_USART_TX	BRD_GPIO_AF(PORT_A, 9, 4)
#define GPIO_USART_RX	BRD_GPIO_AF(PORT_A, 10, 4)

#endif
