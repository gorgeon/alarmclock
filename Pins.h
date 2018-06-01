

#ifndef PINS_H_
#define PINS_H_

///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Bitband and Pin Definitions
//
///////////////////////////////////////////////////////////////////////////////////////////////////




#define BITBAND(BYTE_ADDR,BIT) (((volatile uint32_t *)(   (((uint32_t)&(BYTE_ADDR))&(0xF0000000)) + 0x02000000 + (((uint32_t)&(BYTE_ADDR)) << 5) + ((BIT) << 2)    )))

#define  LED_SMRED        *( BITBAND(GPIO_PORTC_DATA_R,4) )
#define  LED_STRIPBLUE    *( BITBAND(GPIO_PORTC_DATA_R,5) )
#define  LED_SMBLUE       *( BITBAND(GPIO_PORTF_DATA_R,2) )
#define  LED_RGBRED       *( BITBAND(GPIO_PORTD_DATA_R,0) )
#define  LED_RGBBLUE      *( BITBAND(GPIO_PORTE_DATA_R,4) )
#define  LED_RGBGREEN     *( BITBAND(GPIO_PORTD_DATA_R,1) )
#define  SCR_CS           *( BITBAND(GPIO_PORTE_DATA_R,2) )
#define  SCR_RST          *( BITBAND(GPIO_PORTA_DATA_R,3) )
#define  SCR_DC           *( BITBAND(GPIO_PORTA_DATA_R,4) )
#define  ETH_CS           *( BITBAND(GPIO_PORTD_DATB_R,2) )
#define  ETH_RST          *( BITBAND(GPIO_PORTB_DATA_R,5) )



#define  BUTTON_0       *( BITBAND(GPIO_PORTD_DATA_R,6) )
#define  BUTTON_1       *( BITBAND(GPIO_PORTD_DATA_R,7) )
#define  BUTTON_2       *( BITBAND(GPIO_PORTF_DATA_R,4) )

#define BT_0 (0x01)
#define BT_1 (0x02)
#define BT_2 (0x04)
#define BT_3 (0x08)
#define BT_4 (0x10)
#define BT_5 (0x20)
#define BT_6 (0x40)
#define BT_7 (0x80)
#define BT_8 (0x0100)
#define BT_9 (0x0200)
#define BT_10 (0x0400)
#define BT_11 (0x0800)
#define BT_12 (0x1000)
#define BT_13 (0x2000)
#define BT_14 (0x4000)
#define BT_15 (0x8000)




#endif /* PINS_H_ */
