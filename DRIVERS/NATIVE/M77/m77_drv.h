/***********************  I n c l u d e  -  F i l e  ************************
 *
 *         Name: m077_drv.h
 *      Project: M077 VxWorks driver
 *
 *       Author: ag
 *        $Date: 2006/08/22 09:15:20 $
 *    $Revision: 1.2 $
 *
 *  Description: M77 VxWorks driver definitions
 *
 *     Required: BBIS
 *     Switches:
 *
 *-------------------------------[ History ]---------------------------------
 *
 * $Log: m77_drv.h,v $
 * Revision 1.2  2006/08/22 09:15:20  cs
 * added support for M45N / M69N
 *
 * Revision 1.1  2002/07/18 12:05:12  agromann
 * Initial Revision
 *
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2002 by MEN mikro elektronik GmbH, Nuernberg, Germany
 ****************************************************************************/

#ifndef _M077_DRV_H_
#define _M077_DRV_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE


/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/

    /* offsets for main register */
#define M77_HOLD_DLL_REG        0x00
#define M77_IER_DLM_REG         0x02
#define M77_ISR_FCR_REG         0x04
#define M77_LCR_REG             0x06
#define M77_MCR_REG             0x08
#define M77_LSR_REG             0x0a
#define M77_MSR_REG             0x0c
#define M77_SPR_REG             0x0e

    /* offset for 650 compatible registers */
#define M77_EFR_OFFSET			0x04	/* enhanced features register */
#define M77_XON1_OFFSET			0x08	/* XON1 flow control character */
#define M77_XON2_OFFSET			0x0a	/* XON2 flow control character */
#define M77_XOFF1_OFFSET		0x0c	/* XOFF1 flow control character */
#define M77_XOFF2_OFFSET		0x0e    /* XOFF2 flow control character */
#define M77_LCR_650_ACCESS_KEY  0xbf    /* access key for 650 compatible registers */

	/* equates for extended feature register */
#define M77_EFR_FLOWCTL_CTS		0x80	/* automatic CTS Flow control enable */
#define M77_EFR_FLOWCTL_RTS		0x40	/* automatic RTS Flow control enable */
#define M77_EFR_SPEC_CHAR_DET	0x20	/* special character detect enable */
#define M77_EFR_ENHANCED_MODE	0x10	/* Enhanced mode enable */

    /* offsets for 950 specific registers */
#define M77_ASR_OFFSET			0x02		/* additional status register */
#define M77_RFL_OFFSET			0x06		/* receiver FIFO fill level */
#define M77_TFL_OFFSET			0x08		/* transmitter FIFO fill level */
#define M77_ICR_OFFSET			0x0a		/* indexed control register */
#define M77_ACR_OFFSET			0x00		/* index of ARC register */
#define M77_CPR_OFFSET   		0x01		/* clock prescaler register */
#define M77_TCR_OFFSET   		0x02		/* times clock register */
#define M77_CKS_OFFSET   		0x03		/* clock select register */
#define M77_TTL_OFFSET   		0x04		/* transmitter int trigger level */
#define M77_RTL_OFFSET   		0x05		/* receiver int trigger level */
#define M77_FCL_OFFSET   		0x06		/* automatic flow control lower trigger level */
#define M77_FCH_OFFSET   		0x07		/* automatic flow control higher trigger level */
#define M77_CSR_INDEX			0x0c		/* index of CSR register */
#define M77_ID1_OFFSET   		0x08		/* hardwired ID byte 1 */
#define M77_ID2_OFFSET   		0x09		/* hardwired ID byte 2 */
#define M77_ID3_OFFSET   		0x0a		/* hardwired ID byte 3 */
#define M77_REV_OFFSET   		0x0b		/* hardwired revision byte */

    /* equates for additional control register */
#define M77_ACR_RX_DIS				0x01		/* enable receiver */
#define M77_ACR_TX_DIS				0x02		/* enable transmitter */
#define M77_ACR_AUTO_DSR_EN			0x04		/* enable auto DSR flow control */
#define M77_ACR_DTR_MASK			0x18		/* mask bits setting function of DTR pin */
#define M77_ACR_DTR_NORM			0x00     	/* DTR: compatible to 16C450, C550, C650, C750 */
#define M77_ACR_DTR_RX_CTL			0x08    	/* DTR: automatic flow control */
#define M77_ACR_DTR_HD_L_DRIVER_CTL	0x10    	/* DTR: activate line driver control in HD-mode,
														low active EN pin */
#define M77_ACR_DTR_HD_H_DRIVER_CTL	0x18    	/* DTR: activate line driver control in HD-mode,
														high active EN pin */
#define M77_ACR_950_TRIG_EN			0x20		/* enable 950 interrupt trigger level */
#define M77_ACR_ICR_READ_EN			0x40		/* enable read of ICR register */
#define M77_ACR_950_READ_EN			0x80		/* enable read of 950 registers */



    /* equates for FIFO Control Register */
#define FCR_EN                  0x01		/* enable xmit and rcvr */
#define FIFO_ENABLE             FCR_EN
#define FCR_RXCLR               0x02		/* clears rcvr fifo */
#define RxCLEAR                 FCR_RXCLR
#define FCR_TXCLR               0x04		/* clears xmit fifo */
#define TxCLEAR                 FCR_TXCLR
#define FCR_DMA                 0x08		/* dma */
#define FCR_RXTRIG_L            0x40		/* rcvr fifo trigger lvl low */
#define FCR_RXTRIG_H            0x80		/* rcvr fifo trigger lvl high */
#define FCR_DMA_MODE            0x08 		/* Set to use 550 Tx Trigger Levels */

#define FCR_RX_TRIGGER_16		0x00	/* receiver FIFO trigger level = 16 Byte */
#define FCR_RX_TRIGGER_32		0x40	/* receiver FIFO trigger level = 32 Byte */
#define FCR_RX_TRIGGER_112		0x80	/* receiver FIFO trigger level = 112 Byte */
#define FCR_RX_TRIGGER_120		0xC0	/* receiver FIFO trigger level = 120 Byte */
#define FCR_TX_TRIGGER_16		0x00	/* transmitter FIFO trigger level = 16 Byte */
#define FCR_TX_TRIGGER_32		0x10	/* transmitter FIFO trigger level = 32 Byte */
#define FCR_TX_TRIGGER_64		0x20	/* transmitter FIFO trigger level = 64 Byte */
#define FCR_TX_TRIGGER_112		0x30	/* transmitter FIFO trigger level = 112 Byte */

#define M77_FIFO_ENABLE         0x01
#define M77_FIFO_DISABLE        0x00


    /* equates for Line Control Register */
#define M77_LCR_CS5				0x00		/* 5 bits data size */
#define M77_LCR_CS6				0x01		/* 6 bits data size */
#define M77_LCR_CS7				0x02		/* 7 bits data size */
#define M77_LCR_CS8				0x03		/* 8 bits data size */
#define M77_LCR_1_STB			0x00
#define M77_LCR_2_STB			0x04
#define M77_LCR_PEN				0x08		/* parity enable */
#define M77_LCR_PDIS			0x00		/* parity disable */
#define M77_LCR_EPS				0x10		/* even parity slect */
#define M77_LCR_SP				0x20		/* stick parity select */
#define M77_LCR_SBRK			0x40		/* break control bit */
#define M77_LCR_DLAB			0x80		/* divisor latch access enable */
#define M77_LCR_DL_ACCESS_KEY	0x80

    /* equates for interrupt enable register */
#define M77_IER_RXRDY			0x01		/* receiver data ready */
#define M77_IER_TBE				0x02		/* transmit bit enable */
#define M77_IER_LSI				0x04		/* line status interrupts */
#define M77_IER_MSI				0x08		/* modem status interrupts */

    /* equates for interrupt status register */
#define M77_ISR_MASK			0x2f     /* ISR mask to cut of INT level 6 */
#define M77_ISR_NO_INT_PEND		0x01     /* ISR value if no int is pending */
#define M77_ISR_RX_ERROR		0x06     /* ISR value for int lavel 1 RX Error */
#define M77_ISR_RX_DATA_AVAIL	0x04     /* ISR value for int level 2a RX data avail */
#define M77_ISR_RX_TIMEOUT		0x0c     /* ISR value for int level 2b RX timeout */
#define M77_ISR_THR_EMPTY		0x02     /* ISR value for int level 3 TX queue empty */
#define M77_ISR_MODEM_STATE		0x00     /* ISR value for int level 4 TX queue empty */
#define M77_ISR_RTSCTS_STATE	0x20     /* ISR value for int level 6 RTS or CTS change of state */

    /* equates for line status register */
#define M77_LSR_RXRDY			0x01     /* bit mask for RxRDY in LSR reg */

    /* equates for modem control register */
#define M77_MCR_DTR				0x01     /* mask to access DTR bit */
#define M77_MCR_RTS				0x02     /* mask to access RTS bit */
#define M77_MCR_INT_EN			0x08     /* external interrupt enable */
#define M77_MCR_LOOP			0x10     /* internal loopback enable */
#define M77_MCR_NORM_RTS_CTS_EN 0x20     /* normal mode: enable RTS and CTS flow control */
#define M77_MCR_ENH_XONALL_EN   0x20     /* enhanced mode: enable XON-Any flow control */

    /* equates for modem status register */
#define M77_MSR_DCTS			0x01     /* CTS changed */
#define M77_MSR_DDSR			0x02     /* DSR changed */
#define M77_MSR_TRI 			0x04     /* trailing Ring Indicator edge */
#define M77_MSR_DDCD			0x08     /* DCD changed */
#define M77_MSR_CTS				0x10     /* CTS status */
#define M77_MSR_DSR				0x20     /* DSR status */
#define M77_MSR_RI 				0x40     /* Ring Indicator status */
#define M77_MSR_DCD				0x80     /* DCD status */


	/* Size of complete register block (offset of second controller on M45N) */
#define M45N_CTRL_REG_BLOCK_SIZE 0x80

    /* offset for M77 HW line driver configuration register & ISR register */
#define M77_DCR_REG_BASE		0x40		/* offset of Driver Configuration Register */
#define M77_IRQ_REG				0x48
#define M45N_TCR_REG			0x40 		/* offset of Tristate Contol Register 1*/
#define M45N_TCR1_REG			0x40 		/* offset of Tristate Contol Register 1*/
#define M45N_TCR2_REG			(0x40 + M45N_CTRL_REG_BLOCK_SIZE)

    /* equates for M77 configuration/interrupt register */
#define M77_IRQ_CLEAR			0x01
#define M77_IRQ_EN				0x02
#define M77_TX_EN				0x04

    /* equates for M45N tristate configuration register */
#define M45N_TCR1_CHAN0			0x01		/* channel 0 is tristate */
#define M45N_TCR1_CHAN1			0x02		/* channel 1 is tristate */
#define M45N_TCR1_CHAN23		0x04		/* channels 2 and 3 are tristate */
#define M45N_TCR2_CHAN45		0x01		/* channels 4 and 5 are tristate */
#define M45N_TCR2_CHAN67		0x02		/* channels 6 and 7 are tristate */

/* ioctl function */

#define M77_TX_FIFO_LEVEL		0x102
#define M77_RX_FIFO_LEVEL		0x103
#define M77_NO_FIFO				0x104
#define M77_ECHO_SUPPRESS		0x105
#define M77_PHYS_INT_SET		0x106
#define M77_GET_LAST_ERR		0x107
#define M77_REG_DUMP            0x109
#define M77_HS_HIGH_FIFO_LEVEL	0x110
#define M77_HS_LOW_FIFO_LEVEL	0x111
#define M77_MODEM_AUTO_RTS 		0x112
#define M77_MODEM_AUTO_CTS 		0x113
#define M77_MODEM_AUTO_DSR		0x114
#define M77_MODEM_AUTO_DTR		0x115
#define M77_MODEM_XONXOFF		0x116
#define M77_TRISTATE			0x117

/* ioctl arguments */
#define M77_RS423              0x00     /* M77_PHYS_INT_SET arg for RS423 */
#define M77_RS422_HD           0x01     /* M77_PHYS_INT_SET arg for RS422 half duplex */
#define M77_RS422_FD           0x02     /* M77_PHYS_INT_SET arg for RS422 full duplex */
#define M77_RS485_HD           0x03     /* M77_PHYS_INT_SET arg for RS485 half duplex */
#define M77_RS485_FD           0x04     /* M77_PHYS_INT_SET arg for RS485 full duplex */
#define M77_RS232              0x07     /* M77_PHYS_INT_SET arg for RS232 */

#define M77_RX_EN              0x08     /* RX_EN bit mask */

/* automatic Xon Xoff flow control values */
#define M77_XON_CHAR			17		/* Xon character = ^Q */
#define M77_XOFF_CHAR			19		/* Xoff character = ^S */

/* Flag flow control */
#define M77_MODEM_HS_NONE		0x00
#define M77_MODEM_HS_AUTO_RTS	0x01
#define M77_MODEM_HS_AUTO_CTS	0x02
#define M77_MODEM_HS_AUTO_DSR	0x04
#define M77_MODEM_HS_AUTO_DTR	0x08
#define M77_MODEM_HS_XONXOFF	0x10

/* default handshake FIFO levels */
#define M77_DEF_FCL				0x40
#define M77_DEF_FCH				0x64

#endif	/* _ASMLANGUAGE */
#ifdef __cplusplus
}
#endif

#endif /* _M077_DRV_H_ */
