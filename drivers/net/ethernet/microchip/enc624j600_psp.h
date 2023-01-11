/* SPDX-License-Identifier: GPL-2.0 */
/*
 * enc624j600_psp.h: Register definitions for 16-Bit PSP mode
 *
 */

#ifndef _ENC624J600_PSP_H
#define _ENC624J600_PSP_H

/* Command mapping SPI -> PSP */
#define SETETHRST	0xCA	/* System Reset */
#define FCDISABLE	0xE0	/* Flow Control Disable */
#define FCSINGLE	0xE2	/* Flow Control Single */
#define FCMULTIPLE	0xE4	/* Flow Control Multiple */
#define FCCLEAR		0xE6	/* Flow Control Clear */
#define SETPKTDEC	0xCC	/* Decrement Packet Counter */
#define DMASTOP		0xD2	/* DMA Stop */
#define DMACKSUM	0xD8	/* DMA Start Checksum */
#define DMACKSUMS	0xDA	/* DMA Start Checksum with Seed */
#define DMACOPY		0xDC	/* DMA Start Copy */
#define DMACOPYS	0xDE	/* DMA Start Copy and Checksum with Seed */
#define SETTXRTS	0xD4	/* Request Packet Transmission */
#define ENABLERX	0xE8	/* Enable RX */
#define DISABLERX	0xEA	/* Disable RX */
#define SETEIE		0xEC	/* Enable Interrupts */
#define CLREIE		0xEE	/* Disable Interrupts */

/* 16-Bit PSP mapping (SFR) */
#define SFR_REG_BASE	0x7E00
#define SFR_REG_COUNT	0xA0

#define ETXST		0x00
#define ETXLEN		0x02
#define ERXST		0x04
#define ERXTAIL		0x06
#define ERXHEAD		0x08
#define EDMAST		0x0A
#define EDMALEN		0x0C
#define	EDMADST		0x0E
#define EDMACS		0x10
#define ETXSTAT		0x12
#define ETXWIRE		0x14
#define EUDAST		0x16
#define EUDAND		0x18
#define ESTAT		0x1A
#define EIR		0x1C
#define ECON1		0x1E
#define EHT1		0x20
#define EHT2		0x22
#define EHT3		0x24
#define EHT4		0x26
#define EPMM1		0x28
#define EPMM2		0x2A
#define EPMM3		0x2C
#define EPMM4		0x2E
#define EPMCS		0x30
#define EPMO		0x32
#define ERXFCON		0x34
#define MACON1		0x40
#define MACON2		0x42
#define MABBIPG		0x44
#define MAIPG		0x46
#define MACLCON		0x48
#define MAMXFL		0x4A
#define	MICMD		0x52
#define MIREGADR	0x54
#define MAADR3		0x60
#define MAADR2		0x62
#define MAADR1		0x64
#define MIWR		0x66
#define MIRD		0x68
#define MISTAT		0x6A
#define EPAUS		0x6C
#define ECON2		0x6E
#define ERXWM		0x70
#define EIE		0x72
#define EIDLED		0x74
#define EGPDATA		0x80
#define ERXDATA		0x82
#define EUDADATA	0x84
#define EGPRDPT		0x86
#define EGPWRPT		0x88
#define ERXRDPT		0x8A
#define ERXWRPT		0x8C
#define EUDARDPT	0x8E
#define EUDAWRPT	0x90

#define SFR_SET_OFFSET	0x0100
#define SFR_CLR_OFFSET	0x0180

/* Register bit definitions */
/* ESTAT */
#define INT		BIT(15)
#define FCIDLE		BIT(14)
#define RXBUSY		BIT(13)
#define CLKRDY		BIT(12)
#define PHYDPX		BIT(10)
#define PHYLNK		BIT(8)

/* EIR */
#define CRYPTEN		BIT(15)
#define MODEXIF		BIT(14)
#define HASHIF		BIT(13)
#define AESIF		BIT(12)
#define LINKIF		BIT(11)
#define PKTIF		BIT(6)
#define DMAIF		BIT(5)
#define TXIF		BIT(3)
#define TXABTIF		BIT(2)
#define RXABTIF		BIT(1)
#define PCFULIF		BIT(0)

/* ECON1 */
#define MODEXST		BIT(15)
#define HASHEN		BIT(14)
#define HASHOP		BIT(13)
#define HASHLST		BIT(12)
#define AESST		BIT(11)
#define AESOP1		BIT(10)
#define AESOP0		BIT(9)
#define PKTDEC		BIT(8)
#define FCOP1		BIT(7)
#define FCOP0		BIT(6)
#define DMAST		BIT(5)
#define DMACPY		BIT(4)
#define DMACSSD		BIT(3)
#define DMANOCS		BIT(2)
#define TXRTS		BIT(1)
#define RXEN		BIT(0)

/* ETXSTAT */
#define LATECOL		BIT(10)
#define MAXCOL		BIT(9)
#define EXDEFER		BIT(8)
#define ETXSTATL_DEFER	BIT(7)
#define CRCBAD		BIT(4)
#define COLCNT_MASK	0xF

/* ERXFCON */
#define HTEN		BIT(15)
#define MPEN		BIT(14)
#define NOTPM		BIT(12)
#define PMEN3		BIT(11)
#define PMEN2		BIT(10)
#define PMEN1		BIT(9)
#define PMEN0		BIT(8)
#define CRCEEN		BIT(7)
#define CRCEN		BIT(6)
#define RUNTEEN		BIT(5)
#define RUNTEN		BIT(4)
#define UCEN		BIT(3)
#define NOTMEEN		BIT(2)
#define MCEN		BIT(1)
#define BCEN		BIT(0)

/* MACON1 */
#define LOOPBK		BIT(4)
#define RXPAUS		BIT(2)
#define PASSALL		BIT(1)

/* MACON2 */
#define MACON2_DEFER	BIT(14)
#define BPEN		BIT(13)
#define NOBKOFF		BIT(12)
#define PADCFG2		BIT(7)
#define PADCFG1		BIT(6)
#define PADCFG0		BIT(5)
#define TXCRCEN		BIT(4)
#define PHDREN		BIT(3)
#define HFRMEN		BIT(2)
#define MACON2_RSV1	BIT(1)
#define FULDPX		BIT(0)

/* MAIPG */
/* value of the high byte is given by the reserved bits,
 * value of the low byte is recomended setting of the
 * IPG parameter.
 */
#define MAIPGH_VAL	0x0C
#define MAIPGL_VAL	0x12

/* MIREGADRH */
#define MIREGADR_VAL	BIT(8)

/* MIREGADRL */
#define PHREG_MASK	0x1F

/* MICMD */
#define MIISCAN		BIT(1)
#define MIIRD		BIT(0)

/* MISTAT */
#define NVALID		BIT(2)
#define SCAN		BIT(1)
#define BUSY		BIT(0)

/* ECON2 */
#define ETHEN		BIT(15)
#define STRCH		BIT(14)
#define TXMAC		BIT(13)
#define SHA1MD5		BIT(12)
#define COCON3		BIT(11)
#define COCON2		BIT(10)
#define COCON1		BIT(9)
#define COCON0		BIT(8)
#define AUTOFC		BIT(7)
#define TXRST		BIT(6)
#define RXRST		BIT(5)
#define ETHRST		BIT(4)
#define MODLEN1		BIT(3)
#define MODLEN0		BIT(2)
#define AESLEN1		BIT(1)
#define AESLEN0		BIT(0)

/* EIE */
#define INTIE		BIT(15)
#define MODEXIE		BIT(14)
#define HASHIE		BIT(13)
#define AESIE		BIT(12)
#define LINKIE		BIT(11)
#define PKTIE		BIT(6)
#define DMAIE		BIT(5)
#define TXIE		BIT(3)
#define TXABTIE		BIT(2)
#define RXABTIE		BIT(1)
#define PCFULIE		BIT(0)

/* EIDLED */
#define LACFG3		BIT(15)
#define LACFG2		BIT(14)
#define LACFG1		BIT(13)
#define LACFG0		BIT(12)
#define LBCFG3		BIT(11)
#define LBCFG2		BIT(10)
#define LBCFG1		BIT(9)
#define LBCFG0		BIT(8)
#define DEVID_SHIFT	5
#define DEVID_MASK	(0x7 << DEVID_SHIFT)
#define REVID_SHIFT	0
#define REVID_MASK	(0x1F << REVID_SHIFT)

/* PHY registers */
#define PHCON1		0x00
#define PHSTAT1		0x01
#define PHANA		0x04
#define PHANLPA		0x05
#define PHANE		0x06
#define PHCON2		0x11
#define PHSTAT2		0x1B
#define PHSTAT3		0x1F

/* PHCON1 */
#define PRST		BIT(15)
#define PLOOPBK		BIT(14)
#define SPD100		BIT(13)
#define ANEN		BIT(12)
#define PSLEEP		BIT(11)
#define RENEG		BIT(9)
#define PFULDPX		BIT(8)

/* PHSTAT1 */
#define FULL100		BIT(14)
#define HALF100		BIT(13)
#define FULL10		BIT(12)
#define HALF10		BIT(11)
#define ANDONE		BIT(5)
#define LRFAULT		BIT(4)
#define ANABLE		BIT(3)
#define LLSTAT		BIT(2)
#define EXTREGS		BIT(0)

/* PHSTAT2 */
#define PLRITY		BIT(4)

/* PHSTAT3 */
#define PHY3SPD100	BIT(3)
#define PHY3DPX		BIT(4)
#define SPDDPX_SHIFT	2
#define SPDDPX_MASK	(0x7 << SPDDPX_SHIFT)

/* PHANA */
/* Default value for PHY initialization*/
#define PHANA_DEFAULT	0x05E1

/* PHANE */
#define PDFLT		BIT(4)
#define LPARCD		BIT(1)
#define LPANABL		BIT(0)

#define EUDAST_TEST_VAL 0x1234

#define TSV_SIZE	7

#define ENCX24J600_DEV_ID 0x1

/* Configuration */

/* Led is on when the link is present and driven low
 * temporarily when packet is TX'd or RX'd
 */
#define LED_A_SETTINGS	0xC

/* Led is on if the link is in 100 Mbps mode */
#define LED_B_SETTINGS	0x8

/* maximum ethernet frame length
 * Currently not used as a limit anywhere
 * (we're using the "huge frame enable" feature of
 * enc424j600).
 */
#define MAX_FRAMELEN	1518

/* Size in bytes of the receive buffer in enc424j600.
 * Must be word aligned (even).
 */
#define RX_BUFFER_SIZE	(15 * MAX_FRAMELEN)

/* Start of the general purpose area in sram */
#define SRAM_GP_START	0x0

/* SRAM size */
#define SRAM_SIZE	0x6000

/* Start of the receive buffer */
#define ERXST_VAL (SRAM_SIZE - RX_BUFFER_SIZE)

#define RSV_RXLONGEVDROPEV	16
#define RSV_CARRIEREV		18
#define RSV_CRCERROR		20
#define RSV_LENCHECKERR		21
#define RSV_LENOUTOFRANGE	22
#define RSV_RXOK		23
#define RSV_RXMULTICAST		24
#define RSV_RXBROADCAST		25
#define RSV_DRIBBLENIBBLE	26
#define RSV_RXCONTROLFRAME	27
#define RSV_RXPAUSEFRAME	28
#define RSV_RXUNKNOWNOPCODE	29
#define RSV_RXTYPEVLAN		30

#define RSV_RUNTFILTERMATCH	31
#define RSV_NOTMEFILTERMATCH	32
#define RSV_HASHFILTERMATCH	33
#define RSV_MAGICPKTFILTERMATCH	34
#define RSV_PTRNMTCHFILTERMATCH	35
#define RSV_UNICASTFILTERMATCH	36

#define RSV_SIZE		8
#define RSV_BITMASK(x)		(1 << ((x) - 16))
#define RSV_GETBIT(x, y)	(((x) & RSV_BITMASK(y)) ? 1 : 0)

struct rsv {
	u16 next_packet;
	u16 len;
	u32 rxstat;
};

/* Put RX buffer at 0 as suggested by the Errata datasheet */

#define RXSTART_INIT		ERXST_VAL
#define RXEND_INIT		0x5FFF

#endif
