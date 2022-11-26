/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_SMSCPHY_H__
#define __LINUX_SMSCPHY_H__

#define MII_LAN83C185_ISF 29 /* Interrupt Source Flags */
#define MII_LAN83C185_IM  30 /* Interrupt Mask */
#define MII_LAN83C185_CTRL_STATUS 17 /* Mode/Status Register */
#define MII_LAN83C185_SPECIAL_MODES 18 /* Special Modes Register */

#define MII_LAN83C185_ISF_INT1 (1<<1) /* Auto-Negotiation Page Received */
#define MII_LAN83C185_ISF_INT2 (1<<2) /* Parallel Detection Fault */
#define MII_LAN83C185_ISF_INT3 (1<<3) /* Auto-Negotiation LP Ack */
#define MII_LAN83C185_ISF_INT4 (1<<4) /* Link Down */
#define MII_LAN83C185_ISF_INT5 (1<<5) /* Remote Fault Detected */
#define MII_LAN83C185_ISF_INT6 (1<<6) /* Auto-Negotiation complete */
#define MII_LAN83C185_ISF_INT7 (1<<7) /* ENERGYON */
#define MII_LAN83C185_ISF_INT8 (1<<8) /* Wake on LAN */

#define MII_LAN83C185_ISF_INT_ALL (0x0e)

#define MII_LAN83C185_ISF_INT_PHYLIB_EVENTS \
	(MII_LAN83C185_ISF_INT6 | MII_LAN83C185_ISF_INT4 | \
	 MII_LAN83C185_ISF_INT7)

#define MII_LAN83C185_EDPWRDOWN (1 << 13) /* EDPWRDOWN */
#define MII_LAN83C185_ENERGYON  (1 << 1)  /* ENERGYON */

#define MII_LAN83C185_MODE_MASK      0xE0
#define MII_LAN83C185_MODE_POWERDOWN 0xC0 /* Power Down mode */
#define MII_LAN83C185_MODE_ALL       0xE0 /* All capable mode */

/* MMD 3 Registers */
#define	LAN8742_MMD3_WAKEUP_CTRL	(32784)
#define	LAN8742_MMD3_WUCSR_LED2_AS_NPME BIT(12)
#define	LAN8742_MMD3_WUCSR_WOL		BIT(8)
#define	LAN8742_MMD3_WUCSR_PFDA_FR	BIT(7)
#define	LAN8742_MMD3_WUCSR_WUFR		BIT(6)
#define	LAN8742_MMD3_WUCSR_MPR		BIT(5)
#define	LAN8742_MMD3_WUCSR_BCAST_FR	BIT(4)
#define	LAN8742_MMD3_WUCSR_MPEN		BIT(1)

#define	LAN8742_MMD3_WAKEUP_FILTER	(32785)
#define	LAN8742_MMD3_WUF_CFGA_FE	BIT(15)
#define	LAN8742_MMD3_WUF_CFGA_AME	BIT(10)

#define	LAN8742_MMD3_MAC_ADDRA		(32865)
#define	LAN8742_MMD3_MAC_ADDRB		(32866)
#define	LAN8742_MMD3_MAC_ADDRC		(32867)
#define	LAN8742_MMD3_PME_ASSERT_DELAY	(32868)

#endif /* __LINUX_SMSCPHY_H__ */
