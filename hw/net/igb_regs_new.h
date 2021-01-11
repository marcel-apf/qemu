/*
 * QEMU Intel 82576 SR/IOV capable Ethernet NIC Emulation
 * Copyright(c) 2020 RedHat.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 */

#ifndef HW_IGB_REGS_H
#define HW_IGB_REGS_H

#define E1000_RDBAL0        0xC000 /* RX Descriptor Base Low Queue 0; RW */
#define E1000_RDBAL0_ALT    0x2800
#define E1000_RDBAL1        0xC040 /* RX Descriptor Base Low Queue 1; RW */
#define E1000_RDBAL1_ALT    0x2900
#define E1000_RDBAL2        0xC080 /* RX Descriptor Base Low Queue 2; RW */
#define E1000_RDBAL2_ALT    0x2A00
#define E1000_RDBAL3        0xC0C0 /* RX Descriptor Base Low Queue 3; RW */
#define E1000_RDBAL3_ALT    0x2B00
#define E1000_RDBAL4        0xC100 /* RX Descriptor Base Low Queue 4; RW */
#define E1000_RDBAL5        0xC140 /* RX Descriptor Base Low Queue 5; RW */
#define E1000_RDBAL6        0xC180 /* RX Descriptor Base Low Queue 6; RW */
#define E1000_RDBAL7        0xC1C0 /* RX Descriptor Base Low Queue 7; RW */
#define E1000_RDBAL8        0xC200 /* RX Descriptor Base Low Queue 8; RW */
#define E1000_RDBAL9        0xC240 /* RX Descriptor Base Low Queue 9; RW */
#define E1000_RDBAL10       0xC280 /* RX Descriptor Base Low Queue 10; RW */
#define E1000_RDBAL11       0xC2C0 /* RX Descriptor Base Low Queue 11; RW */
#define E1000_RDBAL12       0xC300 /* RX Descriptor Base Low Queue 12; RW */
#define E1000_RDBAL13       0xC340 /* RX Descriptor Base Low Queue 13; RW */
#define E1000_RDBAL14       0xC380 /* RX Descriptor Base Low Queue 14; RW */
#define E1000_RDBAL15       0xC3C0 /* RX Descriptor Base Low Queue 15; RW */

#define E1000_CTRL_ALT      0x0004
#define E1000_ICR_ALT       0x00C0
#define E1000_ICS_ALT       0x00C8
#define E1000_IMS_ALT       0x00D0
#define E1000_IMC_ALT       0x00D8
#define E1000_IAM_ALT       0x00E0
#define E1000_FCRTL_ALT     0x0168
#define E1000_RDBAH0_ALT    0x2804
#define E1000_RDLEN0_ALT    0x2808
#define E1000_SRRCTL0_ALT   0x280C
#define E1000_RDH0_ALT      0x2810
#define E1000_RDT0_ALT      0x2818
#define E1000_RXDCTL0_ALT   0x2828
#define E1000_RXCTL0_ALT    0x2814
#define E1000_RQDPC0_ALT    0x2830
#define E1000_RDBAH1_ALT    0x2904
#define E1000_RDBAH2_ALT    0x2A04
#define E1000_RDBAH3_ALT    0x2B04
#define E1000_RDLEN1_ALT    0x2908
#define E1000_RDLEN2_ALT    0x2A08
#define E1000_RDLEN3_ALT    0x2B08
#define E1000_SRRCTL1_ALT   0x290C
#define E1000_SRRCTL2_ALT   0x2A0C
#define E1000_SRRCTL3_ALT   0x2B0C
#define E1000_RDH1_ALT      0x2910
#define E1000_RDH2_ALT      0x2A10
#define E1000_RDH3_ALT      0x2B10
#define E1000_RDT1_ALT      0x2918
#define E1000_RDT2_ALT      0x2A18
#define E1000_RDT3_ALT      0x2B18
#define E1000_RXDCTL1_ALT   0x2928
#define E1000_RXDCTL2_ALT   0x2A28
#define E1000_RXDCTL3_ALT   0x2B28
#define E1000_RXCTL1_ALT    0x2914
#define E1000_RXCTL2_ALT    0x2A14
#define E1000_RXCTL3_ALT    0x2B14
#define E1000_RQDPC1_ALT    0x2930
#define E1000_RQDPC2_ALT    0x2A30
#define E1000_RQDPC3_ALT    0x2B30
#define E1000_MTA_ALT       0x0200
#define E1000_VFTA_ALT      0x0600
#define E1000_TDBAL0_ALT    0x3800
#define E1000_TDBAH0_ALT    0x3804
#define E1000_TDLEN0_ALT    0x3808
#define E1000_TDH0_ALT      0x3810
#define E1000_TDT0_ALT      0x3818
#define E1000_TXDCTL0_ALT   0x3828
#define E1000_TXCTL0_ALT    0x3814
#define E1000_TDWBAL0_ALT   0x3838
#define E1000_TDWBAH0_ALT   0x383C
#define E1000_TDBAL1_ALT    0x3900
#define E1000_TDBAL2_ALT    0x3A00
#define E1000_TDBAL3_ALT    0x3B00
#define E1000_TDBAH1_ALT    0x3904
#define E1000_TDBAH2_ALT    0x3A04
#define E1000_TDBAH3_ALT    0x3B04
#define E1000_TDLEN1_ALT    0x3908
#define E1000_TDLEN2_ALT    0x3A08
#define E1000_TDLEN3_ALT    0x3B08
#define E1000_TDH1_ALT      0x3910
#define E1000_TDH2_ALT      0x3A10
#define E1000_TDH3_ALT      0x3B10
#define E1000_TDT1_ALT      0x3918
#define E1000_TDT2_ALT      0x3A18
#define E1000_TDT3_ALT      0x3B18
#define E1000_TXDCTL1_ALT   0x3928
#define E1000_TXDCTL2_ALT   0x3A28
#define E1000_TXDCTL3_ALT   0x3B28
#define E1000_TXCTL1_ALT    0x3914
#define E1000_TXCTL2_ALT    0x3A14
#define E1000_TXCTL3_ALT    0x3B14
#define E1000_TDWBAL1_ALT   0x3938
#define E1000_TDWBAL2_ALT   0x3A38
#define E1000_TDWBAL3_ALT   0x3B38
#define E1000_TDWBAH1_ALT   0x393C
#define E1000_TDWBAH2_ALT   0x3A3C
#define E1000_TDWBAH3_ALT   0x3B3C

/* Interrupts */

#define IGB_INT_TXDW        0x00000001 /* Transmit Descriptor Written Back */
#define IGB_INT_LSC         0x00000004 /* Link Status Change Interrupt */
#define IGB_INT_RXDMT0      0x00000010 /* Receive Descriptor Minimum Threshold Hit */
#define IGB_INT_MACSEC      0x00000020 /* MACSec */
#define IGB_INT_RX0         0x00000040 /* Receiver Overrun */
#define IGB_INT_RXDW        0x00000080 /* Receiver Descriptor Write Back */
#define IGB_INT_VMMB        0x00000100 /* Mailbox */
#define IGB_INT_GPI_SDP0    0x00000800 /* General Purpose, SDP0 pin */
#define IGB_INT_GPI_SDP1    0x00001000 /* General Purpose, SDP1 pin */
#define IGB_INT_GPI_SDP2    0x00002000 /* General Purpose, SDP2 pin */
#define IGB_INT_GPI_SDP3    0x00004000 /* General Purpose, SDP3 pin */
#define IGB_INT_PTRAP       0x00008000 /* Probe Trap */
#define IGB_INT_MNG         0x00040000 /* Management Event */
#define IGB_INT_OMED        0x00100000 /* Other Media Energy Detected */
#define IGB_INT_FER         0x00400000 /* Fatal Error */
#define IGB_INT_NFER        0x00800000 /* Non Fatal Error */
#define IGB_INT_CSRTO       0x01000000 /* CSR access Time Out Indication */
#define IGB_INT_SCE         0x02000000 /* Storm Control Event */
#define IGB_INT_SW_WD       0x04000000 /* Software Watchdog */
#define IGB_INT_OUTSYNC     0x10000000 /* DMA Tx Out of Sync */
#define IGB_INT_TCP_TIMER   0x40000000 /* TCP Timer */
#define IGB_INT_INTA        0x80000000 /* Interrupt Asserted */

/* Extended Interrupts */

#define IGB_EINT_MSIX_MASK      0x01FFFFFF /* Bits used in MSI-X mode */
#define IGB_EINT_LEGACY_MASK    0x4000FFFF /* Bits used in non MSI-X mode */

#define IGB_EINT_RXTXQ00        0x00000001 /* Receive/Transmit Queues */
#define IGB_EINT_RXTXQ01        0x00000002
#define IGB_EINT_RXTXQ02        0x00000004
#define IGB_EINT_RXTXQ03        0x00000008
#define IGB_EINT_RXTXQ04        0x00000010
#define IGB_EINT_RXTXQ05        0x00000020
#define IGB_EINT_RXTXQ06        0x00000040
#define IGB_EINT_RXTXQ07        0x00000080
#define IGB_EINT_RXTXQ08        0x00000100
#define IGB_EINT_RXTXQ09        0x00000200
#define IGB_EINT_RXTXQ10        0x00000400
#define IGB_EINT_RXTXQ11        0x00000800
#define IGB_EINT_RXTXQ12        0x00001000
#define IGB_EINT_RXTXQ13        0x00002000
#define IGB_EINT_RXTXQ14        0x00004000
#define IGB_EINT_TCP_TIMER      0x40000000 /* TCP Timer Expired */
#define IGB_EINT_OTHER_CAUSE    0x80000000 /* Interrupt Cause Active */

/* General Purpose Interrupt Enable */

#define IGB_GPIE_NSICR          0x00000001 /* Non Selective Interrupt Clear On Read */
#define IGB_GPIE_MULTIPLE_MSIX  0x00000010
#define IGB_GPIE_EIAME          0x40000000 /* Extended Interrupt Auto Mask Enable */
#define IGB_GPIE_PBA_SUPPORT    0x80000000

/* Transmit & Receive */

#define IGB_XDBAL_MASK  (~(BIT(7) - 1)) /* TDBAL and RDBAL Registers Mask */
#define IGB_TDT_MASK    0xFFFF          /* Transmit Descriptor Tail Mask */

/* Physical Function Mailbox */

#define E1000_PFMAILBOX_STS     0x00000001 /* Status/Command from PF Ready */
#define E1000_PFMAILBOX_ACK     0x00000002 /* VF Message Received */
#define E1000_PFMAILBOX_VFU     0x00000004 /* Buffer Taken by VF */
#define E1000_PFMAILBOX_PFU     0x00000008 /* Buffer Taken by PF */
#define E1000_PFMAILBOX_RVFU    0x00000010 /* Reset VFU  */

/* Virtual Function Mailbox */

#define E1000_VFMAILBOX_REQ     0x00000001 /* Request for PF Ready */
#define E1000_VFMAILBOX_ACK     0x00000002 /* PF Message Received */
#define E1000_VFMAILBOX_VFU     0x00000004 /* Buffer Taken by VF */
#define E1000_VFMAILBOX_PFU     0x00000008 /* Buffer Taken by PF */
#define E1000_VFMAILBOX_PF_STS  0x00000010 /* PF wrote a message in the mailbox */
#define E1000_VFMAILBOX_PF_ACK  0x00000020 /* PF acknowledged the VF previous message */
#define E1000_VFMAILBOX_RSTI    0x00000040 /* PF had reset the shared resources */
#define E1000_VFMAILBOX_RSTD    0x00000080 /* PF software reset completed */

#endif // HW_IGB_REGS_H
