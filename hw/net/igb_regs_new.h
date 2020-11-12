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

#endif // HW_IGB_REGS_H
