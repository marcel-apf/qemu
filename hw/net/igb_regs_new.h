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

/* Receive Registers */

#if 0
#define E1000_RCTL          0x0100 /* RX Control; RW */
#define E1000_FCRTL         0x2160 /* Flow Control Receive Threshold Low; RW */
#define E1000_FCRTL_ALT     0x0168
#define E1000_FCRTH         0x2168 /* Flow Control Receive Threshold Low; RW */
#define E1000_PBDIAG        0x2458 /* PB Diagnostic; RW */
#define E1000_RXPBS         0x2404 /* RX Packet Buffer Size; RW */
#define E1000_PBRWAC        0x24E8 /* RX Packet Buffer Wrap Around Counter; RO */
#define E1000_FCRTV         0x2460 /* Flow Control Refresh Timer Value; RW */
#define E1000_DRXMXOD       0x2540 /* DMA RX Max Total Allow Size Requests; RW */
#endif
#define E1000_RDBAL0        0xC000 /* RX Descriptor Base Low Queue 0; RW */
#define E1000_RDBAL0_ALT    0x2800
#define E1000_RDBAH0        0xC004 /* RX Descriptor Base High Queue 0; RW */
#define E1000_RDBAH0_ALT    0x2804
#define E1000_RDLEN0        0xC008 /* RX Descriptor Ring Length Queue 0; RW */
#define E1000_RDLEN0_ALT    0x2808
#define E1000_SRRCTL0       0xC00C /* Split and Replication Receive Control Queue 0; RW */
#define E1000_SRRCTL0_ALT   0x280C
#define E1000_RDH0          0xC010 /* RX Descriptor Head Queue 0; RW */
#define E1000_RDH0_ALT      0x2810
#define E1000_RDT0          0xC018 /* RX Descriptor Tail Queue 0; RW */
#define E1000_RDT0_ALT      0x2818
#define E1000_RXDCTL0       0xC028 /* RX Descriptor Control Queue 0; RW */
#define E1000_RXDCTL0_ALT   0x2828
#define E1000_RXCTL0        0xC014 /* RX DCA Control Queue 0; RW */
#define E1000_RXCTL0_ALT    0x2814
#define E1000_RQDPC0        0xC030 /* RX Queue Drop Packet Count Queue 0; RC */
#define E1000_RQDPC0_ALT    0x2830
#define E1000_RDBAL1        0xC040 /* RX Descriptor Base Low Queue 1; RW */
#define E1000_RDBAL1_ALT    0x2900
#define E1000_RDBAL2        0xC080 /* RX Descriptor Base Low Queue 2; RW */
#define E1000_RDBAL2_ALT    0x2A00
#define E1000_RDBAL3        0xC0C0 /* RX Descriptor Base Low Queue 3; RW */
#define E1000_RDBAL3_ALT    0x2B00
#define E1000_RDBAH1        0xC044 /* RX Descriptor Base High Queue 1; RW */
#define E1000_RDBAH1_ALT    0x2904
#define E1000_RDBAH2        0xC084 /* RX Descriptor Base High Queue 2; RW */
#define E1000_RDBAH2_ALT    0x2A04
#define E1000_RDBAH3        0xC0C4 /* RX Descriptor Base High Queue 3; RW */
#define E1000_RDBAH3_ALT    0x2B04
#define E1000_RDLEN1        0xC048 /* RX Descriptor Ring Length Queue 1; RW */
#define E1000_RDLEN1_ALT    0x2908
#define E1000_RDLEN2        0xC088 /* RX Descriptor Ring Length Queue 2; RW */
#define E1000_RDLEN2_ALT    0x2A08
#define E1000_RDLEN3        0xC0C8 /* RX Descriptor Ring Length Queue 3; RW */
#define E1000_RDLEN3_ALT    0x2B08
#define E1000_SRRCTL1       0xC04C /* Split and Replication Receive Control Queue 1; RW */
#define E1000_SRRCTL1_ALT   0x290C
#define E1000_SRRCTL2       0xC08C /* Split and Replication Receive Control 2; RW */
#define E1000_SRRCTL2_ALT   0x2A0C
#define E1000_SRRCTL3       0xC0CC /* Split and Replication Receive Control 3; RW */
#define E1000_SRRCTL3_ALT   0x2B0C
#define E1000_RDH1          0xC050 /* RX Descriptor Head Queue 1; RW */
#define E1000_RDH1_ALT      0x2910
#define E1000_RDH2          0xC090 /* RX Descriptor Head Queue 2; RW */
#define E1000_RDH2_ALT      0x2A10
#define E1000_RDH3          0xC0D0 /* RX Descriptor Head Queue 3; RW */
#define E1000_RDH3_ALT      0x2B10
#define E1000_RDT1          0xC058 /* RX Descriptor Tail Queue 1; RW */
#define E1000_RDT1_ALT      0x2918
#define E1000_RDT2          0xC098 /* RX Descriptor Tail Queue 2; RW */
#define E1000_RDT2_ALT      0x2A18
#define E1000_RDT3          0xC0D8 /* RX Descriptor Tail Queue 3; RW */
#define E1000_RDT3_ALT      0x2B18
#define E1000_RXDCTL1       0xC068 /* RX Descriptor Control Queue 1; RW */
#define E1000_RXDCTL1_ALT   0x2928
#define E1000_RXDCTL2       0xC0A8 /* RX Descriptor Control Queue 2; RW */
#define E1000_RXDCTL2_ALT   0x2A28
#define E1000_RXDCTL3       0xC0E8 /* RX Descriptor Control Queue 3; RW */
#define E1000_RXDCTL3_ALT   0x2B28
#define E1000_RXCTL1        0xC054 /* RX DCA Control Queue 1; RW */
#define E1000_RXCTL1_ALT    0x2914
#define E1000_RXCTL2        0xC094 /* RX DCA Control Queue 2; RW */
#define E1000_RXCTL2_ALT    0x2A14
#define E1000_RXCTL3        0xC0D4 /* RX DCA Control Queue 3; RW */
#define E1000_RXCTL3_ALT    0x2B14
#define E1000_RQDPC1        0xC070 /* RX Drop Packet Count Queue 1; RC */
#define E1000_RQDPC1_ALT    0x2930
#define E1000_RQDPC2        0xC0B0 /* RX Drop Packet Count Queue 2; RC */
#define E1000_RQDPC2_ALT    0x2A30
#define E1000_RQDPC3        0xC0F0 /* RX Drop Packet Count Queue 3; RC */
#define E1000_RQDPC3_ALT    0x2B30
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
#define E1000_RDBAH4        0xC104 /* RX Descriptor Base High Queue 4; RW */
#define E1000_RDBAH5        0xC144 /* RX Descriptor Base High Queue 5; RW */
#define E1000_RDBAH6        0xC184 /* RX Descriptor Base High Queue 6; RW */
#define E1000_RDBAH7        0xC1C4 /* RX Descriptor Base High Queue 7; RW */
#define E1000_RDBAH8        0xC204 /* RX Descriptor Base High Queue 8; RW */
#define E1000_RDBAH9        0xC244 /* RX Descriptor Base High Queue 9; RW */
#define E1000_RDBAH10       0xC284 /* RX Descriptor Base High Queue 10; RW */
#define E1000_RDBAH11       0xC2C4 /* RX Descriptor Base High Queue 11; RW */
#define E1000_RDBAH12       0xC304 /* RX Descriptor Base High Queue 12; RW */
#define E1000_RDBAH13       0xC344 /* RX Descriptor Base High Queue 13; RW */
#define E1000_RDBAH14       0xC384 /* RX Descriptor Base High Queue 14; RW */
#define E1000_RDBAH15       0xC3C4 /* RX Descriptor Base High Queue 15; RW */
#define E1000_RDLEN4        0xC108 /* RX Descriptor Ring Length Queue 4; RW */
#define E1000_RDLEN5        0xC148 /* RX Descriptor Ring Length Queue 5; RW */
#define E1000_RDLEN6        0xC188 /* RX Descriptor Ring Length Queue 6; RW */
#define E1000_RDLEN7        0xC1C8 /* RX Descriptor Ring Length Queue 7; RW */
#define E1000_RDLEN8        0xC208 /* RX Descriptor Ring Length Queue 8; RW */
#define E1000_RDLEN9        0xC248 /* RX Descriptor Ring Length Queue 9; RW */
#define E1000_RDLEN10       0xC288 /* RX Descriptor Ring Length Queue 10; RW */
#define E1000_RDLEN11       0xC2C8 /* RX Descriptor Ring Length Queue 11; RW */
#define E1000_RDLEN12       0xC308 /* RX Descriptor Ring Length Queue 12; RW */
#define E1000_RDLEN13       0xC348 /* RX Descriptor Ring Length Queue 13; RW */
#define E1000_RDLEN14       0xC388 /* RX Descriptor Ring Length Queue 14; RW */
#define E1000_RDLEN15       0xC3C8 /* RX Descriptor Ring Length Queue 15; RW */
#define E1000_SRRCTL4       0xC10C /* Split and Replication Receive Control Queue 4; RW */
#define E1000_SRRCTL5       0xC14C /* Split and Replication Receive Control Queue 5; RW */
#define E1000_SRRCTL6       0xC18C /* Split and Replication Receive Control Queue 6; RW */
#define E1000_SRRCTL7       0xC1CC /* Split and Replication Receive Control Queue 7; RW */
#define E1000_SRRCTL8       0xC20C /* Split and Replication Receive Control Queue 8; RW */
#define E1000_SRRCTL9       0xC24C /* Split and Replication Receive Control Queue 9; RW */
#define E1000_SRRCTL10      0xC28C /* Split and Replication Receive Control Queue 10; RW */
#define E1000_SRRCTL11      0xC2CC /* Split and Replication Receive Control Queue 11; RW */
#define E1000_SRRCTL12      0xC30C /* Split and Replication Receive Control Queue 12; RW */
#define E1000_SRRCTL13      0xC34C /* Split and Replication Receive Control Queue 13; RW */
#define E1000_SRRCTL14      0xC38C /* Split and Replication Receive Control Queue 14; RW */
#define E1000_SRRCTL15      0xC3CC /* Split and Replication Receive Control Queue 15; RW */
#define E1000_RDH4          0xC110 /* RX Descriptor Head Queue 4; RW */
#define E1000_RDH5          0xC150 /* RX Descriptor Head Queue 5; RW */
#define E1000_RDH6          0xC190 /* RX Descriptor Head Queue 6; RW */
#define E1000_RDH7          0xC1D0 /* RX Descriptor Head Queue 7; RW */
#define E1000_RDH8          0xC210 /* RX Descriptor Head Queue 8; RW */
#define E1000_RDH9          0xC250 /* RX Descriptor Head Queue 9; RW */
#define E1000_RDH10         0xC290 /* RX Descriptor Head Queue 10; RW */
#define E1000_RDH11         0xC2D0 /* RX Descriptor Head Queue 11; RW */
#define E1000_RDH12         0xC310 /* RX Descriptor Head Queue 12; RW */
#define E1000_RDH13         0xC350 /* RX Descriptor Head Queue 13; RW */
#define E1000_RDH14         0xC390 /* RX Descriptor Head Queue 14; RW */
#define E1000_RDH15         0xC3D0 /* RX Descriptor Head Queue 15; RW */
#define E1000_RDT4          0xC118 /* RX Descriptor Tail Queue 4; RW */
#define E1000_RDT5          0xC158 /* RX Descriptor Tail Queue 5; RW */
#define E1000_RDT6          0xC198 /* RX Descriptor Tail Queue 6; RW */
#define E1000_RDT7          0xC1D8 /* RX Descriptor Tail Queue 7; RW */
#define E1000_RDT8          0xC218 /* RX Descriptor Tail Queue 8; RW */
#define E1000_RDT9          0xC258 /* RX Descriptor Tail Queue 9; RW */
#define E1000_RDT10         0xC298 /* RX Descriptor Tail Queue 10; RW */
#define E1000_RDT11         0xC2D8 /* RX Descriptor Tail Queue 11; RW */
#define E1000_RDT12         0xC318 /* RX Descriptor Tail Queue 12; RW */
#define E1000_RDT13         0xC358 /* RX Descriptor Tail Queue 13; RW */
#define E1000_RDT14         0xC398 /* RX Descriptor Tail Queue 14; RW */
#define E1000_RDT15         0xC3D8 /* RX Descriptor Tail Queue 15; RW */
#define E1000_RXDCTL4       0xC128 /* RX Descriptor Control Queue 4; RW */
#define E1000_RXDCTL5       0xC168 /* RX Descriptor Control Queue 5; RW */
#define E1000_RXDCTL6       0xC1A8 /* RX Descriptor Control Queue 6; RW */
#define E1000_RXDCTL7       0xC1E8 /* RX Descriptor Control Queue 7; RW */
#define E1000_RXDCTL8       0xC228 /* RX Descriptor Control Queue 8; RW */
#define E1000_RXDCTL9       0xC268 /* RX Descriptor Control Queue 9; RW */
#define E1000_RXDCTL10      0xC2A8 /* RX Descriptor Control Queue 10; RW */
#define E1000_RXDCTL11      0xC2E8 /* RX Descriptor Control Queue 11; RW */
#define E1000_RXDCTL12      0xC328 /* RX Descriptor Control Queue 12; RW */
#define E1000_RXDCTL13      0xC368 /* RX Descriptor Control Queue 13; RW */
#define E1000_RXDCTL14      0xC3A8 /* RX Descriptor Control Queue 14; RW */
#define E1000_RXDCTL15      0xC3E8 /* RX Descriptor Control Queue 15; RW */
#define E1000_RXCTL4        0xC114 /* RX DCA Control Queue 4; RW */
#define E1000_RXCTL5        0xC154 /* RX DCA Control Queue 5; RW */
#define E1000_RXCTL6        0xC194 /* RX DCA Control Queue 6; RW */
#define E1000_RXCTL7        0xC1D4 /* RX DCA Control Queue 7; RW */
#define E1000_RXCTL8        0xC214 /* RX DCA Control Queue 8; RW */
#define E1000_RXCTL9        0xC254 /* RX DCA Control Queue 9; RW */
#define E1000_RXCTL10       0xC294 /* RX DCA Control Queue 10; RW */
#define E1000_RXCTL11       0xC2D4 /* RX DCA Control Queue 11; RW */
#define E1000_RXCTL12       0xC314 /* RX DCA Control Queue 12; RW */
#define E1000_RXCTL13       0xC354 /* RX DCA Control Queue 13; RW */
#define E1000_RXCTL14       0xC394 /* RX DCA Control Queue 14; RW */
#define E1000_RXCTL15       0xC3D4 /* RX DCA Control Queue 15; RW */
#if 0
#define E1000_RQDPC4        0xC130 /* RX Drop Packet Count Queue 4; RC */
#define E1000_RQDPC5        0xC170 /* RX Drop Packet Count Queue 5; RC */
#define E1000_RQDPC6        0xC1B0 /* RX Drop Packet Count Queue 6; RC */
#define E1000_RQDPC7        0xC1F0 /* RX Drop Packet Count Queue 7; RC */
#define E1000_RQDPC8        0xC230 /* RX Drop Packet Count Queue 8; RC */
#define E1000_RQDPC9        0xC270 /* RX Drop Packet Count Queue 9; RC */
#define E1000_RQDPC10       0xC2B0 /* RX Drop Packet Count Queue 10; RC */
#define E1000_RQDPC11       0xC2F0 /* RX Drop Packet Count Queue 11; RC */
#define E1000_RQDPC12       0xC330 /* RX Drop Packet Count Queue 12; RC */
#define E1000_RQDPC13       0xC370 /* RX Drop Packet Count Queue 13; RC */
#define E1000_RQDPC14       0xC3B0 /* RX Drop Packet Count Queue 14; RC */
#define E1000_RQDPC15       0xC3F0 /* RX Drop Packet Count Queue 15; RC */
#define E1000_RXCSUM        0x5000 /* Receive Checksum Control RW */
#endif
#define E1000_RLPML         0x5004 /* Receive Long packet maximal length RW */
#if 0
#define E1000_RFCTL         0x5008 /* Receive Filter Control Register RW */
#define E1000_MTA           0x5200 /* TODO: 0x53FC Multicast Table Array [127:0]; RW */
#define E1000_MTA_ALT       0x0200 /* TODO: 0x03FC */
#endif
#define E1000_RA            0x5400 /* Receive Address; RW */
#define E1000_RA_ALT        0x0040 /* */
#define E1000_RA_VF         0x54E0 /* Receive Address for the VF "Pool List"; RW */
#if 0
#define E1000_PSRTYPE       0x5480 /* TODO: 0x549C Packet Split Receive Type [7:0]; RW */
#define E1000_RPLPSRTYPE    0x54C0 /* Replicated Packet Split Receive type RW */
#define E1000_VT_CTL        0x581C /* Next Generation VMDq Control; RW */
#define E1000_VFTA          0x5600 /* TODO: 0x57FC VLAN Filter Table Array [127:0]; RW */
#define E1000_VFTA_ALT      0x0600 /* TODO: 0x07FC */
#define E1000_MRQC          0x5818 /* Multiple Receive Queues Command; RW */
#define E1000_RETA          0x5C00 /* TODO: 0x5C7C Redirection Table; RW */
#define E1000_RSSRK         0x5C80 /* TODO: 0x5CA4 RSS Random Key; RW */
#endif

/* Transmit Registers */

#if 0
#define E1000_TXPBS         0x3404 /* TX Packet Buffer Size; RW */
#define E1000_PBTWAC        0x34E8 /* TX Packet Buffer Wrap Around Counter; RO */
#define E1000_TCTL          0x0400 /* TX Control; RW */
#define E1000_TCTL_EXT      0x0404 /* TCTL_EXT TX Control Extended; RW */
#define E1000_TIPG          0x0410 /* TX IPG; RW */
#define E1000_DTXCTL        0x3590 /* DMA TX Control; RW */
#define E1000_DTXTCPFLGL    0x359C /* DMA TX TCP Flags Control Low; RW */
#define E1000_DTXTCPFLGH    0x35A0 /* DMA TX TCP Flags Control High; RW */
#define E1000_DTXMXSZRQ     0x3540 /* DMA TX Max Total Allow Size Requests; RW */
#endif
#define E1000_TDBAL0        0xE000 /* TX Descriptor Base Low 0; RW */
#define E1000_TDBAL0_ALT    0x3800
#define E1000_TDBAH0        0xE004 /* TX Descriptor Base High 0; RW */
#define E1000_TDBAH0_ALT    0x3804
#define E1000_TDLEN0        0xE008 /* TX Descriptor Ring Length 0; RW */
#define E1000_TDLEN0_ALT    0x3808
#define E1000_TDH0          0xE010 /* TX Descriptor Head Queue 0; RW */
#define E1000_TDH0_ALT      0x3810
#define E1000_TDT0          0xE018 /* TX Descriptor Tail Queue 0; RW */
#define E1000_TDT0_ALT      0x3818
#define E1000_TXDCTL0       0xE028 /* TX Descriptor Control Queue 0; RW */
#define E1000_TXDCTL0_ALT   0x3828
#define E1000_TXCTL0        0xE014 /* TX DCA Control Queue 0 RW */
#define E1000_TXCTL0_ALT    0x3814
#define E1000_TDWBAL0       0xE038 /* TX Descriptor Completion Write–Back Address Low Queue 0; RW */
#define E1000_TDWBAL0_ALT   0x3838
#define E1000_TDWBAH0       0xE03C /* TX Descriptor Completion Write–Back Address High Queue 0; RW */
#define E1000_TDWBAH0_ALT   0x383C
#define E1000_TDBAL1        0xE040 /* TX Descriptor Base Low Queue 1; RW */
#define E1000_TDBAL1_ALT    0x3900
#define E1000_TDBAL2        0xE080 /* TX Descriptor Base Low Queue 2; RW */
#define E1000_TDBAL2_ALT    0x3A00
#define E1000_TDBAL3        0xE0C0 /* TX Descriptor Base Low Queue 3; RW */
#define E1000_TDBAL3_ALT    0x3B00
#define E1000_TDBAH1        0xE044 /* TX Descriptor Base High Queue 1; RW */
#define E1000_TDBAH1_ALT    0x3904
#define E1000_TDBAH2        0xE084 /* TX Descriptor Base High Queue 2; RW */
#define E1000_TDBAH2_ALT    0x3A04
#define E1000_TDBAH3        0xE0C4 /* TX Descriptor Base High Queue 3; RW */
#define E1000_TDBAH3_ALT    0x3B04
#define E1000_TDLEN1        0xE048 /* TX Descriptor Ring Length Queue 1; RW */
#define E1000_TDLEN1_ALT    0x3908
#define E1000_TDLEN2        0xE088 /* TX Descriptor Ring Length Queue 2; RW */
#define E1000_TDLEN2_ALT    0x3A08
#define E1000_TDLEN3        0xE0C8 /* TX Descriptor Ring Length Queue 3; RW */
#define E1000_TDLEN3_ALT    0x3B08
#define E1000_TDH1          0xE050 /* TX Descriptor Head Queue 1; RW */
#define E1000_TDH1_ALT      0x3910
#define E1000_TDH2          0xE090 /* TX Descriptor Head Queue 2; RW */
#define E1000_TDH2_ALT      0x3A10
#define E1000_TDH3          0xE0D0 /* TX Descriptor Head Queue 3; RW */
#define E1000_TDH3_ALT      0x3B10
#define E1000_TDT1          0xE058 /* TX Descriptor Tail Queue 1; RW */
#define E1000_TDT1_ALT      0x3918
#define E1000_TDT2          0xE098 /* TX Descriptor Tail Queue 2; RW */
#define E1000_TDT2_ALT      0x3A18
#define E1000_TDT3          0xE0D8 /* TX Descriptor Tail Queue 3; RW */
#define E1000_TDT3_ALT      0x3B18
#define E1000_TXDCTL1       0xE068 /* TX Descriptor Control Queue 1; RW */
#define E1000_TXDCTL1_ALT   0x3928
#define E1000_TXDCTL2       0xE0A8 /* TX Descriptor Control Queue 2; RW */
#define E1000_TXDCTL2_ALT   0x3A28
#define E1000_TXDCTL3       0xE0E8 /* TX Descriptor Control 3; RW */
#define E1000_TXDCTL3_ALT   0x3B28
#define E1000_TXCTL1        0xE054 /* TX DCA Control Queue 1; RW */
#define E1000_TXCTL1_ALT    0x3914
#define E1000_TXCTL2        0xE094 /* TX DCA Control Queue 2; RW */
#define E1000_TXCTL2_ALT    0x3A14
#define E1000_TXCTL3        0xE0D4 /* TX DCA Control Queue 3; RW */
#define E1000_TXCTL3_ALT    0x3B14
#define E1000_TDWBAL1       0xE078 /* TX Descriptor Completion Write–Back Address Low Queue 1; RW */
#define E1000_TDWBAL1_ALT   0x3938
#define E1000_TDWBAL2       0xE0B8 /* TX Descriptor Completion Write–Back Address Low Queue 2; RW */
#define E1000_TDWBAL2_ALT   0x3A38
#define E1000_TDWBAL3       0xE0F8 /* TX Descriptor Completion Write–Back Address Low Queue 3; RW */
#define E1000_TDWBAL3_ALT   0x3B38
#define E1000_TDWBAH1       0xE07C /* TX Descriptor Completion Write–Back Address High Queue 1; RW */
#define E1000_TDWBAH1_ALT   0x393C
#define E1000_TDWBAH2       0xE0BC /* TX Descriptor Completion Write–Back Address High Queue 2; RW */
#define E1000_TDWBAH2_ALT   0x3A3C
#define E1000_TDWBAH3       0xE0FC /* TX Descriptor Completion Write–Back Address High Queue 3; RW */
#define E1000_TDWBAH3_ALT   0x3B3C
#define E1000_TDBAL4        0xE100 /* TX Descriptor Base Low Queue 4; RW */
#define E1000_TDBAL5        0xE140 /* TX Descriptor Base Low Queue 5; RW */
#define E1000_TDBAL6        0xE180 /* TX Descriptor Base Low Queue 6; RW */
#define E1000_TDBAL7        0xE1C0 /* TX Descriptor Base Low Queue 7; RW */
#define E1000_TDBAL8        0xE200 /* TX Descriptor Base Low Queue 8; RW */
#define E1000_TDBAL9        0xE240 /* TX Descriptor Base Low Queue 9; RW */
#define E1000_TDBAL10       0xE280 /* TX Descriptor Base Low Queue 10; RW */
#define E1000_TDBAL11       0xE2C0 /* TX Descriptor Base Low Queue 11; RW */
#define E1000_TDBAL12       0xE300 /* TX Descriptor Base Low Queue 12; RW */
#define E1000_TDBAL13       0xE340 /* TX Descriptor Base Low Queue 13; RW */
#define E1000_TDBAL14       0xE380 /* TX Descriptor Base Low Queue 14; RW */
#define E1000_TDBAL15       0xE3C0 /* TX Descriptor Base Low Queue 15; RW */
#define E1000_TDBAH4        0xE104 /* TX Descriptor Base High Queue 4; RW */
#define E1000_TDBAH5        0xE144 /* TX Descriptor Base High Queue 5; RW */
#define E1000_TDBAH6        0xE184 /* TX Descriptor Base High Queue 6; RW */
#define E1000_TDBAH7        0xE1C4 /* TX Descriptor Base High Queue 7; RW */
#define E1000_TDBAH8        0xE204 /* TX Descriptor Base High Queue 8; RW */
#define E1000_TDBAH9        0xE244 /* TX Descriptor Base High Queue 9; RW */
#define E1000_TDBAH10       0xE284 /* TX Descriptor Base High Queue 10; RW */
#define E1000_TDBAH11       0xE2C4 /* TX Descriptor Base High Queue 11; RW */
#define E1000_TDBAH12       0xE304 /* TX Descriptor Base High Queue 12; RW */
#define E1000_TDBAH13       0xE344 /* TX Descriptor Base High Queue 13; RW */
#define E1000_TDBAH14       0xE384 /* TX Descriptor Base High Queue 14; RW */
#define E1000_TDBAH15       0xE3C4 /* TX Descriptor Base High Queue 15; RW */
#define E1000_TDLEN4        0xE108 /* TX Descriptor Ring Length Queue 4; RW */
#define E1000_TDLEN5        0xE148 /* TX Descriptor Ring Length Queue 5; RW */
#define E1000_TDLEN6        0xE188 /* TX Descriptor Ring Length Queue 6; RW */
#define E1000_TDLEN7        0xE1C8 /* TX Descriptor Ring Length Queue 7; RW */
#define E1000_TDLEN8        0xE208 /* TX Descriptor Ring Length Queue 8; RW */
#define E1000_TDLEN9        0xE248 /* TX Descriptor Ring Length Queue 9; RW */
#define E1000_TDLEN10       0xE288 /* TX Descriptor Ring Length Queue 10; RW */
#define E1000_TDLEN11       0xE2C8 /* TX Descriptor Ring Length Queue 11; RW */
#define E1000_TDLEN12       0xE308 /* TX Descriptor Ring Length Queue 12; RW */
#define E1000_TDLEN13       0xE348 /* TX Descriptor Ring Length Queue 13; RW */
#define E1000_TDLEN14       0xE388 /* TX Descriptor Ring Length Queue 14; RW */
#define E1000_TDLEN15       0xE3C8 /* TX Descriptor Ring Length Queue 15; RW */
#define E1000_TDH4          0xE110 /* TX Descriptor Head Queue 4; RW */
#define E1000_TDH5          0xE150 /* TX Descriptor Head Queue 5; RW */
#define E1000_TDH6          0xE190 /* TX Descriptor Head Queue 6; RW */
#define E1000_TDH7          0xE1D0 /* TX Descriptor Head Queue 7; RW */
#define E1000_TDH8          0xE210 /* TX Descriptor Head Queue 8; RW */
#define E1000_TDH9          0xE250 /* TX Descriptor Head Queue 9; RW */
#define E1000_TDH10         0xE290 /* TX Descriptor Head Queue 10; RW */
#define E1000_TDH11         0xE2D0 /* TX Descriptor Head Queue 11; RW */
#define E1000_TDH12         0xE310 /* TX Descriptor Head Queue 12; RW */
#define E1000_TDH13         0xE350 /* TX Descriptor Head Queue 13; RW */
#define E1000_TDH14         0xE390 /* TX Descriptor Head Queue 14; RW */
#define E1000_TDH15         0xE3D0 /* TX Descriptor Head Queue 15; RW */
#define E1000_TDT4          0xE118 /* TX Descriptor Tail Queue 4; RW */
#define E1000_TDT5          0xE158 /* TX Descriptor Tail Queue 5; RW */
#define E1000_TDT6          0xE198 /* TX Descriptor Tail Queue 6; RW */
#define E1000_TDT7          0xE1D8 /* TX Descriptor Tail Queue 7; RW */
#define E1000_TDT8          0xE218 /* TX Descriptor Tail Queue 8; RW */
#define E1000_TDT9          0xE258 /* TX Descriptor Tail Queue 9; RW */
#define E1000_TDT10         0xE298 /* TX Descriptor Tail Queue 10; RW */
#define E1000_TDT11         0xE2D8 /* TX Descriptor Tail Queue 11; RW */
#define E1000_TDT12         0xE318 /* TX Descriptor Tail Queue 12; RW */
#define E1000_TDT13         0xE358 /* TX Descriptor Tail Queue 13; RW */
#define E1000_TDT14         0xE398 /* TX Descriptor Tail Queue 14; RW */
#define E1000_TDT15         0xE3D8 /* TX Descriptor Tail Queue 15; RW */
#define E1000_TXDCTL4       0xE128 /* TX Descriptor Control Queue 4; RW */
#define E1000_TXDCTL5       0xE168 /* TX Descriptor Control Queue 5; RW */
#define E1000_TXDCTL6       0xE1A8 /* TX Descriptor Control Queue 6; RW */
#define E1000_TXDCTL7       0xE1E8 /* TX Descriptor Control Queue 7; RW */
#define E1000_TXDCTL8       0xE228 /* TX Descriptor Control Queue 8; RW */
#define E1000_TXDCTL9       0xE268 /* TX Descriptor Control Queue 9; RW */
#define E1000_TXDCTL10      0xE2A8 /* TX Descriptor Control Queue 10; RW */
#define E1000_TXDCTL11      0xE2E8 /* TX Descriptor Control Queue 11; RW */
#define E1000_TXDCTL12      0xE328 /* TX Descriptor Control Queue 12; RW */
#define E1000_TXDCTL13      0xE368 /* TX Descriptor Control Queue 13; RW */
#define E1000_TXDCTL14      0xE3A8 /* TX Descriptor Control Queue 14; RW */
#define E1000_TXDCTL15      0xE3E8 /* TX Descriptor Control Queue 15; RW */
#define E1000_TDWBAL4       0xE138 /* TX Descriptor Completion Write–Back Address Low Queue 4; RW */
#define E1000_TDWBAL5       0xE178 /* TX Descriptor Completion Write–Back Address Low Queue 5; RW */
#define E1000_TDWBAL6       0xE1B8 /* TX Descriptor Completion Write–Back Address Low Queue 6; RW */
#define E1000_TDWBAL7       0xE1F8 /* TX Descriptor Completion Write–Back Address Low Queue 7; RW */
#define E1000_TDWBAL8       0xE238 /* TX Descriptor Completion Write–Back Address Low Queue 8; RW */
#define E1000_TDWBAL9       0xE278 /* TX Descriptor Completion Write–Back Address Low Queue 9; RW */
#define E1000_TDWBAL10      0xE2B8 /* TX Descriptor Completion Write–Back Address Low Queue 10; RW */
#define E1000_TDWBAL11      0xE2F8 /* TX Descriptor Completion Write–Back Address Low Queue 11; RW */
#define E1000_TDWBAL12      0xE338 /* TX Descriptor Completion Write–Back Address Low Queue 12; RW */
#define E1000_TDWBAL13      0xE378 /* TX Descriptor Completion Write–Back Address Low Queue 13; RW */
#define E1000_TDWBAL14      0xE3B8 /* TX Descriptor Completion Write–Back Address Low Queue 14; RW */
#define E1000_TDWBAL15      0xE3F8 /* TX Descriptor Completion Write–Back Address Low Queue 15; RW */
#define E1000_TDWBAH4       0xE13C /* TX Descriptor Completion Write–Back Address High Queue 4; RW */
#define E1000_TDWBAH5       0xE17C /* TX Descriptor Completion Write–Back Address High Queue 5; RW */
#define E1000_TDWBAH6       0xE1BC /* TX Descriptor Completion Write–Back Address High Queue 6; RW */
#define E1000_TDWBAH7       0xE1FC /* TX Descriptor Completion Write–Back Address High Queue 7; RW */
#define E1000_TDWBAH8       0xE23C /* TX Descriptor Completion Write–Back Address High Queue 8; RW */
#define E1000_TDWBAH9       0xE27C /* TX Descriptor Completion Write–Back Address High Queue 9; RW */
#define E1000_TDWBAH10      0xE2BC /* TX Descriptor Completion Write–Back Address High Queue 10; RW */
#define E1000_TDWBAH11      0xE2FC /* TX Descriptor Completion Write–Back Address High Queue 11; RW */
#define E1000_TDWBAH12      0xE33C /* TX Descriptor Completion Write–Back Address High Queue 12; RW */
#define E1000_TDWBAH13      0xE37C /* TX Descriptor Completion Write–Back Address High Queue 13; RW */
#define E1000_TDWBAH14      0xE3BC /* TX Descriptor Completion Write–Back Address High Queue 14; RW */
#define E1000_TDWBAH15      0xE3FC /* TX Descriptor Completion Write–Back Address High Queue 15; RW */
#define E1000_TXCTL4        0xE114 /* TX DCA Control Queue 4; RW */
#define E1000_TXCTL5        0xE154 /* TX DCA Control Queue 5; RW */
#define E1000_TXCTL6        0xE194 /* TX DCA Control Queue 6; RW */
#define E1000_TXCTL7        0xE1D4 /* TX DCA Control Queue 7; RW */
#define E1000_TXCTL8        0xE214 /* TX DCA Control Queue 8; RW */
#define E1000_TXCTL9        0xE254 /* TX DCA Control Queue 9; RW */
#define E1000_TXCTL10       0xE294 /* TX DCA Control Queue 10; RW */
#define E1000_TXCTL11       0xE2D4 /* TX DCA Control Queue 11; RW */
#define E1000_TXCTL12       0xE314 /* TX DCA Control Queue 12; RW */
#define E1000_TXCTL13       0xE354 /* TX DCA Control Queue 13; RW */
#define E1000_TXCTL14       0xE394 /* TX DCA Control Queue 14; RW */
#define E1000_TXCTL15       0xE3D4 /* TX DCA Control Queue 15; RW */

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
