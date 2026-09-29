/*
 * Copyright (c) 2026 UMass Robotics Club
 * SPDX-License-Identifier: Apache-2.0
 *
 * Lets the upstream Microchip GMAC driver source (written against
 * gmac_registers_t / GMAC_* for the PIC32CX SG) run on the PIC32CZ CA "ETH"
 * block, the same idea as modules/hal/microchip/.../sam/common/override_gmac.h.
 *
 * Why this is enough: the CA's eth_registers_t is a small wrapper (CTRLA,
 * CTRLB, EVCTRL, SYNCB, WPCTRL at 0x000-0x030) followed by the same Cadence
 * GEM core at +0x1000. Every core register the driver touches has an ETH_
 * twin at exactly +0x1000 (checked field by field against both HAL headers),
 * so pointing the driver at the ETH base and renaming is correct: the +0x1000
 * is already inside eth_registers_t.
 *
 * Generated from the GMAC_* names the driver uses that exist in the
 * pic32cx_sg41 gmac.h (71 names). 64 map 1:1. The last 7 are named field
 * values the CA header doesn't name; the field positions are identical and the
 * values come from the PIC32CZ CA datasheet (DS60001749L, NCFGR.CLK and
 * DCFGR.FBLDO tables).
 */

#ifndef MEGA_CAN_GMAC_PIC32CZ_COMPAT_H_
#define MEGA_CAN_GMAC_PIC32CZ_COMPAT_H_

#include <soc.h>

typedef eth_registers_t gmac_registers_t;

#define GMAC_AE                        ETH_AE
#define GMAC_CSE                       ETH_CSE
#define GMAC_DCFGR                     ETH_DCFGR
#define GMAC_DCFGR_DRBS                ETH_DCFGR_DRBS
#define GMAC_DCFGR_FBLDO_INCR4         ETH_DCFGR_FBLDO(4)
#define GMAC_DCFGR_RXBMS               ETH_DCFGR_RXBMS
#define GMAC_DCFGR_TXCOEN_Msk          ETH_DCFGR_TXCOEN_Msk
#define GMAC_EC                        ETH_EC
#define GMAC_FCSE                      ETH_FCSE
#define GMAC_HRB                       ETH_HRB
#define GMAC_HRT                       ETH_HRT
#define GMAC_IDR                       ETH_IDR
#define GMAC_IER                       ETH_IER
#define GMAC_IER_HRESP_Msk             ETH_IER_HRESP_Msk
#define GMAC_IER_RCOMP_Msk             ETH_IER_RCOMP_Msk
#define GMAC_IER_RLEX_Msk              ETH_IER_RLEX_Msk
#define GMAC_IER_ROVR_Msk              ETH_IER_ROVR_Msk
#define GMAC_IER_RXUBR_Msk             ETH_IER_RXUBR_Msk
#define GMAC_IER_TCOMP_Msk             ETH_IER_TCOMP_Msk
#define GMAC_IER_TFC_Msk               ETH_IER_TFC_Msk
#define GMAC_IER_TUR_Msk               ETH_IER_TUR_Msk
#define GMAC_IHCE                      ETH_IHCE
#define GMAC_ISR                       ETH_ISR
#define GMAC_ISR_RCOMP_Msk             ETH_ISR_RCOMP_Msk
#define GMAC_ISR_TCOMP_Msk             ETH_ISR_TCOMP_Msk
#define GMAC_JR                        ETH_JR
#define GMAC_LFFE                      ETH_LFFE
#define GMAC_MCF                       ETH_MCF
#define GMAC_MFR                       ETH_MFR
#define GMAC_MFT                       ETH_MFT
#define GMAC_NCFGR                     ETH_NCFGR
#define GMAC_NCFGR_CLK_MCK16           ETH_NCFGR_CLK(1)
#define GMAC_NCFGR_CLK_MCK32           ETH_NCFGR_CLK(2)
#define GMAC_NCFGR_CLK_MCK48           ETH_NCFGR_CLK(3)
#define GMAC_NCFGR_CLK_MCK64           ETH_NCFGR_CLK(4)
#define GMAC_NCFGR_CLK_MCK8            ETH_NCFGR_CLK(0)
#define GMAC_NCFGR_CLK_MCK96           ETH_NCFGR_CLK(5)
#define GMAC_NCFGR_FD_Msk              ETH_NCFGR_FD_Msk
#define GMAC_NCFGR_LFERD_Msk           ETH_NCFGR_LFERD_Msk
#define GMAC_NCFGR_MAXFS_Msk           ETH_NCFGR_MAXFS_Msk
#define GMAC_NCFGR_MTIHEN_Msk          ETH_NCFGR_MTIHEN_Msk
#define GMAC_NCFGR_RFCS_Msk            ETH_NCFGR_RFCS_Msk
#define GMAC_NCFGR_RXCOEN_Msk          ETH_NCFGR_RXCOEN_Msk
#define GMAC_NCFGR_SPD_Msk             ETH_NCFGR_SPD_Msk
#define GMAC_NCR                       ETH_NCR
#define GMAC_NCR_CLRSTAT_Msk           ETH_NCR_CLRSTAT_Msk
#define GMAC_NCR_MPE_Msk               ETH_NCR_MPE_Msk
#define GMAC_NCR_RXEN_Msk              ETH_NCR_RXEN_Msk
#define GMAC_NCR_TSTART_Msk            ETH_NCR_TSTART_Msk
#define GMAC_NCR_TXEN_Msk              ETH_NCR_TXEN_Msk
#define GMAC_OFR                       ETH_OFR
#define GMAC_RBQB                      ETH_RBQB
#define GMAC_RBQB_ADDR_Msk             ETH_RBQB_ADDR_Msk
#define GMAC_ROE                       ETH_ROE
#define GMAC_RRE                       ETH_RRE
#define GMAC_RSE                       ETH_RSE
#define GMAC_RSR                       ETH_RSR
#define GMAC_RSR_BNA_Msk               ETH_RSR_BNA_Msk
#define GMAC_RSR_RESETVALUE            ETH_RSR_RESETVALUE
#define GMAC_SAB                       ETH_SAB
#define GMAC_SAB_ADDR                  ETH_SAB_ADDR
#define GMAC_SAT                       ETH_SAT
#define GMAC_SAT_ADDR                  ETH_SAT_ADDR
#define GMAC_SCF                       ETH_SCF
#define GMAC_TBQB                      ETH_TBQB
#define GMAC_TBQB_ADDR_Msk             ETH_TBQB_ADDR_Msk
#define GMAC_TCE                       ETH_TCE
#define GMAC_TUR                       ETH_TUR
#define GMAC_UCE                       ETH_UCE
#define GMAC_UFR                       ETH_UFR
#define GMAC_UR                        ETH_UR

#endif /* MEGA_CAN_GMAC_PIC32CZ_COMPAT_H_ */
