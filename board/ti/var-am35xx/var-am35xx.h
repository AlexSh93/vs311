/*
 * var-am35xx.h - Header file for the Variscite VAR-SOM-AM35 SOM.
 *
 * Author: Alex Bikhdriker <alex@variscite.com>
 *
 * Based on ti/am3517evm/am3517evm.h
 *
 * Copyright (C) 2010 Variscite LTD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef _VAR_AM35XX_H_
#define _VAR_AM35XX_H_

#include <asm/arch/dss.h>

const omap3_sysinfo sysinfo = {
	DDR_DISCRETE,
	"VAR-SOM-AM35xx Board",
	"NAND",
};

#define FRAME_BUFFER_ADDR    0x80500000

/*
 * Configure VENC in DSS for Beagle to generate Color Bar
 *
 * Kindly refer to OMAP TRM for definition of these values.
 */
static const struct venc_config venc_config_std_tv = {
       .status                                 = 0x0000001B,
       .f_control                              = 0x00000040,
       .vidout_ctrl                            = 0x00000000,
       .sync_ctrl                              = 0x00008000,
       .llen                                   = 0x00008359,
       .flens                                  = 0x0000020C,
       .hfltr_ctrl                             = 0x00000000,
       .cc_carr_wss_carr                       = 0x043F2631,
       .c_phase                                = 0x00000024,
       .gain_u                                 = 0x00000130,
       .gain_v                                 = 0x00000198,
       .gain_y                                 = 0x000001C0,
       .black_level                            = 0x0000006A,
       .blank_level                            = 0x0000005C,
       .x_color                                = 0x00000000,
       .m_control                              = 0x00000001,
       .bstamp_wss_data                        = 0x0000003F,
       .s_carr                                 = 0x21F07C1F,
       .line21                                 = 0x00000000,
       .ln_sel                                 = 0x00000015,
       .l21__wc_ctl                            = 0x00001400,
       .htrigger_vtrigger                      = 0x00000000,
       .savid__eavid                           = 0x069300F4,
       .flen__fal                              = 0x0016020C,
       .lal__phase_reset                       = 0x00060107,
       .hs_int_start_stop_x                    = 0x008D034E,
       .hs_ext_start_stop_x                    = 0x000F0359,
       .vs_int_start_x                         = 0x01A00000,
       .vs_int_stop_x__vs_int_start_y          = 0x020501A0,
       .vs_int_stop_y__vs_ext_start_x          = 0x01AC0024,
       .vs_ext_stop_x__vs_ext_start_y          = 0x020D01AC,
       .vs_ext_stop_y                          = 0x00000006,
       .avid_start_stop_x                      = 0x03480079,
       .avid_start_stop_y                      = 0x02040024,
       .fid_int_start_x__fid_int_start_y       = 0x0001008A,
       .fid_int_offset_y__fid_ext_start_x      = 0x01AC0106,
       .fid_ext_start_y__fid_ext_offset_y      = 0x01060006,
       .tvdetgp_int_start_stop_x               = 0x00140001,
       .tvdetgp_int_start_stop_y               = 0x00010001,
       .gen_ctrl                               = 0x00FF0000,
       .output_control                         = 0x0000000D,
       .dac_b__dac_c                           = 0x00000000,
       .height_width                           = 0x00ef027f
};

/*
 * Configure Timings for DVI D
 */
static const struct panel_config dvid_cfg = {
       .timing_h       = 0x0ff03f31, /* Horizontal timing */
       .timing_v       = 0x01400504, /* Vertical timing */
       .pol_freq       = 0x00007028, /* Pol Freq */
       .divisor        = 0x00010006, /* 72Mhz Pixel Clock */
       .lcd_size       = 0x02ff03ff, /* 1024x768 */
       .panel_type     = 0x01, /* TFT */
       .data_lines     = 0x03, /* 24 Bit RGB */
       .load_mode      = 0x02 /* Frame Mode */
};


static const struct panel_config urt_cfg = {
       .timing_h       = 0x01101d1b, /* Horizontal timing */
       .timing_v       = 0x01400b02, /* Vertical timing */
       .pol_freq       = 0x00023000, /* Pol Freq */
       .divisor        = 0x0001000d, /* 33Mhz Pixel Clock */
       .lcd_size       = 0x01df031f, /* 800x480 */
       .panel_type     = 0x01, /* TFT */
       .data_lines     = 0x03, /* 24 Bit RGB */
       .load_mode      = 0x02 /* Frame Mode */////
};

/* different LCD types for u-boot splash image demonstration */
/* define CONFIG_LCD_URT for URT UMSH-8272MD-1T LCD */
/* define CONFIG_LCD_WAG02 for DataImage WAG02 LCD */

#define CONFIG_LCD_URT

#ifdef CONFIG_VIDEO_BACKGROUND
#define SPLASH_SOLID_COLOR	0x00FF8000
#endif /* CONFIG_VIDEO_BACKGROUND */

/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * M0   - Mode 0
 * The commented string gives the final mux configuration for that pin
 */
#define MUX_VAR_AM35XX() \
	/* SDRC */\
	MUX_VAL(CP(SDRC_D0),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D1),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D2),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D3),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D4),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D5),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D6),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D7),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D8),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D9),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D10),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D11),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D12),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D13),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D14),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D15),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D16),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D17),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D18),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D19),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D20),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D21),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D22),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D23),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D24),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D25),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D26),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D27),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D28),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D29),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D30),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_D31),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_CLK),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_DQS0),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_DQS1),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_DQS2),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_DQS3),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SDRC_DQS0N),		(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(SDRC_DQS1N),		(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(SDRC_DQS2N),		(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(SDRC_DQS3N),		(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(SDRC_CKE0),		(M0)) \
	MUX_VAL(CP(SDRC_CKE1),		(M0)) \
	MUX_VAL(CP(STRBEN_DLY0),	(IEN  | PTD | EN  | M0)) /*sdrc_strben_dly0*/\
	MUX_VAL(CP(STRBEN_DLY1),	(IEN  | PTD | EN  | M0)) /*sdrc_strben_dly1*/\
	/* GPMC */\
	MUX_VAL(CP(GPMC_A1),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_A2),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_A3),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_A4),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_A5),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_A6),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_A7),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_A8),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_A9),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_A10),		(IDIS | PTU | DIS | M4)) /* GPIO_43 LCD buffer enable */ \
	MUX_VAL(CP(GPMC_D0),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D1),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D2),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D3),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D4),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D5),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D6),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D7),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D8),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D9),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D10),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D11),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D12),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D13),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D14),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_D15),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_NCS0),		(IDIS | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_NCS1),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_NCS2),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_NCS3),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_NCS4),		(IEN | PTU | EN  | M4))\
	MUX_VAL(CP(GPMC_NCS5),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_NCS6),		(IEN | PTU | EN  | M4))/* GPIO_57 TS_PenIRQn */\
	MUX_VAL(CP(GPMC_NCS7),		(IEN | PTU | EN  | M4)) /* GPIO_58 ETHERNET RESET */\
	MUX_VAL(CP(GPMC_CLK),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(GPMC_NADV_ALE),	(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(GPMC_NOE),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(GPMC_NWE),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(GPMC_NBE0_CLE),	(IDIS | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_NBE1),		(IEN  | PTU | DIS  | M4))/* GPIO_61 SD-CARD CD */ \
	MUX_VAL(CP(GPMC_NWP),		(IDIS  | PTU | EN | M4))/* GPIO_62 Nand write protect, keep enabled */ \
	MUX_VAL(CP(GPMC_WAIT0),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(GPMC_WAIT1),		(IEN | PTU | EN  | M4))\
	MUX_VAL(CP(GPMC_WAIT2),		(IEN  | PTU | EN  | M4))\
	MUX_VAL(CP(GPMC_WAIT3),		(IEN | PTU | EN  | M4))/* GPIO_65 SD-CARD WP */\
	/* DSS */\
	MUX_VAL(CP(DSS_PCLK),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_HSYNC),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_VSYNC),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_ACBIAS),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA0),		(IEN | PTU | EN  | M4))\
	MUX_VAL(CP(DSS_DATA1),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(DSS_DATA2),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(DSS_DATA3),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA4),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA5),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA6),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA7),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA8),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(DSS_DATA9),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(DSS_DATA10),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA11),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA12),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA13),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA14),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA15),		(IDIS | PTD | DIS | M0))\
	MUX_VAL(CP(DSS_DATA16),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(DSS_DATA17),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(DSS_DATA18),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(DSS_DATA19),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA20),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA21),		(IDIS | PTD | DIS | M0))  \
	MUX_VAL(CP(DSS_DATA22),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(DSS_DATA23),		(IDIS | PTD | DIS | M0)) \
	/* CAMERA */\
	MUX_VAL(CP(CAM_HS),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(CAM_VS),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(CAM_XCLKA),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_PCLK),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(CAM_FLD),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D0),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D1),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D2),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D3),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D4),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D5),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D6),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D7),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D8),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D9),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D10),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_D11),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_XCLKB),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_WEN),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CAM_STROBE),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CSI2_DX0),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CSI2_DY0),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CSI2_DX1),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CSI2_DY1),		(IEN  | PTD | EN  | M4)) \
	/* MMC */\
	MUX_VAL(CP(MMC1_CLK),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(MMC1_CMD),		(IEN  | PTU | DIS | M0)) \
	MUX_VAL(CP(MMC1_DAT0),		(IEN  | PTU | DIS | M0)) \
	MUX_VAL(CP(MMC1_DAT1),		(IEN  | PTU | DIS | M0)) \
	MUX_VAL(CP(MMC1_DAT2),		(IEN  | PTU | DIS | M0)) \
	MUX_VAL(CP(MMC1_DAT3),		(IEN  | PTU | DIS | M0)) \
	MUX_VAL(CP(MMC1_DAT4),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MMC1_DAT5),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MMC1_DAT6),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MMC1_DAT7),		(IEN  | PTU | EN  | M4)) \
	\
	MUX_VAL(CP(MMC2_CLK),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(MMC2_CMD),		(IDIS | PTD | DIS | M4)) /* GPIO_131 LCD Enable */ \
	MUX_VAL(CP(MMC2_DAT0),		(IDIS | PTD | DIS  | M4)) /* GPIO_132 USB host Enable */\
	MUX_VAL(CP(MMC2_DAT1),		(IDIS  | PTD | DIS  | M4)) /*GPIO_133 HDMI PD */\
	MUX_VAL(CP(MMC2_DAT2),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MMC2_DAT3),		(IEN  | PTU | EN  | M4))\
	/* McBSP */\
	MUX_VAL(CP(MCBSP_CLKS),		(IEN  | PTU | DIS | M0)) \
	MUX_VAL(CP(MCBSP1_CLKR),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(MCBSP1_FSR),		(IDIS | PTU | EN  | M0)) \
	MUX_VAL(CP(MCBSP1_DX),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(MCBSP1_DR),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(MCBSP1_FSX),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(MCBSP1_CLKX),	(IEN  | PTD | DIS | M0)) \
	\
	MUX_VAL(CP(MCBSP2_FSX),		(IEN  | PTU | EN  | M4))\
	MUX_VAL(CP(MCBSP2_CLKX),	(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MCBSP2_DR),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MCBSP2_DX),		(IEN  | PTU | EN  | M4))\
	\
	MUX_VAL(CP(MCBSP3_DX),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MCBSP3_DR),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MCBSP3_CLKX),	(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MCBSP3_FSX),		(IEN  | PTU | EN  | M4))\
	\
	MUX_VAL(CP(MCBSP4_CLKX),	(IDIS | PTD | DIS | M4)) /*GPIO_152 USB phy2 reset*/\
	MUX_VAL(CP(MCBSP4_DR),		(IEN | PTU | EN | M4)) /*GPIO_153*/\
	MUX_VAL(CP(MCBSP4_DX),		(IDIS | PTD | DIS | M4)) /*GPIO_154 USB phy1 reset*/\
	MUX_VAL(CP(MCBSP4_FSX),		(IEN | PTU | EN | M4)) /*GPIO_155 TS_BUSY*/\
	/* UART */\
	MUX_VAL(CP(UART1_TX),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(UART1_RTS),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(UART1_CTS),		(IEN  | PTU | DIS | M0)) \
	MUX_VAL(CP(UART1_RX),		(IEN  | PTD | DIS | M0)) \
	\
	MUX_VAL(CP(UART2_CTS),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(UART2_RTS),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(UART2_TX),		(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(UART2_RX),		(IEN  | PTD | DIS | M0)) \
	\
	MUX_VAL(CP(UART3_CTS_RCTX),	(IEN  | PTU | DIS | M0)) \
	MUX_VAL(CP(UART3_RTS_SD),	(IDIS | PTD | DIS | M0)) \
	MUX_VAL(CP(UART3_RX_IRRX),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(UART3_TX_IRTX),	(IDIS | PTD | DIS | M0)) \
	/* I2C */\
	MUX_VAL(CP(I2C1_SCL),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(I2C1_SDA),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(I2C2_SCL),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(I2C2_SDA),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(I2C3_SCL),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(I2C3_SDA),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(I2C4_SCL),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(I2C4_SDA),		(IEN  | PTU | EN  | M0)) \
	/* McSPI */\
	MUX_VAL(CP(MCSPI1_CLK),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(MCSPI1_SIMO),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(MCSPI1_SOMI),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(MCSPI1_CS0),		(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(MCSPI1_CS1),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MCSPI1_CS2),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MCSPI1_CS3),		(IEN  | PTU | EN  | M4)) \
	MUX_VAL(CP(MCSPI2_CLK),		(IEN  | PTD | DIS | M3)) /* HSUSB2_dat7 */\
	MUX_VAL(CP(MCSPI2_SIMO),	(IEN  | PTD | DIS | M3)) /* HSUSB2_dat4 */\
	MUX_VAL(CP(MCSPI2_SOMI),	(IEN  | PTD | DIS | M3)) /* HSUSB2_dat5 */\
	MUX_VAL(CP(MCSPI2_CS0),		(IEN  | PTD | DIS | M3)) /* HSUSB2_dat6 */\
	MUX_VAL(CP(MCSPI2_CS1),		(IEN  | PTD | DIS | M3)) /* HSUSB2_dat3 */\
	/* CCDC */\
	MUX_VAL(CP(CCDC_PCLK),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_FIELD),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_HD),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_VD),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_WEN),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_DATA0),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_DATA1),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_DATA2),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_DATA3),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_DATA4),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_DATA5),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_DATA6),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(CCDC_DATA7),		(IEN  | PTD | EN  | M4)) \
	/* RMII */\
	MUX_VAL(CP(RMII_MDIO_DATA),	(IEN  |  M0)) \
	MUX_VAL(CP(RMII_MDIO_CLK),	(M0)) \
	MUX_VAL(CP(RMII_RXD0)	,	(IEN  | PTD | M0)) \
	MUX_VAL(CP(RMII_RXD1),		(IEN  | PTD | M0)) \
	MUX_VAL(CP(RMII_CRS_DV),	(IEN  | PTD | M0)) \
	MUX_VAL(CP(RMII_RXER),		(PTD | M0)) \
	MUX_VAL(CP(RMII_TXD0),		(PTD | M0)) \
	MUX_VAL(CP(RMII_TXD1),		(PTD | M0)) \
	MUX_VAL(CP(RMII_TXEN),		(PTD | M0)) \
	MUX_VAL(CP(RMII_50MHZ_CLK),	(IEN  | PTD | EN  | M0)) \
	/* HECC */\
	MUX_VAL(CP(HECC1_TXD),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(HECC1_RXD),		(IEN  | PTD | EN  | M4)) \
	/* HSUSB */\
	MUX_VAL(CP(HSUSB0_CLK),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(HSUSB0_STP),		(IDIS | PTU | EN  | M0)) \
	MUX_VAL(CP(HSUSB0_DIR),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(HSUSB0_NXT),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(HSUSB0_DATA0),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(HSUSB0_DATA1),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(HSUSB0_DATA2),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(HSUSB0_DATA3),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(HSUSB0_DATA4),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(HSUSB0_DATA6),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(HSUSB0_DATA7),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(USB0_DRVBUS),	(IEN  | PTD | EN  | M0)) \
	/* HDQ */\
	MUX_VAL(CP(HDQ_SIO),		(IEN  | PTD | EN  | M4)) \
	/* Control and debug */\
	MUX_VAL(CP(SYS_32K),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(SYS_CLKREQ),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SYS_NIRQ),		(IEN  | PTD | EN  | M4)) \
	MUX_VAL(CP(SYS_NRESWARM),     	(IEN  | PTU | DIS | M4)) /*SYS_nRESWARM */\
	MUX_VAL(CP(SYS_BOOT0),		(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(SYS_BOOT1),		(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(SYS_BOOT2),		(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(SYS_BOOT3),		(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(SYS_BOOT4),		(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(SYS_BOOT5),		(IEN  | PTD | DIS | M4))\
	MUX_VAL(CP(SYS_BOOT6),		(IEN  | PTD | DIS | M4))\
	MUX_VAL(CP(SYS_BOOT7),		(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(SYS_BOOT8),		(IEN  | PTD | DIS | M4)) \
	\
	MUX_VAL(CP(SYS_OFF_MODE),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(SYS_CLKOUT1),	(IEN  | PTD | DIS | M4))\
	MUX_VAL(CP(SYS_CLKOUT2),	(IEN | PTD | DIS | M4))\
	/* JTAG */\
	MUX_VAL(CP(JTAG_nTRST),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(JTAG_TCK),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(JTAG_TMS),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(JTAG_TDI),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(JTAG_EMU0),		(IEN | PTU | EN  | M4)) \
	MUX_VAL(CP(JTAG_EMU1),		(IEN | PTU | EN  | M4))\
	/* ETK (ES2 onwards) */\
	MUX_VAL(CP(ETK_CLK_ES2),	(IDIS  | PTD | DIS | M3))/* hsusb1_stp */ \
	MUX_VAL(CP(ETK_CTL_ES2),	(IDIS  | PTD | DIS | M3)) /*hsusb1_clk*/\
	MUX_VAL(CP(ETK_D0_ES2 ),	(IEN  | PTD | EN | M3)) \
	MUX_VAL(CP(ETK_D1_ES2 ),	(IEN  | PTD | EN | M3)) \
	MUX_VAL(CP(ETK_D2_ES2 ),	(IEN  | PTD | EN | M3)) \
	MUX_VAL(CP(ETK_D3_ES2 ),	(IEN  | PTD | EN | M3)) \
	MUX_VAL(CP(ETK_D4_ES2 ),	(IEN  | PTD | EN | M3)) \
	MUX_VAL(CP(ETK_D5_ES2 ),	(IEN  | PTD | EN | M3)) \
	MUX_VAL(CP(ETK_D6_ES2 ),	(IEN  | PTD | EN | M3)) \
	MUX_VAL(CP(ETK_D7_ES2 ),	(IEN  | PTD | EN | M3)) \
	MUX_VAL(CP(ETK_D8_ES2 ),	(IEN  | PTD | EN | M3))\
	MUX_VAL(CP(ETK_D9_ES2 ),	(IEN  | PTD | EN | M3)) \
	MUX_VAL(CP(ETK_D10_ES2),	(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(ETK_D11_ES2),	(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(ETK_D12_ES2),	(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(ETK_D13_ES2),	(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(ETK_D14_ES2),	(IEN  | PTD | DIS | M4)) \
	MUX_VAL(CP(ETK_D15_ES2),	(IEN  | PTD | DIS | M4))\
	/* Die to Die */\
	MUX_VAL(CP(D2D_MCAD34),		(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(D2D_MCAD35),		(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(D2D_MCAD36),		(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(D2D_CLK26MI),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_NRESPWRON),	(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(D2D_NRESWARM),	(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(D2D_ARM9NIRQ),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_UMA2P6FIQ),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_SPINT),		(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(D2D_FRINT),		(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(D2D_DMAREQ0),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_DMAREQ1),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_DMAREQ2),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_DMAREQ3),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_N3GTRST),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_N3GTDI),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_N3GTDO),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_N3GTMS),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_N3GTCK),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_N3GRTCK),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_MSTDBY),		(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(D2D_SWAKEUP),	(IEN  | PTD | EN  | M0)) \
	MUX_VAL(CP(D2D_IDLEREQ),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_IDLEACK),	(IEN  | PTU | EN  | M0)) \
	MUX_VAL(CP(D2D_MWRITE),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_SWRITE),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_MREAD),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_SREAD),		(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_MBUSFLAG),	(IEN  | PTD | DIS | M0)) \
	MUX_VAL(CP(D2D_SBUSFLAG),	(IEN  | PTD | DIS | M0)) \

#endif /* _VAR_AM35XX_H_ */
