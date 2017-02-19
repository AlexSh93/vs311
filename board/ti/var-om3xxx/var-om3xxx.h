/*
 * (C) Copyright 2008
 * Nishanth Menon <menon.nishanth@gmail.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef _VAR_OM3XXX_H_
#define _VAR_OM3XXX_H_

#include <asm/arch/dss.h>

const omap3_sysinfo sysinfo = {
	DDR_STACKED,
	//DDR_DISCRETE,
	"VAR-SOM-OM3xxx Board",
#if defined(CONFIG_ENV_IS_IN_ONENAND)
	"OneNAND",
#else
	"NAND",
#endif
};

static void setup_net_chip(void);

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
       .load_mode      = 0x02 /* Frame Mode */
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
#define MUX_EVM() \
 /*SDRC*/\
	MUX_VAL(CP(SDRC_D0),		(IEN  | PTD | DIS | M0)) /*SDRC_D0*/\
	MUX_VAL(CP(SDRC_D1),		(IEN  | PTD | DIS | M0)) /*SDRC_D1*/\
	MUX_VAL(CP(SDRC_D2),		(IEN  | PTD | DIS | M0)) /*SDRC_D2*/\
	MUX_VAL(CP(SDRC_D3),		(IEN  | PTD | DIS | M0)) /*SDRC_D3*/\
	MUX_VAL(CP(SDRC_D4),		(IEN  | PTD | DIS | M0)) /*SDRC_D4*/\
	MUX_VAL(CP(SDRC_D5),		(IEN  | PTD | DIS | M0)) /*SDRC_D5*/\
	MUX_VAL(CP(SDRC_D6),		(IEN  | PTD | DIS | M0)) /*SDRC_D6*/\
	MUX_VAL(CP(SDRC_D7),		(IEN  | PTD | DIS | M0)) /*SDRC_D7*/\
	MUX_VAL(CP(SDRC_D8),		(IEN  | PTD | DIS | M0)) /*SDRC_D8*/\
	MUX_VAL(CP(SDRC_D9),		(IEN  | PTD | DIS | M0)) /*SDRC_D9*/\
	MUX_VAL(CP(SDRC_D10),		(IEN  | PTD | DIS | M0)) /*SDRC_D10*/\
	MUX_VAL(CP(SDRC_D11),		(IEN  | PTD | DIS | M0)) /*SDRC_D11*/\
	MUX_VAL(CP(SDRC_D12),		(IEN  | PTD | DIS | M0)) /*SDRC_D12*/\
	MUX_VAL(CP(SDRC_D13),		(IEN  | PTD | DIS | M0)) /*SDRC_D13*/\
	MUX_VAL(CP(SDRC_D14),		(IEN  | PTD | DIS | M0)) /*SDRC_D14*/\
	MUX_VAL(CP(SDRC_D15),		(IEN  | PTD | DIS | M0)) /*SDRC_D15*/\
	MUX_VAL(CP(SDRC_D16),		(IEN  | PTD | DIS | M0)) /*SDRC_D16*/\
	MUX_VAL(CP(SDRC_D17),		(IEN  | PTD | DIS | M0)) /*SDRC_D17*/\
	MUX_VAL(CP(SDRC_D18),		(IEN  | PTD | DIS | M0)) /*SDRC_D18*/\
	MUX_VAL(CP(SDRC_D19),		(IEN  | PTD | DIS | M0)) /*SDRC_D19*/\
	MUX_VAL(CP(SDRC_D20),		(IEN  | PTD | DIS | M0)) /*SDRC_D20*/\
	MUX_VAL(CP(SDRC_D21),		(IEN  | PTD | DIS | M0)) /*SDRC_D21*/\
	MUX_VAL(CP(SDRC_D22),		(IEN  | PTD | DIS | M0)) /*SDRC_D22*/\
	MUX_VAL(CP(SDRC_D23),		(IEN  | PTD | DIS | M0)) /*SDRC_D23*/\
	MUX_VAL(CP(SDRC_D24),		(IEN  | PTD | DIS | M0)) /*SDRC_D24*/\
	MUX_VAL(CP(SDRC_D25),		(IEN  | PTD | DIS | M0)) /*SDRC_D25*/\
	MUX_VAL(CP(SDRC_D26),		(IEN  | PTD | DIS | M0)) /*SDRC_D26*/\
	MUX_VAL(CP(SDRC_D27),		(IEN  | PTD | DIS | M0)) /*SDRC_D27*/\
	MUX_VAL(CP(SDRC_D28),		(IEN  | PTD | DIS | M0)) /*SDRC_D28*/\
	MUX_VAL(CP(SDRC_D29),		(IEN  | PTD | DIS | M0)) /*SDRC_D29*/\
	MUX_VAL(CP(SDRC_D30),		(IEN  | PTD | DIS | M0)) /*SDRC_D30*/\
	MUX_VAL(CP(SDRC_D31),		(IEN  | PTD | DIS | M0)) /*SDRC_D31*/\
	MUX_VAL(CP(SDRC_CLK),		(IEN  | PTD | DIS | M0)) /*SDRC_CLK*/\
	MUX_VAL(CP(SDRC_DQS0),		(IEN  | PTD | DIS | M0)) /*SDRC_DQS0*/\
	MUX_VAL(CP(SDRC_DQS1),		(IEN  | PTD | DIS | M0)) /*SDRC_DQS1*/\
	MUX_VAL(CP(SDRC_DQS2),		(IEN  | PTD | DIS | M0)) /*SDRC_DQS2*/\
	MUX_VAL(CP(SDRC_DQS3),		(IEN  | PTD | DIS | M0)) /*SDRC_DQS3*/\
 /*GPMC*/\
	MUX_VAL(CP(GPMC_A1),		(IDIS | PTU | EN  | M0)) /*GPMC_A1*/\
	MUX_VAL(CP(GPMC_A2),		(IDIS | PTU | EN  | M0)) /*GPMC_A2*/\
	MUX_VAL(CP(GPMC_A3),		(IDIS | PTU | EN  | M0)) /*GPMC_A3*/\
	MUX_VAL(CP(GPMC_A4),		(IDIS | PTU | EN  | M0)) /*GPMC_A4*/\
	MUX_VAL(CP(GPMC_A5),		(IDIS | PTU | EN  | M0)) /*GPMC_A5*/\
	MUX_VAL(CP(GPMC_A6),		(IDIS | PTU | EN  | M0)) /*GPMC_A6*/\
	MUX_VAL(CP(GPMC_A7),		(IDIS | PTU | EN  | M0)) /*GPMC_A7*/\
	MUX_VAL(CP(GPMC_A8),		(IDIS | PTU | EN  | M0)) /*GPMC_A8*/\
	MUX_VAL(CP(GPMC_A9),		(IDIS | PTU | EN  | M0)) /*GPMC_A9*/\
	MUX_VAL(CP(GPMC_A10),		(IDIS | PTU | EN  | M0)) /*GPMC_A10*/\
	MUX_VAL(CP(GPMC_D0),		(IEN  | PTU | EN  | M0)) /*GPMC_D0*/\
	MUX_VAL(CP(GPMC_D1),		(IEN  | PTU | EN  | M0)) /*GPMC_D1*/\
	MUX_VAL(CP(GPMC_D2),		(IEN  | PTU | EN  | M0)) /*GPMC_D2*/\
	MUX_VAL(CP(GPMC_D3),		(IEN  | PTU | EN  | M0)) /*GPMC_D3*/\
	MUX_VAL(CP(GPMC_D4),		(IEN  | PTU | EN  | M0)) /*GPMC_D4*/\
	MUX_VAL(CP(GPMC_D5),		(IEN  | PTU | EN  | M0)) /*GPMC_D5*/\
	MUX_VAL(CP(GPMC_D6),		(IEN  | PTU | EN  | M0)) /*GPMC_D6*/\
	MUX_VAL(CP(GPMC_D7),		(IEN  | PTU | EN  | M0)) /*GPMC_D7*/\
	MUX_VAL(CP(GPMC_D8),		(IEN  | PTU | EN  | M0)) /*GPMC_D8*/\
	MUX_VAL(CP(GPMC_D9),		(IEN  | PTU | EN  | M0)) /*GPMC_D9*/\
	MUX_VAL(CP(GPMC_D10),		(IEN  | PTU | EN  | M0)) /*GPMC_D10*/\
	MUX_VAL(CP(GPMC_D11),		(IEN  | PTU | EN  | M0)) /*GPMC_D11*/\
	MUX_VAL(CP(GPMC_D12),		(IEN  | PTU | EN  | M0)) /*GPMC_D12*/\
	MUX_VAL(CP(GPMC_D13),		(IEN  | PTU | EN  | M0)) /*GPMC_D13*/\
	MUX_VAL(CP(GPMC_D14),		(IEN  | PTU | EN  | M0)) /*GPMC_D14*/\
	MUX_VAL(CP(GPMC_D15),		(IEN  | PTU | EN  | M0)) /*GPMC_D15*/\
	MUX_VAL(CP(GPMC_NCS0),		(IDIS | PTU | EN  | M0)) /*GPMC_nCS0*/\
	MUX_VAL(CP(GPMC_NCS1),		(IDIS | PTU | EN  | M0)) /*GPMC_nCS1*/\
	MUX_VAL(CP(GPMC_NCS3),      (IDIS | PTU | DIS  | M4)) /*GPMC_nCS3*/\
	MUX_VAL(CP(GPMC_NCS4),		(IEN  | PTU | EN  | M0)) /*GPMC_nCS4*/\
	MUX_VAL(CP(GPMC_NCS5),      (IDIS | PTD | DIS  | M0)) /*GPMC_nCS5*/\
	MUX_VAL(CP(GPMC_NCS6),      (IDIS | PTD | DIS  | M4)) /*GPMC_nCS6*/\
	MUX_VAL(CP(GPMC_NCS7),		(IEN  | PTU | EN  | M0)) /*GPMC_nCS7*/\
	MUX_VAL(CP(GPMC_CLK),       (IDIS | PTD | DIS | M0)) /*GPMC_CLK*/\
	MUX_VAL(CP(GPMC_NADV_ALE),	(IDIS | PTD | DIS | M0)) /*GPMC_nADV_ALE*/\
	MUX_VAL(CP(GPMC_NOE),		(IDIS | PTD | DIS | M0)) /*GPMC_nOE*/\
	MUX_VAL(CP(GPMC_NWE),		(IDIS | PTD | DIS | M0)) /*GPMC_nWE*/\
	MUX_VAL(CP(GPMC_NBE0_CLE),	(IDIS | PTU | EN  | M0)) /*GPMC_nBE0_CLE*/\
	MUX_VAL(CP(GPMC_NBE1),		(IEN  | PTU | EN  | M0)) /*GPMC_nBE1*/\
	MUX_VAL(CP(GPMC_NWP),		(IEN  | PTD | DIS | M0)) /*GPMC_nWP*/\
	MUX_VAL(CP(GPMC_WAIT0),		(IEN  | PTU | EN  | M0)) /*GPMC_WAIT0*/\
	MUX_VAL(CP(GPMC_WAIT3),     (IDIS | PTU | DIS | M4)) /*GPIO_65*/\
 /*DSS*/\
	MUX_VAL(CP(DSS_PCLK),		(IDIS | PTD | DIS | M0)) /*DSS_PCLK*/\
	MUX_VAL(CP(DSS_HSYNC),		(IDIS | PTD | DIS | M0)) /*DSS_HSYNC*/\
	MUX_VAL(CP(DSS_VSYNC),		(IDIS | PTD | DIS | M0)) /*DSS_VSYNC*/\
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
	MUX_VAL(CP(DSS_DATA20),		(IDIS | PTD | DIS | M0)) /*DSS_DATA20*/\
	MUX_VAL(CP(DSS_DATA21),		(IDIS | PTD | DIS | M0)) /*DSS_DATA21*/\
	MUX_VAL(CP(DSS_DATA22),		(IDIS | PTD | DIS | M0)) /*DSS_DATA22*/\
	MUX_VAL(CP(DSS_DATA23),		(IDIS | PTD | DIS | M0)) /*DSS_DATA23*/\
 /*CAMERA*/\
	MUX_VAL(CP(CAM_HS),		(IEN  | PTU | EN  | M0)) /*CAM_HS */\
	MUX_VAL(CP(CAM_VS),		(IEN  | PTU | EN  | M0)) /*CAM_VS */\
	MUX_VAL(CP(CAM_XCLKA),		(IDIS | PTD | DIS | M0)) /*CAM_XCLKA*/\
	MUX_VAL(CP(CAM_PCLK),		(IEN  | PTU | EN  | M0)) /*CAM_PCLK*/\
	MUX_VAL(CP(CAM_FLD),		(IDIS | PTD | DIS | M4)) /*GPIO_98*/\
								 /* - CAM_RESET*/\
	MUX_VAL(CP(CAM_D0),		(IEN  | PTD | DIS | M0)) /*CAM_D0*/\
	MUX_VAL(CP(CAM_D1),		(IEN  | PTD | DIS | M0)) /*CAM_D1*/\
	MUX_VAL(CP(CAM_D2),		(IEN  | PTD | DIS | M0)) /*CAM_D2*/\
	MUX_VAL(CP(CAM_D3),		(IEN  | PTD | DIS | M0)) /*CAM_D3*/\
	MUX_VAL(CP(CAM_D4),		(IEN  | PTD | DIS | M0)) /*CAM_D4*/\
	MUX_VAL(CP(CAM_D5),		(IEN  | PTD | DIS | M0)) /*CAM_D5*/\
	MUX_VAL(CP(CAM_D6),		(IEN  | PTD | DIS | M0)) /*CAM_D6*/\
	MUX_VAL(CP(CAM_D7),		(IEN  | PTD | DIS | M0)) /*CAM_D7*/\
	MUX_VAL(CP(CAM_D8),		(IEN  | PTD | DIS | M0)) /*CAM_D8*/\
	MUX_VAL(CP(CAM_D9),		(IEN  | PTD | DIS | M0)) /*CAM_D9*/\
	MUX_VAL(CP(CAM_D10),		(IEN  | PTD | DIS | M0)) /*CAM_D10*/\
	MUX_VAL(CP(CAM_D11),		(IEN  | PTD | DIS | M0)) /*CAM_D11*/\
	MUX_VAL(CP(CAM_XCLKB),		(IDIS | PTD | DIS | M0)) /*CAM_XCLKB*/\
	MUX_VAL(CP(CAM_WEN),		(IEN  | PTD | DIS | M4)) /*GPIO_167*/\
	MUX_VAL(CP(CAM_STROBE),		(IDIS | PTD | DIS | M0)) /*CAM_STROBE*/\
 /*Audio Interface */\
	MUX_VAL(CP(MCBSP2_FSX),		(IEN  | PTD | DIS | M0)) /*McBSP2_FSX*/\
	MUX_VAL(CP(MCBSP2_CLKX),	(IEN  | PTD | DIS | M0)) /*McBSP2_CLKX*/\
	MUX_VAL(CP(MCBSP2_DR),		(IEN  | PTD | DIS | M0)) /*McBSP2_DR*/\
	MUX_VAL(CP(MCBSP2_DX),		(IDIS | PTD | DIS | M0)) /*McBSP2_DX*/\
 /*Expansion card  */\
	MUX_VAL(CP(MMC1_CLK),		(IDIS | PTU | EN  | M0)) /*MMC1_CLK*/\
	MUX_VAL(CP(MMC1_CMD),		(IEN  | PTU | EN  | M0)) /*MMC1_CMD*/\
	MUX_VAL(CP(MMC1_DAT0),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT0*/\
	MUX_VAL(CP(MMC1_DAT1),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT1*/\
	MUX_VAL(CP(MMC1_DAT2),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT2*/\
	MUX_VAL(CP(MMC1_DAT3),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT3*/\
	MUX_VAL(CP(MMC1_DAT4),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT4*/\
	MUX_VAL(CP(MMC2_DAT5),      (IDIS | PTU | EN  | M4)) /*GPIO_137 Eth reset*/\
	MUX_VAL(CP(MMC2_CMD),       (IDIS | PTD | DIS | M4)) /*LCD enable*/\
	MUX_VAL(CP(MMC2_DAT0),      (IDIS | PTD | DIS | M4)) /*USB enable*/\
	MUX_VAL(CP(MMC2_DAT1),      (IDIS | PTD | DIS | M4)) /*HDMI ~enable*/\
	MUX_VAL(CP(MMC1_DAT6),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT6*/\
	MUX_VAL(CP(MMC1_DAT7),		(IEN  | PTU | EN  | M0)) /*MMC1_DAT7*/\
 /*Wireless LAN */\
	MUX_VAL(CP(MCBSP3_FSX),     (IEN  | PTU | EN | M1)) /*UART2_CTS*/\
	MUX_VAL(CP(MCBSP3_CLKX),    (IDIS  | PTD | DIS | M1)) /*UART2_RTS*/\
	MUX_VAL(CP(MCBSP3_DX),      (IDIS | PTD | DIS | M1)) /*UART2_TX*/\
	MUX_VAL(CP(MCBSP3_DR),      (IEN  | PTU | EN | M1)) /*UART2_RX*/\
 /*Modem Interface */\
	MUX_VAL(CP(UART1_TX),		(IDIS | PTD | DIS | M0)) /*UART1_TX*/\
	MUX_VAL(CP(UART1_RTS),		(IDIS | PTD | DIS | M0)) /*UART1_RTS*/\
	MUX_VAL(CP(UART1_CTS),		(IEN  | PTU | DIS | M0)) /*UART1_CTS*/\
	MUX_VAL(CP(UART1_RX),		(IEN  | PTD | DIS | M0)) /*UART1_RX*/\
 /*Serial Interface*/\
	MUX_VAL(CP(UART3_CTS_RCTX),	(IEN  | PTD | EN  | M0)) /*UART3_CTS_*/\
	MUX_VAL(CP(UART3_RTS_SD),	(IDIS | PTD | DIS | M0)) /*UART3_RTS_SD */\
	MUX_VAL(CP(UART3_RX_IRRX),	(IEN  | PTD | DIS | M0)) /*UART3_RX_IRRX*/\
	MUX_VAL(CP(UART3_TX_IRTX),	(IDIS | PTD | DIS | M0)) /*UART3_TX_IRTX*/\
	/*OTG Interface*/\
	MUX_VAL(CP(HSUSB0_CLK),		(IEN  | PTD | DIS | M0)) /*HSUSB0_CLK*/\
	MUX_VAL(CP(HSUSB0_STP),		(IDIS | PTU | EN  | M0)) /*HSUSB0_STP*/\
	MUX_VAL(CP(HSUSB0_DIR),		(IEN  | PTD | DIS | M0)) /*HSUSB0_DIR*/\
	MUX_VAL(CP(HSUSB0_NXT),		(IEN  | PTD | DIS | M0)) /*HSUSB0_NXT*/\
	MUX_VAL(CP(HSUSB0_DATA0),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA0*/\
	MUX_VAL(CP(HSUSB0_DATA1),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA1*/\
	MUX_VAL(CP(HSUSB0_DATA2),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA2*/\
	MUX_VAL(CP(HSUSB0_DATA3),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA3*/\
	MUX_VAL(CP(HSUSB0_DATA4),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA4*/\
	MUX_VAL(CP(HSUSB0_DATA5),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA5*/\
	MUX_VAL(CP(HSUSB0_DATA6),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA6*/\
	MUX_VAL(CP(HSUSB0_DATA7),	(IEN  | PTD | DIS | M0)) /*HSUSB0_DATA7*/\
	MUX_VAL(CP(I2C1_SCL),		(IEN  | PTU | EN  | M0)) /*I2C1_SCL*/\
	MUX_VAL(CP(I2C1_SDA),		(IEN  | PTU | EN  | M0)) /*I2C1_SDA*/\
	MUX_VAL(CP(I2C2_SCL),		(IEN  | PTU | EN  | M0)) /*I2C2_SCL*/\
	MUX_VAL(CP(I2C2_SDA),		(IEN  | PTU | EN  | M0)) /*I2C2_SDA*/\
	MUX_VAL(CP(I2C3_SCL),		(IEN  | PTU | EN  | M0)) /*I2C3_SCL*/\
	MUX_VAL(CP(I2C3_SDA),		(IEN  | PTU | EN  | M0)) /*I2C3_SDA*/\
	MUX_VAL(CP(I2C4_SCL),		(IEN  | PTU | EN  | M0)) /*I2C4_SCL*/\
	MUX_VAL(CP(I2C4_SDA),		(IEN  | PTU | EN  | M0)) /*I2C4_SDA*/\
	MUX_VAL(CP(HDQ_SIO),		(IEN  | PTU | EN  | M0)) /*HDQ_SIO*/\
	MUX_VAL(CP(MCSPI1_CLK),		(IEN  | PTD | DIS | M0)) /*McSPI1_CLK*/\
	MUX_VAL(CP(MCSPI1_SIMO),	(IEN  | PTD | DIS | M0)) /*McSPI1_SIMO  */\
	MUX_VAL(CP(MCSPI1_SOMI),	(IEN  | PTD | DIS | M0)) /*McSPI1_SOMI  */\
	MUX_VAL(CP(MCSPI1_CS0),		(IEN  | PTD | EN  | M0)) /*McSPI1_CS0*/\
	MUX_VAL(CP(MCSPI1_CS3),     (IEN  | PTD | DIS  | M4)) /*GPIO_177 (touch IRQ)*/\
 /*Control and debug */\
	MUX_VAL(CP(SYS_32K),		(IEN  | PTD | DIS | M0)) /*SYS_32K*/\
	MUX_VAL(CP(SYS_CLKREQ),		(IEN  | PTD | DIS | M0)) /*SYS_CLKREQ*/\
	MUX_VAL(CP(SYS_NIRQ),		(IEN  | PTU | EN  | M0)) /*SYS_nIRQ*/\
	MUX_VAL(CP(SYS_BOOT0),		(IEN  | PTD | DIS | M4)) /*GPIO_2*/\
	MUX_VAL(CP(SYS_BOOT1),		(IEN  | PTD | DIS | M4)) /*GPIO_3 */\
	MUX_VAL(CP(SYS_BOOT2),		(IEN  | PTD | DIS | M0)) /*GPIO_4*/\
	MUX_VAL(CP(SYS_BOOT3),		(IEN  | PTD | DIS | M0)) /*GPIO_5*/\
	MUX_VAL(CP(SYS_BOOT4),		(IEN  | PTD | DIS | M0)) /*GPIO_6*/\
	MUX_VAL(CP(SYS_BOOT5),		(IEN  | PTD | DIS | M0)) /*GPIO_7*/\
	MUX_VAL(CP(SYS_BOOT6),		(IDIS | PTD | DIS | M4)) /*GPIO_8*/\
								 /* - VIO_1V8*/\
	MUX_VAL(CP(SYS_OFF_MODE),	(IEN  | PTD | DIS | M0)) /*SYS_OFF_MODE*/\
	MUX_VAL(CP(SYS_CLKOUT1),	(IEN  | PTD | DIS | M0)) /*SYS_CLKOUT1*/\
	MUX_VAL(CP(SYS_CLKOUT2),	(IEN  | PTU | EN  | M0)) /*SYS_CLKOUT2*/\
	MUX_VAL(CP(JTAG_nTRST),		(IEN  | PTD | DIS | M0)) /*JTAG_nTRST*/\
	MUX_VAL(CP(JTAG_TCK),		(IEN  | PTD | DIS | M0)) /*JTAG_TCK*/\
	MUX_VAL(CP(JTAG_TMS),		(IEN  | PTD | DIS | M0)) /*JTAG_TMS*/\
	MUX_VAL(CP(JTAG_TDI),		(IEN  | PTD | DIS | M0)) /*JTAG_TDI*/\
	MUX_VAL(CP(JTAG_EMU0),		(IEN  | PTD | DIS | M0)) /*JTAG_EMU0*/\
	MUX_VAL(CP(JTAG_EMU1),		(IEN  | PTD | DIS | M0)) /*JTAG_EMU1*/\
	/* USB EHCI port#1 */\
	MUX_VAL(CP(ETK_CLK),        (IDIS | PTU | EN  | M3)) /*HSUSB1_STP*/\
	MUX_VAL(CP(ETK_CTL),        (IDIS | PTD | DIS | M3)) /*HSUSB1_CLK*/\
	MUX_VAL(CP(ETK_D0 ),        (IEN  | PTD | DIS | M3)) /*HSUSB1_DATA0*/\
	MUX_VAL(CP(ETK_D1 ),        (IEN  | PTD | DIS | M3)) /*HSUSB1_DATA1*/\
	MUX_VAL(CP(ETK_D2 ),        (IEN  | PTD | DIS | M3)) /*HSUSB1_DATA2*/\
	MUX_VAL(CP(ETK_D3 ),        (IEN  | PTD | DIS | M3)) /*HSUSB1_DATA7*/\
	MUX_VAL(CP(ETK_D4 ),        (IEN  | PTD | DIS | M3)) /*HSUSB1_DATA4*/\
	MUX_VAL(CP(ETK_D5 ),        (IEN  | PTD | DIS | M3)) /*HSUSB1_DATA5*/\
	MUX_VAL(CP(ETK_D6 ),        (IEN  | PTD | DIS | M3)) /*HSUSB1_DATA6*/\
	MUX_VAL(CP(ETK_D7 ),        (IEN  | PTD | DIS | M3)) /*HSUSB1_DATA3*/\
	MUX_VAL(CP(ETK_D8 ),        (IEN  | PTD | DIS | M3)) /*HSUSB1_DIR*/\
	MUX_VAL(CP(ETK_D9 ),        (IEN  | PTD | DIS | M3)) /*HSUSB1_NXT*/\
	MUX_VAL(CP(ETK_D10),        (IEN  | PTD | DIS | M4)) /*GPIO_24 USB PHY reset*/\
	MUX_VAL(CP(ETK_D14_ES2),    (IDIS | PTD | DIS | M4)) /*GPIO_28 (Backlight)*/\
	MUX_VAL(CP(ETK_D15_ES2),    (IEN  | PTD | DIS | M4)) /*GPIO_29 (Eth IRQ)*/\
	MUX_VAL(CP(SDRC_CKE0),      (IDIS | PTU | EN  | M0)) /*sdrc_cke0 */

#endif /* _VAR_OM3XXX_H_ */
