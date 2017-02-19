/*
 * (C) Copyright 2010
 * Variscite LTD., <www.variscite.com>
 * Alex Bikhdriker <alex@variscite.com>
 *
 * Referred to Linux DSS driver files for OMAP3
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation's version 2 of
 * the License.
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

#include <common.h>
#include <asm/io.h>
#include <asm/arch/dss.h>

/*
 * VENC configuration
 */
void omap3_dss_venc_config(const struct venc_config *venc_cfg)
{
       dss_write_reg(VENC_STATUS, venc_cfg->status);
       dss_write_reg(VENC_F_CONTROL, venc_cfg->f_control);
       dss_write_reg(VENC_VIDOUT_CTRL, venc_cfg->vidout_ctrl);
       dss_write_reg(VENC_SYNC_CTRL, venc_cfg->sync_ctrl);
       dss_write_reg(VENC_LLEN, venc_cfg->llen);
       dss_write_reg(VENC_FLENS, venc_cfg->flens);
       dss_write_reg(VENC_HFLTR_CTRL, venc_cfg->hfltr_ctrl);
       dss_write_reg(VENC_CC_CARR_WSS_CARR, venc_cfg->cc_carr_wss_carr);
       dss_write_reg(VENC_C_PHASE, venc_cfg->c_phase);
       dss_write_reg(VENC_GAIN_U, venc_cfg->gain_u);
       dss_write_reg(VENC_GAIN_V, venc_cfg->gain_v);
       dss_write_reg(VENC_GAIN_Y, venc_cfg->gain_y);
       dss_write_reg(VENC_BLACK_LEVEL, venc_cfg->black_level);
       dss_write_reg(VENC_BLANK_LEVEL, venc_cfg->blank_level);
       dss_write_reg(VENC_X_COLOR, venc_cfg->x_color);
       dss_write_reg(VENC_M_CONTROL, venc_cfg->m_control);
       dss_write_reg(VENC_BSTAMP_WSS_DATA, venc_cfg->bstamp_wss_data);
       dss_write_reg(VENC_S_CARR, venc_cfg->s_carr);
       dss_write_reg(VENC_LINE21, venc_cfg->line21);
       dss_write_reg(VENC_LN_SEL, venc_cfg->ln_sel);
       dss_write_reg(VENC_L21__WC_CTL, venc_cfg->l21__wc_ctl);
       dss_write_reg(VENC_HTRIGGER_VTRIGGER, venc_cfg->htrigger_vtrigger);
       dss_write_reg(VENC_SAVID__EAVID, venc_cfg->savid__eavid);
       dss_write_reg(VENC_FLEN__FAL, venc_cfg->flen__fal);
       dss_write_reg(VENC_LAL__PHASE_RESET, venc_cfg->lal__phase_reset);
       dss_write_reg(VENC_HS_INT_START_STOP_X,
                               venc_cfg->hs_int_start_stop_x);
       dss_write_reg(VENC_HS_EXT_START_STOP_X,
                               venc_cfg->hs_ext_start_stop_x);
       dss_write_reg(VENC_VS_INT_START_X, venc_cfg->vs_int_start_x);
       dss_write_reg(VENC_VS_INT_STOP_X__VS_INT_START_Y,
                       venc_cfg->vs_int_stop_x__vs_int_start_y);
       dss_write_reg(VENC_VS_INT_STOP_Y__VS_EXT_START_X,
                       venc_cfg->vs_int_stop_y__vs_ext_start_x);
       dss_write_reg(VENC_VS_EXT_STOP_X__VS_EXT_START_Y,
                       venc_cfg->vs_ext_stop_x__vs_ext_start_y);
       dss_write_reg(VENC_VS_EXT_STOP_Y, venc_cfg->vs_ext_stop_y);
       dss_write_reg(VENC_AVID_START_STOP_X, venc_cfg->avid_start_stop_x);
       dss_write_reg(VENC_AVID_START_STOP_Y, venc_cfg->avid_start_stop_y);
       dss_write_reg(VENC_FID_INT_START_X__FID_INT_START_Y,
                               venc_cfg->fid_int_start_x__fid_int_start_y);
       dss_write_reg(VENC_FID_INT_OFFSET_Y__FID_EXT_START_X,
                               venc_cfg->fid_int_offset_y__fid_ext_start_x);
       dss_write_reg(VENC_FID_EXT_START_Y__FID_EXT_OFFSET_Y,
                               venc_cfg->fid_ext_start_y__fid_ext_offset_y);
       dss_write_reg(VENC_TVDETGP_INT_START_STOP_X,
                               venc_cfg->tvdetgp_int_start_stop_x);
       dss_write_reg(VENC_TVDETGP_INT_START_STOP_Y,
                               venc_cfg->tvdetgp_int_start_stop_y);
       dss_write_reg(VENC_GEN_CTRL, venc_cfg->gen_ctrl);
       dss_write_reg(VENC_OUTPUT_CONTROL, venc_cfg->output_control);
       dss_write_reg(VENC_DAC_B__DAC_C, venc_cfg->dac_b__dac_c);
       dss_write_reg(DISPC_SIZE_DIG, venc_cfg->height_width);
       dss_write_reg(DSS_CONTROL, VENC_DSS_CONFIG);
}

/*
 * Configure Panel Specific parameters
 */
void omap3_dss_panel_config(const struct panel_config *panel_cfg)
{
       dss_write_reg(DISPC_TIMING_H, panel_cfg->timing_h);
       dss_write_reg(DISPC_TIMING_V, panel_cfg->timing_v);
       dss_write_reg(DISPC_POL_FREQ, panel_cfg->pol_freq);
       dss_write_reg(DISPC_DIVISOR, panel_cfg->divisor);
       dss_write_reg(DISPC_SIZE_LCD, panel_cfg->lcd_size);
       dss_write_reg(DISPC_CONFIG,
               (panel_cfg->load_mode << FRAME_MODE_OFFSET));
       dss_write_reg(DISPC_CONTROL,
               ((panel_cfg->panel_type << TFTSTN_OFFSET) |
               (panel_cfg->data_lines << DATALINES_OFFSET)));
}

/*
 * Enable LCD and DIGITAL OUT in DSS
 */
void omap3_dss_enable(void)
{
       u32 l = 0;

       l = dss_read_reg(DISPC_CONTROL);
       l |= DISPC_ENABLE;

       dss_write_reg(DISPC_CONTROL, l);
}

/*
 * Set Background Color in DISPC
 */
void omap3_dss_set_background_col(u32 color)
{
       dss_write_reg(DISPC_DEFAULT_COLOR0, color);
}

/*
 * Fill frame buffer with splash screen data
 */
void omap3_fill_framebuffer(unsigned int fb_addr, unsigned char *bit_data, unsigned int size)
{
	memset((unsigned char*)fb_addr, 0xff, (800 * 480 *2));
	/* copy splash image row data to the frame buffer */
	memcpy((unsigned char*)fb_addr + (165 * 800 * 2), bit_data, size);

	/* set pointer to the frame buffer */
	dss_write_reg(DISPC_GFX_BA0, fb_addr);
	dss_write_reg(DISPC_GFX_BA1, fb_addr);

	dss_write_reg(DISPC_GFX_SIZE, 0x01df031f);
	dss_write_reg(DISPC_GFX_ATTRIBUTES, 0xD); /* 0x13 */
	dss_write_reg(DISPC_GFX_FIFO_THRESHOLD, 0x00fc00c0);
	dss_write_reg(DISPC_GFX_FIFO_SIZE_STATUS, 0x00000400);
	dss_write_reg(DISPC_GFX_ROW_INC, 0x00000001);
	dss_write_reg(DISPC_GFX_PIXEL_INC, 0x00000001);
	dss_write_reg(DISPC_GFX_WINDOW_SKIP, 0x00000000);
	dss_write_reg(DISPC_GFX_TABLE_BA, 0x807ff000);
}
