/*
 * (C) Copyright 2004-2008
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *  Yura Dinkevich <yurid@variscite.com>
 *
 * Derived from EVM code by
 *	Manikandan Pillai <mani.pillai@ti.com>
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
#include <common.h>
#include <netdev.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/arch/mem.h>
#include <asm/arch/mux.h>
#include <asm/arch/omap_gpmc.h>
#include <asm/arch/sys_proto.h>
#include <i2c.h>
#include "var-om3xxx.h"

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */
	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_OMAP3EVM;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

	return 0;
}

/* splash screen row data */
#ifdef CONFIG_VIDEO_SPLASH
#include "var_16_splash.c"
#endif /* CONFIG_VIDEO_SPLASH */

/*
 * Configure DSS to display background color on DVID
 * Configure VENC to display color bar on S-Video
 */
#ifdef CONFIG_VIDEO_OMAP3
void display_init(void)
{
	struct panel_config lcd_cfg;

#if defined(CONFIG_LCD_URT)
	memcpy(&lcd_cfg, &urt_cfg, sizeof(lcd_cfg));
#elif defined(CONFIG_LCD_WAG02)
	memcpy(&lcd_cfg, &wag02_cfg, sizeof(lcd_cfg));
#else
	error Unknown display type
#endif
//	if (is_cpu_family(CPU_OMAP36XX))
	//	lcd_cfg.divisor = 0x00010003;
	omap3_dss_panel_config(&lcd_cfg);

#if defined(CONFIG_VIDEO_SPLASH)
	omap3_fill_framebuffer(FRAME_BUFFER_ADDR, splash_data, splash_data_size);
#elif defined(CONFIG_VIDEO_BACKGROUND)
	omap3_dss_set_background_col(SPLASH_SOLID_COLOR);
#endif	
}
#endif /* CONFIG_VIDEO_OMAP3 */

/******************************************************************************
 * Routine: set_baseboard_gpios
 * Description:
 * enable LCD and USB; disable HDMI
 *
 *****************************************************************************/
void set_baseboard_gpios(void)
{
	struct gpio *gpio5_base = (struct gpio *)OMAP34XX_GPIO5_BASE;

	/* shut the backlight */
#ifndef CONFIG_VIDEO_OMAP3
	struct gpio *gpio1_base = (struct gpio *)OMAP34XX_GPIO1_BASE;

	/* backlight off */
	writel(GPIO28, &gpio1_base->cleardataout);
	/* Make GPIO 28 as output pin */
	writel(readl(&gpio1_base->oe) & ~(GPIO28), &gpio1_base->oe);
#endif /* CONFIG_VIDEO_OMAP3 */

	/* Make GPIO 131, 132 and 133 as output pin */
	writel(readl(&gpio5_base->oe) & ~(GPIO3|GPIO4|GPIO5), &gpio5_base->oe);
	writel(GPIO4, &gpio5_base->cleardataout);
	writel(GPIO5, &gpio5_base->cleardataout);
}

/*
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 */
int misc_init_r(void)
{
	/* enable LCD and USB; disable HDMI */
	set_baseboard_gpios();
	
	/* initialize display controller */
#ifdef CONFIG_VIDEO_OMAP3	
	display_init();
#endif /* CONFIG_VIDEO_OMAP3 */
	
#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif

#if defined(CONFIG_CMD_NET)
	setup_net_chip();
#endif

	/* enable display output */
#ifdef CONFIG_VIDEO_OMAP3
	omap3_dss_enable();
#endif /* CONFIG_VIDEO_OMAP3 */

	return 0;
}

/*
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers specific to the
 *		hardware. Many pins need to be moved from protect to primary
 *		mode.
 */
void set_muxconf_regs(void)
{
	MUX_EVM();
}

/*
 * Routine: setup_net_chip
 * Description: Setting up the configuration GPMC registers specific to the
 *		Ethernet hardware.
 */
static void setup_net_chip(void)
{
	struct gpio *gpio5_base = (struct gpio *)OMAP34XX_GPIO5_BASE;
	struct ctrl *ctrl_base = (struct ctrl *)OMAP34XX_CTRL_BASE;

	/* Configure GPMC registers */
	writel(NET_GPMC_CONFIG1, &gpmc_cfg->cs[5].config1);
	writel(NET_GPMC_CONFIG2, &gpmc_cfg->cs[5].config2);
	writel(NET_GPMC_CONFIG3, &gpmc_cfg->cs[5].config3);
	writel(NET_GPMC_CONFIG4, &gpmc_cfg->cs[5].config4);
	writel(NET_GPMC_CONFIG5, &gpmc_cfg->cs[5].config5);
	writel(NET_GPMC_CONFIG6, &gpmc_cfg->cs[5].config6);
	writel(NET_GPMC_CONFIG7, &gpmc_cfg->cs[5].config7);

	/* Enable off mode for NWE in PADCONF_GPMC_NWE register */
	writew(readw(&ctrl_base ->gpmc_nwe) | 0x0E00, &ctrl_base->gpmc_nwe);
	/* Enable off mode for NOE in PADCONF_GPMC_NADV_ALE register */
	writew(readw(&ctrl_base->gpmc_noe) | 0x0E00, &ctrl_base->gpmc_noe);
	/* Enable off mode for ALE in PADCONF_GPMC_NADV_ALE register */
	writew(readw(&ctrl_base->gpmc_nadv_ale) | 0x0E00,
		&ctrl_base->gpmc_nadv_ale);

	/* Make GPIO 137 as output pin */
	writel(readl(&gpio5_base->oe) & ~(GPIO9), &gpio5_base->oe);

	/* Now send a pulse on the GPIO pin */
	writel(GPIO9, &gpio5_base->setdataout);
	udelay(1);
	writel(GPIO9, &gpio5_base->cleardataout);
	udelay(1);
	writel(GPIO9, &gpio5_base->setdataout);
}

int board_eth_init(bd_t *bis)
{
	int rc = 0;

#ifdef CONFIG_SMC911X
	rc = smc911x_initialize(0, CONFIG_SMC911X_BASE);
#endif
	return rc;
}
