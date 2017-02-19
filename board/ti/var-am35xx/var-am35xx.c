/*
 * var-am35xx.c - board file for Variscite VAR-AM35xx SOM.
 *
 * Author: Alex Bikhdriker <alex@variscite.com>
 *
 * Based on ti/am3517evm/am3517evm.c
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

#include <common.h>
#include <netdev.h>
#include <asm/io.h>
#include <asm/arch/mem.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/emac_defs.h>
#include <asm/arch/gpio.h>
#include <i2c.h>
#include <asm/mach-types.h>
#include "var-am35xx.h"


#define AM3517_IP_SW_RESET	0x48002598

#define CPGMACSS_SW_RST		(1 << 1)



/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */
	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_OMAP3517EVM;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

	return 0;
}

/* splash screen row data */
#ifdef CONFIG_VIDEO_SPLASH
#include "var_16_splash.c"
#endif /* CONFIG_VIDEO_SPLASH */


#define LCD_PWR_ENn		131
#define LCD_BKLIGHT_EN		186
#define LCD_LVL_SFHT_BUF_ENn	43
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
	if (is_cpu_family(CPU_OMAP36XX))
		lcd_cfg.divisor = 0x00010003;
	omap3_dss_panel_config(&lcd_cfg);

#if defined(CONFIG_VIDEO_SPLASH)
	omap3_fill_framebuffer(FRAME_BUFFER_ADDR, splash_data, splash_data_size);
#elif defined(CONFIG_VIDEO_BACKGROUND)
	omap3_dss_set_background_col(SPLASH_SOLID_COLOR);
#endif	
}
#endif /* CONFIG_VIDEO_OMAP3 */

/*
 * Routine: misc_init_r
 * Description: Init ethernet (done here so udelay works)
 */
int misc_init_r(void)
{
	u32 reset;



	/* initialize display controller */
#ifdef CONFIG_VIDEO_OMAP3	
	display_init();
#endif /* CONFIG_VIDEO_OMAP3 */

#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif

	dieid_num_r();

	omap_request_gpio(2);
	omap_set_gpio_direction(2,1);

	omap_request_gpio(3);
	omap_set_gpio_direction(3,1);

	omap_request_gpio(4);
	omap_set_gpio_direction(4,1);

	omap_request_gpio(5);
	omap_set_gpio_direction(5,1);

	omap_request_gpio(6);
	omap_set_gpio_direction(6,1);

	omap_request_gpio(7);
	omap_set_gpio_direction(7,1);

	omap_request_gpio(8);
	omap_set_gpio_direction(8,1);

	omap_request_gpio(30);
	omap_set_gpio_direction(30,1);


// proper initializatoin of i2c1 for proper TPS operation.
	I2C_SET_BUS(1);
	i2c_probe(0x48);

#if defined(CONFIG_DRIVER_TI_EMAC)


	/*ensure that the module is out of reset*/
	reset = readl(AM3517_IP_SW_RESET);
	reset &= (~CPGMACSS_SW_RST);
	writel(reset,AM3517_IP_SW_RESET);

#endif


#ifdef CONFIG_VIDEO_OMAP3
	/* enable DSS output */
	omap_request_gpio(LCD_LVL_SFHT_BUF_ENn);
	omap_set_gpio_direction(LCD_LVL_SFHT_BUF_ENn,0);
	omap_set_gpio_dataout(LCD_LVL_SFHT_BUF_ENn,0);

	/* enable LCD VCC*/
	omap_request_gpio(LCD_PWR_ENn);
	omap_set_gpio_direction(LCD_PWR_ENn,0);
	omap_set_gpio_dataout(LCD_PWR_ENn,0);

	/* enable display output */
	omap3_dss_enable();

	/* enable LCD backlight */
	omap_request_gpio(LCD_BKLIGHT_EN);
	omap_set_gpio_direction(LCD_BKLIGHT_EN,0);
	omap_set_gpio_dataout(LCD_BKLIGHT_EN,1);
#endif /* CONFIG_VIDEO_OMAP3 */

	return 0;
}


/*
 * Initializes on-chip ethernet controllers.
 * to override, implement board_eth_init()
 */
int cpu_eth_init(bd_t *bis)
{
#if defined(CONFIG_DRIVER_TI_EMAC)
	printf("davinci_emac_initialize\n");
	davinci_emac_initialize();
#endif
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
	MUX_VAR_AM35XX();
}
