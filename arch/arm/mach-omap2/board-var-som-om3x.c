/*
 * linux/arch/arm/mach-omap2/board-var-som-om3x.c.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds.h>
#include <linux/interrupt.h>
#include <linux/mtd/nand.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl.h>
#include <linux/usb/otg.h>
#include <linux/smsc911x.h>

#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mmc/host.h>

#ifdef CONFIG_WL12XX_PLATFORM_DATA
#include <linux/wl12xx.h>
#include <linux/regulator/fixed.h>
#endif
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/usb.h>
#include <plat/common.h>
#include <plat/mcspi.h>
#include <plat/display.h>
#include <plat/omap-pm.h>
#include <plat/gpmc.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "sdram-hynix-h8kds0un0mer-4em.h"
#include "hsmmc.h"
#include "board-flash.h"
#ifdef CONFIG_SND_SOC_WL1271BT
#include "control.h"
#endif
#define NAND_BLOCK_SIZE		SZ_128K

// CustomBoard v2.1 and forth
#define VER_2_1

/* FIXME: add VAR-OM-3x configuration: touchscreen on/off and
 * list of supported touchscreens. Define symbol VAR_TS. Then
 * protect all touchscreen changes by symbol CONFIG_VAR_TS instead of
 * CONFIG_INPUT_TOUCHSCREEN.
 * Vladik, 6.07.2011
 */
#ifdef CONFIG_INPUT_TOUCHSCREEN
#ifdef CONFIG_TOUCHSCREEN_ADS7846
#define VAR_SOM_OM3X_TS_GPIO	177
#else
#error Unsupported touchscreen device.
#endif /* CONFIG_TOUCHSCREEN_ADS7846 */
#endif /* CONFIG_INPUT_TOUCHSCREEN */

#define VAR_SOM_OM3X_ETHR_GPIO_IRQ	29
#define VAR_SOM_OM3X_ETHR_START		0x2c000000
#define VAR_SOM_OM3X_ETHR_SIZE		1024
#define VAR_SOM_OM3X_ETHR_ID_REV	0x50
#define VAR_SOM_OM3X_SMSC911X_CS	5
#define VAR_SOM_OM3X_WLAN_PMENA_GPIO	11
#define VAR_SOM_OM3X_WLAN_IRQ_GPIO	25

/*
 * VAR_SOM_OM3X LCD Panel control signals
 */

#define VAR_SOM_OM3X_LCD_PANEL_BKLIGHT_GPIO	28

#ifdef VER_2_1
#define VAR_SOM_OM3X_DVI_PANEL_EN_GPIO		26
#define VAR_SOM_OM3X_USB3_PWR_ENn		167
#else
#define VAR_SOM_OM3X_DVI_PANEL_EN_GPIO		133
#define VAR_SOM_OM3X_BUFFER_DISABLE_GPIO	26
#define VAR_SOM_OM3X_LCD_PANEL_ENVDD		131
#endif


#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource VAR_SOM_OM3X_smsc911x_resources[] = {
	[0] =	{
		.start	= VAR_SOM_OM3X_ETHR_START,
		.end	= (VAR_SOM_OM3X_ETHR_START + VAR_SOM_OM3X_ETHR_SIZE - 1),
		.flags	= IORESOURCE_MEM,
	},
	[1] =	{
		.start	= OMAP_GPIO_IRQ(VAR_SOM_OM3X_ETHR_GPIO_IRQ),
		.end	= OMAP_GPIO_IRQ(VAR_SOM_OM3X_ETHR_GPIO_IRQ),
		.flags	= (IORESOURCE_IRQ | IRQF_TRIGGER_LOW),
 	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface  = PHY_INTERFACE_MODE_MII,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags          = (SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS),
};

static struct platform_device VAR_SOM_OM3X_smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(VAR_SOM_OM3X_smsc911x_resources),
	.resource	= &VAR_SOM_OM3X_smsc911x_resources[0],
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static inline void __init VAR_SOM_OM3X_init_smsc911x(void)
{
	int eth_cs;
	struct clk *l3ck;
	unsigned int rate;

	eth_cs = VAR_SOM_OM3X_SMSC911X_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);
	platform_device_register(&VAR_SOM_OM3X_smsc911x_device);
}

#else
static inline void __init VAR_SOM_OM3X_init_smsc911x(void) { return; }
#endif




#ifdef CONFIG_PANEL_VARISCITE
static int lcd_enabled;
#endif
static int dvi_enabled;


static void __init VAR_SOM_OM3X_display_init(void)
{
	int r;
#ifndef VER_2_1
// LCD Power and level shifter are always on in CustomBoard v2.1
	r = gpio_request(VAR_SOM_OM3X_LCD_PANEL_ENVDD, "lcd_panel_envdd");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_envdd\n");
		return;
	}
	gpio_direction_output(VAR_SOM_OM3X_LCD_PANEL_ENVDD, 0);
	gpio_set_value(VAR_SOM_OM3X_LCD_PANEL_ENVDD, 0);

	r = gpio_request(VAR_SOM_OM3X_BUFFER_DISABLE_GPIO, "lcd_panel_envdd");
	if (r) {
		printk(KERN_ERR "failed to get voltage buffer gpio\n");
		return;
	}
	gpio_direction_output(VAR_SOM_OM3X_BUFFER_DISABLE_GPIO, 0);

	gpio_set_value(VAR_SOM_OM3X_BUFFER_DISABLE_GPIO, 0);
#endif
	omap_mux_init_gpio(VAR_SOM_OM3X_DVI_PANEL_EN_GPIO, OMAP_PIN_INPUT_PULLUP);
	r = gpio_request(VAR_SOM_OM3X_DVI_PANEL_EN_GPIO, "lcd_panel_envdd");
	if (r) {
		printk(KERN_ERR "failed to get dvi_panel_env\n");
		return;
	}

	gpio_direction_output(VAR_SOM_OM3X_DVI_PANEL_EN_GPIO, 0);
	

	gpio_set_value(VAR_SOM_OM3X_LCD_PANEL_BKLIGHT_GPIO, 0);
	gpio_set_value(VAR_SOM_OM3X_DVI_PANEL_EN_GPIO, 0);
}

static int VAR_SOM_OM3X_enable_lcd(struct omap_dss_device *dssdev)
{
	printk(KERN_DEBUG "VAR_SOM_OM3X_enable_lcd\n");
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}

	gpio_set_value(VAR_SOM_OM3X_LCD_PANEL_BKLIGHT_GPIO, 1);
	lcd_enabled = 1;
	return 0;
}

static void VAR_SOM_OM3X_disable_lcd(struct omap_dss_device *dssdev)
{
	printk(KERN_DEBUG "VAR_SOM_OM3X_disable_lcd\n");
	gpio_set_value(VAR_SOM_OM3X_LCD_PANEL_BKLIGHT_GPIO, 0);
	lcd_enabled = 0;
}

static struct omap_dss_device VAR_SOM_OM3X_lcd_device = {
	.name			= "lcd",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.driver_name		= "URT_UMSH8272MD",

	.phy.dpi.data_lines	= 24,
	.max_backlight_level	= 100,
	.platform_enable	= VAR_SOM_OM3X_enable_lcd,
	.platform_disable	= VAR_SOM_OM3X_disable_lcd,
};



static int VAR_SOM_OM3X_enable_dvi(struct omap_dss_device *dssdev)
{
#ifdef CONFIG_PANEL_VARISCITE
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}
#endif

	gpio_set_value(VAR_SOM_OM3X_DVI_PANEL_EN_GPIO, 1);
	dvi_enabled = 1;

	return 0;
}

static void VAR_SOM_OM3X_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(VAR_SOM_OM3X_DVI_PANEL_EN_GPIO, 0);

	dvi_enabled = 0;
}

static struct omap_dss_device VAR_SOM_OM3X_dvi_device = {
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= VAR_SOM_OM3X_enable_dvi,
	.platform_disable	= VAR_SOM_OM3X_disable_dvi,
};

static struct omap_dss_device *VAR_SOM_OM3X_dss_devices[] = {
#ifdef CONFIG_PANEL_VARISCITE
	&VAR_SOM_OM3X_lcd_device,
#endif
	&VAR_SOM_OM3X_dvi_device,
};

static struct omap_dss_board_info VAR_SOM_OM3X_dss_data = {
	.num_devices	= ARRAY_SIZE(VAR_SOM_OM3X_dss_devices),
	.devices	= VAR_SOM_OM3X_dss_devices,
#ifdef CONFIG_PANEL_VARISCITE
	.default_device	= &VAR_SOM_OM3X_lcd_device,
#else
	.default_device	= &VAR_SOM_OM3X_dvi_device,
#endif
};

static struct platform_device VAR_SOM_OM3X_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &VAR_SOM_OM3X_dss_data,
	},
};






static struct regulator_consumer_supply VAR_SOM_OM3X_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply VAR_SOM_OM3X_vsim_supply = {
	.supply			= "vmmc_aux",
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data VAR_SOM_OM3X_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &VAR_SOM_OM3X_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data VAR_SOM_OM3X_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &VAR_SOM_OM3X_vsim_supply,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= 65,
	},
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	{
	.name           = "wl1271",
	.mmc            = 2,
	.caps           = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
	.gpio_wp        = -EINVAL,
	.gpio_cd        = -EINVAL,
	.nonremovable   = true,
	},
#endif
	{}	/* Terminator */
};


static int VAR_SOM_OM3X_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	omap_mux_init_gpio(65, OMAP_PIN_INPUT);
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters */
	VAR_SOM_OM3X_vmmc1_supply.dev = mmc[0].dev;
	VAR_SOM_OM3X_vsim_supply.dev = mmc[0].dev;

	return 0;
}

static struct twl4030_gpio_platform_data VAR_SOM_OM3X_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= VAR_SOM_OM3X_twl_gpio_setup,
};

static struct twl4030_usb_data VAR_SOM_OM3X_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};



static struct twl4030_madc_platform_data VAR_SOM_OM3X_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data VAR_SOM_OM3X_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data VAR_SOM_OM3X_codec_data = {
	.audio_mclk = 26000000,
	.audio = &VAR_SOM_OM3X_audio_data,
};

static struct regulator_consumer_supply VAR_SOM_OM3X_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &VAR_SOM_OM3X_dss_device.dev,
};

/* VDAC for DSS driving S-Video */
static struct regulator_init_data VAR_SOM_OM3X_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &VAR_SOM_OM3X_vdda_dac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply VAR_SOM_OM3X_vpll2_supply =
	REGULATOR_SUPPLY("vdds_dsi", "omapdss");

static struct regulator_init_data VAR_SOM_OM3X_vpll2 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &VAR_SOM_OM3X_vpll2_supply,
};

/* VAUX2 for USB */
static struct regulator_consumer_supply VAR_SOM_OM3X_vaux2_supplies = {
	.supply		= "hsusb1",
};

static struct regulator_init_data VAR_SOM_OM3X_vaux2 = {
	.constraints = {
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= 1,
	.consumer_supplies		= &VAR_SOM_OM3X_vaux2_supplies,
};


/* VIO_1V8 is required for diff modules: ads7846 on SPI, pull-ups, etc... */
static struct regulator_consumer_supply VAR_SOM_OM3X_vio_supply[] = {
	REGULATOR_SUPPLY("vcc", "spi1.0"),
	REGULATOR_SUPPLY("vio_1v8", NULL),
};

static struct regulator_init_data VAR_SOM_OM3X_vio = {
	.constraints = {
		.min_uV                 = 1800000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		.valid_ops_mask         = REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(VAR_SOM_OM3X_vio_supply),
	.consumer_supplies      = VAR_SOM_OM3X_vio_supply,
};

static struct regulator_consumer_supply var_som_om44_vmmc2_supply =
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1");

/* VMMC2 for driving the WL12xx module */
static struct regulator_init_data var_som_om44_vmmc2 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &var_som_om44_vmmc2_supply
};

static struct fixed_voltage_config var_som_om44_vwlan = {
	.supply_name            = "vwl1271",
	.microvolts             = 1800000, /* 1.80V */
	.gpio                   = VAR_SOM_OM3X_WLAN_PMENA_GPIO,
	.startup_delay          = 70000, /* 70ms */
	.enable_high            = 1,
	.enabled_at_boot        = 0,
	.init_data              = &var_som_om44_vmmc2,
};

/*static struct platform_device omap3evm_wlan_regulator = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data  = &var_som_om44_vwlan,
	},
};

struct wl12xx_platform_data omap3evm_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(VAR_SOM_OM3X_WLAN_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_26,  //26 MHz 
};*/

static struct twl4030_platform_data VAR_SOM_OM3X_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.madc		= &VAR_SOM_OM3X_madc_data,
	.usb		= &VAR_SOM_OM3X_usb_data,
	.gpio		= &VAR_SOM_OM3X_gpio_data,
	.codec		= &VAR_SOM_OM3X_codec_data,
	.vdac		= &VAR_SOM_OM3X_vdac,
	.vpll2		= &VAR_SOM_OM3X_vpll2,
	.vaux2          = &VAR_SOM_OM3X_vaux2,
	.vio		= &VAR_SOM_OM3X_vio,
};

static struct i2c_board_info __initdata VAR_SOM_OM3X_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &VAR_SOM_OM3X_twldata,
	},
};

static int __init VAR_SOM_OM3X_i2c_init(void)
{
	/*
	 * REVISIT: These entries can be set in omap3evm_twl_data
	 * after a merge with MFD tree
	 */
	VAR_SOM_OM3X_twldata.vmmc1 = &VAR_SOM_OM3X_vmmc1;
	VAR_SOM_OM3X_twldata.vsim = &VAR_SOM_OM3X_vsim;

	omap_register_i2c_bus(1, 2600, VAR_SOM_OM3X_i2c_boardinfo,
			ARRAY_SIZE(VAR_SOM_OM3X_i2c_boardinfo));
	/* Bus 2 is used for Camera/Sensor interface */
	omap_register_i2c_bus(2, 400, NULL, 0);

	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

#ifdef CONFIG_INPUT_TOUCHSCREEN
#ifdef CONFIG_TOUCHSCREEN_ADS7846
static void ads7846_dev_init(void)
{
	if (gpio_request(VAR_SOM_OM3X_TS_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

	gpio_direction_input(VAR_SOM_OM3X_TS_GPIO);
	gpio_set_debounce(VAR_SOM_OM3X_TS_GPIO, 310);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(VAR_SOM_OM3X_TS_GPIO);
}

static struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 10,
	.debounce_tol		= 3,
	.debounce_rep		= 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.settle_delay_usecs	= 150,
	.wakeup				= true,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

static struct spi_board_info VAR_SOM_OM3X_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(VAR_SOM_OM3X_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};
#endif /* CONFIG_TOUCHSCREEN_ADS7846 */
#endif /* CONFIG_INPUT_TOUCHSCREEN */

static struct spi_board_info mcspi_board_info[] = {
	// spi 4.0
	{
		.modalias	= "spidev",
		.max_speed_hz	= 48000000, //48 Mbps
		.bus_num	= 2,
		.chip_select	= 0,
	},
};

static struct omap_board_config_kernel VAR_SOM_OM3X_config[] __initdata = {
};

static void __init VAR_SOM_OM3X_init_irq(void)
{
	omap_board_config = VAR_SOM_OM3X_config;
	omap_board_config_size = ARRAY_SIZE(VAR_SOM_OM3X_config);
	omap2_init_common_infrastructure();

	omap2_init_common_devices(mt46h32m32lf6_sdrc_params, NULL);

	omap_init_irq();
	gpmc_init();
}

#ifdef CONFIG_SND_SOC_WL1271BT
/* WL1271 Audio */
static struct platform_device wl1271bt_audio_device = {
	.name		= "wl1271bt",
	.id		= -1,
};

static struct platform_device wl1271bt_codec_device = {
	.name		= "wl1271bt-dummy-codec",
	.id		= -1,
};

static void wl1271bt_clk_setup(void)
{
	u16 reg;
	u32 val;

	/*
	 * Set DEVCONF0 register to connect
	 * MCBSP1_CLKR -> MCBSP1_CLKX & MCBSP1_FSR -> MCBSP1_FSX
	 */
	reg = OMAP2_CONTROL_DEVCONF0;
	val = omap_ctrl_readl(reg);
	val = val | 0x18;
	omap_ctrl_writel(val, reg);
}
#endif
static struct platform_device *VAR_SOM_OM3X_devices[] __initdata = {
	&VAR_SOM_OM3X_dss_device,
};

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	/* PHY reset GPIO will be runtime programmed based on EVM version */
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};


/*
 * NAND
 */
static struct mtd_partition VAR_SOM_OM3X_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 10 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",

		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x1c0000 */
		.size		= 6 * (64 * 2048),
	},
	{
		.name		= "Kernel-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 40 * (64 * 2048),
	},
	{
		.name		= "File System - NAND",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x780000 */
	},
};

/* CUS Pkg. */
#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
#ifdef CONFIG_KEYBOARD_TWL4030
	/*OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
				OMAP_PIN_OFF_WAKEUPENABLE),*/
#endif
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WLAN IRQ - GPIO 25 */
	//OMAP3_MUX(ETK_D11, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),

	/* WLAN POWER ENABLE - GPIO 11 */
	//OMAP3_MUX(JTAG_EMU0, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* MMC2 SDIO pin muxes for WL12xx */
	/*OMAP3_MUX(SDMMC2_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),*/
#endif
#ifdef CONFIG_INPUT_TOUCHSCREEN
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	/*OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
				OMAP_PIN_OFF_WAKEUPENABLE),
	OMAP3_MUX(MCSPI1_CS3, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),  */               /* GPIO177, default touchscreen pendown */
#endif
#endif
	OMAP3_MUX(ETK_D10, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),			/* GPIO24, USB */
	OMAP3_MUX(ETK_D12, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),			/* GPIO26, voltage buffer disable */
	OMAP3_MUX(ETK_D14, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),			/* GPIO28, LCD backlight */
	OMAP3_MUX(ETK_D15, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),			/* GPIO29, ETH0 */
	OMAP3_MUX(SDMMC2_CMD, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),		/* GPIO131, LCD ENVDD */

	/* McBSP3 */
	OMAP3_MUX(MCBSP3_DX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT | OMAP_PULL_ENA |OMAP_PULL_UP ),
	OMAP3_MUX(MCBSP3_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP3_CLKX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP3_FSX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),


	/* McBSP1 */
	OMAP3_MUX(MCBSP1_CLKR, 	OMAP_MUX_MODE0 | OMAP_PIN_INPUT ),
	OMAP3_MUX(MCBSP1_FSR, 	OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_DX, 	OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT | OMAP_PULL_ENA |OMAP_PULL_UP ),
	OMAP3_MUX(MCBSP1_DR, 	OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_FSX, 	OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP1_CLKX, 	OMAP_MUX_MODE0 | OMAP_PIN_INPUT ),

	/* KEY_INT */
	OMAP3_MUX(CAM_D1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
	/* ADC_SYNC */
	OMAP3_MUX(CAM_D11, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	/* McSPI2 */
	OMAP3_MUX(MCSPI2_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT ),
	OMAP3_MUX(MCSPI2_CS0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PULL_ENA |OMAP_PULL_UP ),
	OMAP3_MUX(MCSPI2_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT ),
	

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static void __init VAR_SOM_OM3X_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CUS);

	// USB phy enable
	if (gpio_request(24,"GPIO24")<0)
		printk(KERN_ERR "Can't get GPIO24 for USB phy reset\n");
	gpio_direction_output(24, 1);
	gpio_set_value(24, 1);

	if (gpio_request(28,"GPIO24")<0)
		printk(KERN_ERR "Can't get GPIO24 for USB phy reset\n");
	gpio_direction_output(28, 1);
	gpio_set_value(28, 1);

	omap_mux_init_gpio(137, OMAP_PIN_OUTPUT);
	if (gpio_request(137,"eth_rst")<0)
		printk(KERN_ERR "Can't get 137 eth_rst\n");
	gpio_direction_output(137, 1);
	gpio_set_value(137, 1);

#ifdef VER_2_1
	omap_mux_init_gpio(VAR_SOM_OM3X_USB3_PWR_ENn, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(VAR_SOM_OM3X_USB3_PWR_ENn,"USB3_PWR_ENn")<0)
		printk(KERN_ERR "Can't get 167 for USB3 power enable\n");
	gpio_direction_output(VAR_SOM_OM3X_USB3_PWR_ENn, 1);
	gpio_set_value(VAR_SOM_OM3X_USB3_PWR_ENn, 1);
#endif


	VAR_SOM_OM3X_i2c_init();

	platform_add_devices(VAR_SOM_OM3X_devices, ARRAY_SIZE(VAR_SOM_OM3X_devices));

	/* SPI */
	spi_register_board_info( mcspi_board_info,
		ARRAY_SIZE( mcspi_board_info ));

#ifdef CONFIG_INPUT_TOUCHSCREEN
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	spi_register_board_info(VAR_SOM_OM3X_spi_board_info,
				ARRAY_SIZE(VAR_SOM_OM3X_spi_board_info));
#endif
#endif

	omap_serial_init();


	omap_mux_init_gpio(VAR_SOM_OM3X_DVI_PANEL_EN_GPIO, OMAP_PIN_INPUT_PULLUP);

	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);
#ifdef CONFIG_INPUT_TOUCHSCREEN
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	//ads7846_dev_init();
#endif
#endif
	VAR_SOM_OM3X_init_smsc911x();

	VAR_SOM_OM3X_display_init();
	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	/* NAND */
	board_nand_init(VAR_SOM_OM3X_nand_partitions,
			ARRAY_SIZE(VAR_SOM_OM3X_nand_partitions),
			0, NAND_BUSWIDTH_16);
}

MACHINE_START(OMAP3EVM, "VARISCITE VAR-SOM-OM3X")
	/* Maintainer: Syed Mohammed Khasim - Texas Instruments */
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= VAR_SOM_OM3X_init_irq,
	.init_machine	= VAR_SOM_OM3X_init,
	.timer		= &omap_timer,
MACHINE_END
