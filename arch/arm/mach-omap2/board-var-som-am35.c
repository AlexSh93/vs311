/*
 * linux/arch/arm/mach-omap2/board-var-som-am35.c
 *
 * Copyright (C) 2010,2011 Variscite LTD.
 * Author: Alex Bikhdriker <alex@variscite.com>
 * Modified: Vladik Goytin
 *
 * Based on mach-omap2/board-am3517evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/mmc/host.h>
#include <linux/etherdevice.h>
#include <linux/davinci_emac.h>
#include <linux/regulator/machine.h>
#include <linux/can/platform/ti_hecc.h>

#include <linux/spi/spi.h>
#include <plat/mcspi.h>
#include <linux/spi/ads7846.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#include "mux.h"
#include "control.h"
#include "hsmmc.h"
#include "board-flash.h"

// CustomBoard ver 2.1
#define CUSTOM_VER_2_1 


#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE        SZ_128K

/* GPIOs Definitions */



// LCD
#ifndef CUSTOM_VER_2_1
// LCD Power and level shifters are always on at CustomBoard v2.1
#define LCD_LVL_SFHT_BUF_ENn	43
#define LCD_PWR_ENn		131
#endif

#define LCD_BKLIGHT_EN		186

#ifdef CUSTOM_VER_2_1
#define HDMI_TRCVR_PDn		43
#else
#define HDMI_TRCVR_PDn		133

#endif


// USB
#ifdef CUSTOM_VER_2_1
#define USB_HOST_PWR_EN		98
#else
#define USB_HOST_PWR_EN		132
#endif

#define USB_PHY1_RESET		154
#define USB_PHY2_RESET		152


// SD-Card
#define SD_CARD_CD		61
#define SD_CARD_WP		65



static struct mtd_partition var_am35_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "xloader-nand",
		.offset		= 0,
		.size		= 4*(SZ_128K),
		.mask_flags	= MTD_WRITEABLE
	},
	{
		.name		= "uboot-nand",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 14*(SZ_128K),
		.mask_flags	= MTD_WRITEABLE
	},
	{
		.name		= "params-nand",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2*(SZ_128K)
	},
	{
		.name		= "linux-nand",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 40*(SZ_128K)
	},
	{
		.name		= "jffs2-nand",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,
	},
};


/* 
 * Ethernet Controller
 */

#define AM35XX_PHY_MASK		(0xF)
#define AM35XX_MDIO_FREQUENCY   (1000000)

static struct mdio_platform_data am3517_evm_mdio_pdata = {
	.bus_freq	= AM35XX_MDIO_FREQUENCY,
};

static struct resource am3517_mdio_resources[] = {
	{
		.start	= AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET,
		.end	= AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET +
			  SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device am3517_mdio_device = {
	.name		= "davinci_mdio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_mdio_resources),
	.resource	= am3517_mdio_resources,
	.dev.platform_data = &am3517_evm_mdio_pdata,
};


static struct emac_platform_data am3517_emac_pdata = {
	.rmii_en        = 1,
};

static int __init eth_addr_setup(char *str)
{
	int i;

	if(str == NULL)
		return 0;
	for(i = 0; i <  ETH_ALEN; i++)
		am3517_emac_pdata.mac_addr[i] = simple_strtol(&str[i*3],
							(char **)NULL, 16);
	return 1;
}

/* Get MAC address from kernel boot parameter eth=AA:BB:CC:DD:EE:FF */
__setup("eth=", eth_addr_setup);

static struct resource am3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x3FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_emac_device = {
	.name           = "davinci_emac",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(am3517_emac_resources),
	.resource       = am3517_emac_resources,
};

static void am3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR | AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
		AM35XX_CPGMAC_C0_RX_THRESH_CLR );
	omap_ctrl_writel(regval,AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval,AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}


static const uint8_t zero_mac[ETH_ALEN];
static const uint8_t var_mac[ETH_ALEN] = { 0xF8, 0xDC, 0x7A, 0x00, 0x00, 0x00 };
void am3517_ethernet_init(struct emac_platform_data *pdata)
{
	unsigned int regval;

	pdata->ctrl_reg_offset		= AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset	= AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset		= AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->ctrl_ram_size		= AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version			= EMAC_VERSION_2;
	pdata->hw_ram_addr		= AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable 	= am3517_enable_ethernet_int;
	pdata->interrupt_disable	= am3517_disable_ethernet_int;
	am3517_emac_device.dev.platform_data	= pdata;

	if ( !memcmp(pdata->mac_addr, zero_mac, ETH_ALEN) ) {
		memcpy(pdata->mac_addr, var_mac, ETH_ALEN);
		printk(KERN_ERR "error: no MAC address on cmd. line, set to F8:DC:7A:00:00:00\n");
	}

	platform_device_register(&am3517_mdio_device);
	platform_device_register(&am3517_emac_device);
	clk_add_alias(NULL, dev_name(&am3517_mdio_device.dev),
		NULL, &am3517_emac_device.dev);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval,AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}




static int lcd_enabled;
static int dvi_enabled;

static void __init am3517_display_init(void)
{
#ifdef CONFIG_PANEL_VARISCITE
	int r;

	/* disable LCD backlight */
	r = gpio_request(LCD_BKLIGHT_EN, "LCD_BKLIGHT_EN");
	if (r) {
		printk(KERN_ERR "failed to get LCD_BKLIGHT_EN gpio\n");
		goto err_1;
	}
	omap_mux_init_gpio(LCD_BKLIGHT_EN, OMAP_PIN_OUTPUT);
	gpio_direction_output(LCD_BKLIGHT_EN, 1);
	gpio_set_value(LCD_BKLIGHT_EN, 0);

#ifndef CUSTOM_VER_2_1
// LCD power and level shifter are always on at CustomBoard v2.1

	/* Enable VIO-> 3.3v level shifter */
	r = gpio_request(LCD_LVL_SFHT_BUF_ENn, "LCD_LVL_SFHT_BUF_ENn");
	if (r) {
		printk(KERN_ERR "failed to get LCD_LVL_SFHT_BUF_ENn gpio\n");
		goto err_2;
	}
	omap_mux_init_gpio(LCD_LVL_SFHT_BUF_ENn, OMAP_PIN_OUTPUT);
	gpio_direction_output(LCD_LVL_SFHT_BUF_ENn, 1);
	gpio_set_value(LCD_LVL_SFHT_BUF_ENn, 0);
	
	/* Enable LCD panel VCC */

	r = gpio_request(LCD_PWR_ENn, "LCD_PWR_ENn");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PWR_ENn\n");
		goto err_3;
	}

	gpio_direction_output(LCD_PWR_ENn, 1);
	gpio_set_value(LCD_PWR_ENn, 0);
#endif

	/* Disable HDMI transceiver */
	r = gpio_request(HDMI_TRCVR_PDn, "HDMI_TRCVR_PDn");
	if (r) {
		printk(KERN_ERR "failed to get HDMI_TRCVR_PDn\n");
		goto err_4;
	}
	omap_mux_init_gpio(HDMI_TRCVR_PDn, OMAP_PIN_OUTPUT);
	gpio_direction_output(HDMI_TRCVR_PDn, 1);
	gpio_set_value(HDMI_TRCVR_PDn, 0);

	return;

err_4:
	gpio_free(HDMI_TRCVR_PDn);
#ifndef CUSTOM_VER_2_1
err_3:
	gpio_free(LCD_LVL_SFHT_BUF_ENn);
err_2:
	gpio_free(LCD_PWR_ENn);
#endif
err_1:
	gpio_free(LCD_BKLIGHT_EN);
#endif /* CONFIG_PANEL_VARISCITE */
}

static int am3517_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}

	gpio_set_value(LCD_BKLIGHT_EN, 1);
	lcd_enabled = 1;

	return 0;
}

static void am3517_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_BKLIGHT_EN, 0);
	lcd_enabled = 0;
}

static struct omap_dss_device am3517_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "URT_UMSH8272MD",
	.phy.dpi.data_lines 	= 24,
	.platform_enable	= am3517_panel_enable_lcd,
	.platform_disable	= am3517_panel_disable_lcd,
};

/* 
 * TV Output
 */

static int am3517_panel_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void am3517_panel_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device am3517_tv_device = {
	.type 			= OMAP_DISPLAY_TYPE_VENC,
	.name 			= "tv",
	.driver_name		= "venc",
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable	= am3517_panel_enable_tv,
	.platform_disable	= am3517_panel_disable_tv,
};

/* 
 * DVI/HDMI Output
 */

static int am3517_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}
	dvi_enabled = 1;
	gpio_set_value(HDMI_TRCVR_PDn, 1);
	return 0;
}

static void am3517_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	dvi_enabled = 0;
	gpio_set_value(HDMI_TRCVR_PDn, 0);
}

static struct omap_dss_device am3517_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.phy.dpi.data_lines	= 24,
	.platform_enable	= am3517_panel_enable_dvi,
	.platform_disable	= am3517_panel_disable_dvi,
};

static struct omap_dss_device *am3517_dss_devices[] = {
	&am3517_lcd_device,
	&am3517_tv_device,
	&am3517_dvi_device,
};

static struct omap_dss_board_info am3517_dss_data = {
	.num_devices	= ARRAY_SIZE(am3517_dss_devices),
	.devices	= am3517_dss_devices,
	.default_device	= &am3517_lcd_device,
};

struct platform_device am3517_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data	= &am3517_dss_data,
	},
};


/*
 * Touch Panel: ADS 7846
 */
#ifdef CONFIG_TOUCHSCREEN_ADS7846

#define ADS7846_PENDOWN_GPIO	57

static void ads7846_dev_init(void)
{
	if (gpio_request(ADS7846_PENDOWN_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

	gpio_direction_input(ADS7846_PENDOWN_GPIO);

	gpio_set_debounce(ADS7846_PENDOWN_GPIO, 1);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(ADS7846_PENDOWN_GPIO);
}

struct ads7846_platform_data ads7846_config = {
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
	.wakeup			= true,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

struct spi_board_info var_omap3530_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(ADS7846_PENDOWN_GPIO),
		.platform_data		= &ads7846_config,
	},
};
#endif /* CONFIG_TOUCHSCREEN_ADS7846 */


static struct i2c_board_info __initdata var_am35_i2c1_boardinfo[] = {
	/* Audio Codec */
	{
		I2C_BOARD_INFO("tlv320aic23", 0x1A),
	},
	/* RTC */
	{
		I2C_BOARD_INFO("ds1307", 0x68), /* 0xD0 */
		.type		= "ds1307",
	},
};




static int __init am3517_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, NULL,0);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);

	return 0;
}

/*
 * HECC information
 */

#define CAN_STB		214
static void hecc_phy_control(int on)
{
	int r;

	r = gpio_request(CAN_STB, "can_stb");
	if (r) {
		printk(KERN_ERR "failed to get can_stb \n");
		return;
	}

	gpio_direction_output(CAN_STB, (on==1)?0:1);
}


static struct resource am3517_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_35XX_HECC0_IRQ,
		.end	= INT_35XX_HECC0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_hecc_device = {
	.name			= "ti_hecc",
	.id			= -1,	/* FIXME: it was +1 in the previous version */
	.num_resources		= ARRAY_SIZE(am3517_hecc_resources),
	.resource		= am3517_hecc_resources,
};

static struct ti_hecc_platform_data am3517_hecc_pdata = {
	.scc_hecc_offset	= AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset		= AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset	= AM35XX_HECC_RAM_OFFSET,
	.mbx_offset		= AM35XX_HECC_MBOX_OFFSET,
	.int_line		= AM35XX_HECC_INT_LINE,
	.version		= AM35XX_HECC_VERSION,
	.transceiver_switch	= hecc_phy_control
};

static void am3517_hecc_init(struct ti_hecc_platform_data *pdata)
{
        am3517_hecc_device.dev.platform_data = pdata;
        platform_device_register(&am3517_hecc_device);
}


/*
 * Board initialization
 */
static struct omap_board_config_kernel am3517_config[] __initdata = {
};

static struct platform_device *am3517_devices[] __initdata = {
	&am3517_dss_device,
};

static void __init am3517_init_irq(void)
{
	omap_board_config = am3517_config;
	omap_board_config_size = ARRAY_SIZE(am3517_config);

	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	.mode		= MUSB_OTG,
	.power		= 500,
};

static __init void am3517_musb_init(void)
{
	u32 devconf2;

	/*
	 * Set up USB clock/mode in the DEVCONF2 register.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&musb_board_data);
}


static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY, 
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = USB_PHY1_RESET,
	.reset_gpio_port[1]  = USB_PHY2_RESET,		
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
#ifndef CUSTOM_VER_2_1
	OMAP3_MUX(SDMMC2_CMD, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
#endif

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= SD_CARD_CD,
		.gpio_wp	= SD_CARD_WP
	},
	{}      /* Terminator */
};

static void __init am3517_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	am3517_i2c_init();
	platform_add_devices(am3517_devices, ARRAY_SIZE(am3517_devices));

	omap_serial_init();

	am3517_musb_init();

	/* Configure EHCI ports */
	omap_mux_init_gpio(USB_HOST_PWR_EN, OMAP_PIN_OUTPUT);
	gpio_request(USB_HOST_PWR_EN, "USB_HOST_PWR_EN");
	gpio_direction_output(USB_HOST_PWR_EN, 1);
	gpio_set_value(USB_HOST_PWR_EN, 1);

	omap_mux_init_gpio(USB_PHY1_RESET, OMAP_PIN_OUTPUT); // ECHI port 1 reset
	omap_mux_init_gpio(USB_PHY2_RESET, OMAP_PIN_OUTPUT); // ECHI port 2 reset

	usb_ehci_init(&ehci_pdata);

	am3517_display_init();

	board_nand_init(var_am35_nand_partitions,
			ARRAY_SIZE(var_am35_nand_partitions),
			0, NAND_BUSWIDTH_16);

#ifdef CONFIG_TOUCHSCREEN_ADS7846
	/* register SPI interface */
	spi_register_board_info(var_omap3530_spi_board_info,
				ARRAY_SIZE(var_omap3530_spi_board_info));

	/* configure  GPIO for pendown interrupt */
	omap_mux_init_gpio(ADS7846_PENDOWN_GPIO, OMAP_PIN_INPUT_PULLUP);
	/* ADS7846 init function */
	ads7846_dev_init();
#endif /* CONFIG_TOUCHSCREEN_ADS7846 */

	i2c_register_board_info(1, var_am35_i2c1_boardinfo, ARRAY_SIZE(var_am35_i2c1_boardinfo));

	/*Ethernet*/
	am3517_ethernet_init(&am3517_emac_pdata);

	am3517_hecc_init(&am3517_hecc_pdata); // available on CustomBoard V2.0 and forth

	//am3517_musb_init();

	/* MMC init */
	omap_mux_init_gpio(SD_CARD_CD, OMAP_PIN_INPUT);
	omap_mux_init_gpio(SD_CARD_WP, OMAP_PIN_INPUT);
	omap2_hsmmc_init(mmc);
}


MACHINE_START(OMAP3517EVM, "VARISCITE VAR-SOM-AM35")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= am3517_init_irq,
	.init_machine	= am3517_init,
	.timer		= &omap_timer,
MACHINE_END
