# var-som-2.6.37

 /-----------------uImage--------------------/
 * export PATH=<path to U-boot>/tools:$PATH
 * make ARCH=arm var-som-om3x_defconfig
 * make ARCH=arm menuconfig
 * make ARCH=arm -j4 CROSS_COMPILE=arm-none-linux-gnueabi- uImage

/*не забыть*/
 * export PATH=/opt/toolchain/arm-2009q1/bin:$PATH
 * export PATH=/home/alex/Desktop/for-omap/toolchain/arm-2009q1/bin:$PATH
 * export PATH=/home/alex/Desktop/for-omap/uboot/u-boot-2009.11-psp03.00.01.06.03.sdk/tools:$PATH
