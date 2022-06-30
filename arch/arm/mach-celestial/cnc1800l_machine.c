#include <linux/clocksource.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/irqchip.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/smp.h>
#include <asm/smp_scu.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/psci.h>
#include <linux/arm-smccc.h>

#define CNC1800L_VIRT_UART0 0xfec00000

static struct map_desc cnc1800l_of_io_desc[] __initdata = {
#ifdef CONFIG_DEBUG_CELESTIAL_UART
	{
		.virtual = CNC1800L_VIRT_UART0,
		.pfn = __phys_to_pfn(0x801f1000),
		.length = SZ_4K,
		.type = MT_DEVICE,
	},
#endif
};

static void __init cnc1800l_of_map_io(void)
{
	iotable_init(cnc1800l_of_io_desc, ARRAY_SIZE(cnc1800l_of_io_desc));
}


static void __init cnc1800l_init_machine(void)
{
    return;
}

static const char * const cnc1800l_dt_match[] = {
	"cavium,cnc1800l",
	NULL
};

DT_MACHINE_START(CNC1800L, "Cavium Celestial CNC1800L")
	.map_io		= cnc1800l_of_map_io,
	.init_machine	= cnc1800l_init_machine,
	.dt_compat	= cnc1800l_dt_match,
MACHINE_END
