#include <linux/module.h>
#include <linux/pci.h>

#define MODULENAME "r8169_ver0"
#define PFX MODULENAME ": "

MODULE_AUTHOR("yurai");
MODULE_DESCRIPTION("RealTek");
MODULE_LICENSE("GPL");

#define dprintk(fmt, args...) \
    do { printk(KERN_DEBUG PFX fmt, ## args); } while (0)

enum cfg_version {
    RTL_CFG_1 = 0x01,
};

static const struct pci_device_id rtl8169_pci_tbl[] = {

	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK,	0x8168), 0, 0, RTL_CFG_1 },
	{0,},
};

MODULE_DEVICE_TABLE(pci, rtl8169_pci_tbl);

static void rtl_shutdown(struct pci_dev *pdev)
{
    (void)pdev;
    dprintk("rtl_shutdown");
}

static void rtl_remove_one(struct pci_dev *pdev)
{
    (void)pdev;
    dprintk("rtl_remove_one");
}

static int rtl_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    (void)pdev; (void)ent;
    dprintk("rtl_init_one");
    return 0;
}

#define RTL8169_PM_OPS	NULL
static struct pci_driver rtl8169_pci_driver = {
	.name		= MODULENAME,
	.id_table	= rtl8169_pci_tbl,
	.probe		= rtl_init_one,
	.remove		= rtl_remove_one,
	.shutdown	= rtl_shutdown,
	.driver.pm	= RTL8169_PM_OPS,
};

module_pci_driver(rtl8169_pci_driver);
