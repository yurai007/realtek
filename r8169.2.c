#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>

/*
 * Iteration 1 - allocating netdevice + handling PCI (enabling device,
                 handling I/O regions, dma flag) + proper cleanup

 * Without pci_set_drvdata in rtl_init_one crash in rtl_remove_one
   because pdev->dev have garbage!

 * Iteration 2 - simple MMIO communication with device + obtaining MAC address
                 + dev->features

 * Without netif_carrier_off after net_device registration whole OS freeze (
   probably kernel think may xmit). Same situation during rmmod when unregister_netdev
   is absent

 * OK case:

 *
root@yurai-PC:realtek# ip a
8: eth0: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc pfifo_fast state DOWN group default qlen 1000
    link/ether 90:2b:34:1a:5f:04 brd ff:ff:ff:ff:ff:ff


root@yurai-PC:realtek# ethtool -i eth0
driver: r8169_ver2
version:
firmware-version:
bus-info: 0000:04:00.0
supports-statistics: no
supports-test: no
supports-eeprom-access: no
supports-register-dump: no
supports-priv-flags: no


[  868.229979] r8169_ver2: rtl_init_one: pdev = ffff88021561f000, ent = ffffffffc03c4480
[  868.229983] r8169_ver2: rtl_init_one  pdev: vendor = 10ec, device = 8168, class = 20000, dma_mask = ffffffff, enable_cnt = 0
[  868.229987] r8169_ver2 Gigabit Ethernet driver 2.3LK-NAPI loaded
[  868.229996] r8169_ver2: rtl_init_one: dev = ffff8802107d1000, tp = ffff8802107d18c0
[  868.230000] r8169_ver2 0000:04:00.0: can't disable ASPM; OS doesn't have ASPM control
[  868.230070] r8169_ver2: rtl_init_one:  pci_enable_device ok
[  868.230072] r8169_ver2: rtl_init_one:  pdev: enable_cnt = 1
[  868.230094] r8169_ver2: rtl_init_one:  ioremap ok
[  868.230096] r8169_ver2: after ioremap: tp->mmio_addr = ffffc90004ea0000
[  868.230098] r8169_ver2: after ioremap: region len = 4096, region start = 00000000fbbff000
[  868.230102] r8169_ver2: mac_version = 0x21
[  868.230129] r8169_ver2: rtl_init_one: vlan_features ok
[  868.230465] r8169_ver2 0000:04:00.0 eth0: ??? at 0xffffc90004ea0000, 90:2b:34:1a:5f:04
[  868.230467] r8169_ver2: rtl_init_one:  ok
[  868.247957] IPv6: ADDRCONF(NETDEV_UP): eth0: link is not ready

    // TO DO: tp->features
 */

#define MODULENAME "r8169_ver2"
#define PFX MODULENAME ": "
#define RTL8169_VERSION "2.3LK-NAPI"
#define ETH_ADDR_LEN 6
#define R8169_REGS_SIZE		256

#define FIRMWARE_8168E_1	"rtl_nic/rtl8168e-1.fw"
#define FIRMWARE_8168E_2	"rtl_nic/rtl8168e-2.fw"
#define FIRMWARE_8168E_3	"rtl_nic/rtl8168e-3.fw"

#define JUMBO_9K	(9*1024 - ETH_HLEN - 2)

MODULE_AUTHOR("yurai");
MODULE_DESCRIPTION("RealTek");
MODULE_LICENSE("GPL");

#define dprintk(fmt, args...) \
    do { printk(KERN_DEBUG PFX fmt, ## args); } while (0)

#define R8169_MSG_DEFAULT \
    (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_IFUP | NETIF_MSG_IFDOWN)

/* write/read MMIO register */
#define RTL_W8(reg, val8)	writeb ((val8), ioaddr + (reg))
#define RTL_W16(reg, val16)	writew ((val16), ioaddr + (reg))
#define RTL_W32(reg, val32)	writel ((val32), ioaddr + (reg))
#define RTL_R8(reg)         readb (ioaddr + (reg))
#define RTL_R16(reg)		readw (ioaddr + (reg))
#define RTL_R32(reg)		readl (ioaddr + (reg))

enum rtl_flag {
    RTL_FLAG_TASK_ENABLED,
    RTL_FLAG_TASK_SLOW_PENDING,
    RTL_FLAG_TASK_RESET_PENDING,
    RTL_FLAG_TASK_PHY_PENDING,
    RTL_FLAG_MAX
};

enum cfg_version {
    RTL_CFG_0 = 0x00,
    RTL_CFG_1,
    RTL_CFG_2
};

enum features {
    RTL_FEATURE_WOL		= (1 << 0),
    RTL_FEATURE_MSI		= (1 << 1),
    RTL_FEATURE_GMII	= (1 << 2),
};

enum rtl_register_content {
    PCIDAC		= (1 << 4),
};

enum mac_version {
    RTL_GIGA_MAC_VER_32 = 0x19,
    RTL_GIGA_MAC_VER_33 = 0x20,
    RTL_GIGA_MAC_VER_34 = 0x21,
    RTL_GIGA_MAC_NONE   = 0xff,
};

enum rtl_registers {
    MAC0		= 0,	// Ethernet hardware address
    ChipCmd		= 0x37,
    IntrMask	= 0x3c,
    IntrStatus	= 0x3e,
    TxConfig	= 0x40,
    RxConfig	= 0x44,
    Cfg9346		= 0x50,
    Config1		= 0x52,
    Config2		= 0x53,
    Config3		= 0x54,
    Config4		= 0x55,
    Config5		= 0x56,
    PHYAR		= 0x60,
    CPlusCmd	= 0xe0,

    MaxTxPacketSize	= 0xec,	/* 8101/8168. Unit of 128 bytes. */
};

enum rtl_tx_desc_version {
    RTL_TD_1	= 1,
};

static void rtl_hw_start_8168(struct net_device *dev) {}
static void rtl_hw_start_8169(struct net_device *dev) {}
static int rtl8169_set_features(struct net_device *dev,
                netdev_features_t features) { return 0; }


static const struct rtl_cfg_info {
    void (*hw_start)(struct net_device *);
    unsigned int region;
    unsigned int align;
    u16 event_slow;
    unsigned features;
} rtl_cfg_infos [] = {
    [RTL_CFG_0] = {
        .hw_start	= rtl_hw_start_8169,
        .region		= 1,
        .align		= 0,
        .event_slow	= 0, //SYSErr | LinkChg | RxOverflow | RxFIFOOver,
        .features	= RTL_FEATURE_GMII,
    },
    [RTL_CFG_1] = {
        .hw_start	= rtl_hw_start_8168,
        .region		= 2,
        .align		= 8,
        .event_slow	= 0, //SYSErr | LinkChg | RxOverflow,
        .features	= RTL_FEATURE_GMII | RTL_FEATURE_MSI,
    }
};

#define _R(NAME,TD,FW,SZ,B) {	\
    .name = NAME,		\
    .txd_version = TD,	\
    .fw_name = FW,		\
    .jumbo_max = SZ,	\
    .jumbo_tx_csum = B	\
}

static const struct
{
    const char *name;
    enum rtl_tx_desc_version txd_version;
    const char *fw_name;
    u16 jumbo_max;
    bool jumbo_tx_csum;
} rtl_chip_infos[] =
{
    [RTL_GIGA_MAC_VER_32] =
        _R("RTL8168e/8111e",	RTL_TD_1, FIRMWARE_8168E_1,
                            JUMBO_9K, false),
    [RTL_GIGA_MAC_VER_33] =
        _R("RTL8168e/8111e",	RTL_TD_1, FIRMWARE_8168E_2,
                            JUMBO_9K, false),
    [RTL_GIGA_MAC_VER_34] =
        _R("RTL8168evl/8111evl",RTL_TD_1, FIRMWARE_8168E_3,
                            JUMBO_9K, false)
};

#undef _R

static const struct net_device_ops rtl_netdev_ops = {
    .ndo_open		= NULL,
    .ndo_stop		= NULL,
    .ndo_get_stats64	= NULL,
    .ndo_start_xmit		= NULL,
    .ndo_tx_timeout		= NULL,
    .ndo_validate_addr	= NULL,
    .ndo_change_mtu		= NULL,
    .ndo_fix_features	= NULL,
    .ndo_set_features	= rtl8169_set_features,
    .ndo_set_mac_address	= NULL,
    .ndo_do_ioctl		= NULL,
    .ndo_set_rx_mode	= NULL,
#ifdef CONFIG_NET_POLL_CONTROLLER
    .ndo_poll_controller	= NULL,
#endif

};

struct rtl8169_private {
    void __iomem *mmio_addr;	/* memory map physical address */
    struct pci_dev *pci_dev;
    struct net_device *dev;
    u32 msg_enable;
    u16 cp_cmd;

    uint8_t *hw_addr;
    uint16_t mac_version;
    uint8_t dev_addr[ETH_ADDR_LEN];
    bool stopped;

    struct {
        DECLARE_BITMAP(flags, RTL_FLAG_MAX);
        struct mutex mutex;
        struct work_struct work;
    } wk;
};

static const struct pci_device_id rtl8169_pci_tbl[] = {

	{ PCI_DEVICE(PCI_VENDOR_ID_REALTEK,	0x8168), 0, 0, RTL_CFG_1 },
	{0,},
};

MODULE_DEVICE_TABLE(pci, rtl8169_pci_tbl);

int use_dac = 0;

static struct {
    u32 msg_enable;
} debug = { -1 };

static void rtl_shutdown(struct pci_dev *pdev)
{
    (void)pdev;
    dprintk("rtl_shutdown");
}

static void rtl8169_release_board(struct pci_dev *pdev, struct net_device *dev,
                  void __iomem *ioaddr)
{
    iounmap(ioaddr);
    pci_release_regions(pdev);
    pci_clear_mwi(pdev);
    pci_disable_device(pdev);
    free_netdev(dev);
}

static void rtl_remove_one(struct pci_dev *pdev)
{
    dprintk("rtl_remove_one");
    void __iomem *ioaddr = NULL;
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8169_private *tp = netdev_priv(dev);
    unregister_netdev(dev);
    ioaddr = tp->mmio_addr;
    rtl8169_release_board(pdev, dev, ioaddr);
    dprintk("rtl_remove_one:    ok");
}

static void rtl8169_get_mac_version(struct rtl8169_private *tp, struct net_device *dev)
{
    void __iomem *ioaddr = tp->mmio_addr;

    static const struct rtl_mac_info
    {
        u32 mask;
        u32 val;
        int mac_version;
    }
    mac_info[] =
    {
        /* 8168E family. */
        { 0x7c800000, 0x2c800000,	RTL_GIGA_MAC_VER_34 },
        { 0x7cf00000, 0x2c200000,	RTL_GIGA_MAC_VER_33 },
        { 0x7cf00000, 0x2c100000,	RTL_GIGA_MAC_VER_32 },
        { 0x7c800000, 0x2c000000,	RTL_GIGA_MAC_VER_33 },
        // Catch-all
        { 0x00000000, 0x00000000,	RTL_GIGA_MAC_NONE   }
    };
    const struct rtl_mac_info *p = mac_info;
    u32 reg;

    reg = RTL_R32(TxConfig);
    while ((reg & p->mask) != p->val)
        p++;
    tp->mac_version = p->mac_version;

    if (tp->mac_version == RTL_GIGA_MAC_NONE) {
        netif_err(tp, probe, dev, "unknown MAC\n");
        // there is no support for default versions like VER1 or VER11 now
    }
}


static int rtl_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    unsigned id = ent->driver_data;
    const struct rtl_cfg_info *cfg = rtl_cfg_infos + id;
    const unsigned int region = cfg->region;
    struct rtl8169_private *tp;
    struct net_device *dev;
    void __iomem *ioaddr;
    int rc = 0;

    dprintk("rtl_init_one: pdev = %p, ent = %p", pdev, ent);
    dprintk("rtl_init_one  pdev: vendor = %x, device = %x, class = %x, dma_mask = %llx, enable_cnt = %d",
            pdev->vendor, pdev->device, pdev->class, pdev->dma_mask, pdev->enable_cnt.counter);

    if (netif_msg_drv(&debug)) {
        printk(KERN_INFO "%s Gigabit Ethernet driver %s loaded\n",
               MODULENAME, RTL8169_VERSION);
    }

    dev = alloc_etherdev(sizeof (*tp));
    if (!dev) {
        rc = -ENOMEM;
        goto out;
    }

    SET_NETDEV_DEV(dev, &pdev->dev);
    dev->netdev_ops = &rtl_netdev_ops;
    tp = netdev_priv(dev);
    dprintk("rtl_init_one: dev = %p, tp = %p", dev, tp);
    tp->dev = dev;
    tp->pci_dev = pdev;
    tp->msg_enable = netif_msg_init(debug.msg_enable, R8169_MSG_DEFAULT);


    /* disable ASPM completely as that cause random device stop working
         * problems as well as full system hangs for some PCIe devices users */
    pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S | PCIE_LINK_STATE_L1 |
                           PCIE_LINK_STATE_CLKPM);

    /* enable device (incl. PCI PM wakeup and hotplug setup) */
    rc = pci_enable_device(pdev);
    if (rc < 0) {
        netif_err(tp, probe, dev, "enable failure\n");
        goto err_out_free_dev_1;
    }
    dprintk("rtl_init_one:  pci_enable_device ok");
    dprintk("rtl_init_one:  pdev: enable_cnt = %d", pdev->enable_cnt.counter);

    /*
      If the PCI device can use the PCI Memory-Write-Invalidate transaction,
      call pci_set_mwi().  This enables the PCI_COMMAND bit for Mem-Wr-Inval
      and also ensures that the cache line size register is set correctly.
     */
    if (pci_set_mwi(pdev) < 0)
        netif_info(tp, probe, dev, "Mem-Wr-Inval unavailable\n");

    /* make sure PCI base addr 1 is MMIO */
    if (!(pci_resource_flags(pdev, region) & IORESOURCE_MEM)) {
        netif_err(tp, probe, dev,
                  "region #%d not an MMIO resource, aborting\n",
                  region);
        rc = -ENODEV;
        goto err_out_mwi_2;
    }

    /* check for weird/broken PCI region reporting */
    if (pci_resource_len(pdev, region) < R8169_REGS_SIZE) {
        netif_err(tp, probe, dev,
                  "Invalid PCI region size(s), aborting\n");
        rc = -ENODEV;
        goto err_out_mwi_2;
    }

    // /proc/iomem ?
    rc = pci_request_regions(pdev, MODULENAME);
    if (rc < 0) {
        netif_err(tp, probe, dev, "could not request regions\n");
        goto err_out_mwi_2;
    }
    tp->cp_cmd = 0;

    /* Inform about capabilities (64bit/32bit)
     */
    if ((sizeof(dma_addr_t) > 4) &&
            !pci_set_dma_mask(pdev, DMA_BIT_MASK(64)) && use_dac) {
        tp->cp_cmd |= PCIDAC;
        dev->features |= NETIF_F_HIGHDMA;
    }
    else
    {
        rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
        if (rc < 0) {
            netif_err(tp, probe, dev, "DMA configuration failed\n");
            goto err_out_free_res_3;
        }
    }

    /* ioremap MMIO region */
    ioaddr = ioremap(pci_resource_start(pdev, region), R8169_REGS_SIZE);
    if (!ioaddr) {
        netif_err(tp, probe, dev, "cannot remap MMIO, aborting\n");
        rc = -EIO;
        goto err_out_free_res_3;
    }
    tp->mmio_addr = ioaddr;
    /* show dump equivalent with cat /proc/ioports and cat /proc/iomem (which actually
       was done by kernel on startup :) */
    dprintk("rtl_init_one:  ioremap ok");
    dprintk("after ioremap: tp->mmio_addr = %p", tp->mmio_addr);
    dprintk("after ioremap: region len = %llu, region start = %p",
            pci_resource_len(pdev, region), (void*)pci_resource_start(pdev, region));

    if (!pci_is_pcie(pdev))
        netif_info(tp, probe, dev, "not PCI Express\n");

    /* Identify chip attached to board */
    rtl8169_get_mac_version(tp, dev);
    dprintk("mac_version = 0x%02x\n", tp->mac_version);

    /* pci_set_master() will enable DMA by setting the bus master bit
    in the PCI_COMMAND register */
    pci_set_master(pdev);

    int i;
    for (i = 0; i < ETH_ALEN; i++)
        dev->dev_addr[i] = RTL_R8(MAC0 + i);

    /* don't enable SG, IP_CSUM and TSO by default - it might not work
     * properly for all devices */
    dev->features |= NETIF_F_RXCSUM |
        NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX;

    dev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
        NETIF_F_RXCSUM | NETIF_F_HW_VLAN_CTAG_TX |
        NETIF_F_HW_VLAN_CTAG_RX;
    dev->vlan_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
        NETIF_F_HIGHDMA;

    dprintk("rtl_init_one: vlan_features ok\n");

    // because tp->txd_version == RTL_TD_1
    // tp->tso_csum = rtl8169_tso_csum_v2;
    dev->hw_features |= NETIF_F_IPV6_CSUM | NETIF_F_TSO6;

    dev->hw_features |= NETIF_F_RXALL;
    dev->hw_features |= NETIF_F_RXFCS;

    rc = register_netdev(dev);
    if (rc < 0)
        goto err_out_msi_4;

    pci_set_drvdata(pdev, dev);

    netif_info(tp, probe, dev, "??? at 0x%p, %pM\n",
            ioaddr, dev->dev_addr); //rtl_chip_infos[chipset].name,

    dprintk("rtl_init_one:  ok");

    netif_carrier_off(dev);

out:
    return rc;

err_out_msi_4:
    iounmap(ioaddr);
err_out_free_res_3:
    pci_release_regions(pdev);
err_out_mwi_2:
    pci_clear_mwi(pdev);
    pci_disable_device(pdev);
err_out_free_dev_1:
    free_netdev(dev);
    goto out;
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
