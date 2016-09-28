#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/pm_runtime.h>

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

   TO DO: rtl8169_set_features is empty and tp->cp_cmd is missing
          (look at __rtl8169_set_features)

 * Iteration 3 - open() (work requred for rtl_task -> rtl_slow_event_work ->
                         __rtl8169_check_link_status) +
                 PHY initialization (via MDIO in rtl8168e_1_hw_phy_config)
                 + autonegotiation + rtl8169_set_speed (without link partner).


 * never ever return -ENOMEM from open() - consequences are strange and hard to debug
   (rtl_open is called many times, and no rtl8169_close during rmmod!)

 * Iteration 4

 * Every plug-in/plug-out generate interrupt triggering rtl8169_interrupt.
   Check this before/after plugging:
   cat /proc/interrupts | grep --color=auto "eth1"

   1. rtl_init_one -> rtl_try_msi -> pci_enable_msi sets MSI for line=27 (IRQ=27)
      I need this for link_status!

   2. rtl_open -> request_irq for MSI. I need this as well.

   3. Because flow is:
        rtl8169_interrupt ->
        rtl8169_interrupt: napi ->
        start rtl8169_poll ->
        start rtl_slow_event_work

   This fucking NAPI is required too!

   4. Ok case from r8169.origin.c.
      rtl8169_interrupt  was triggered during plugging cable.

   r8169: start rtl_phy_work
   r8169: start rtl_phy_work
   r8169: start rtl_phy_work
   r8169: start rtl_phy_work
   r8169: start rtl_phy_work
   r8169: start rtl_phy_work
   r8169: start rtl8169_interrupt
   r8169: start rtl_slow_event_work
   r8169: __rtl8169_check_link_status;  0
   r8169 0000:01:00.0 eth1: link up
   r8169: __rtl8169_check_link_status is done;  0
   r8169: start rtl8169_interrupt
   r8169: start rtl8169_interrupt
   r8169: start rtl8169_interrupt
   r8169: start rtl8169_interrupt
   r8169: start rtl8169_interrupt

[12451.749863] r8169_ver4: rtl_init_one: pdev = ffff88007c3ec000, ent = ffffffffc037b8e0
[12451.749866] r8169_ver4: rtl_init_one  pdev: vendor = 10ec, device = 8168, class = 20000,
                           dma_mask = ffffffff, enable_cnt = 0
[12451.749871] r8169_ver4 Gigabit Ethernet driver 2.3LK-NAPI loaded
[12451.749882] r8169_ver4: rtl_init_one: dev = ffff8800788b1000, tp = ffff8800788b18c0
[12451.749887] r8169_ver4 0000:01:00.0: can't disable ASPM; OS doesn't have ASPM control
[12451.750178] r8169_ver4: rtl_init_one:  pci_enable_device ok
[12451.750180] r8169_ver4: rtl_init_one:  pdev: enable_cnt = 1
[12451.750206] r8169_ver4: rtl_init_one:  ioremap ok
[12451.750207] r8169_ver4: after ioremap: tp->mmio_addr = ffffc90000006000
[12451.750210] r8169_ver4: after ioremap: region len = 4096, region start = 00000000d0004000
[12451.750212] r8169_ver4: mac_version = 0x20
[12451.750220] r8169_ver4: rtl_init_one: vlan_features ok
[12451.752639] r8169_ver4 0000:01:00.0 eth1: ??? at 0xffffc90000006000, 80:1f:02:4c:0a:12
[12451.752643] r8169_ver4: rtl_init_one:  ok
[12451.757304] r8169_ver4: start rtl_open
[12451.757310] r8169_ver4: start rtl8169_init_phy
[12451.757312] r8169_ver4: mac_version = 0x20
[12451.772149] r8169_ver4: rtl8169_set_speed
[12451.772154] r8169_ver4: rtl8169_set_speed_xmii:
[12451.772525] r8169_ver4: Half duplex
[12451.772526] r8169_ver4: Full duplex
[12451.772773] r8169_ver4: auto_nego = de1, giga_ctrl = 300, speed = 1000
[12451.772898] r8169_ver4: rtl8169_init_phy is done
[12451.772900] r8169_ver4: rtl_open:  ok
[12461.788036] r8169_ver4: start rtl_task
[12461.788042] r8169_ver4: start rtl_phy_work
[12461.788414] r8169_ver4: rtl_phy_work:  ok
[12471.804042] r8169_ver4: start rtl_task
[12471.804047] r8169_ver4: start rtl_phy_work
[12471.804419] r8169_ver4: rtl_phy_work:  ok
[12481.820037] r8169_ver4: start rtl_task
[12481.820043] r8169_ver4: start rtl_phy_work
[12481.820415] r8169_ver4: rtl_phy_work:  ok
[12490.973520] r8169_ver4: rtl_remove_one
[12490.973536] r8169_ver4: start rtl8169_close
[12490.973541] r8169_ver4: rtl8169_close:  ok
[12490.992434] r8169_ver4: rtl_remove_one:    ok

TO DO: No interrupts. Compare configuration on register level with r8169.origin
       in probe() and open()
*/

#define MODULENAME "r8169_ver4"
#define PFX MODULENAME ": "
#define RTL8169_VERSION "2.3LK-NAPI"
#define ETH_ADDR_LEN 6
#define R8169_REGS_SIZE		256

#define FIRMWARE_8168E_1	"rtl_nic/rtl8168e-1.fw"
#define FIRMWARE_8168E_2	"rtl_nic/rtl8168e-2.fw"
#define FIRMWARE_8168E_3	"rtl_nic/rtl8168e-3.fw"

#define JUMBO_9K	(9*1024 - ETH_HLEN - 2)
#define RTL8169_PHY_TIMEOUT	(10*HZ)

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

#define RTL_EVENT_NAPI_RX	(RxOK | RxErr)
#define RTL_EVENT_NAPI_TX	(TxOK | TxErr)
#define RTL_EVENT_NAPI		(RTL_EVENT_NAPI_RX | RTL_EVENT_NAPI_TX)
#define R8169_NAPI_WEIGHT	64

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
    /* InterruptStatusBits */
    SYSErr		= 0x8000,
    LinkChg		= 0x0020,
    TxErr		= 0x0008,
    TxOK		= 0x0004,
    RxErr		= 0x0002,
    RxOK		= 0x0001,
    /* Cfg9346Bits */
    Cfg9346_Lock	= 0x00,
    Cfg9346_Unlock	= 0xc0,

    /* CPlusCmd p.31 */
    PCIDAC		= (1 << 4),
    /* rtl8169_PHYstatus */
    LinkStatus	= 0x02,
    /* Config2 register p. 25 */
    MSIEnable	= (1 << 5),	/* 8169 only. Reserved in the 8168. */
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
    PHYstatus	= 0x6c,
    CPlusCmd	= 0xe0,

    MaxTxPacketSize	= 0xec,	/* 8101/8168. Unit of 128 bytes. */
};

enum rtl_tx_desc_version {
    RTL_TD_1	= 1,
};

static void rtl_hw_start_8168(struct net_device *dev) {}
static void rtl_hw_start_8169(struct net_device *dev) {}
static int rtl8169_set_features(struct net_device *dev,
                netdev_features_t features)
{
    (void)dev; (void)features;
    return 0;
}
static int rtl_open(struct net_device *dev);
static int rtl8169_close(struct net_device *dev);
static int rtl8169_set_speed_xmii(struct net_device *dev,
                  u8 autoneg, u16 speed, u8 duplex, u32 adv);
static netdev_tx_t rtl8169_start_xmit(struct sk_buff *skb,
                                      struct net_device *dev)
{
    (void)skb; (void)dev;
    return NETDEV_TX_OK;
}


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
    .ndo_open		= rtl_open,
    .ndo_stop		= rtl8169_close,
    .ndo_get_stats64	= NULL,
    .ndo_start_xmit		= rtl8169_start_xmit,
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
    struct napi_struct napi;
    u32 msg_enable;
    u16 cp_cmd;

    u16 event_slow;

    uint8_t *hw_addr;
    uint16_t mac_version;
    uint8_t dev_addr[ETH_ADDR_LEN];
    bool stopped;
    struct timer_list timer;

    struct {
        DECLARE_BITMAP(flags, RTL_FLAG_MAX);
        struct mutex mutex;
        struct work_struct work;
    } wk;

    struct mdio_ops {
        void (*write)(struct rtl8169_private *, int, int);
        int (*read)(struct rtl8169_private *, int);
    } mdio_ops;

    void (*phy_reset_enable)(struct rtl8169_private *tp);
    unsigned int (*link_ok)(void __iomem *);
    int (*set_speed)(struct net_device *, u8 aneg, u16 sp, u8 dpx, u32 adv);
    unsigned int (*phy_reset_pending)(struct rtl8169_private *tp);

    unsigned features;
    struct mii_if_info mii;
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

static void rtl_disable_msi(struct pci_dev *pdev, struct rtl8169_private *tp)
{
    if (tp->features & RTL_FEATURE_MSI) {
        pci_disable_msi(pdev);
        tp->features &= ~RTL_FEATURE_MSI;
    }
}

static void rtl_remove_one(struct pci_dev *pdev)
{
    dprintk("rtl_remove_one");
    void __iomem *ioaddr = NULL;
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8169_private *tp = netdev_priv(dev);

    netif_napi_del(&tp->napi);
    unregister_netdev(dev);
    rtl_disable_msi(pdev, tp);

    ioaddr = tp->mmio_addr;
    rtl8169_release_board(pdev, dev, ioaddr);
    dprintk("rtl_remove_one:    ok");
}

static u16 rtl_get_events(struct rtl8169_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    return RTL_R16(IntrStatus);
}

static void rtl_ack_events(struct rtl8169_private *tp, u16 bits)
{
    void __iomem *ioaddr = tp->mmio_addr;
    RTL_W16(IntrStatus, bits);
    mmiowb();
}

static void rtl_irq_disable(struct rtl8169_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    RTL_W16(IntrMask, 0);
    mmiowb();
}

static void rtl_irq_enable(struct rtl8169_private *tp, u16 bits)
{
    void __iomem *ioaddr = tp->mmio_addr;
    RTL_W16(IntrMask, bits);
}

static void rtl_irq_enable_all(struct rtl8169_private *tp)
{
    rtl_irq_enable(tp, RTL_EVENT_NAPI | tp->event_slow);
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

static void rtl_writephy(struct rtl8169_private *tp, int location, u32 val)
{
    tp->mdio_ops.write(tp, location, val);
}

static int rtl_readphy(struct rtl8169_private *tp, int location)
{
    return tp->mdio_ops.read(tp, location);
}

struct rtl_cond
{
    bool (*check)(struct rtl8169_private *);
    const char *msg;
};

static void rtl_udelay(unsigned int d)
{
    udelay(d);
}

static bool rtl_loop_wait(struct rtl8169_private *tp, const struct rtl_cond *c,
                          void (*delay)(unsigned int), unsigned int d, int n,
                          bool high)
{
    int i;
    for (i = 0; i < n; i++) {
        delay(d);
        if (c->check(tp) == high)
            return true;
    }
    netif_err(tp, drv, tp->dev, "%s == %d (loop: %d, delay: %d).\n",
              c->msg, !high, n, d);
    return false;
}

static bool rtl_udelay_loop_wait_high(struct rtl8169_private *tp,
                                      const struct rtl_cond *c,
                                      unsigned int d, int n)
{
    return rtl_loop_wait(tp, c, rtl_udelay, d, n, true);
}

static bool rtl_udelay_loop_wait_low(struct rtl8169_private *tp,
                                     const struct rtl_cond *c,
                                     unsigned int d, int n)
{
    return rtl_loop_wait(tp, c, rtl_udelay, d, n, false);
}

static bool rtl_msleep_loop_wait_low(struct rtl8169_private *tp,
                     const struct rtl_cond *c,
                     unsigned int d, int n)
{
    return rtl_loop_wait(tp, c, msleep, d, n, false);
}

#define DECLARE_RTL_COND(name)                          \
static bool name ## _check(struct rtl8169_private *);	\
                                                        \
static const struct rtl_cond name = {                   \
    .check	= name ## _check,                           \
    .msg	= #name                                     \
};                                                      \
                                                        \
static bool name ## _check(struct rtl8169_private *tp)

DECLARE_RTL_COND(rtl_phyar_cond)
{
    void __iomem *ioaddr = tp->mmio_addr;
    return RTL_R32(PHYAR) & 0x80000000;
}

DECLARE_RTL_COND(rtl_phy_reset_cond)
{
    return tp->phy_reset_pending(tp);
}

static unsigned int rtl8169_xmii_reset_pending(struct rtl8169_private *tp)
{
    return rtl_readphy(tp, MII_BMCR) & BMCR_RESET;
}

static void rtl8169_xmii_reset_enable(struct rtl8169_private *tp)
{
    unsigned int val;

    val = rtl_readphy(tp, MII_BMCR) | BMCR_RESET;
    rtl_writephy(tp, MII_BMCR, val & 0xffff);
}

static unsigned int rtl8169_xmii_link_ok(void __iomem *ioaddr)
{
    return RTL_R8(PHYstatus) & LinkStatus;
}

static void r8169_mdio_write(struct rtl8169_private *tp, int reg, int value)
{
    void __iomem *ioaddr = tp->mmio_addr;
    RTL_W32(PHYAR, 0x80000000 | (reg & 0x1f) << 16 | (value & 0xffff));
    rtl_udelay_loop_wait_low(tp, &rtl_phyar_cond, 25, 20);
    /*
         * According to hardware specs a 20us delay is required after write
         * complete indication, but before sending next command.
         */
    udelay(20);
}

static int r8169_mdio_read(struct rtl8169_private *tp, int reg)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int value;
    RTL_W32(PHYAR, 0x0 | (reg & 0x1f) << 16);
    value = rtl_udelay_loop_wait_high(tp, &rtl_phyar_cond, 25, 20) ?
                RTL_R32(PHYAR) & 0xffff : ~0;
    /*
         * According to hardware specs a 20us delay is required after read
         * complete indication, but before sending next command.
         */
    udelay(20);
    return value;
}

static void rtl_schedule_task(struct rtl8169_private *tp, enum rtl_flag flag)
{
    if (!test_and_set_bit(flag, tp->wk.flags))
        schedule_work(&tp->wk.work);
}

static void rtl8169_phy_timer(unsigned long __opaque)
{
    struct net_device *dev = (struct net_device *)__opaque;
    struct rtl8169_private *tp = netdev_priv(dev);

    rtl_schedule_task(tp, RTL_FLAG_TASK_PHY_PENDING);
}

/* Cfg9346_Unlock assumed. */
static unsigned rtl_try_msi(struct rtl8169_private *tp,
                            const struct rtl_cfg_info *cfg)
{
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned msi = 0;
    u8 cfg2;

    cfg2 = RTL_R8(Config2) & ~MSIEnable;
    if (cfg->features & RTL_FEATURE_MSI)
    {
        if (pci_enable_msi(tp->pci_dev))
        {
            netif_info(tp, hw, tp->dev, "no MSI. Back to INTx.\n");
        }
        else
        {
            cfg2 |= MSIEnable;
            msi = RTL_FEATURE_MSI;
        }
    }
    return msi;
}

static int rtl8169_poll(struct napi_struct *napi, int budget)
{
    dprintk("start rtl8169_poll\n");
    struct rtl8169_private *tp = container_of(napi, struct rtl8169_private, napi);
    //struct net_device *dev = tp->dev;
    u16 enable_mask = RTL_EVENT_NAPI | tp->event_slow;
    int work_done = 0;
    u16 status;

    status = rtl_get_events(tp);
    rtl_ack_events(tp, status & ~tp->event_slow);

//    if (status & RTL_EVENT_NAPI_RX)
//        work_done = rtl_rx(dev, tp, (u32) budget);

//    if (status & RTL_EVENT_NAPI_TX)
//        rtl_tx(dev, tp);

    if (status & tp->event_slow) {
        enable_mask &= ~tp->event_slow;

        rtl_schedule_task(tp, RTL_FLAG_TASK_SLOW_PENDING);
    }

    if (work_done < budget) {
        napi_complete(napi);

        rtl_irq_enable(tp, enable_mask);
        mmiowb();
    }
    return work_done;
}

static int rtl_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    unsigned id = ent->driver_data;
    const struct rtl_cfg_info *cfg = rtl_cfg_infos + id;
    const unsigned int region = cfg->region;
    struct rtl8169_private *tp;
    struct mii_if_info *mii;
    struct net_device *dev;
    void __iomem *ioaddr;
    int rc = 0, chipset;

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


    mii = &tp->mii;
    mii->supports_gmii = !!(cfg->features & RTL_FEATURE_GMII);


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


    rtl_irq_disable(tp);

    /* pci_set_master() will enable DMA by setting the bus master bit
    in the PCI_COMMAND register */
    pci_set_master(pdev);

    // handling interrupts via MSI is crucial for link up/down
    RTL_W8(Cfg9346, Cfg9346_Unlock);
    tp->features |= rtl_try_msi(tp, cfg);
    RTL_W8(Cfg9346, Cfg9346_Lock);

    tp->mdio_ops.write = r8169_mdio_write;
    tp->mdio_ops.read = r8169_mdio_read;
    tp->set_speed = rtl8169_set_speed_xmii;
    tp->phy_reset_pending = rtl8169_xmii_reset_pending;
    tp->phy_reset_enable = rtl8169_xmii_reset_enable;
    tp->link_ok = rtl8169_xmii_link_ok;

    /* wk.mutex is used implicitly through rtl_lock_work/rtl_unlock_work.
       It protectes work.
     */
    mutex_init(&tp->wk.mutex);

    int i;
    for (i = 0; i < ETH_ALEN; i++)
        dev->dev_addr[i] = RTL_R8(MAC0 + i);

    // handling NAPI is crucial for link up/down
    netif_napi_add(dev, &tp->napi, rtl8169_poll, R8169_NAPI_WEIGHT);

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

    /*
     * Timer is required for putting work in global queue by schedule_work()
       which is called by rtl8169_phy_timer -> rtl_schedule_task
     */
    init_timer(&tp->timer);
    tp->timer.data = (unsigned long) dev;
    tp->timer.function = rtl8169_phy_timer;

    rc = register_netdev(dev);
    if (rc < 0)
        goto err_out_msi_4;

    pci_set_drvdata(pdev, dev);

    chipset = tp->mac_version;
    netif_info(tp, probe, dev, "%s at 0x%p, %pM, XID %08x IRQ %d\n",
           rtl_chip_infos[chipset].name, ioaddr, dev->dev_addr,
           (u32)(RTL_R32(TxConfig) & 0x9cf0f8ff), pdev->irq);

    dprintk("rtl_init_one:  ok");

    netif_carrier_off(dev);

out:
    return rc;

err_out_msi_4:
    netif_napi_del(&tp->napi);
    rtl_disable_msi(pdev, tp);
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

static int rtl8169_set_speed_xmii(struct net_device *dev,
                  u8 autoneg, u16 speed, u8 duplex, u32 adv)
{
    dprintk("rtl8169_set_speed_xmii:    \n");

    struct rtl8169_private *tp = netdev_priv(dev);
    int giga_ctrl, bmcr;
    int rc = -EINVAL;

    rtl_writephy(tp, 0x1f, 0x0000);

    if (autoneg == AUTONEG_ENABLE)
    {
        int auto_nego;
        auto_nego = rtl_readphy(tp, MII_ADVERTISE);
        auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL |
                ADVERTISE_100HALF | ADVERTISE_100FULL);

        if (adv & ADVERTISED_10baseT_Half)
            auto_nego |= ADVERTISE_10HALF;
        if (adv & ADVERTISED_10baseT_Full)
            auto_nego |= ADVERTISE_10FULL;
        if (adv & ADVERTISED_100baseT_Half)
            auto_nego |= ADVERTISE_100HALF;
        if (adv & ADVERTISED_100baseT_Full)
            auto_nego |= ADVERTISE_100FULL;

        auto_nego |= ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM;

        giga_ctrl = rtl_readphy(tp, MII_CTRL1000);
        giga_ctrl &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);

        /* The 8100e/8101e/8102e do Fast Ethernet only. */
        if (tp->mii.supports_gmii)
        {
            if (adv & ADVERTISED_1000baseT_Half)
            {
                giga_ctrl |= ADVERTISE_1000HALF;
                dprintk("Half duplex\n");
            }
            if (adv & ADVERTISED_1000baseT_Full)
            {
                giga_ctrl |= ADVERTISE_1000FULL;
                dprintk("Full duplex\n");
            }
        }
        else
            if (adv & (ADVERTISED_1000baseT_Half |
                       ADVERTISED_1000baseT_Full))
            {
                netif_info(tp, link, dev,
                           "PHY does not support 1000Mbps\n");
                goto out;
            }

        bmcr = BMCR_ANENABLE | BMCR_ANRESTART;

        rtl_writephy(tp, MII_ADVERTISE, auto_nego);
        rtl_writephy(tp, MII_CTRL1000, giga_ctrl);

        dprintk("auto_nego = %x, giga_ctrl = %x, speed = %d\n",
                auto_nego, giga_ctrl, speed);
    }
    else
    {
        giga_ctrl = 0;

        if (speed == SPEED_10)
            bmcr = 0;
        else if (speed == SPEED_100)
            bmcr = BMCR_SPEED100;
        else
            goto out;

        if (duplex == DUPLEX_FULL)
            bmcr |= BMCR_FULLDPLX;
    }

    rtl_writephy(tp, MII_BMCR, bmcr);
    rc = 0;
out:
    return rc;
}

static int rtl8169_set_speed(struct net_device *dev,
                 u8 autoneg, u16 speed, u8 duplex, u32 advertising)
{
    dprintk("rtl8169_set_speed\n");
    struct rtl8169_private *tp = netdev_priv(dev);
    int ret;

    // call to rtl8169_set_speed_xmii
    ret = tp->set_speed(dev, autoneg, speed, duplex, advertising);
    if (ret < 0)
        goto out;

    if (netif_running(dev) && (autoneg == AUTONEG_ENABLE) &&
        (advertising & ADVERTISED_1000baseT_Full)) {
        mod_timer(&tp->timer, jiffies + RTL8169_PHY_TIMEOUT);
    }
out:
    return ret;
}

struct phy_reg
{
    u16 reg;
    u16 val;
};

static void rtl_writephy_batch(struct rtl8169_private *tp,
                   const struct phy_reg *regs, int len)
{
    while (len-- > 0)
    {
        rtl_writephy(tp, regs->reg, regs->val);
        regs++;
    }
}

static void rtl_w0w1_phy(struct rtl8169_private *tp, int reg_addr, int p, int m)
{
    int val;
    val = rtl_readphy(tp, reg_addr);
    rtl_writephy(tp, reg_addr, (val & ~m) | p);
}

static void rtl8168e_1_hw_phy_config(struct rtl8169_private *tp)
{
    static const struct phy_reg phy_reg_init[] =
    {
        /* Enable Delay cap */
    { 0x1f, 0x0005 },
    { 0x05, 0x8b80 },
    { 0x06, 0xc896 },
    { 0x1f, 0x0000 },

    /* Channel estimation fine tune */
    { 0x1f, 0x0001 },
    { 0x0b, 0x6c20 },
    { 0x07, 0x2872 },
    { 0x1c, 0xefff },
    { 0x1f, 0x0003 },
    { 0x14, 0x6420 },
    { 0x1f, 0x0000 },

    /* Update PFM & 10M TX idle timer */
    { 0x1f, 0x0007 },
    { 0x1e, 0x002f },
    { 0x15, 0x1919 },
    { 0x1f, 0x0000 },

    { 0x1f, 0x0007 },
    { 0x1e, 0x00ac },
    { 0x18, 0x0006 },
    { 0x1f, 0x0000 }
    };

    rtl_writephy_batch(tp, phy_reg_init, ARRAY_SIZE(phy_reg_init));

    /* DCO enable for 10M IDLE Power */
    rtl_writephy(tp, 0x1f, 0x0007);
    rtl_writephy(tp, 0x1e, 0x0023);
    rtl_w0w1_phy(tp, 0x17, 0x0006, 0x0000);
    rtl_writephy(tp, 0x1f, 0x0000);

    /* For impedance matching */
    rtl_writephy(tp, 0x1f, 0x0002);
    rtl_w0w1_phy(tp, 0x08, 0x8000, 0x7f00);
    rtl_writephy(tp, 0x1f, 0x0000);

    /* PHY auto speed down */
    rtl_writephy(tp, 0x1f, 0x0007);
    rtl_writephy(tp, 0x1e, 0x002d);
    rtl_w0w1_phy(tp, 0x18, 0x0050, 0x0000);
    rtl_writephy(tp, 0x1f, 0x0000);
    rtl_w0w1_phy(tp, 0x14, 0x8000, 0x0000);

    rtl_writephy(tp, 0x1f, 0x0005);
    rtl_writephy(tp, 0x05, 0x8b86);
    rtl_w0w1_phy(tp, 0x06, 0x0001, 0x0000);
    rtl_writephy(tp, 0x1f, 0x0000);

    rtl_writephy(tp, 0x1f, 0x0005);
    rtl_writephy(tp, 0x05, 0x8b85);
    rtl_w0w1_phy(tp, 0x06, 0x0000, 0x2000);
    rtl_writephy(tp, 0x1f, 0x0007);
    rtl_writephy(tp, 0x1e, 0x0020);
    rtl_w0w1_phy(tp, 0x15, 0x0000, 0x1100);
    rtl_writephy(tp, 0x1f, 0x0006);
    rtl_writephy(tp, 0x00, 0x5a00);
    rtl_writephy(tp, 0x1f, 0x0000);
    rtl_writephy(tp, 0x0d, 0x0007);
    rtl_writephy(tp, 0x0e, 0x003c);
    rtl_writephy(tp, 0x0d, 0x4007);
    rtl_writephy(tp, 0x0e, 0x0000);
    rtl_writephy(tp, 0x0d, 0x0000);
}

static void rtl_hw_phy_config(struct net_device *dev)
{
    struct rtl8169_private *tp = netdev_priv(dev);

    dprintk("mac_version = 0x%02x\n", tp->mac_version);
    rtl8168e_1_hw_phy_config(tp);
}

static void rtl8169_phy_reset(struct net_device *dev,
                  struct rtl8169_private *tp)
{
    tp->phy_reset_enable(tp);
    rtl_msleep_loop_wait_low(tp, &rtl_phy_reset_cond, 1, 100);
}

static void rtl8169_init_phy(struct net_device *dev, struct rtl8169_private *tp)
{
    dprintk("start rtl8169_init_phy");

    rtl_hw_phy_config(dev);

    pci_write_config_byte(tp->pci_dev, PCI_LATENCY_TIMER, 0x40);

    rtl8169_phy_reset(dev, tp);

    rtl8169_set_speed(dev, AUTONEG_ENABLE, SPEED_1000, DUPLEX_FULL,
              ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
              ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |
              (tp->mii.supports_gmii ?
               ADVERTISED_1000baseT_Half |
               ADVERTISED_1000baseT_Full : 0));

    dprintk("rtl8169_init_phy is done");
}

static void __rtl8169_check_link_status(struct net_device *dev,
                                        struct rtl8169_private *tp,
                                        void __iomem *ioaddr, bool pm)
{
    dprintk("__rtl8169_check_link_status");
    if (tp->link_ok(ioaddr))
    {
        //rtl_link_chg_patch(tp);
        /* This is to cancel a scheduled suspend if there's one. */
        if (pm)
            pm_request_resume(&tp->pci_dev->dev);
        netif_carrier_on(dev);
        if (net_ratelimit())
            netif_info(tp, ifup, dev, "link up\n");
    }
    else
    {
        netif_carrier_off(dev);
        netif_info(tp, ifdown, dev, "link down\n");
        if (pm)
            pm_schedule_suspend(&tp->pci_dev->dev, 5000);
    }
    dprintk("__rtl8169_check_link_status is done");
}

static void rtl8169_pcierr_interrupt(struct net_device *dev)
{
    (void)dev;
    dprintk("ERROR: rtl8169_pcierr_interrupt. Not implemented yet\n");
}

static void rtl_slow_event_work(struct rtl8169_private *tp)
{
    dprintk("start rtl_slow_event_work\n");

    struct net_device *dev = tp->dev;
    u16 status;

    status = rtl_get_events(tp) & tp->event_slow;
    rtl_ack_events(tp, status);

    if (unlikely(status & SYSErr))
        rtl8169_pcierr_interrupt(dev);

    if (status & LinkChg)
        __rtl8169_check_link_status(dev, tp, tp->mmio_addr, true);

    rtl_irq_enable_all(tp);
}

static irqreturn_t rtl8169_interrupt(int irq, void *dev_instance)
{
    dprintk("start rtl8169_interrupt\n");

    struct net_device *dev = dev_instance;
    struct rtl8169_private *tp = netdev_priv(dev);
    int handled = 0;
    u16 status;

    status = rtl_get_events(tp);
    if (status && status != 0xffff) {
        status &= RTL_EVENT_NAPI | tp->event_slow;
        if (status) {
            dprintk("rtl8169_interrupt: napi\n");

            handled = 1;

            rtl_irq_disable(tp);
            napi_schedule(&tp->napi);
        }
    }
    return IRQ_RETVAL(handled);
}

static void rtl_reset_work(struct rtl8169_private *tp)
{
    (void)tp;
    dprintk("rtl_reset_work\n");
}

static void rtl_phy_work(struct rtl8169_private *tp)
{
    dprintk("start rtl_phy_work\n");

    struct timer_list *timer = &tp->timer;
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long timeout = RTL8169_PHY_TIMEOUT;

    if (tp->phy_reset_pending(tp))
    {
        /*
        * A busy loop could burn quite a few cycles on nowadays CPU.
        * Let's delay the execution of the timer for a few ticks.
        */
        timeout = HZ/10;
        goto out_mod_timer;
    }

    if (tp->link_ok(ioaddr))
        return;

    netif_dbg(tp, link, tp->dev, "PHY reset until link up\n");
    tp->phy_reset_enable(tp);

    dprintk("rtl_phy_work:  ok\n");

out_mod_timer:
    mod_timer(timer, jiffies + timeout);
}

static void rtl_lock_work(struct rtl8169_private *tp)
{
    mutex_lock(&tp->wk.mutex);
}

static void rtl_unlock_work(struct rtl8169_private *tp)
{
    mutex_unlock(&tp->wk.mutex);
}

static void rtl_task(struct work_struct *work)
{
    dprintk("start rtl_task\n");
    static const struct
    {
        int bitnr;
        void (*action)(struct rtl8169_private *);
    }
    rtl_work[] =
    {
        /* XXX - keep rtl_slow_event_work() as first element. */
        { RTL_FLAG_TASK_SLOW_PENDING,	rtl_slow_event_work },
        { RTL_FLAG_TASK_RESET_PENDING,	rtl_reset_work },
        { RTL_FLAG_TASK_PHY_PENDING,	rtl_phy_work }
    };

    struct rtl8169_private *tp =
            container_of(work, struct rtl8169_private, wk.work);
    struct net_device *dev = tp->dev;
    int i;

    rtl_lock_work(tp);

    if (!netif_running(dev) ||
            !test_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags))
        goto out_unlock;

    for (i = 0; i < ARRAY_SIZE(rtl_work); i++) {
        bool pending;

        pending = test_and_clear_bit(rtl_work[i].bitnr, tp->wk.flags);
        if (pending)
            rtl_work[i].action(tp);
    }

out_unlock:
    rtl_unlock_work(tp);
}

static void rtl8169_check_link_status(struct net_device *dev,
                                      struct rtl8169_private *tp,
                                      void __iomem *ioaddr)
{
    __rtl8169_check_link_status(dev, tp, ioaddr, false);
}


static int rtl_open(struct net_device *dev)
{
    dprintk("start rtl_open");
    struct rtl8169_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    void __iomem *ioaddr = tp->mmio_addr;
    int retval = 0; //-ENOMEM;

    pm_runtime_get_sync(&pdev->dev);

    INIT_WORK(&tp->wk.work, rtl_task);

    smp_mb();

    // it's for triggering rtl_slow_event_work (through NAPI)
    retval = request_irq(pdev->irq, rtl8169_interrupt,
                         (tp->features & RTL_FEATURE_MSI) ? 0 : IRQF_SHARED,
                         dev->name, dev);
    if (retval < 0)
        goto out;

    rtl_lock_work(tp);
    set_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags);

    // NAPI for rtl_slow_event_work
    napi_enable(&tp->napi);

    rtl8169_init_phy(dev, tp);
    netif_start_queue(dev);

    rtl_unlock_work(tp);

    pm_runtime_put_noidle(&pdev->dev);

    rtl8169_check_link_status(dev, tp, ioaddr);

    dprintk("rtl_open:  ok");

out:
    return retval;
}

static void rtl8169_down(struct net_device *dev)
{
    struct rtl8169_private *tp = netdev_priv(dev);
    del_timer_sync(&tp->timer);
    napi_disable(&tp->napi);
    netif_stop_queue(dev);
}

static int rtl8169_close(struct net_device *dev)
{
    dprintk("start rtl8169_close");
    struct rtl8169_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    pm_runtime_get_sync(&pdev->dev);

    rtl_lock_work(tp);
    clear_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags);

    rtl8169_down(dev);

    rtl_unlock_work(tp);

    cancel_work_sync(&tp->wk.work);

    free_irq(pdev->irq, dev);

    pm_runtime_put_sync(&pdev->dev);
    dprintk("rtl8169_close:  ok");
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
