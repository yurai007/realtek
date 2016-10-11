#ifndef R8169_H
#define R8169_H

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

  * Iteration 5

  * Registers
    - IntrMask; rtl_irq_enable()/rtl_irq_disable()
    - IntrStatus; rtl_(ack/get)_events()
    - PHYAR; MDIO, PHY configuration (used heavly in hw_config and speed_xmii)

  * it was enaugh to set missing tp->event_slow values (for IntrStatus register
    via rtl_(ack/get)_events). After this device started to generate interrupts triggering
    link status checking (/proc/interrupts counters). Flow:

    r8169_ver5: start rtl8169_interrupt
    r8169_ver5: rtl8169_interrupt: napi
    r8169_ver5: start rtl8169_poll
    r8169_ver5: start rtl_task
    r8169_ver5: start rtl_slow_event_work
    r8169_ver5: rtl8169_check_link_status
    r8169_ver5 0000:01:00.0 eth1: link up
    r8169_ver5: rtl8169_check_link_status is done

  * Register dumps via start_tracing/stop_tracing_and_dump

1. RTL_INIT_ONE

r8169_ver5: MMIO access history:
// TxConfig; Get mac_version
r8169_ver5: R0: 40 -> 2f200700
// IntrMask; rtl_irq_enable/rtl_irq_disable
r8169_ver5: W1: 3c := 0
// Cfg9346; Unlock, check MSI capability; Lock
r8169_ver5: W2: 50 := c0
r8169_ver5: R3: 53 -> 1c
r8169_ver5: W4: 50 := 0
// Get MAC address
r8169_ver5: R5: 00 -> 80
r8169_ver5: R6: 01 -> 1f
r8169_ver5: R7: 02 -> 2
r8169_ver5: R8: 03 -> 4c
r8169_ver5: R9: 04 -> a
r8169_ver5: R10: 05 -> 12
// TxConfig again for XID
r8169_ver5: R11: 40 -> 2f200700

2. RTL_OPEN

Many (~300) mdio writes/reads by PHYAR. Everything in rtl8168e_1_hw_phy_config.

It works. Finally it works! :)

r8169_ver5: rtl_init_one: pdev = ffff88007c3ec000, ent = ffffffffc00c8ac0
r8169_ver5: rtl_init_one  pdev: vendor = 10ec, device = 8168, class = 20000, dma_mask = ffffffff, enable_cnt = 0
r8169_ver5 Gigabit Ethernet driver 2.3LK-NAPI loaded
r8169_ver5: rtl_init_one: dev = ffff88007881c000, tp = ffff88007881c8c0
r8169_ver5 0000:01:00.0: can't disable ASPM; OS doesn't have ASPM control
r8169_ver5: rtl_init_one:  pci_enable_device ok
r8169_ver5: rtl_init_one:  pdev: enable_cnt = 1
r8169_ver5: rtl_init_one:  ioremap ok
r8169_ver5: after ioremap: tp->mmio_addr = ffffc90000006000
r8169_ver5: after ioremap: region len = 4096, region start = 00000000d0004000
r8169_ver5: mac_version = 0x20
r8169_ver5: rtl_init_one: vlan_features ok
r8169_ver5 0000:01:00.0 eth1: RTL8168e/8111e at 0xffffc90000006000, 80:1f:02:4c:0a:12, XID 0c200000 IRQ 27
r8169_ver5: rtl_init_one:  ok
r8169_ver5: start rtl_open
r8169_ver5: start rtl8169_init_phy
r8169_ver5: mac_version = 0x20
r8169_ver5: rtl8169_set_speed
r8169_ver5: rtl8169_set_speed_xmii:
r8169_ver5: Half duplex
r8169_ver5: Full duplex
r8169_ver5: auto_nego = de1, giga_ctrl = 300, speed = 1000
r8169_ver5: rtl8169_init_phy is done
r8169_ver5: __rtl8169_check_link_status
r8169_ver5: start rtl8169_interrupt
r8169_ver5 0000:01:00.0 eth1: link down
r8169_ver5: __rtl8169_check_link_status is done
r8169_ver5: rtl_open:  ok
r8169_ver5: rtl8169_interrupt: napi
r8169_ver5: start rtl8169_poll
r8169_ver5: start rtl_task
r8169_ver5: start rtl_slow_event_work
r8169_ver5: __rtl8169_check_link_status
r8169_ver5 0000:01:00.0 eth1: link down
r8169_ver5: __rtl8169_check_link_status is done
r8169_ver5: start rtl8169_interrupt
r8169_ver5: rtl8169_interrupt: napi
r8169_ver5: start rtl8169_poll
r8169_ver5: start rtl_task
r8169_ver5: start rtl_slow_event_work
r8169_ver5: __rtl8169_check_link_status
r8169_ver5 0000:01:00.0 eth1: link up
r8169_ver5: __rtl8169_check_link_status is done
r8169_ver5: start rtl_task
r8169_ver5: start rtl_phy_work
r8169_ver5: start rtl8169_interrupt
r8169_ver5: rtl8169_interrupt: napi
r8169_ver5: start rtl8169_poll
r8169_ver5: start rtl_task
r8169_ver5: start rtl_slow_event_work
r8169_ver5: __rtl8169_check_link_status
r8169_ver5 0000:01:00.0 eth1: link down
r8169_ver5: __rtl8169_check_link_status is done
r8169_ver5: start rtl8169_interrupt
r8169_ver5: rtl8169_interrupt: napi
r8169_ver5: start rtl8169_poll
r8169_ver5: start rtl_task
r8169_ver5: start rtl_slow_event_work
r8169_ver5: __rtl8169_check_link_status
r8169_ver5 0000:01:00.0 eth1: link up

13: eth1: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc pfifo_fast state UP group default qlen 1000
    link/ether 80:1f:02:4c:0a:12 brd ff:ff:ff:ff:ff:ff
    inet6 fe80::821f:2ff:fe4c:a12/64 scope link tentative
       valid_lft forever preferred_lft forever
*/

#define MODULENAME "r8169_ver7"
#define RTL8169_VERSION "2.3LK-NAPI"
#define DEVICE_NAME "RTL8168e/8111e"
#define ETH_ADDR_LEN 6
#define R8169_REGS_SIZE		256

#define JUMBO_9K	(9*1024 - ETH_HLEN - 2)
#define RTL8169_PHY_TIMEOUT	(10*HZ)

#define dprintk(fmt, args...) \
    do { printk(KERN_DEBUG MODULENAME ": " fmt, ## args); } while (0)

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
    RTL_CFG_1
};

enum features {
    RTL_FEATURE_MSI		= (1 << 1),
    RTL_FEATURE_GMII	= (1 << 2),
};

enum rtl_register_content {
    /* InterruptStatusBits */
    SYSErr		= 0x8000,
    LinkChg		= 0x0020,
    RxFIFOOver	= 0x0040,
    RxOverflow	= 0x0010,
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
    Cfg9346		= 0x50,
    Config2		= 0x53,
    PHYAR		= 0x60,
    PHYstatus	= 0x6c,
    CPlusCmd	= 0xe0,
};

enum rtl_tx_desc_version {
    RTL_TD_1	= 1,
};

static int rtl_open(struct net_device *dev);
static int rtl8169_close(struct net_device *dev);
static int rtl8169_set_speed_xmii(struct net_device *dev,
                   u16 speed, u32 adv);

struct rtl_cfg_info {
    unsigned int region;
    unsigned int align;
    u16 event_slow;
    unsigned features;
};

struct rtl8169_private {
    void __iomem *mmio_addr;	/* memory map physical address */
    struct pci_dev *pci_dev;
    struct net_device *dev;
    struct napi_struct napi;

    // for netif_info and netif_err
    u32 msg_enable;
    u16 event_slow;
    uint16_t mac_version;
    struct timer_list timer;

    struct {
        DECLARE_BITMAP(flags, RTL_FLAG_MAX);
        struct mutex mutex;
        struct work_struct work;
    } wk;

    unsigned features;
    struct mii_if_info mii;
};

struct rtl_cond
{
    bool (*check)(struct rtl8169_private *);
    const char *msg;
};

#define DECLARE_RTL_COND(name)                          \
static bool name ## _check(struct rtl8169_private *);	\
                                                        \
static const struct rtl_cond name = {                   \
    .check	= name ## _check,                           \
    .msg	= #name                                     \
};                                                      \
                                                        \
static bool name ## _check(struct rtl8169_private *tp)

struct phy_reg
{
    u16 reg;
    u16 val;
};

static int rtl8169_set_features(struct net_device *dev,
                netdev_features_t features)
{
    (void)dev; (void)features;
    return 0;
}

static netdev_tx_t rtl8169_start_xmit(struct sk_buff *skb,
                                      struct net_device *dev)
{
    (void)skb; (void)dev;
    return NETDEV_TX_OK;
}

static void rtl_shutdown(struct pci_dev *pdev)
{
    (void)pdev;
    dprintk("rtl_shutdown");
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

static void rtl_udelay(unsigned int d)
{
    udelay(d);
}

static unsigned int rtl8169_xmii_link_ok(void __iomem *ioaddr)
{
    return RTL_R8(PHYstatus) & LinkStatus;
}

DECLARE_RTL_COND(rtl_phyar_cond)
{
    void __iomem *ioaddr = tp->mmio_addr;
    return RTL_R32(PHYAR) & 0x80000000;
}

static void rtl_disable_msi(struct pci_dev *pdev, struct rtl8169_private *tp)
{
    if (tp->features & RTL_FEATURE_MSI) {
        pci_disable_msi(pdev);
        tp->features &= ~RTL_FEATURE_MSI;
    }
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

static unsigned rtl_try_msi(struct rtl8169_private *tp,
                            const struct rtl_cfg_info *cfg)
{
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned msi = 0;
    u8 cfg2;

    RTL_W8(Cfg9346, Cfg9346_Unlock);
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
    RTL_W8(Cfg9346, Cfg9346_Lock);
    return msi;
}

static void rtl_writephy(struct rtl8169_private *tp, int location, u32 val)
{
    r8169_mdio_write(tp, location, val);
}

static int rtl_readphy(struct rtl8169_private *tp, int location)
{
    return r8169_mdio_read(tp, location);
}

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

static void rtl_lock_work(struct rtl8169_private *tp)
{
    mutex_lock(&tp->wk.mutex);
}

static void rtl_unlock_work(struct rtl8169_private *tp)
{
    mutex_unlock(&tp->wk.mutex);
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

static void rtl8169_get_mac_address(struct rtl8169_private *tp, struct net_device *dev)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int i;
    for (i = 0; i < ETH_ALEN; i++)
        dev->dev_addr[i] = RTL_R8(MAC0 + i);
}

#endif

