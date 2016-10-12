#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/pm_runtime.h>

#include "r8169.h"

MODULE_AUTHOR("yurai");
MODULE_DESCRIPTION("RealTek");
MODULE_LICENSE("GPL");

static const struct rtl_cfg_info rtl_cfg_infos[] = {
    [RTL_CFG_0] = {
        .region		= 1,
        .align		= 0,
        .event_slow	= SYSErr | LinkChg | RxOverflow | RxFIFOOver,
        .features	= RTL_FEATURE_GMII,
    },
    [RTL_CFG_1] = {
        .region		= 2,
        .align		= 8,
        .event_slow	= SYSErr | LinkChg | RxOverflow,
        .features	= RTL_FEATURE_GMII | RTL_FEATURE_MSI,
    }
};

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
    .ndo_poll_controller	= NULL,
};

static const struct pci_device_id rtl8169_pci_tbl[] = {

    { PCI_DEVICE(PCI_VENDOR_ID_REALTEK,	0x8168), 0, 0, RTL_CFG_1 },
    {0,},
};

MODULE_DEVICE_TABLE(pci, rtl8169_pci_tbl);

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

    netif_napi_del(&tp->napi);
    unregister_netdev(dev);
    rtl_disable_msi(pdev, tp);

    ioaddr = tp->mmio_addr;
    rtl8169_release_board(pdev, dev, ioaddr);
    dprintk("rtl_remove_one:    ok");
}

static unsigned int rtl8169_xmii_reset_pending(struct rtl8169_private *tp)
{
    return rtl_readphy(tp, MII_BMCR) & BMCR_RESET;
}

DECLARE_RTL_COND(rtl_phy_reset_cond)
{
    return rtl8169_xmii_reset_pending(tp);
}

static void rtl8169_xmii_reset_enable(struct rtl8169_private *tp)
{
    unsigned int val;

    val = rtl_readphy(tp, MII_BMCR) | BMCR_RESET;
    rtl_writephy(tp, MII_BMCR, val & 0xffff);
}

static void rtl_schedule_task(struct rtl8169_private *tp, enum rtl_flag flag)
{
    dprintk("start rtl_schedule_task\n");
    if (!test_and_set_bit(flag, tp->wk.flags))
        schedule_work(&tp->wk.work);
}

static void rtl8169_phy_timer(unsigned long __opaque)
{
    dprintk("start rtl8169_phy_timer\n");
    struct net_device *dev = (struct net_device *)__opaque;
    struct rtl8169_private *tp = netdev_priv(dev);

    rtl_schedule_task(tp, RTL_FLAG_TASK_PHY_PENDING);
}

static int rtl8169_poll(struct napi_struct *napi, int budget)
{
    dprintk("start rtl8169_poll\n");
    struct rtl8169_private *tp = container_of(napi, struct rtl8169_private, napi);
    u16 enable_mask = RTL_EVENT_NAPI | tp->event_slow;
    int work_done = 0;
    u16 status;

    status = rtl_get_events(tp);
    rtl_ack_events(tp, status & ~tp->event_slow);

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

static int rtl_init_pci(struct rtl8169_private *tp, const struct rtl_cfg_info *cfg)
{
    const unsigned int region = cfg->region;
    struct net_device *dev = tp->dev;
    struct pci_dev *pdev = tp->pci_dev;
    int rc = 0;
    /* disable ASPM completely as that cause random device stop working
         * problems as well as full system hangs for some PCIe devices users */
    pci_disable_link_state(pdev, PCIE_LINK_STATE_L0S | PCIE_LINK_STATE_L1 |
                           PCIE_LINK_STATE_CLKPM);

    /* enable device (incl. PCI PM wakeup and hotplug setup) */
    rc = pci_enable_device(pdev);
    if (rc < 0) {
        netif_err(tp, probe, dev, "enable failure\n");
        goto out;
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

    /* Inform about capabilities (64bit/32bit) */
    rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
    if (rc < 0) {
        netif_err(tp, probe, dev, "DMA configuration failed\n");
        goto err_out_free_res_3;
    }
    /* ioremap MMIO region */
    void __iomem *ioaddr = ioremap(pci_resource_start(pdev, region), R8169_REGS_SIZE);
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

out:
    return rc;

err_out_free_res_3:
    pci_release_regions(pdev);
err_out_mwi_2:
    pci_clear_mwi(pdev);
    pci_disable_device(pdev);
    goto out;
}

static int rtl_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    unsigned id = ent->driver_data;
    const struct rtl_cfg_info *cfg = rtl_cfg_infos + id;

    struct rtl8169_private *tp;
    struct mii_if_info *mii;
    struct net_device *dev;
    void __iomem *ioaddr;
    int rc = 0;

    dprintk("rtl_init_one: pdev = %p, ent = %p", pdev, ent);
    dprintk("rtl_init_one  pdev: vendor = %x, device = %x, class = %x, dma_mask = %llx, enable_cnt = %d",
            pdev->vendor, pdev->device, pdev->class, pdev->dma_mask, pdev->enable_cnt.counter);

    printk(KERN_INFO "%s Gigabit Ethernet driver %s loaded\n",
               MODULENAME, RTL8169_VERSION);

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
    // for logging purpose from netif* macros
    tp->msg_enable = netif_msg_init(-1, NETIF_MSG_DRV | NETIF_MSG_PROBE |
                                    NETIF_MSG_IFUP | NETIF_MSG_IFDOWN);
    mii = &tp->mii;
    mii->supports_gmii = !!(cfg->features & RTL_FEATURE_GMII);

    rtl_init_pci(tp, cfg);
    ioaddr = tp->mmio_addr;

    /* Identify chip attached to board */
    rtl8169_get_mac_version(tp, dev);
    dprintk("mac_version = 0x%02x\n", tp->mac_version);

    rtl_irq_disable(tp);

    /* pci_set_master() will enable DMA by setting the bus master bit
    in the PCI_COMMAND register */
    pci_set_master(pdev);

    // handling interrupts via MSI is crucial for link up/down
    // unlocking/locking inside
    tp->features |= rtl_try_msi(tp, cfg);

    /* wk.mutex is used implicitly through rtl_lock_work/rtl_unlock_work.
       It protectes work.
     */
    mutex_init(&tp->wk.mutex);

    rtl8169_get_mac_address(tp, dev);

    // handling NAPI is crucial for link up/down
    netif_napi_add(dev, &tp->napi, rtl8169_poll, R8169_NAPI_WEIGHT);

    /* don't enable SG, IP_CSUM and TSO by default - it might not work
     * properly for all devices */
    dev->features |= NETIF_F_RXCSUM |
        NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX;
    dev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
        NETIF_F_RXCSUM | NETIF_F_HW_VLAN_CTAG_TX |
        NETIF_F_HW_VLAN_CTAG_RX;
    dev->hw_features |= NETIF_F_IPV6_CSUM | NETIF_F_TSO6 | NETIF_F_RXALL | NETIF_F_RXFCS;
    dev->vlan_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
        NETIF_F_HIGHDMA;
    dprintk("rtl_init_one: features ok\n");
    /* tp->event_slow is used for rtl_ack_events/rtl_get_events. tp->event_slow
       value is stored in IntrStatus register. Cfg->event_slow must be properly set.
    */
    tp->event_slow = cfg->event_slow;

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

    netif_info(tp, probe, dev, "%s at 0x%p, %pM, XID %08x IRQ %d\n",
           DEVICE_NAME, ioaddr, dev->dev_addr,
           (u32)(RTL_R32(TxConfig) & 0x9cf0f8ff), pdev->irq);

    dprintk("rtl_init_one:  ok");

    netif_carrier_off(dev);

out:
    return rc;

err_out_msi_4:
    netif_napi_del(&tp->napi);
    rtl_disable_msi(pdev, tp);
    iounmap(ioaddr);

    pci_release_regions(pdev);

    pci_clear_mwi(pdev);
    pci_disable_device(pdev);
    free_netdev(dev);
    goto out;
}

static int rtl8169_set_speed_xmii(struct net_device *dev, u16 speed, u32 adv)
{
    dprintk("rtl8169_set_speed_xmii:    \n");

    struct rtl8169_private *tp = netdev_priv(dev);
    int giga_ctrl, bmcr;
    int auto_nego;

    rtl_writephy(tp, 0x1f, 0x0000);

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

    bmcr = BMCR_ANENABLE | BMCR_ANRESTART;

    rtl_writephy(tp, MII_ADVERTISE, auto_nego);
    rtl_writephy(tp, MII_CTRL1000, giga_ctrl);

    dprintk("auto_nego = %x, giga_ctrl = %x, speed = %d\n",
            auto_nego, giga_ctrl, speed);

    rtl_writephy(tp, MII_BMCR, bmcr);
    return 0;
}

static int rtl8169_set_speed(struct net_device *dev,
                 u16 speed, u32 advertising)
{
    dprintk("rtl8169_set_speed\n");
    struct rtl8169_private *tp = netdev_priv(dev);

    int ret = rtl8169_set_speed_xmii(dev, speed, advertising);
    if (ret < 0)
        goto out;

    if (netif_running(dev) && (advertising & ADVERTISED_1000baseT_Full))
    {
        mod_timer(&tp->timer, jiffies + RTL8169_PHY_TIMEOUT);
    }
out:
    return ret;
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
    rtl8169_xmii_reset_enable(tp);
    rtl_msleep_loop_wait_low(tp, &rtl_phy_reset_cond, 1, 100);
}

static void rtl8169_init_phy(struct net_device *dev, struct rtl8169_private *tp)
{
    rtl_hw_phy_config(dev);
    pci_write_config_byte(tp->pci_dev, PCI_LATENCY_TIMER, 0x40);

    rtl8169_phy_reset(dev, tp);
    rtl8169_set_speed(dev, SPEED_1000,
              ADVERTISED_10baseT_Half | ADVERTISED_10baseT_Full |
              ADVERTISED_100baseT_Half | ADVERTISED_100baseT_Full |
              (tp->mii.supports_gmii ?
                    ADVERTISED_1000baseT_Half |
                    ADVERTISED_1000baseT_Full : 0)
              );
}

static void rtl8169_check_link_status(struct net_device *dev,
                                      struct rtl8169_private *tp,
                                      void __iomem *ioaddr, bool pm)
{
    dprintk("rtl8169_check_link_status");
    if (rtl8169_xmii_link_ok(ioaddr))
    {
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
    u16 status = rtl_get_events(tp) & tp->event_slow;
    rtl_ack_events(tp, status);

    if (unlikely(status & SYSErr))
        rtl8169_pcierr_interrupt(dev);

    if (status & LinkChg)
        rtl8169_check_link_status(dev, tp, tp->mmio_addr, true);

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

/*
* A busy loop could burn quite a few cycles on nowadays CPU.
* Let's delay the execution of the timer for a few ticks.
*/
static void rtl_phy_work(struct rtl8169_private *tp)
{
    dprintk("start rtl_phy_work\n");

    struct timer_list *timer = &tp->timer;
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long timeout = RTL8169_PHY_TIMEOUT;

    if (rtl8169_xmii_reset_pending(tp)) {
        timeout = HZ/10;
        goto out_mod_timer;
    }

    if (rtl8169_xmii_link_ok(ioaddr))
        return;
    rtl8169_xmii_reset_enable(tp);

out_mod_timer:
    mod_timer(timer, jiffies + timeout);
}

static void rtl_task(struct work_struct *work)
{
    dprintk("start rtl_task\n");
    struct rtl8169_private *tp = container_of(work, struct rtl8169_private, wk.work);
    struct net_device *dev = tp->dev;

    rtl_lock_work(tp);

    if (!netif_running(dev) || !test_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags))
        goto out_unlock;

    bool pending = test_and_clear_bit(RTL_FLAG_TASK_SLOW_PENDING, tp->wk.flags);
    if (pending)
        rtl_slow_event_work(tp);
    pending = test_and_clear_bit(RTL_FLAG_TASK_PHY_PENDING, tp->wk.flags);
    if (pending)
        rtl_phy_work(tp);

out_unlock:
    rtl_unlock_work(tp);
}

static int rtl_open(struct net_device *dev)
{
    dprintk("start rtl_open");
    struct rtl8169_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    void __iomem *ioaddr = tp->mmio_addr;
    int retval = 0;

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
    rtl_irq_enable_all(tp);
    netif_start_queue(dev);
    rtl_unlock_work(tp);

    pm_runtime_put_noidle(&pdev->dev);

    rtl8169_check_link_status(dev, tp, ioaddr, false);

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
