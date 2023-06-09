#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/platform_device.h>

/* From AP processor to realtime processor */
#define MFISARIICR0 0x0 /* tx */
#define MFISARIICR1 0x8 /* rxdone */
/* From AP realtime to AP processor */
#define MFISAREICR0 0x4 /* txdone */
#define MFISAREICR1 0xc /* rx */

#define INT_BIT BIT(0)
#define TX_BIT BIT(1)

enum {
	IPCC_IRQ_RX,
	IPCC_IRQ_TX,
	IPCC_IRQ_NUM,
};

struct rcar_ipcc {
	struct mbox_controller controller;
	void __iomem *reg_base;
	struct clk *clk;
	spinlock_t lock; /* protect access to IPCC registers */
	int irqs[IPCC_IRQ_NUM];
};

static inline void rcar_ipcc_set_bits(spinlock_t *lock, void __iomem *reg,
				       u32 mask)
{
	unsigned long flags;

	spin_lock_irqsave(lock, flags);
	writel_relaxed(readl_relaxed(reg) | mask, reg);
	spin_unlock_irqrestore(lock, flags);
}

static inline void rcar_ipcc_clr_bits(spinlock_t *lock, void __iomem *reg,
				       u32 mask)
{
	unsigned long flags;

	spin_lock_irqsave(lock, flags);
	writel_relaxed(readl_relaxed(reg) & ~mask, reg);
	spin_unlock_irqrestore(lock, flags);
}

static irqreturn_t rcar_ipcc_rx_irq(int irq, void *data)
{
	struct rcar_ipcc *ipcc = data;
	uint32_t status, chan = 1;
	irqreturn_t ret = IRQ_NONE;

	/* clear irq */
	rcar_ipcc_clr_bits(&ipcc->lock, ipcc->reg_base + MFISAREICR1, INT_BIT);
	
	status = readl_relaxed(ipcc->reg_base + MFISAREICR1);
	if (status & TX_BIT) {
		mbox_chan_received_data(&ipcc->controller.chans[chan], NULL);
		
		/* raise irq on remoteproc rx done */
		rcar_ipcc_set_bits(&ipcc->lock, ipcc->reg_base + MFISARIICR1,
				   INT_BIT);
		ret = IRQ_HANDLED;
	}
	return ret;
}

static irqreturn_t rcar_ipcc_tx_irq(int irq, void *data)
{
	struct rcar_ipcc *ipcc = data;
	irqreturn_t ret = IRQ_NONE;

	uint32_t chan = 0;
	uint32_t status;
	/* clear irq */
	rcar_ipcc_clr_bits(&ipcc->lock, ipcc->reg_base + MFISAREICR0, INT_BIT);
	
	status = readl_relaxed(ipcc->reg_base + MFISARIICR0);
	if (status & TX_BIT) {
		rcar_ipcc_clr_bits(&ipcc->lock, ipcc->reg_base + MFISARIICR0, TX_BIT);
		mbox_chan_txdone(&ipcc->controller.chans[chan], 0);
		ret = IRQ_HANDLED;
	}
	return ret;
}

static int rcar_ipcc_send_data(struct mbox_chan *link, void *data)
{
	struct rcar_ipcc *ipcc = container_of(link->mbox, struct rcar_ipcc,
					       controller);
	uint32_t status;

	status = readl_relaxed(ipcc->reg_base + MFISARIICR0);
	if (status & TX_BIT) {
		dev_err(ipcc->controller.dev, "ERROR tx channel is busy !");
		return -EBUSY;
	}

	/* set channel occupied, and raise irq on remoteproc */
	rcar_ipcc_set_bits(&ipcc->lock, ipcc->reg_base + MFISARIICR0,
				TX_BIT|INT_BIT);
	return 0;
}

static int rcar_ipcc_startup(struct mbox_chan *link)
{
	struct rcar_ipcc *ipcc = container_of(link->mbox, struct rcar_ipcc,
					       controller);
	int ret;

	ret = clk_prepare_enable(ipcc->clk);
	if (ret) {
		dev_err(ipcc->controller.dev, "can not enable the clock\n");
		return ret;
	}

	return 0;
}

static void rcar_ipcc_shutdown(struct mbox_chan *link)
{
	struct rcar_ipcc *ipcc = container_of(link->mbox, struct rcar_ipcc,
					       controller);

	clk_disable_unprepare(ipcc->clk);
}

static const struct mbox_chan_ops rcar_ipcc_ops = {
	.send_data	= rcar_ipcc_send_data,
	.startup	= rcar_ipcc_startup,
	.shutdown	= rcar_ipcc_shutdown,
};

static int rcar_ipcc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rcar_ipcc *ipcc;
	struct resource *res;
	static const char * const irq_name[] = {"rx", "tx"};
	irq_handler_t irq_thread[] = {rcar_ipcc_rx_irq, rcar_ipcc_tx_irq};
	int ret;
	unsigned int i;

	if (!np) {
		dev_err(dev, "No DT found\n");
		return -ENODEV;
	}

	ipcc = devm_kzalloc(dev, sizeof(*ipcc), GFP_KERNEL);
	if (!ipcc)
		return -ENOMEM;

	spin_lock_init(&ipcc->lock);
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ipcc->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ipcc->reg_base))
		return PTR_ERR(ipcc->reg_base);

	/* clock */
	ipcc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ipcc->clk))
		return PTR_ERR(ipcc->clk);

	/* irq */
	for (i = 0; i < IPCC_IRQ_NUM; i++) {
		ipcc->irqs[i] = platform_get_irq_byname(pdev, irq_name[i]);
		if (ipcc->irqs[i] < 0) {
			if (ipcc->irqs[i] != -EPROBE_DEFER)
				dev_err(dev, "no IRQ specified %s\n",
					irq_name[i]);
			ret = ipcc->irqs[i];
			goto err_clk;
		}

		ret = devm_request_threaded_irq(dev, ipcc->irqs[i], NULL,
						irq_thread[i], IRQF_ONESHOT,
						dev_name(dev), ipcc);
		if (ret) {
			dev_err(dev, "failed to request irq %d (%d)\n", i, ret);
			goto err_clk;
		}
	}

	ipcc->controller.dev = dev;
	ipcc->controller.txdone_irq = true;
	ipcc->controller.ops = &rcar_ipcc_ops;
	ipcc->controller.num_chans = 2;
	ipcc->controller.chans = devm_kcalloc(dev, ipcc->controller.num_chans,
						sizeof(*ipcc->controller.chans),
						GFP_KERNEL);

	ret = devm_mbox_controller_register(dev, &ipcc->controller);
	if (ret)
		goto err_clk;
	
	platform_set_drvdata(pdev, ipcc);

	return 0;

err_clk:
	return ret;
}

static int rcar_ipcc_remove(struct platform_device *pdev)
{	
	return 0;
}

static const struct of_device_id rcar_ipcc_of_match[] = {
	{ .compatible = "renesas,rcar-ipcc" },
	{},
};
MODULE_DEVICE_TABLE(of, rcar_ipcc_of_match);

static struct platform_driver rcar_ipcc_driver = {
	.driver = {
		.name = "rcar-ipcc",
		.of_match_table = rcar_ipcc_of_match,
	},
	.probe		= rcar_ipcc_probe,
	.remove		= rcar_ipcc_remove,
};

module_platform_driver(rcar_ipcc_driver);

MODULE_AUTHOR("Julien Massot <julien.massot@iot.bzh>");
MODULE_DESCRIPTION("Renesas RCAR IPCC driver");
MODULE_LICENSE("GPL v2");
