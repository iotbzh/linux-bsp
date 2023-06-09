#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/workqueue.h>

#include "remoteproc_internal.h"

#define RCAR_RX_VQ_ID 0
#define RSC_TBL_SIZE		1024

struct rcar_syscon {
	struct regmap *map;
	u32 reg;
	u32 mask;
};

struct rcar_rproc {
	struct device			*dev;
	struct rproc			*rproc;
	struct delayed_work		rproc_work;
	struct mbox_client              cl;
	struct mbox_chan		*tx_ch;
	struct mbox_chan		*rx_ch;
	struct workqueue_struct         *workqueue;
	struct work_struct              vq_work;
	struct rcar_syscon              rsctbl;
	void __iomem                    *rsc_va;
};

static void rcar_rproc_vq_work(struct work_struct *work)
{
	struct rcar_rproc *priv = container_of(work, struct rcar_rproc, vq_work);
	struct rproc *rproc = priv->rproc;
	if (rproc_vq_interrupt(rproc, RCAR_RX_VQ_ID) == IRQ_NONE)
		dev_dbg(&rproc->dev, "no message found in vq%d\n", RCAR_RX_VQ_ID);
};

static void rcar_rproc_rx_callback(struct mbox_client *cl, void *msg)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct rcar_rproc *priv = rproc->priv;

	queue_work(priv->workqueue, &priv->vq_work);
}

static void rcar_rproc_free_mbox(struct rproc *rproc)
{
	struct rcar_rproc *priv = rproc->priv;

	if (priv->tx_ch) {
		mbox_free_channel(priv->tx_ch);
		priv->tx_ch = NULL;
	}

	if (priv->rx_ch) {
		mbox_free_channel(priv->rx_ch);
		priv->rx_ch = NULL;
	}
}

static int rcar_rproc_request_mbox(struct rproc *rproc)
{
	struct rcar_rproc *priv = rproc->priv;
	struct device *dev = &rproc->dev;
	struct mbox_client *cl;
	int ret = 0;

	cl = &priv->cl;
	cl->dev = dev->parent;
	cl->tx_block = true;
	cl->tx_tout = 500;
	cl->knows_txdone = false;
	cl->rx_callback = rcar_rproc_rx_callback;

	priv->tx_ch = mbox_request_channel_byname(cl, "tx");
	if (IS_ERR(priv->tx_ch)) {
		if (PTR_ERR(priv->tx_ch) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		ret = PTR_ERR(priv->tx_ch);
		dev_dbg(cl->dev, "failed to request mbox tx chan, ret %d\n",
			ret);
		goto err_out;
	}

	priv->rx_ch = mbox_request_channel_byname(cl, "rx");
	if (IS_ERR(priv->rx_ch)) {
		if (PTR_ERR(priv->rx_ch) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		ret = PTR_ERR(priv->rx_ch);
		dev_dbg(cl->dev, "failed to request mbox tx chan, ret %d\n",
			ret);
		goto err_out;
	}
	INIT_WORK(&priv->vq_work, rcar_rproc_vq_work);

	return ret;

err_out:
	if (!IS_ERR(priv->tx_ch))
		mbox_free_channel(priv->tx_ch);
	if (!IS_ERR(priv->rx_ch))
		mbox_free_channel(priv->rx_ch);

	return ret;
}

static void rcar_rproc_kick(struct rproc *rproc, int vqid)
{
	struct rcar_rproc *priv = rproc->priv;
	int err;

	if (!priv->tx_ch)
		return;
	err = mbox_send_message(priv->tx_ch, (void *)&vqid);
	if (err < 0)
		dev_err(&rproc->dev, "%s: failed (err:%d)\n",
			__func__, err);
	return;
}

static int rcar_rproc_attach(struct rproc *rproc)
{
	return 0;
}

static struct rproc_ops rcar_rproc_ops = {
	.attach		= rcar_rproc_attach,
	.kick		= rcar_rproc_kick,
};

static int rcar_rproc_pa_to_da(struct rproc *rproc, phys_addr_t pa, u64 *da)
{
	/* On RCAR da = pa right ? */
	*da = pa;
	return 0;
};

static int rcar_rproc_da_to_pa(struct rproc *rproc, u64 da, phys_addr_t *pa)
{
	/* On RCAR pa = da right ? */
	*pa = da;
	return 0;
};

static int rcar_rproc_mem_alloc(struct rproc *rproc,
				 struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	va = ioremap_wc(mem->dma, mem->len);
	if (IS_ERR_OR_NULL(va)) {
		dev_err(dev, "Unable to map memory region: %pa+%lx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;

	return 0;
}

static int rcar_rproc_mem_release(struct rproc *rproc,
				   struct rproc_mem_entry *mem)
{
	dev_dbg(rproc->dev.parent, "unmap memory: %pa\n", &mem->dma);
	iounmap(mem->va);

	return 0;
}

static int rcar_rproc_parse_memory_regions(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	u64 da;

	/* Register associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {

		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		if (rcar_rproc_pa_to_da(rproc, rmem->base, &da) < 0) {
			dev_err(dev, "memory region not valid %pa\n",
				&rmem->base);
			return -EINVAL;
		}

		mem = rproc_mem_entry_init(dev, NULL,
					   (dma_addr_t)rmem->base,
					   rmem->size, da,
					   rcar_rproc_mem_alloc,
					   rcar_rproc_mem_release,
					   it.node->name);

		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
	}

	return 0;
};

static int rcar_rproc_get_syscon(struct device_node *np, const char *prop,
				  struct rcar_syscon *syscon)
{
	int err = 0;
	syscon->map = syscon_regmap_lookup_by_phandle(np, prop);
	if (IS_ERR(syscon->map)) {
		err = -EPROBE_DEFER;
		syscon->map = NULL;
		goto out;
	}

	err = of_property_read_u32_index(np, prop, 1, &syscon->reg);
	if (err)
		goto out;

	err = of_property_read_u32_index(np, prop, 2, &syscon->mask);

out:
	return err;
}

/*
 * Remoteproc is supposed to fill in the resource table address in the syscon register.
 */
static int rcar_rproc_get_loaded_rsc_table(struct platform_device *pdev,
					struct rproc *rproc, struct rcar_rproc *priv)
{
	struct device *dev = &pdev->dev;
	phys_addr_t rsc_pa;
	u32 rsc_da;
	int err;

	/* See if we can get the resource table */
	err = rcar_rproc_get_syscon(dev->of_node, "rcar,syscfg-rsc-tbl",
				     &priv->rsctbl);
	if (err) {
		/* no rsc table syscon */
		dev_warn(dev, "rsc tbl syscon not supported\n");
		return err;
	}

	err = regmap_read(priv->rsctbl.map, priv->rsctbl.reg, &rsc_da);
	if (err) {
		dev_err(dev, "failed to read rsc tbl addr\n");
		return err;
	}

	if (!rsc_da) {
		/* no rsc table */
		dev_err(dev, "Ressource table empty does device has booted yet ?");
		return -ENOENT;
	}

	err = rcar_rproc_da_to_pa(rproc, rsc_da, &rsc_pa);
	if (err)
		return err;
	/*FIXME: need to unmap */
	priv->rsc_va = ioremap_wc(rsc_pa, RSC_TBL_SIZE);
	if (IS_ERR_OR_NULL(priv->rsc_va)) {
		dev_err(dev, "Unable to map memory region: %pa+%x\n",
			&rsc_pa, RSC_TBL_SIZE);
		priv->rsc_va = NULL;
		return -ENOMEM;
	}

	/*
	 * The resource table is already loaded in device memory, no need
	 * to work with a cached table.
	 */
	rproc->cached_table = NULL;
	/* Assuming the resource table fits in 1kB is fair */
	rproc->table_sz = RSC_TBL_SIZE;
	rproc->table_ptr = (struct resource_table *)priv->rsc_va;

	return 0;
};

static int rcar_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct rcar_rproc *priv;
	struct rproc *rproc;
	int ret;

	rproc = rproc_alloc(dev, np->name, &rcar_rproc_ops,
			    NULL, sizeof(*priv));

	if (!rproc)
		return -ENOMEM;

	priv = rproc->priv;
	priv->rproc = rproc;
	priv->dev = dev;

	dev_set_drvdata(dev, rproc);

	ret = rcar_rproc_get_loaded_rsc_table(pdev, rproc, priv);
	if (ret)
		goto free_rproc;

	ret = rcar_rproc_parse_memory_regions(rproc);
	if (ret)
		goto free_rproc;

	/* Assume rproc is loaded by another component e.g u-boot */
	rproc->state = RPROC_DETACHED;

	priv->workqueue = create_workqueue(dev_name(dev));
	if (!priv->workqueue) {
		dev_err(dev, "cannot create workqueue\n");
		ret = -ENOMEM;
		goto free_resources;
	}

	ret = rcar_rproc_request_mbox(rproc);
	if (ret)
		goto free_wkq;

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "rproc_add failed\n");
		goto free_mb;
	}

	return 0;

free_mb:
	rcar_rproc_free_mbox(rproc);
free_wkq:
	destroy_workqueue(priv->workqueue);
free_resources:
	rproc_resource_cleanup(rproc);
free_rproc:
	rproc_free(rproc);

	return ret;
}

static int rcar_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct rcar_rproc *priv = rproc->priv;

	rproc_del(rproc);
	rcar_rproc_free_mbox(rproc);
	destroy_workqueue(priv->workqueue);
	if (priv->rsc_va)
		iounmap(priv->rsc_va);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id rcar_rproc_of_match[] = {
	{ .compatible = "renesas,rcar-cr7" },
	{},
};

MODULE_DEVICE_TABLE(of, rcar_rproc_of_match);

static struct platform_driver rcar_rproc_driver = {
	.probe = rcar_rproc_probe,
	.remove = rcar_rproc_remove,
	.driver = {
		.name = "rcar-rproc",
		.of_match_table = rcar_rproc_of_match,
	},
};

module_platform_driver(rcar_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas Gen3 RCAR remote processor control driver");
MODULE_AUTHOR("Julien Massot <julien.massot@iot.bzh>");
