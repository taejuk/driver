#include <linux/module.h>
#include <linux/pci.h>

#define NACC_VENDOR_ID  0x1234
#define NACC_DEVICE_ID  0x11e8

struct nacc_dev {
    struct pci_dev *pdev;
    void __iomem *bar0;
    int irq_count;
};

static irqreturn_t nacc_isr(int irq, void *data)
{
	struct nacc_dev *nacc = data;
	u32 status = ioread32(nacc->bar0 + 0x24);

	if (!status)
		return IRQ_NONE;

	iowrite32(status, nacc->bar0 + 0x64);   /* ack */
	nacc->irq_count++;
	return IRQ_HANDLED;
}

static int nacc_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct nacc_dev *nacc;
    int ret;

    nacc = devm_kzalloc(&pdev->dev, sizeof(*nacc), GFP_KERNEL);
    if(!nacc) return -ENOMEM;

    nacc->pdev = pdev;

    ret = pcim_enable_device(pdev);

    if(ret) return ret;
    // BAR 공간을 안전하게 예약하고 이를 가상 메모리에 매핑
    ret = pcim_iomap_regions(pdev, BIT(0), KBUILD_MODNAME);
    if(ret) return ret;
    // BAR별 가상 주소 I/O 매핑 테이블 반환
    nacc->bar0 = pcim_iomap_table(pdev)[0];
    pci_set_master(pdev);
    pci_set_drvdata(pdev, nacc);

    dev_info(&pdev->dev, "BAR0 phys=%pa len=%llu virt=%p\n",
		 &pdev->resource[0].start,
		 (unsigned long long)pci_resource_len(pdev, 0),
		 nacc->bar0);

	dev_info(&pdev->dev, "ID register = 0x%08x\n", ioread32(nacc->bar0));

    ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI | PCI_IRQ_INTX);
    if (ret < 0)
        return ret;

    dev_info(&pdev->dev, "irq vectors=%d, msi_enabled=%d\n",
        ret, pdev->msi_enabled);

    ret = devm_request_irq(&pdev->dev, pci_irq_vector(pdev, 0),
                nacc_isr, IRQF_SHARED, KBUILD_MODNAME, nacc);
    iowrite32(0x01, nacc->bar0 + 0x60);
	return 0;
}

static void nacc_remove(struct pci_dev *pdev)
{
    dev_info(&pdev->dev, "removed\n");
}

static const struct pci_device_id nacc_ids[] = {
    {PCI_DEVICE(NACC_VENDOR_ID, NACC_DEVICE_ID)},
    {}
};
MODULE_DEVICE_TABLE(pci, nacc_ids);

static struct pci_driver nacc_driver = {
	.name     = KBUILD_MODNAME,
	.id_table = nacc_ids,
	.probe    = nacc_probe,
	.remove   = nacc_remove,
};
module_pci_driver(nacc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Minimal PCI probe for virtual accelerator");
