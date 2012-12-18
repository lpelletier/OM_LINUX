typedef struct vpu_mem_desc {
	u32 size;
	dma_addr_t phy_addr;
	u32 cpu_addr;		/* cpu address to free the dma mem */
	u32 virt_uaddr;		/* virtual user space address */
} vpu_mem_desc;

#define VPU_IOC_MAGIC  'V'

#define VPU_IOC_PHYMEM_ALLOC	_IO(VPU_IOC_MAGIC, 0)
#define VPU_IOC_PHYMEM_FREE	_IO(VPU_IOC_MAGIC, 1)
#define VPU_IOC_WAIT4INT	_IO(VPU_IOC_MAGIC, 2)
#define VPU_IOC_PHYMEM_DUMP	_IO(VPU_IOC_MAGIC, 3)
#define VPU_IOC_REG_DUMP	_IO(VPU_IOC_MAGIC, 4)
#define VPU_IOC_LHD		_IO(VPU_IOC_MAGIC, 5)
#define VPU_IOC_VL2CC_FLUSH	_IO(VPU_IOC_MAGIC, 6)

int vl2cc_init(u32 vl2cc_hw_base);
void vl2cc_enable(void);
void vl2cc_flush(void);
void vl2cc_disable(void);
void vl2cc_cleanup(void);
