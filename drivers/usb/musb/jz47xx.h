
#include "musb_core.h"

#define VSTATE_POWER		(1 << 0)
#define VSTATE_CONNECT		(1 << 1)
#define VSTATE_DISCONNECT 	(1 << 2)
#define VSTATE_DEV_ENABLE	(1 << 3)
#define VSTATE_DEV_DISABLE	(1 << 4)

struct jz47xx_musb_glue {
	int vstate;
	int gpio_otg_id;
	int gpio_vdetect;
	struct device *dev;
	struct musb *musb;
	struct platform_device *musb_device;

	struct delayed_work otgid_work;
	struct workqueue_struct * otgid_queue;

	struct delayed_work vstate_work;
	struct workqueue_struct * vstate_queue;
	volatile int vstate_irq_flag;

	struct delayed_work vsense_work;
	struct workqueue_struct * vsense_queue;

	void (*usb_power_callback)(void);
	void (*usb_connect_callback)(void);
	void (*usb_disconnect_callback)(void);
	void (*set_vbus)(int on);
	irqreturn_t (*irq_dma_handler)(int,void *);
};

static inline struct jz47xx_musb_glue *
				otgid_to_glue(struct delayed_work *delayed)
{
	return container_of(delayed, struct jz47xx_musb_glue, otgid_work);
}

static inline struct jz47xx_musb_glue *
				vstate_to_glue(struct delayed_work *delayed)
{
	return container_of(delayed, struct jz47xx_musb_glue, vstate_work);
}

static inline struct jz47xx_musb_glue *
				vsense_to_glue(struct delayed_work *delayed)
{
	return container_of(delayed, struct jz47xx_musb_glue, vsense_work);
}

static inline void *musb_to_glue(struct musb *musb)
{
	return dev_get_drvdata(musb->controller->parent);
}

