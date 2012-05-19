#ifdef CONFIG_KEYBOARD_HA2605
#ifndef _LINUX_HA2605_H
#define _LINUX_HA2605_H

#define	HA_KEY_NAME	"ha2605_key"

struct touch_keys_button {
    int code;
};

struct ha2605_platform_data {
	u16	intr;	/* irq number	  */
    struct touch_keys_button *buttons;
	int nbuttons;

};
#endif /* _LINUX_HA2605_H */
#endif
