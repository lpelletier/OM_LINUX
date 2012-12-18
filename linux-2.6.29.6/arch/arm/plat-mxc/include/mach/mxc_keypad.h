#ifndef __MACH_MXC_KEYPAD_H
#define __MACH_MXC_KEYPAD_H

#include <linux/input.h>

#define MAX_MATRIX_KEY_ROWS    (8)
#define MAX_MATRIX_KEY_COLS    (8)

struct mxc_keypad_platform_data {
	u16 output_pins;
	u16 input_pins;
	void (*init)(struct input_dev *idev, struct platform_device *pdev);
	void (*exit)(struct platform_device *pdev);
	void (*handle_key)(struct input_dev *, int col, int row, int down);
	int (*is_modifier_key)(int col, int row);
};

extern void mxc_set_keypad_info(struct mxc_keypad_platform_data *info);

#endif
