
#define SOC_SHIFT_LEFT		8
#define SOC_SHIFT_RIGHT		13
#define SOC_SHIFT_MAX		18
#define SOC_SHIFT_INVERT	26

#define SOC_ENUM_EXT(xname, xenum, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_ext, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&xenum }

#define SOC_ENUM_SINGLE_EXT(xmask, xtexts) \
{	.mask = xmask, .texts = xtexts }

#define SOC_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_enum_double, .put = snd_soc_put_enum_double, \
	.private_value = (unsigned long)&xenum }

#define SOC_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xmask, xtexts) \
{	.reg = xreg, .shift_l = xshift_l, .shift_r = xshift_r, \
	.mask = xmask, .texts = xtexts }
#define SOC_ENUM_SINGLE(xreg, xshift, xmask, xtexts) \
	SOC_ENUM_DOUBLE(xreg, xshift, xshift, xmask, xtexts)

#define SOC_SINGLE_VALUE(reg, shift, max, invert) ((reg) | ((shift) << SOC_SHIFT_LEFT) |\
	((shift) << SOC_SHIFT_RIGHT) | ((max) << SOC_SHIFT_MAX) | ((invert) << SOC_SHIFT_INVERT))

#define SOC_DOUBLE(xname, reg, shift_left, shift_right, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw, \
	.put = snd_soc_put_volsw, \
	.private_value = (reg) | ((shift_left) << SOC_SHIFT_LEFT) | \
		((shift_right) << SOC_SHIFT_RIGHT) | ((max) << SOC_SHIFT_MAX) | ((invert) << SOC_SHIFT_INVERT) }

#define SOC_SINGLE_EXT(xname, xreg, xshift, xmask, xinvert,\
	 xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmask, xinvert) }

#define SOC_SINGLE(xname, reg, shift, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = snd_soc_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }

struct soc_enum {
	unsigned short reg;
	unsigned short reg2;
	unsigned char shift_l;
	unsigned char shift_r;
	unsigned int mask;
	const char **texts;
	void *dapm;
};

static unsigned int snd_soc_read(int reg);
static void snd_soc_write(int reg, int val);

static int snd_soc_info_enum_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = e->mask;

	if (uinfo->value.enumerated.item > e->mask - 1)
		uinfo->value.enumerated.item = e->mask - 1;
	strcpy(uinfo->value.enumerated.name,
		e->texts[uinfo->value.enumerated.item]);
	return 0;
}

static int snd_soc_info_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	int max = (kcontrol->private_value >> SOC_SHIFT_MAX) & 0xff;
	int shift = (kcontrol->private_value >> SOC_SHIFT_LEFT) & 0x1f;
	int rshift = (kcontrol->private_value >> SOC_SHIFT_RIGHT) & 0x1f;

	if (max == 1)
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = shift == rshift ? 1 : 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}

static int snd_soc_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int reg = kcontrol->private_value & 0xff;
	int shift = (kcontrol->private_value >> SOC_SHIFT_LEFT) & 0x1f;
	int rshift = (kcontrol->private_value >> SOC_SHIFT_RIGHT) & 0x1f;
	int max = (kcontrol->private_value >> SOC_SHIFT_MAX) & 0xff;
	int mask = (1 << fls(max)) - 1;
	int invert = (kcontrol->private_value >> SOC_SHIFT_INVERT) & 0x01;

	ucontrol->value.integer.value[0] =
		(snd_soc_read(reg) >> shift) & mask;
	if (shift != rshift)
		ucontrol->value.integer.value[1] =
			(snd_soc_read(reg) >> rshift) & mask;
	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
		if (shift != rshift)
			ucontrol->value.integer.value[1] =
				max - ucontrol->value.integer.value[1];
	}

	return 0;
}

static int snd_soc_update_bits(unsigned short reg,
				unsigned int mask, unsigned int value)
{
	int change;
	unsigned int old, new;

//	mutex_lock(&io_mutex);
	old = snd_soc_read(reg);
	new = (old & ~mask) | value;
	change = old != new;
	if (change)
		snd_soc_write(reg, new);

//	mutex_unlock(&io_mutex);
	return change;
}

static int snd_soc_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	int reg = kcontrol->private_value & 0xff;
	int shift = (kcontrol->private_value >> SOC_SHIFT_LEFT) & 0x1f;
	int rshift = (kcontrol->private_value >> SOC_SHIFT_RIGHT) & 0x1f;
	int max = (kcontrol->private_value >> SOC_SHIFT_MAX) & 0xff;
	int mask = (1 << fls(max)) - 1;
	int invert = (kcontrol->private_value >> SOC_SHIFT_INVERT) & 0x01;
	unsigned int val, val2, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = max - val;
	val_mask = mask << shift;
	val = val << shift;
	if (shift != rshift) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		if (invert)
			val2 = max - val2;
		val_mask |= mask << rshift;
		val |= val2 << rshift;
	}
	return snd_soc_update_bits(reg, val_mask, val);
}

static int snd_soc_info_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = e->shift_l == e->shift_r ? 1 : 2;
	uinfo->value.enumerated.items = e->mask;

	if (uinfo->value.enumerated.item > e->mask - 1)
		uinfo->value.enumerated.item = e->mask - 1;
	strcpy(uinfo->value.enumerated.name,
		e->texts[uinfo->value.enumerated.item]);
	return 0;
}

static int snd_soc_get_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val, bitmask;

	for (bitmask = 1; bitmask < e->mask; bitmask <<= 1)
		;
	val = snd_soc_read(e->reg);
	ucontrol->value.enumerated.item[0]
		= (val >> e->shift_l) & (bitmask - 1);
	if (e->shift_l != e->shift_r)
		ucontrol->value.enumerated.item[1] =
			(val >> e->shift_r) & (bitmask - 1);

	return 0;
}

static int snd_soc_put_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int val;
	unsigned int mask, bitmask;

	for (bitmask = 1; bitmask < e->mask; bitmask <<= 1)
		;
	if (ucontrol->value.enumerated.item[0] > e->mask - 1)
		return -EINVAL;
	val = ucontrol->value.enumerated.item[0] << e->shift_l;
	mask = (bitmask - 1) << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->mask - 1)
			return -EINVAL;
		val |= ucontrol->value.enumerated.item[1] << e->shift_r;
		mask |= (bitmask - 1) << e->shift_r;
	}

	return snd_soc_update_bits(e->reg, mask, val);
}
