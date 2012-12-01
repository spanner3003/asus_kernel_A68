/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/wait.h>
#include <linux/mfd/wcd9xxx/core.h>
#include <linux/mfd/wcd9xxx/wcd9xxx_registers.h>
#include <linux/mfd/wcd9xxx/wcd9310_registers.h>
#include <linux/mfd/wcd9xxx/pdata.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include "wcd9310.h"

//Bruno++ Audio debug mode
#include <linux/proc_fs.h>
//Bruno++ Audio debug mode

//Bruno++ for P01
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include <linux/microp_notify.h>
#endif
//Bruno++ for P01

#define WCD9310_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000)

#define NUM_DECIMATORS 10
#define NUM_INTERPOLATORS 7
#define BITS_PER_REG 8
#define TABLA_CFILT_FAST_MODE 0x00
#define TABLA_CFILT_SLOW_MODE 0x40

#define SLIM_CLOSE_TIMEOUT 1000

#define TABLA_I2S_MASTER_MODE_MASK 0x08

#define AIF1_PB 1
#define AIF1_CAP 2
#define AIF2_PB 3
#define AIF2_CAP 4
#define AIF3_CAP 5
#define AIF3_PB  6

#define NUM_CODEC_DAIS 6
#define TABLA_COMP_DIGITAL_GAIN_OFFSET 3

//ASUS HANS++
#include <linux/switch.h>
#include <linux/jiffies.h>
#define JACK_IN_DET 6
#define HS_HOOK_DET 62
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)
#define PM8921_GPIO_BASE                NR_GPIO_IRQS
#define MSM_GPIO_TO_INT(n) (NR_MSM_IRQS + (n))
struct timer_list hs_timer;
struct timer_list button_timer;
struct tabla_priv *g_tabla;
static unsigned long hs_jiffies;
static unsigned long button_jiffies;
static struct work_struct button_press_work;
static struct work_struct button_release_work;
static struct work_struct report_hs_event_work;
static int hook_irq_balance = 1;
static int jack_irq_balance = 0;
int g_bDebugMode = 0;
EXPORT_SYMBOL(g_bDebugMode);
static struct wake_lock jack_in_wake_lock;

extern int g_flag_csvoice_fe_connected;
extern int FMStatus;
//ASUS HANS--

struct tabla_codec_dai_data {
	u32 rate;
	u32 *ch_num;
	u32 ch_act;
	u32 ch_tot;
	u32 ch_mask;
	wait_queue_head_t dai_wait;
};

#define TABLA_MCLK_RATE_12288KHZ 12288000
#define TABLA_MCLK_RATE_9600KHZ 9600000

#define TABLA_GPIO_IRQ_DEBOUNCE_TIME_US 5000

#define TABLA_ACQUIRE_LOCK(x) do { mutex_lock(&x); } while (0)
#define TABLA_RELEASE_LOCK(x) do { mutex_unlock(&x); } while (0)

static const DECLARE_TLV_DB_SCALE(digital_gain, 0, 1, 0);
static const DECLARE_TLV_DB_SCALE(line_gain, 0, 7, 1);
static const DECLARE_TLV_DB_SCALE(analog_gain, 0, 25, 1);
static struct snd_soc_dai_driver tabla_dai[];
static const DECLARE_TLV_DB_SCALE(aux_pga_gain, 0, 2, 0);
static int tabla_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event);
static int tabla_codec_enable_slimtx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event);


enum tabla_bandgap_type {
	TABLA_BANDGAP_OFF = 0,
	TABLA_BANDGAP_AUDIO_MODE,
	TABLA_BANDGAP_MBHC_MODE,
};

/* Codec supports 2 IIR filters */
enum {
	IIR1 = 0,
	IIR2,
	IIR_MAX,
};
/* Codec supports 5 bands */
enum {
	BAND1 = 0,
	BAND2,
	BAND3,
	BAND4,
	BAND5,
	BAND_MAX,
};

enum {
	COMPANDER_1 = 0,
	COMPANDER_2,
	COMPANDER_MAX,
};

enum {
	COMPANDER_FS_8KHZ = 0,
	COMPANDER_FS_16KHZ,
	COMPANDER_FS_32KHZ,
	COMPANDER_FS_48KHZ,
	COMPANDER_FS_96KHZ,
	COMPANDER_FS_192KHZ,
	COMPANDER_FS_MAX,
};

/* Flags to track of PA and DAC state.
 * PA and DAC should be tracked separately as AUXPGA loopback requires
 * only PA to be turned on without DAC being on. */
enum tabla_priv_ack_flags {
	TABLA_HPHL_PA_OFF_ACK = 0,
	TABLA_HPHR_PA_OFF_ACK,
	TABLA_HPHL_DAC_OFF_ACK,
	TABLA_HPHR_DAC_OFF_ACK
};


struct comp_sample_dependent_params {
	u32 peak_det_timeout;
	u32 rms_meter_div_fact;
	u32 rms_meter_resamp_fact;
};

struct tabla_reg_address {
	u16 micb_4_ctl;
	u16 micb_4_int_rbias;
	u16 micb_4_mbhc;
};

struct hpf_work {
	struct tabla_priv *tabla;
	u32 decimator;
	u8 tx_hpf_cut_of_freq;
	struct delayed_work dwork;
};

static struct hpf_work tx_hpf_work[NUM_DECIMATORS];

struct tabla_priv {
	struct snd_soc_codec *codec;
	struct tabla_reg_address reg_addr;
	u32 adc_count;
	u32 cfilt1_cnt;
	u32 cfilt2_cnt;
	u32 cfilt3_cnt;
	u32 rx_bias_count;
	s32 dmic_1_2_clk_cnt;
	s32 dmic_3_4_clk_cnt;
	s32 dmic_5_6_clk_cnt;

	enum tabla_bandgap_type bandgap_type;
	bool mclk_enabled;
	bool clock_active;
	bool config_mode_active;
    
	struct tabla_mbhc_config mbhc_cfg;

	struct wcd9xxx_pdata *pdata;
	u32 anc_slot;

//ASUS Tim++
    	struct gpio_switch_data *headset_jack;
//ASUS Tim--

//ASUS HANS++
	int button_press;
//ASUS HANS--

	/*track tabla interface type*/
	u8 intf_type;

	/* num of slim ports required */
	struct tabla_codec_dai_data dai[NUM_CODEC_DAIS];

	/*compander*/
	int comp_enabled[COMPANDER_MAX];
	u32 comp_fs[COMPANDER_MAX];

	/* Maintain the status of AUX PGA */
	int aux_pga_cnt;
	u8 aux_l_gain;
	u8 aux_r_gain;
};


static const u32 comp_shift[] = {
	0,
	2,
};

static const int comp_rx_path[] = {
	COMPANDER_1,
	COMPANDER_1,
	COMPANDER_2,
	COMPANDER_2,
	COMPANDER_2,
	COMPANDER_2,
	COMPANDER_MAX,
};

static const struct comp_sample_dependent_params comp_samp_params[] = {
	{
		.peak_det_timeout = 0x2,
		.rms_meter_div_fact = 0x8 << 4,
		.rms_meter_resamp_fact = 0x21,
	},
	{
		.peak_det_timeout = 0x3,
		.rms_meter_div_fact = 0x9 << 4,
		.rms_meter_resamp_fact = 0x28,
	},

	{
		.peak_det_timeout = 0x5,
		.rms_meter_div_fact = 0xB << 4,
		.rms_meter_resamp_fact = 0x28,
	},

	{
		.peak_det_timeout = 0x5,
		.rms_meter_div_fact = 0xB << 4,
		.rms_meter_resamp_fact = 0x28,
	},
};

static unsigned short rx_digital_gain_reg[] = {
	TABLA_A_CDC_RX1_VOL_CTL_B2_CTL,
	TABLA_A_CDC_RX2_VOL_CTL_B2_CTL,
	TABLA_A_CDC_RX3_VOL_CTL_B2_CTL,
	TABLA_A_CDC_RX4_VOL_CTL_B2_CTL,
	TABLA_A_CDC_RX5_VOL_CTL_B2_CTL,
	TABLA_A_CDC_RX6_VOL_CTL_B2_CTL,
	TABLA_A_CDC_RX7_VOL_CTL_B2_CTL,
};


static unsigned short tx_digital_gain_reg[] = {
	TABLA_A_CDC_TX1_VOL_CTL_GAIN,
	TABLA_A_CDC_TX2_VOL_CTL_GAIN,
	TABLA_A_CDC_TX3_VOL_CTL_GAIN,
	TABLA_A_CDC_TX4_VOL_CTL_GAIN,
	TABLA_A_CDC_TX5_VOL_CTL_GAIN,
	TABLA_A_CDC_TX6_VOL_CTL_GAIN,
	TABLA_A_CDC_TX7_VOL_CTL_GAIN,
	TABLA_A_CDC_TX8_VOL_CTL_GAIN,
	TABLA_A_CDC_TX9_VOL_CTL_GAIN,
	TABLA_A_CDC_TX10_VOL_CTL_GAIN,
};

static int tabla_codec_enable_charge_pump(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_OTHR_CTL, 0x01,
			0x01);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLSG_CTL, 0x08, 0x08);
		usleep_range(200, 200);
		snd_soc_update_bits(codec, TABLA_A_CP_STATIC, 0x10, 0x00);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_OTHR_RESET_CTL, 0x10,
			0x10);
		usleep_range(20, 20);
		snd_soc_update_bits(codec, TABLA_A_CP_STATIC, 0x08, 0x08);
		snd_soc_update_bits(codec, TABLA_A_CP_STATIC, 0x10, 0x10);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLSG_CTL, 0x08, 0x00);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_OTHR_CTL, 0x01,
			0x00);
		snd_soc_update_bits(codec, TABLA_A_CP_STATIC, 0x08, 0x00);
		break;
	}
	return 0;
}

static int tabla_get_anc_slot(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	ucontrol->value.integer.value[0] = tabla->anc_slot;
	return 0;
}

static int tabla_put_anc_slot(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	tabla->anc_slot = ucontrol->value.integer.value[0];
	return 0;
}

static int tabla_pa_gain_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 ear_pa_gain;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	ear_pa_gain = snd_soc_read(codec, TABLA_A_RX_EAR_GAIN);

	ear_pa_gain = ear_pa_gain >> 5;

	if (ear_pa_gain == 0x00) {
		ucontrol->value.integer.value[0] = 0;
	} else if (ear_pa_gain == 0x04) {
		ucontrol->value.integer.value[0] = 1;
	} else  {
		pr_err("%s: ERROR: Unsupported Ear Gain = 0x%x\n",
				__func__, ear_pa_gain);
		return -EINVAL;
	}

	pr_debug("%s: ear_pa_gain = 0x%x\n", __func__, ear_pa_gain);

	return 0;
}

static int tabla_pa_gain_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	u8 ear_pa_gain;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	pr_debug("%s: ucontrol->value.integer.value[0]  = %ld\n", __func__,
			ucontrol->value.integer.value[0]);

	switch (ucontrol->value.integer.value[0]) {
	case 0:
		ear_pa_gain = 0x00;
		break;
	case 1:
		ear_pa_gain = 0x80;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, TABLA_A_RX_EAR_GAIN, 0xE0, ear_pa_gain);
	return 0;
}

static int tabla_get_iir_enable_audio_mixer(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int iir_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->reg;
	int band_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->shift;

	ucontrol->value.integer.value[0] =
		snd_soc_read(codec, (TABLA_A_CDC_IIR1_CTL + 16 * iir_idx)) &
		(1 << band_idx);

	pr_debug("%s: IIR #%d band #%d enable %d\n", __func__,
		iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[0]);
	return 0;
}

static int tabla_put_iir_enable_audio_mixer(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int iir_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->reg;
	int band_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->shift;
	int value = ucontrol->value.integer.value[0];

	/* Mask first 5 bits, 6-8 are reserved */
	snd_soc_update_bits(codec, (TABLA_A_CDC_IIR1_CTL + 16 * iir_idx),
		(1 << band_idx), (value << band_idx));

	pr_debug("%s: IIR #%d band #%d enable %d\n", __func__,
		iir_idx, band_idx, value);
	return 0;
}
static uint32_t get_iir_band_coeff(struct snd_soc_codec *codec,
				int iir_idx, int band_idx,
				int coeff_idx)
{
	/* Address does not automatically update if reading */
	snd_soc_write(codec,
		(TABLA_A_CDC_IIR1_COEF_B1_CTL + 16 * iir_idx),
		(band_idx * BAND_MAX + coeff_idx) & 0x1F);

	/* Mask bits top 2 bits since they are reserved */
	return ((snd_soc_read(codec,
		(TABLA_A_CDC_IIR1_COEF_B2_CTL + 16 * iir_idx)) << 24) |
		(snd_soc_read(codec,
		(TABLA_A_CDC_IIR1_COEF_B3_CTL + 16 * iir_idx)) << 16) |
		(snd_soc_read(codec,
		(TABLA_A_CDC_IIR1_COEF_B4_CTL + 16 * iir_idx)) << 8) |
		(snd_soc_read(codec,
		(TABLA_A_CDC_IIR1_COEF_B5_CTL + 16 * iir_idx)))) &
		0x3FFFFFFF;
}

static int tabla_get_iir_band_audio_mixer(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int iir_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->reg;
	int band_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->shift;

	ucontrol->value.integer.value[0] =
		get_iir_band_coeff(codec, iir_idx, band_idx, 0);
	ucontrol->value.integer.value[1] =
		get_iir_band_coeff(codec, iir_idx, band_idx, 1);
	ucontrol->value.integer.value[2] =
		get_iir_band_coeff(codec, iir_idx, band_idx, 2);
	ucontrol->value.integer.value[3] =
		get_iir_band_coeff(codec, iir_idx, band_idx, 3);
	ucontrol->value.integer.value[4] =
		get_iir_band_coeff(codec, iir_idx, band_idx, 4);

	pr_debug("%s: IIR #%d band #%d b0 = 0x%x\n"
		"%s: IIR #%d band #%d b1 = 0x%x\n"
		"%s: IIR #%d band #%d b2 = 0x%x\n"
		"%s: IIR #%d band #%d a1 = 0x%x\n"
		"%s: IIR #%d band #%d a2 = 0x%x\n",
		__func__, iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[0],
		__func__, iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[1],
		__func__, iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[2],
		__func__, iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[3],
		__func__, iir_idx, band_idx,
		(uint32_t)ucontrol->value.integer.value[4]);
	return 0;
}

static void set_iir_band_coeff(struct snd_soc_codec *codec,
				int iir_idx, int band_idx,
				int coeff_idx, uint32_t value)
{
	/* Mask top 3 bits, 6-8 are reserved */
	/* Update address manually each time */
	snd_soc_write(codec,
		(TABLA_A_CDC_IIR1_COEF_B1_CTL + 16 * iir_idx),
		(band_idx * BAND_MAX + coeff_idx) & 0x1F);

	/* Mask top 2 bits, 7-8 are reserved */
	snd_soc_write(codec,
		(TABLA_A_CDC_IIR1_COEF_B2_CTL + 16 * iir_idx),
		(value >> 24) & 0x3F);

	/* Isolate 8bits at a time */
	snd_soc_write(codec,
		(TABLA_A_CDC_IIR1_COEF_B3_CTL + 16 * iir_idx),
		(value >> 16) & 0xFF);

	snd_soc_write(codec,
		(TABLA_A_CDC_IIR1_COEF_B4_CTL + 16 * iir_idx),
		(value >> 8) & 0xFF);

	snd_soc_write(codec,
		(TABLA_A_CDC_IIR1_COEF_B5_CTL + 16 * iir_idx),
		value & 0xFF);
}

static int tabla_put_iir_band_audio_mixer(
					struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int iir_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->reg;
	int band_idx = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->shift;

	set_iir_band_coeff(codec, iir_idx, band_idx, 0,
				ucontrol->value.integer.value[0]);
	set_iir_band_coeff(codec, iir_idx, band_idx, 1,
				ucontrol->value.integer.value[1]);
	set_iir_band_coeff(codec, iir_idx, band_idx, 2,
				ucontrol->value.integer.value[2]);
	set_iir_band_coeff(codec, iir_idx, band_idx, 3,
				ucontrol->value.integer.value[3]);
	set_iir_band_coeff(codec, iir_idx, band_idx, 4,
				ucontrol->value.integer.value[4]);

	pr_debug("%s: IIR #%d band #%d b0 = 0x%x\n"
		"%s: IIR #%d band #%d b1 = 0x%x\n"
		"%s: IIR #%d band #%d b2 = 0x%x\n"
		"%s: IIR #%d band #%d a1 = 0x%x\n"
		"%s: IIR #%d band #%d a2 = 0x%x\n",
		__func__, iir_idx, band_idx,
		get_iir_band_coeff(codec, iir_idx, band_idx, 0),
		__func__, iir_idx, band_idx,
		get_iir_band_coeff(codec, iir_idx, band_idx, 1),
		__func__, iir_idx, band_idx,
		get_iir_band_coeff(codec, iir_idx, band_idx, 2),
		__func__, iir_idx, band_idx,
		get_iir_band_coeff(codec, iir_idx, band_idx, 3),
		__func__, iir_idx, band_idx,
		get_iir_band_coeff(codec, iir_idx, band_idx, 4));
	return 0;
}

static int tabla_compander_gain_offset(
	struct snd_soc_codec *codec, u32 enable,
	unsigned int reg, int mask,	int event)
{
	int pa_mode = snd_soc_read(codec, reg) & mask;
	int gain_offset = 0;
	/*  if PMU && enable is 1-> offset is 3
	 *  if PMU && enable is 0-> offset is 0
	 *  if PMD && pa_mode is PA -> offset is 0: PMU compander is off
	 *  if PMD && pa_mode is comp -> offset is -3: PMU compander is on.
	 */

	if (SND_SOC_DAPM_EVENT_ON(event) && (enable != 0))
		gain_offset = TABLA_COMP_DIGITAL_GAIN_OFFSET;
	if (SND_SOC_DAPM_EVENT_OFF(event) && (pa_mode == 0))
		gain_offset = -TABLA_COMP_DIGITAL_GAIN_OFFSET;
	return gain_offset;
}


static int tabla_config_gain_compander(
				struct snd_soc_codec *codec,
				u32 compander, u32 enable, int event)
{
	int value = 0;
	int mask = 1 << 4;
	int gain = 0;
	int gain_offset;
	if (compander >= COMPANDER_MAX) {
		pr_err("%s: Error, invalid compander channel\n", __func__);
		return -EINVAL;
	}

	if ((enable == 0) || SND_SOC_DAPM_EVENT_OFF(event))
		value = 1 << 4;

	if (compander == COMPANDER_1) {
		gain_offset = tabla_compander_gain_offset(codec, enable,
				TABLA_A_RX_HPH_L_GAIN, mask, event);
		snd_soc_update_bits(codec, TABLA_A_RX_HPH_L_GAIN, mask, value);
		gain = snd_soc_read(codec, TABLA_A_CDC_RX1_VOL_CTL_B2_CTL);
		snd_soc_update_bits(codec, TABLA_A_CDC_RX1_VOL_CTL_B2_CTL,
				0xFF, gain - gain_offset);
		gain_offset = tabla_compander_gain_offset(codec, enable,
				TABLA_A_RX_HPH_R_GAIN, mask, event);
		snd_soc_update_bits(codec, TABLA_A_RX_HPH_R_GAIN, mask, value);
		gain = snd_soc_read(codec, TABLA_A_CDC_RX2_VOL_CTL_B2_CTL);
		snd_soc_update_bits(codec, TABLA_A_CDC_RX2_VOL_CTL_B2_CTL,
				0xFF, gain - gain_offset);
	} else if (compander == COMPANDER_2) {
		gain_offset = tabla_compander_gain_offset(codec, enable,
				TABLA_A_RX_LINE_1_GAIN, mask, event);
		snd_soc_update_bits(codec, TABLA_A_RX_LINE_1_GAIN, mask, value);
		gain = snd_soc_read(codec, TABLA_A_CDC_RX3_VOL_CTL_B2_CTL);
		snd_soc_update_bits(codec, TABLA_A_CDC_RX3_VOL_CTL_B2_CTL,
				0xFF, gain - gain_offset);
		gain_offset = tabla_compander_gain_offset(codec, enable,
				TABLA_A_RX_LINE_3_GAIN, mask, event);
		snd_soc_update_bits(codec, TABLA_A_RX_LINE_3_GAIN, mask, value);
		gain = snd_soc_read(codec, TABLA_A_CDC_RX4_VOL_CTL_B2_CTL);
		snd_soc_update_bits(codec, TABLA_A_CDC_RX4_VOL_CTL_B2_CTL,
				0xFF, gain - gain_offset);
		gain_offset = tabla_compander_gain_offset(codec, enable,
				TABLA_A_RX_LINE_2_GAIN, mask, event);
		snd_soc_update_bits(codec, TABLA_A_RX_LINE_2_GAIN, mask, value);
		gain = snd_soc_read(codec, TABLA_A_CDC_RX5_VOL_CTL_B2_CTL);
		snd_soc_update_bits(codec, TABLA_A_CDC_RX5_VOL_CTL_B2_CTL,
				0xFF, gain - gain_offset);
		gain_offset = tabla_compander_gain_offset(codec, enable,
				TABLA_A_RX_LINE_4_GAIN, mask, event);
		snd_soc_update_bits(codec, TABLA_A_RX_LINE_4_GAIN, mask, value);
		gain = snd_soc_read(codec, TABLA_A_CDC_RX6_VOL_CTL_B2_CTL);
		snd_soc_update_bits(codec, TABLA_A_CDC_RX6_VOL_CTL_B2_CTL,
				0xFF, gain - gain_offset);
	}
	return 0;
}
static int tabla_get_compander(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int comp = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->max;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = tabla->comp_enabled[comp];

	return 0;
}

static int tabla_set_compander(struct snd_kcontrol *kcontrol,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	int comp = ((struct soc_multi_mixer_control *)
					kcontrol->private_value)->max;
	int value = ucontrol->value.integer.value[0];

	if (value == tabla->comp_enabled[comp]) {
		pr_debug("%s: compander #%d enable %d no change\n",
			    __func__, comp, value);
		return 0;
	}
	tabla->comp_enabled[comp] = value;
	return 0;
}


static int tabla_config_compander(struct snd_soc_dapm_widget *w,
						  struct snd_kcontrol *kcontrol,
						  int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	u32 rate = tabla->comp_fs[w->shift];

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (tabla->comp_enabled[w->shift] != 0) {
			/* Enable both L/R compander clocks */
			snd_soc_update_bits(codec,
					TABLA_A_CDC_CLK_RX_B2_CTL,
					0x03 << comp_shift[w->shift],
					0x03 << comp_shift[w->shift]);
			/* Clar the HALT for the compander*/
			snd_soc_update_bits(codec,
					TABLA_A_CDC_COMP1_B1_CTL +
					w->shift * 8, 1 << 2, 0);
			/* Toggle compander reset bits*/
			snd_soc_update_bits(codec,
					TABLA_A_CDC_CLK_OTHR_RESET_CTL,
					0x03 << comp_shift[w->shift],
					0x03 << comp_shift[w->shift]);
			snd_soc_update_bits(codec,
					TABLA_A_CDC_CLK_OTHR_RESET_CTL,
					0x03 << comp_shift[w->shift], 0);
			tabla_config_gain_compander(codec, w->shift, 1, event);
			/* Update the RMS meter resampling*/
			snd_soc_update_bits(codec,
					TABLA_A_CDC_COMP1_B3_CTL +
					w->shift * 8, 0xFF, 0x01);
			/* Wait for 1ms*/
			usleep_range(1000, 1000);
		}
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* Set sample rate dependent paramater*/
		if (tabla->comp_enabled[w->shift] != 0) {
			snd_soc_update_bits(codec, TABLA_A_CDC_COMP1_FS_CFG +
			w->shift * 8, 0x03,	rate);
			snd_soc_update_bits(codec, TABLA_A_CDC_COMP1_B2_CTL +
			w->shift * 8, 0x0F,
			comp_samp_params[rate].peak_det_timeout);
			snd_soc_update_bits(codec, TABLA_A_CDC_COMP1_B2_CTL +
			w->shift * 8, 0xF0,
			comp_samp_params[rate].rms_meter_div_fact);
			snd_soc_update_bits(codec, TABLA_A_CDC_COMP1_B3_CTL +
			w->shift * 8, 0xFF,
			comp_samp_params[rate].rms_meter_resamp_fact);
			/* Compander enable -> 0x370/0x378*/
			snd_soc_update_bits(codec, TABLA_A_CDC_COMP1_B1_CTL +
			w->shift * 8, 0x03, 0x03);
		}
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* Halt the compander*/
		snd_soc_update_bits(codec, TABLA_A_CDC_COMP1_B1_CTL +
			w->shift * 8, 1 << 2, 1 << 2);
		break;
	case SND_SOC_DAPM_POST_PMD:
		/* Restore the gain */
		tabla_config_gain_compander(codec, w->shift,
				tabla->comp_enabled[w->shift], event);
		/* Disable the compander*/
		snd_soc_update_bits(codec, TABLA_A_CDC_COMP1_B1_CTL +
			w->shift * 8, 0x03, 0x00);
		/* Turn off the clock for compander in pair*/
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_RX_B2_CTL,
			0x03 << comp_shift[w->shift], 0);
		break;
	}
	return 0;
}

static const char *tabla_ear_pa_gain_text[] = {"POS_6_DB", "POS_2_DB"};
static const struct soc_enum tabla_ear_pa_gain_enum[] = {
		SOC_ENUM_SINGLE_EXT(2, tabla_ear_pa_gain_text),
};

/*cut of frequency for high pass filter*/
static const char *cf_text[] = {
	"MIN_3DB_4Hz", "MIN_3DB_75Hz", "MIN_3DB_150Hz"
};

static const struct soc_enum cf_dec1_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_TX1_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec2_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_TX2_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec3_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_TX3_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec4_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_TX4_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec5_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_TX5_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec6_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_TX6_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec7_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_TX7_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec8_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_TX8_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec9_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_TX9_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_dec10_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_TX10_MUX_CTL, 4, 3, cf_text);

static const struct soc_enum cf_rxmix1_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_RX1_B4_CTL, 1, 3, cf_text);

static const struct soc_enum cf_rxmix2_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_RX2_B4_CTL, 1, 3, cf_text);

static const struct soc_enum cf_rxmix3_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_RX3_B4_CTL, 1, 3, cf_text);

static const struct soc_enum cf_rxmix4_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_RX4_B4_CTL, 1, 3, cf_text);

static const struct soc_enum cf_rxmix5_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_RX5_B4_CTL, 1, 3, cf_text)
;
static const struct soc_enum cf_rxmix6_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_RX6_B4_CTL, 1, 3, cf_text);

static const struct soc_enum cf_rxmix7_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_RX7_B4_CTL, 1, 3, cf_text);

static const struct snd_kcontrol_new tabla_snd_controls[] = {

	SOC_ENUM_EXT("EAR PA Gain", tabla_ear_pa_gain_enum[0],
		tabla_pa_gain_get, tabla_pa_gain_put),

	SOC_SINGLE_TLV("LINEOUT1 Volume", TABLA_A_RX_LINE_1_GAIN, 0, 12, 1,
		line_gain),
	SOC_SINGLE_TLV("LINEOUT2 Volume", TABLA_A_RX_LINE_2_GAIN, 0, 12, 1,
		line_gain),
	SOC_SINGLE_TLV("LINEOUT3 Volume", TABLA_A_RX_LINE_3_GAIN, 0, 12, 1,
		line_gain),
	SOC_SINGLE_TLV("LINEOUT4 Volume", TABLA_A_RX_LINE_4_GAIN, 0, 12, 1,
		line_gain),
	SOC_SINGLE_TLV("LINEOUT5 Volume", TABLA_A_RX_LINE_5_GAIN, 0, 12, 1,
		line_gain),

	SOC_SINGLE_TLV("HPHL Volume", TABLA_A_RX_HPH_L_GAIN, 0, 12, 1,
		line_gain),
	SOC_SINGLE_TLV("HPHR Volume", TABLA_A_RX_HPH_R_GAIN, 0, 12, 1,
		line_gain),

	SOC_SINGLE_S8_TLV("RX1 Digital Volume", TABLA_A_CDC_RX1_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX2 Digital Volume", TABLA_A_CDC_RX2_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX3 Digital Volume", TABLA_A_CDC_RX3_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX4 Digital Volume", TABLA_A_CDC_RX4_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX5 Digital Volume", TABLA_A_CDC_RX5_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX6 Digital Volume", TABLA_A_CDC_RX6_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),
	SOC_SINGLE_S8_TLV("RX7 Digital Volume", TABLA_A_CDC_RX7_VOL_CTL_B2_CTL,
		-84, 40, digital_gain),

	SOC_SINGLE_S8_TLV("DEC1 Volume", TABLA_A_CDC_TX1_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC2 Volume", TABLA_A_CDC_TX2_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC3 Volume", TABLA_A_CDC_TX3_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC4 Volume", TABLA_A_CDC_TX4_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC5 Volume", TABLA_A_CDC_TX5_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC6 Volume", TABLA_A_CDC_TX6_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC7 Volume", TABLA_A_CDC_TX7_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC8 Volume", TABLA_A_CDC_TX8_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC9 Volume", TABLA_A_CDC_TX9_VOL_CTL_GAIN, -84, 40,
		digital_gain),
	SOC_SINGLE_S8_TLV("DEC10 Volume", TABLA_A_CDC_TX10_VOL_CTL_GAIN, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR1 INP1 Volume", TABLA_A_CDC_IIR1_GAIN_B1_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR1 INP2 Volume", TABLA_A_CDC_IIR1_GAIN_B2_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR1 INP3 Volume", TABLA_A_CDC_IIR1_GAIN_B3_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_S8_TLV("IIR1 INP4 Volume", TABLA_A_CDC_IIR1_GAIN_B4_CTL, -84,
		40, digital_gain),
	SOC_SINGLE_TLV("ADC1 Volume", TABLA_A_TX_1_2_EN, 5, 3, 0, analog_gain),
	SOC_SINGLE_TLV("ADC2 Volume", TABLA_A_TX_1_2_EN, 1, 3, 0, analog_gain),
	SOC_SINGLE_TLV("ADC3 Volume", TABLA_A_TX_3_4_EN, 5, 3, 0, analog_gain),
	SOC_SINGLE_TLV("ADC4 Volume", TABLA_A_TX_3_4_EN, 1, 3, 0, analog_gain),
	SOC_SINGLE_TLV("ADC5 Volume", TABLA_A_TX_5_6_EN, 5, 3, 0, analog_gain),
	SOC_SINGLE_TLV("ADC6 Volume", TABLA_A_TX_5_6_EN, 1, 3, 0, analog_gain),

	SOC_SINGLE_TLV("AUX_PGA_LEFT Volume", TABLA_A_AUX_L_GAIN, 0, 39, 0,
		aux_pga_gain),
	SOC_SINGLE_TLV("AUX_PGA_RIGHT Volume", TABLA_A_AUX_R_GAIN, 0, 39, 0,
		aux_pga_gain),

	SOC_SINGLE("MICBIAS1 CAPLESS Switch", TABLA_A_MICB_1_CTL, 4, 1, 1),
	SOC_SINGLE("MICBIAS2 CAPLESS Switch", TABLA_A_MICB_2_CTL, 4, 1, 1),
	SOC_SINGLE("MICBIAS3 CAPLESS Switch", TABLA_A_MICB_3_CTL, 4, 1, 1),

	SOC_SINGLE_EXT("ANC Slot", SND_SOC_NOPM, 0, 0, 100, tabla_get_anc_slot,
		tabla_put_anc_slot),
	SOC_ENUM("TX1 HPF cut off", cf_dec1_enum),
	SOC_ENUM("TX2 HPF cut off", cf_dec2_enum),
	SOC_ENUM("TX3 HPF cut off", cf_dec3_enum),
	SOC_ENUM("TX4 HPF cut off", cf_dec4_enum),
	SOC_ENUM("TX5 HPF cut off", cf_dec5_enum),
	SOC_ENUM("TX6 HPF cut off", cf_dec6_enum),
	SOC_ENUM("TX7 HPF cut off", cf_dec7_enum),
	SOC_ENUM("TX8 HPF cut off", cf_dec8_enum),
	SOC_ENUM("TX9 HPF cut off", cf_dec9_enum),
	SOC_ENUM("TX10 HPF cut off", cf_dec10_enum),

	SOC_SINGLE("TX1 HPF Switch", TABLA_A_CDC_TX1_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX2 HPF Switch", TABLA_A_CDC_TX2_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX3 HPF Switch", TABLA_A_CDC_TX3_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX4 HPF Switch", TABLA_A_CDC_TX4_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX5 HPF Switch", TABLA_A_CDC_TX5_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX6 HPF Switch", TABLA_A_CDC_TX6_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX7 HPF Switch", TABLA_A_CDC_TX7_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX8 HPF Switch", TABLA_A_CDC_TX8_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX9 HPF Switch", TABLA_A_CDC_TX9_MUX_CTL, 3, 1, 0),
	SOC_SINGLE("TX10 HPF Switch", TABLA_A_CDC_TX10_MUX_CTL, 3, 1, 0),

	SOC_SINGLE("RX1 HPF Switch", TABLA_A_CDC_RX1_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX2 HPF Switch", TABLA_A_CDC_RX2_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX3 HPF Switch", TABLA_A_CDC_RX3_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX4 HPF Switch", TABLA_A_CDC_RX4_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX5 HPF Switch", TABLA_A_CDC_RX5_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX6 HPF Switch", TABLA_A_CDC_RX6_B5_CTL, 2, 1, 0),
	SOC_SINGLE("RX7 HPF Switch", TABLA_A_CDC_RX7_B5_CTL, 2, 1, 0),

	SOC_ENUM("RX1 HPF cut off", cf_rxmix1_enum),
	SOC_ENUM("RX2 HPF cut off", cf_rxmix2_enum),
	SOC_ENUM("RX3 HPF cut off", cf_rxmix3_enum),
	SOC_ENUM("RX4 HPF cut off", cf_rxmix4_enum),
	SOC_ENUM("RX5 HPF cut off", cf_rxmix5_enum),
	SOC_ENUM("RX6 HPF cut off", cf_rxmix6_enum),
	SOC_ENUM("RX7 HPF cut off", cf_rxmix7_enum),

	SOC_SINGLE_EXT("IIR1 Enable Band1", IIR1, BAND1, 1, 0,
	tabla_get_iir_enable_audio_mixer, tabla_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR1 Enable Band2", IIR1, BAND2, 1, 0,
	tabla_get_iir_enable_audio_mixer, tabla_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR1 Enable Band3", IIR1, BAND3, 1, 0,
	tabla_get_iir_enable_audio_mixer, tabla_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR1 Enable Band4", IIR1, BAND4, 1, 0,
	tabla_get_iir_enable_audio_mixer, tabla_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR1 Enable Band5", IIR1, BAND5, 1, 0,
	tabla_get_iir_enable_audio_mixer, tabla_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR2 Enable Band1", IIR2, BAND1, 1, 0,
	tabla_get_iir_enable_audio_mixer, tabla_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR2 Enable Band2", IIR2, BAND2, 1, 0,
	tabla_get_iir_enable_audio_mixer, tabla_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR2 Enable Band3", IIR2, BAND3, 1, 0,
	tabla_get_iir_enable_audio_mixer, tabla_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR2 Enable Band4", IIR2, BAND4, 1, 0,
	tabla_get_iir_enable_audio_mixer, tabla_put_iir_enable_audio_mixer),
	SOC_SINGLE_EXT("IIR2 Enable Band5", IIR2, BAND5, 1, 0,
	tabla_get_iir_enable_audio_mixer, tabla_put_iir_enable_audio_mixer),

	SOC_SINGLE_MULTI_EXT("IIR1 Band1", IIR1, BAND1, 255, 0, 5,
	tabla_get_iir_band_audio_mixer, tabla_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band2", IIR1, BAND2, 255, 0, 5,
	tabla_get_iir_band_audio_mixer, tabla_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band3", IIR1, BAND3, 255, 0, 5,
	tabla_get_iir_band_audio_mixer, tabla_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band4", IIR1, BAND4, 255, 0, 5,
	tabla_get_iir_band_audio_mixer, tabla_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR1 Band5", IIR1, BAND5, 255, 0, 5,
	tabla_get_iir_band_audio_mixer, tabla_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR2 Band1", IIR2, BAND1, 255, 0, 5,
	tabla_get_iir_band_audio_mixer, tabla_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR2 Band2", IIR2, BAND2, 255, 0, 5,
	tabla_get_iir_band_audio_mixer, tabla_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR2 Band3", IIR2, BAND3, 255, 0, 5,
	tabla_get_iir_band_audio_mixer, tabla_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR2 Band4", IIR2, BAND4, 255, 0, 5,
	tabla_get_iir_band_audio_mixer, tabla_put_iir_band_audio_mixer),
	SOC_SINGLE_MULTI_EXT("IIR2 Band5", IIR2, BAND5, 255, 0, 5,
	tabla_get_iir_band_audio_mixer, tabla_put_iir_band_audio_mixer),
	SOC_SINGLE_EXT("COMP1 Switch", SND_SOC_NOPM, 1, COMPANDER_1, 0,
				   tabla_get_compander, tabla_set_compander),
	SOC_SINGLE_EXT("COMP2 Switch", SND_SOC_NOPM, 0, COMPANDER_2, 0,
				   tabla_get_compander, tabla_set_compander),
};

static const struct snd_kcontrol_new tabla_1_x_snd_controls[] = {
	SOC_SINGLE("MICBIAS4 CAPLESS Switch", TABLA_1_A_MICB_4_CTL, 4, 1, 1),
};

static const struct snd_kcontrol_new tabla_2_higher_snd_controls[] = {
	SOC_SINGLE("MICBIAS4 CAPLESS Switch", TABLA_2_A_MICB_4_CTL, 4, 1, 1),
};

static const char *rx_mix1_text[] = {
	"ZERO", "SRC1", "SRC2", "IIR1", "IIR2", "RX1", "RX2", "RX3", "RX4",
		"RX5", "RX6", "RX7"
};

static const char *rx_mix2_text[] = {
	"ZERO", "SRC1", "SRC2", "IIR1", "IIR2"
};

static const char *rx_dsm_text[] = {
	"CIC_OUT", "DSM_INV"
};

static const char *sb_tx1_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC1"
};

static const char *sb_tx2_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC2"
};

static const char *sb_tx3_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC3"
};

static const char *sb_tx4_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC4"
};

static const char *sb_tx5_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC5"
};

static const char *sb_tx6_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC6"
};

static const char const *sb_tx7_to_tx10_mux_text[] = {
	"ZERO", "RMIX1", "RMIX2", "RMIX3", "RMIX4", "RMIX5", "RMIX6", "RMIX7",
		"DEC1", "DEC2", "DEC3", "DEC4", "DEC5", "DEC6", "DEC7", "DEC8",
		"DEC9", "DEC10"
};

static const char *dec1_mux_text[] = {
	"ZERO", "DMIC1", "ADC6",
};

static const char *dec2_mux_text[] = {
	"ZERO", "DMIC2", "ADC5",
};

static const char *dec3_mux_text[] = {
	"ZERO", "DMIC3", "ADC4",
};

static const char *dec4_mux_text[] = {
	"ZERO", "DMIC4", "ADC3",
};

static const char *dec5_mux_text[] = {
	"ZERO", "DMIC5", "ADC2",
};

static const char *dec6_mux_text[] = {
	"ZERO", "DMIC6", "ADC1",
};

static const char const *dec7_mux_text[] = {
	"ZERO", "DMIC1", "DMIC6", "ADC1", "ADC6", "ANC1_FB", "ANC2_FB",
};

static const char *dec8_mux_text[] = {
	"ZERO", "DMIC2", "DMIC5", "ADC2", "ADC5",
};

static const char *dec9_mux_text[] = {
	"ZERO", "DMIC4", "DMIC5", "ADC2", "ADC3", "ADCMB", "ANC1_FB", "ANC2_FB",
};

static const char *dec10_mux_text[] = {
	"ZERO", "DMIC3", "DMIC6", "ADC1", "ADC4", "ADCMB", "ANC1_FB", "ANC2_FB",
};

static const char const *anc_mux_text[] = {
	"ZERO", "ADC1", "ADC2", "ADC3", "ADC4", "ADC5", "ADC6", "ADC_MB",
		"RSVD_1", "DMIC1", "DMIC2", "DMIC3", "DMIC4", "DMIC5", "DMIC6"
};

static const char const *anc1_fb_mux_text[] = {
	"ZERO", "EAR_HPH_L", "EAR_LINE_1",
};

static const char *iir1_inp1_text[] = {
	"ZERO", "DEC1", "DEC2", "DEC3", "DEC4", "DEC5", "DEC6", "DEC7", "DEC8",
	"DEC9", "DEC10", "RX1", "RX2", "RX3", "RX4", "RX5", "RX6", "RX7"
};

static const struct soc_enum rx_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX1_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX1_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx_mix1_inp3_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX1_B2_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx2_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX2_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx2_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX2_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx3_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX3_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx3_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX3_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx4_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX4_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx4_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX4_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx5_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX5_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx5_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX5_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx6_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX6_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx6_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX6_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx7_mix1_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX7_B1_CTL, 0, 12, rx_mix1_text);

static const struct soc_enum rx7_mix1_inp2_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX7_B1_CTL, 4, 12, rx_mix1_text);

static const struct soc_enum rx1_mix2_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX1_B3_CTL, 0, 5, rx_mix2_text);

static const struct soc_enum rx1_mix2_inp2_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX1_B3_CTL, 3, 5, rx_mix2_text);

static const struct soc_enum rx2_mix2_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX2_B3_CTL, 0, 5, rx_mix2_text);

static const struct soc_enum rx2_mix2_inp2_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX2_B3_CTL, 3, 5, rx_mix2_text);

static const struct soc_enum rx3_mix2_inp1_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX3_B3_CTL, 0, 5, rx_mix2_text);

static const struct soc_enum rx3_mix2_inp2_chain_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_RX3_B3_CTL, 3, 5, rx_mix2_text);

static const struct soc_enum rx4_dsm_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_RX4_B6_CTL, 4, 2, rx_dsm_text);

static const struct soc_enum rx6_dsm_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_RX6_B6_CTL, 4, 2, rx_dsm_text);

static const struct soc_enum sb_tx1_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B1_CTL, 0, 9, sb_tx1_mux_text);

static const struct soc_enum sb_tx2_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B2_CTL, 0, 9, sb_tx2_mux_text);

static const struct soc_enum sb_tx3_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B3_CTL, 0, 9, sb_tx3_mux_text);

static const struct soc_enum sb_tx4_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B4_CTL, 0, 9, sb_tx4_mux_text);

static const struct soc_enum sb_tx5_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B5_CTL, 0, 9, sb_tx5_mux_text);

static const struct soc_enum sb_tx6_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B6_CTL, 0, 9, sb_tx6_mux_text);

static const struct soc_enum sb_tx7_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B7_CTL, 0, 18,
			sb_tx7_to_tx10_mux_text);

static const struct soc_enum sb_tx8_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B8_CTL, 0, 18,
			sb_tx7_to_tx10_mux_text);

static const struct soc_enum sb_tx9_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B9_CTL, 0, 18,
			sb_tx7_to_tx10_mux_text);

static const struct soc_enum sb_tx10_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_SB_B10_CTL, 0, 18,
			sb_tx7_to_tx10_mux_text);

static const struct soc_enum dec1_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B1_CTL, 0, 3, dec1_mux_text);

static const struct soc_enum dec2_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B1_CTL, 2, 3, dec2_mux_text);

static const struct soc_enum dec3_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B1_CTL, 4, 3, dec3_mux_text);

static const struct soc_enum dec4_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B1_CTL, 6, 3, dec4_mux_text);

static const struct soc_enum dec5_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B2_CTL, 0, 3, dec5_mux_text);

static const struct soc_enum dec6_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B2_CTL, 2, 3, dec6_mux_text);

static const struct soc_enum dec7_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B2_CTL, 4, 7, dec7_mux_text);

static const struct soc_enum dec8_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B3_CTL, 0, 7, dec8_mux_text);

static const struct soc_enum dec9_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B3_CTL, 3, 8, dec9_mux_text);

static const struct soc_enum dec10_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_TX_B4_CTL, 0, 8, dec10_mux_text);

static const struct soc_enum anc1_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_ANC_B1_CTL, 0, 16, anc_mux_text);

static const struct soc_enum anc2_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_ANC_B1_CTL, 4, 16, anc_mux_text);

static const struct soc_enum anc1_fb_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_ANC_B2_CTL, 0, 3, anc1_fb_mux_text);

static const struct soc_enum iir1_inp1_mux_enum =
	SOC_ENUM_SINGLE(TABLA_A_CDC_CONN_EQ1_B1_CTL, 0, 18, iir1_inp1_text);

static const struct snd_kcontrol_new rx_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP1 Mux", rx_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP2 Mux", rx_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx_mix1_inp3_mux =
	SOC_DAPM_ENUM("RX1 MIX1 INP3 Mux", rx_mix1_inp3_chain_enum);

static const struct snd_kcontrol_new rx2_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP1 Mux", rx2_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx2_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX2 MIX1 INP2 Mux", rx2_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx3_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP1 Mux", rx3_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx3_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX3 MIX1 INP2 Mux", rx3_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx4_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX4 MIX1 INP1 Mux", rx4_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx4_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX4 MIX1 INP2 Mux", rx4_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx5_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX5 MIX1 INP1 Mux", rx5_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx5_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX5 MIX1 INP2 Mux", rx5_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx6_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX6 MIX1 INP1 Mux", rx6_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx6_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX6 MIX1 INP2 Mux", rx6_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx7_mix1_inp1_mux =
	SOC_DAPM_ENUM("RX7 MIX1 INP1 Mux", rx7_mix1_inp1_chain_enum);

static const struct snd_kcontrol_new rx7_mix1_inp2_mux =
	SOC_DAPM_ENUM("RX7 MIX1 INP2 Mux", rx7_mix1_inp2_chain_enum);

static const struct snd_kcontrol_new rx1_mix2_inp1_mux =
	SOC_DAPM_ENUM("RX1 MIX2 INP1 Mux", rx1_mix2_inp1_chain_enum);

static const struct snd_kcontrol_new rx1_mix2_inp2_mux =
	SOC_DAPM_ENUM("RX1 MIX2 INP2 Mux", rx1_mix2_inp2_chain_enum);

static const struct snd_kcontrol_new rx2_mix2_inp1_mux =
	SOC_DAPM_ENUM("RX2 MIX2 INP1 Mux", rx2_mix2_inp1_chain_enum);

static const struct snd_kcontrol_new rx2_mix2_inp2_mux =
	SOC_DAPM_ENUM("RX2 MIX2 INP2 Mux", rx2_mix2_inp2_chain_enum);

static const struct snd_kcontrol_new rx3_mix2_inp1_mux =
	SOC_DAPM_ENUM("RX3 MIX2 INP1 Mux", rx3_mix2_inp1_chain_enum);

static const struct snd_kcontrol_new rx3_mix2_inp2_mux =
	SOC_DAPM_ENUM("RX3 MIX2 INP2 Mux", rx3_mix2_inp2_chain_enum);

static const struct snd_kcontrol_new rx4_dsm_mux =
	SOC_DAPM_ENUM("RX4 DSM MUX Mux", rx4_dsm_enum);

static const struct snd_kcontrol_new rx6_dsm_mux =
	SOC_DAPM_ENUM("RX6 DSM MUX Mux", rx6_dsm_enum);

static const struct snd_kcontrol_new sb_tx1_mux =
	SOC_DAPM_ENUM("SLIM TX1 MUX Mux", sb_tx1_mux_enum);

static const struct snd_kcontrol_new sb_tx2_mux =
	SOC_DAPM_ENUM("SLIM TX2 MUX Mux", sb_tx2_mux_enum);

static const struct snd_kcontrol_new sb_tx3_mux =
	SOC_DAPM_ENUM("SLIM TX3 MUX Mux", sb_tx3_mux_enum);

static const struct snd_kcontrol_new sb_tx4_mux =
	SOC_DAPM_ENUM("SLIM TX4 MUX Mux", sb_tx4_mux_enum);

static const struct snd_kcontrol_new sb_tx5_mux =
	SOC_DAPM_ENUM("SLIM TX5 MUX Mux", sb_tx5_mux_enum);

static const struct snd_kcontrol_new sb_tx6_mux =
	SOC_DAPM_ENUM("SLIM TX6 MUX Mux", sb_tx6_mux_enum);

static const struct snd_kcontrol_new sb_tx7_mux =
	SOC_DAPM_ENUM("SLIM TX7 MUX Mux", sb_tx7_mux_enum);

static const struct snd_kcontrol_new sb_tx8_mux =
	SOC_DAPM_ENUM("SLIM TX8 MUX Mux", sb_tx8_mux_enum);

static const struct snd_kcontrol_new sb_tx9_mux =
	SOC_DAPM_ENUM("SLIM TX9 MUX Mux", sb_tx9_mux_enum);

static const struct snd_kcontrol_new sb_tx10_mux =
	SOC_DAPM_ENUM("SLIM TX10 MUX Mux", sb_tx10_mux_enum);


static int wcd9310_put_dec_enum(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_widget_list *wlist = snd_kcontrol_chip(kcontrol);
	struct snd_soc_dapm_widget *w = wlist->widgets[0];
	struct snd_soc_codec *codec = w->codec;
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int dec_mux, decimator;
	char *dec_name = NULL;
	char *widget_name = NULL;
	char *temp;
	u16 tx_mux_ctl_reg;
	u8 adc_dmic_sel = 0x0;
	int ret = 0;

	if (ucontrol->value.enumerated.item[0] > e->max - 1)
		return -EINVAL;

	dec_mux = ucontrol->value.enumerated.item[0];

	widget_name = kstrndup(w->name, 15, GFP_KERNEL);
	if (!widget_name)
		return -ENOMEM;
	temp = widget_name;

	dec_name = strsep(&widget_name, " ");
	widget_name = temp;
	if (!dec_name) {
		pr_err("%s: Invalid decimator = %s\n", __func__, w->name);
		ret =  -EINVAL;
		goto out;
	}

	ret = kstrtouint(strpbrk(dec_name, "123456789"), 10, &decimator);
	if (ret < 0) {
		pr_err("%s: Invalid decimator = %s\n", __func__, dec_name);
		ret =  -EINVAL;
		goto out;
	}

	dev_dbg(w->dapm->dev, "%s(): widget = %s  dec_name = %s decimator = %u"
		" dec_mux = %u\n", __func__, w->name, dec_name, decimator,
		dec_mux);


	switch (decimator) {
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
		if (dec_mux == 1)
			adc_dmic_sel = 0x1;
		else
			adc_dmic_sel = 0x0;
		break;
	case 7:
	case 8:
	case 9:
	case 10:
		if ((dec_mux == 1) || (dec_mux == 2))
			adc_dmic_sel = 0x1;
		else
			adc_dmic_sel = 0x0;
		break;
	default:
		pr_err("%s: Invalid Decimator = %u\n", __func__, decimator);
		ret = -EINVAL;
		goto out;
	}

	tx_mux_ctl_reg = TABLA_A_CDC_TX1_MUX_CTL + 8 * (decimator - 1);

	snd_soc_update_bits(codec, tx_mux_ctl_reg, 0x1, adc_dmic_sel);

	ret = snd_soc_dapm_put_enum_double(kcontrol, ucontrol);

out:
	kfree(widget_name);
	return ret;
}

#define WCD9310_DEC_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_dapm_get_enum_double, \
	.put = wcd9310_put_dec_enum, \
	.private_value = (unsigned long)&xenum }

static const struct snd_kcontrol_new dec1_mux =
	WCD9310_DEC_ENUM("DEC1 MUX Mux", dec1_mux_enum);

static const struct snd_kcontrol_new dec2_mux =
	WCD9310_DEC_ENUM("DEC2 MUX Mux", dec2_mux_enum);

static const struct snd_kcontrol_new dec3_mux =
	WCD9310_DEC_ENUM("DEC3 MUX Mux", dec3_mux_enum);

static const struct snd_kcontrol_new dec4_mux =
	WCD9310_DEC_ENUM("DEC4 MUX Mux", dec4_mux_enum);

static const struct snd_kcontrol_new dec5_mux =
	WCD9310_DEC_ENUM("DEC5 MUX Mux", dec5_mux_enum);

static const struct snd_kcontrol_new dec6_mux =
	WCD9310_DEC_ENUM("DEC6 MUX Mux", dec6_mux_enum);

static const struct snd_kcontrol_new dec7_mux =
	WCD9310_DEC_ENUM("DEC7 MUX Mux", dec7_mux_enum);

static const struct snd_kcontrol_new dec8_mux =
	WCD9310_DEC_ENUM("DEC8 MUX Mux", dec8_mux_enum);

static const struct snd_kcontrol_new dec9_mux =
	WCD9310_DEC_ENUM("DEC9 MUX Mux", dec9_mux_enum);

static const struct snd_kcontrol_new dec10_mux =
	WCD9310_DEC_ENUM("DEC10 MUX Mux", dec10_mux_enum);

static const struct snd_kcontrol_new iir1_inp1_mux =
	SOC_DAPM_ENUM("IIR1 INP1 Mux", iir1_inp1_mux_enum);

static const struct snd_kcontrol_new anc1_mux =
	SOC_DAPM_ENUM("ANC1 MUX Mux", anc1_mux_enum);

static const struct snd_kcontrol_new anc2_mux =
	SOC_DAPM_ENUM("ANC2 MUX Mux", anc2_mux_enum);

static const struct snd_kcontrol_new anc1_fb_mux =
	SOC_DAPM_ENUM("ANC1 FB MUX Mux", anc1_fb_mux_enum);

static const struct snd_kcontrol_new dac1_switch[] = {
	SOC_DAPM_SINGLE("Switch", TABLA_A_RX_EAR_EN, 5, 1, 0)
};
static const struct snd_kcontrol_new hphl_switch[] = {
	SOC_DAPM_SINGLE("Switch", TABLA_A_RX_HPH_L_DAC_CTL, 6, 1, 0)
};

static const struct snd_kcontrol_new hphl_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", TABLA_A_AUX_L_PA_CONN,
					7, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", TABLA_A_AUX_R_PA_CONN,
					7, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_L_INV Switch",
					TABLA_A_AUX_L_PA_CONN_INV, 7, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R_INV Switch",
					TABLA_A_AUX_R_PA_CONN_INV, 7, 1, 0),
};

static const struct snd_kcontrol_new hphr_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", TABLA_A_AUX_L_PA_CONN,
					6, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", TABLA_A_AUX_R_PA_CONN,
					6, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_L_INV Switch",
					TABLA_A_AUX_L_PA_CONN_INV, 6, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R_INV Switch",
					TABLA_A_AUX_R_PA_CONN_INV, 6, 1, 0),
};

static const struct snd_kcontrol_new lineout1_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", TABLA_A_AUX_L_PA_CONN,
					5, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", TABLA_A_AUX_R_PA_CONN,
					5, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_L_INV Switch",
					TABLA_A_AUX_L_PA_CONN_INV, 5, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R_INV Switch",
					TABLA_A_AUX_R_PA_CONN_INV, 5, 1, 0),
};

static const struct snd_kcontrol_new lineout2_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", TABLA_A_AUX_L_PA_CONN,
					4, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", TABLA_A_AUX_R_PA_CONN,
					4, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_L_INV Switch",
					TABLA_A_AUX_L_PA_CONN_INV, 4, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R_INV Switch",
					TABLA_A_AUX_R_PA_CONN_INV, 4, 1, 0),
};

static const struct snd_kcontrol_new lineout3_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", TABLA_A_AUX_L_PA_CONN,
					3, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", TABLA_A_AUX_R_PA_CONN,
					3, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_L_INV Switch",
					TABLA_A_AUX_L_PA_CONN_INV, 3, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R_INV Switch",
					TABLA_A_AUX_R_PA_CONN_INV, 3, 1, 0),
};

static const struct snd_kcontrol_new lineout4_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", TABLA_A_AUX_L_PA_CONN,
					2, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", TABLA_A_AUX_R_PA_CONN,
					2, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_L_INV Switch",
					TABLA_A_AUX_L_PA_CONN_INV, 2, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R_INV Switch",
					TABLA_A_AUX_R_PA_CONN_INV, 2, 1, 0),
};

static const struct snd_kcontrol_new lineout5_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", TABLA_A_AUX_L_PA_CONN,
					1, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", TABLA_A_AUX_R_PA_CONN,
					1, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_L_INV Switch",
					TABLA_A_AUX_L_PA_CONN_INV, 1, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R_INV Switch",
					TABLA_A_AUX_R_PA_CONN_INV, 1, 1, 0),
};

static const struct snd_kcontrol_new ear_pa_mix[] = {
	SOC_DAPM_SINGLE("AUX_PGA_L Switch", TABLA_A_AUX_L_PA_CONN,
					0, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R Switch", TABLA_A_AUX_R_PA_CONN,
					0, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_L_INV Switch",
					TABLA_A_AUX_L_PA_CONN_INV, 0, 1, 0),
	SOC_DAPM_SINGLE("AUX_PGA_R_INV Switch",
					TABLA_A_AUX_R_PA_CONN_INV, 0, 1, 0),
};

static const struct snd_kcontrol_new lineout3_ground_switch =
	SOC_DAPM_SINGLE("Switch", TABLA_A_RX_LINE_3_DAC_CTL, 6, 1, 0);

static const struct snd_kcontrol_new lineout4_ground_switch =
	SOC_DAPM_SINGLE("Switch", TABLA_A_RX_LINE_4_DAC_CTL, 6, 1, 0);

static void tabla_codec_enable_adc_block(struct snd_soc_codec *codec,
					 int enable)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s %d\n", __func__, enable);

	if (enable) {
		tabla->adc_count++;
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_OTHR_CTL, 0x2, 0x2);
	} else {
		tabla->adc_count--;
		if (!tabla->adc_count)
			snd_soc_update_bits(codec, TABLA_A_CDC_CLK_OTHR_CTL,
					    0x2, 0x0);
	}
}

extern void SetPadCurrentDependOnAudio(bool audioOn);   //avoid P02 mic's noise from charging.
static int tabla_codec_enable_adc(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	u16 adc_reg;
	u8 init_bit_shift;
    int iRetryCount =5;
	pr_debug("%s %d\n", __func__, event);

	if (w->reg == TABLA_A_TX_1_2_EN)
		adc_reg = TABLA_A_TX_1_2_TEST_CTL;
	else if (w->reg == TABLA_A_TX_3_4_EN)
		adc_reg = TABLA_A_TX_3_4_TEST_CTL;
	else if (w->reg == TABLA_A_TX_5_6_EN)
		adc_reg = TABLA_A_TX_5_6_TEST_CTL;
	else {
		pr_err("%s: Error, invalid adc register\n", __func__);
		return -EINVAL;
	}

	if (w->shift == 3)
		init_bit_shift = 6;
	else if  (w->shift == 7)
		init_bit_shift = 7;
	else {
		pr_err("%s: Error, invalid init bit postion adc register\n",
				__func__);
		return -EINVAL;
	}



	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		tabla_codec_enable_adc_block(codec, 1);
		snd_soc_update_bits(codec, adc_reg, 1 << init_bit_shift,
				1 << init_bit_shift);

//Bruno -- for P01 mic
#ifdef CONFIG_EEPROM_NUVOTON        
        if ((w->reg == TABLA_A_TX_3_4_EN) && (w->shift == 7)) {
            printk("[Audio] enable P01 mic!!!\n");
            while (iRetryCount) 
            {
                if((AX_MicroP_setGPIOOutputPin(OUT_uP_AUD_PWR_EN, 1))==0)
                {
                    break;
                }
                iRetryCount--;
                msleep(100);
            }
            SetPadCurrentDependOnAudio(true);
        }        
#endif
//Bruno -- for P01 mic
		break;
	case SND_SOC_DAPM_POST_PMU:

		snd_soc_update_bits(codec, adc_reg, 1 << init_bit_shift, 0x00);

		break;
	case SND_SOC_DAPM_POST_PMD:
		tabla_codec_enable_adc_block(codec, 0);
//Bruno -- for P01 mic
#ifdef CONFIG_EEPROM_NUVOTON        
        if ((w->reg == TABLA_A_TX_3_4_EN) && (w->shift == 7)) {
            printk("[Audio] disable P01 mic!!!\n");
            AX_MicroP_setGPIOOutputPin(OUT_uP_AUD_PWR_EN, 0);        
            SetPadCurrentDependOnAudio(false);
        }        
#endif
//Bruno -- for P01 mic
		break;
	}
	return 0;
}

static void tabla_codec_enable_audio_mode_bandgap(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, TABLA_A_BIAS_CENTRAL_BG_CTL, 0x80,
		0x80);
	snd_soc_update_bits(codec, TABLA_A_BIAS_CENTRAL_BG_CTL, 0x04,
		0x04);
	snd_soc_update_bits(codec, TABLA_A_BIAS_CENTRAL_BG_CTL, 0x01,
		0x01);
	usleep_range(1000, 1000);
	snd_soc_update_bits(codec, TABLA_A_BIAS_CENTRAL_BG_CTL, 0x80,
		0x00);
}

static void tabla_codec_enable_bandgap(struct snd_soc_codec *codec,
	enum tabla_bandgap_type choice)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	/* TODO lock resources accessed by audio streams and threaded
	 * interrupt handlers
	 */

	pr_debug("%s, choice is %d, current is %d\n", __func__, choice,
		tabla->bandgap_type);

	if (tabla->bandgap_type == choice)
		return;

	if ((tabla->bandgap_type == TABLA_BANDGAP_OFF) &&
		(choice == TABLA_BANDGAP_AUDIO_MODE)) {
		tabla_codec_enable_audio_mode_bandgap(codec);
	} else if (choice == TABLA_BANDGAP_OFF) {
		snd_soc_write(codec, TABLA_A_BIAS_CENTRAL_BG_CTL, 0x00);
	} else {
		pr_err("%s: Error, Invalid bandgap settings\n", __func__);
	}
	tabla->bandgap_type = choice;
}

static void tabla_codec_disable_clock_block(struct snd_soc_codec *codec)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	pr_debug("%s\n", __func__);
	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN2, 0x04, 0x00);
	usleep_range(50, 50);
	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN2, 0x02, 0x02);
	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN1, 0x05, 0x00);
	usleep_range(50, 50);
	tabla->clock_active = false;
}

static void tabla_enable_rx_bias(struct snd_soc_codec *codec, u32  enable)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	if (enable) {
		tabla->rx_bias_count++;
		if (tabla->rx_bias_count == 1)
			snd_soc_update_bits(codec, TABLA_A_RX_COM_BIAS,
				0x80, 0x80);
	} else {
		tabla->rx_bias_count--;
		if (!tabla->rx_bias_count)
			snd_soc_update_bits(codec, TABLA_A_RX_COM_BIAS,
				0x80, 0x00);
	}
}

static int tabla_hphr_dac_event(struct snd_soc_dapm_widget *w,
    struct snd_kcontrol *kcontrol, int event)
{
    struct snd_soc_codec *codec = w->codec;

    pr_debug("%s %s %d\n", __func__, w->name, event);

    switch (event) {
    case SND_SOC_DAPM_PRE_PMU:
        snd_soc_update_bits(codec, w->reg, 0x40, 0x40);
        break;
    case SND_SOC_DAPM_POST_PMD:
        snd_soc_update_bits(codec, w->reg, 0x40, 0x00);
        break;
    }
    return 0;
}

static int tabla_hph_pa_event(struct snd_soc_dapm_widget *w,
    struct snd_kcontrol *kcontrol, int event)
{
    pr_debug("%s: event = %d\n", __func__, event);

    switch (event) {
    case SND_SOC_DAPM_PRE_PMU:
        break;

    case SND_SOC_DAPM_POST_PMD:
        pr_debug("%s: sleep 10 ms after %s PA disable.\n", __func__,
                w->name);
        usleep_range(10000, 10000);
        break;
    }
    return 0;
}
static int tabla_codec_enable_config_mode(struct snd_soc_codec *codec,
	int enable)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: enable = %d\n", __func__, enable);
	if (enable) {
		snd_soc_update_bits(codec, TABLA_A_CONFIG_MODE_FREQ, 0x10, 0);
		/* bandgap mode to fast */
		snd_soc_write(codec, TABLA_A_BIAS_CONFIG_MODE_BG_CTL, 0x17);
		usleep_range(5, 5);
		snd_soc_update_bits(codec, TABLA_A_CONFIG_MODE_FREQ, 0x80,
				    0x80);
		snd_soc_update_bits(codec, TABLA_A_CONFIG_MODE_TEST, 0x80,
				    0x80);
		usleep_range(10, 10);
		snd_soc_update_bits(codec, TABLA_A_CONFIG_MODE_TEST, 0x80, 0);
		usleep_range(10000, 10000);
		snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN1, 0x08, 0x08);
	} else {
		snd_soc_update_bits(codec, TABLA_A_BIAS_CONFIG_MODE_BG_CTL, 0x1,
				    0);
		snd_soc_update_bits(codec, TABLA_A_CONFIG_MODE_FREQ, 0x80, 0);
		/* clk source to ext clk and clk buff ref to VBG */
		snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN1, 0x0C, 0x04);
	}
	tabla->config_mode_active = enable ? true : false;

	return 0;
}

static int tabla_codec_enable_clock_block(struct snd_soc_codec *codec,
					  int config_mode)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: config_mode = %d\n", __func__, config_mode);

	/* transit to RCO requires mclk off */
	WARN_ON(snd_soc_read(codec, TABLA_A_CLK_BUFF_EN2) & (1 << 2));
	if (config_mode) {
		/* enable RCO and switch to it */
		tabla_codec_enable_config_mode(codec, 1);
		snd_soc_write(codec, TABLA_A_CLK_BUFF_EN2, 0x02);
		usleep_range(1000, 1000);
	} else {
		/* switch to MCLK */
		snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN1, 0x08, 0x00);
	}

	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN1, 0x01, 0x01);
	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN2, 0x02, 0x00);
	/* on MCLK */
	snd_soc_update_bits(codec, TABLA_A_CLK_BUFF_EN2, 0x04, 0x04);
	snd_soc_update_bits(codec, TABLA_A_CDC_CLK_MCLK_CTL, 0x01, 0x01);
	usleep_range(50, 50);
	tabla->clock_active = true;
	return 0;
}

static int tabla_codec_enable_aux_pga(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		tabla_codec_enable_bandgap(codec,
					TABLA_BANDGAP_AUDIO_MODE);
		tabla_enable_rx_bias(codec, 1);

		snd_soc_update_bits(codec, TABLA_A_AUX_COM_CTL,
						0x08, 0x08);
		/* Enable Zero Cross detect for AUX PGA channel
		 * and set the initial AUX PGA gain to NEG_0P0_DB
		 * to avoid glitches.
		 */
		if (w->reg == TABLA_A_AUX_L_EN) {
			snd_soc_update_bits(codec, TABLA_A_AUX_L_EN,
						0x20, 0x20);
			tabla->aux_l_gain = snd_soc_read(codec,
							TABLA_A_AUX_L_GAIN);
			snd_soc_write(codec, TABLA_A_AUX_L_GAIN, 0x1F);
		} else {
			snd_soc_update_bits(codec, TABLA_A_AUX_R_EN,
						0x20, 0x20);
			tabla->aux_r_gain = snd_soc_read(codec,
							TABLA_A_AUX_R_GAIN);
			snd_soc_write(codec, TABLA_A_AUX_R_GAIN, 0x1F);
		}
		if (tabla->aux_pga_cnt++ == 1
			&& !tabla->mclk_enabled) {
			tabla_codec_enable_clock_block(codec, 1);
			pr_debug("AUX PGA enabled RC osc\n");
		}
		break;

	case SND_SOC_DAPM_POST_PMU:
		if (w->reg == TABLA_A_AUX_L_EN)
			snd_soc_write(codec, TABLA_A_AUX_L_GAIN,
				tabla->aux_l_gain);
		else
			snd_soc_write(codec, TABLA_A_AUX_R_GAIN,
				tabla->aux_r_gain);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		/* Mute AUX PGA channel in use before disabling AUX PGA */
		if (w->reg == TABLA_A_AUX_L_EN) {
			tabla->aux_l_gain = snd_soc_read(codec,
							TABLA_A_AUX_L_GAIN);
			snd_soc_write(codec, TABLA_A_AUX_L_GAIN, 0x1F);
		} else {
			tabla->aux_r_gain = snd_soc_read(codec,
							TABLA_A_AUX_R_GAIN);
			snd_soc_write(codec, TABLA_A_AUX_R_GAIN, 0x1F);
		}
		break;

	case SND_SOC_DAPM_POST_PMD:
		tabla_enable_rx_bias(codec, 0);

		snd_soc_update_bits(codec, TABLA_A_AUX_COM_CTL,
						0x08, 0x00);
		if (w->reg == TABLA_A_AUX_L_EN) {
			snd_soc_write(codec, TABLA_A_AUX_L_GAIN,
					tabla->aux_l_gain);
			snd_soc_update_bits(codec, TABLA_A_AUX_L_EN,
							0x20, 0x00);
		} else {
			snd_soc_write(codec, TABLA_A_AUX_R_GAIN,
					tabla->aux_r_gain);
			snd_soc_update_bits(codec, TABLA_A_AUX_R_EN,
						0x20, 0x00);
		}

		if (tabla->aux_pga_cnt-- == 0) {
				tabla_codec_enable_bandgap(codec,
					TABLA_BANDGAP_OFF);

			if (!tabla->mclk_enabled) {
				tabla_codec_enable_clock_block(codec, 0);
			}
		}
		break;
	}
	return 0;
}

static int tabla_codec_enable_lineout(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	u16 lineout_gain_reg;

	pr_debug("%s %d %s\n", __func__, event, w->name);

	switch (w->shift) {
	case 0:
		lineout_gain_reg = TABLA_A_RX_LINE_1_GAIN;
		break;
	case 1:
		lineout_gain_reg = TABLA_A_RX_LINE_2_GAIN;
		break;
	case 2:
		lineout_gain_reg = TABLA_A_RX_LINE_3_GAIN;
		break;
	case 3:
		lineout_gain_reg = TABLA_A_RX_LINE_4_GAIN;
		break;
	case 4:
		lineout_gain_reg = TABLA_A_RX_LINE_5_GAIN;
		break;
	default:
		pr_err("%s: Error, incorrect lineout register value\n",
			__func__);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, lineout_gain_reg, 0x40, 0x40);
		break;
	case SND_SOC_DAPM_POST_PMU:
		pr_debug("%s: sleeping 16 ms after %s PA turn on\n",
				__func__, w->name);
		usleep_range(16000, 16000);
		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, lineout_gain_reg, 0x40, 0x00);
		break;
	}
	return 0;
}


static int tabla_codec_enable_dmic(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	u8  dmic_clk_en;
	s32 *dmic_clk_cnt;
	unsigned int dmic;
	int ret;

	ret = kstrtouint(strpbrk(w->name, "123456"), 10, &dmic);
	if (ret < 0) {
		pr_err("%s: Invalid DMIC line on the codec\n", __func__);
		return -EINVAL;
	}

	switch (dmic) {
	case 1:
	case 2:
		dmic_clk_en = 0x01;
		dmic_clk_cnt = &(tabla->dmic_1_2_clk_cnt);

		pr_debug("%s() event %d DMIC%d dmic_1_2_clk_cnt %d\n",
			__func__, event,  dmic, *dmic_clk_cnt);

		break;

	case 3:
	case 4:
		dmic_clk_en = 0x04;
		dmic_clk_cnt = &(tabla->dmic_3_4_clk_cnt);

		pr_debug("%s() event %d DMIC%d dmic_3_4_clk_cnt %d\n",
			__func__, event,  dmic, *dmic_clk_cnt);
		break;

	case 5:
	case 6:
		dmic_clk_en = 0x10;
		dmic_clk_cnt = &(tabla->dmic_5_6_clk_cnt);

		pr_debug("%s() event %d DMIC%d dmic_5_6_clk_cnt %d\n",
			__func__, event,  dmic, *dmic_clk_cnt);

		break;

	default:
		pr_err("%s: Invalid DMIC Selection\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:

		(*dmic_clk_cnt)++;
		if (*dmic_clk_cnt == 1)
			snd_soc_update_bits(codec, TABLA_A_CDC_CLK_DMIC_CTL,
					dmic_clk_en, dmic_clk_en);

		break;
	case SND_SOC_DAPM_POST_PMD:

		(*dmic_clk_cnt)--;
		if (*dmic_clk_cnt  == 0)
			snd_soc_update_bits(codec, TABLA_A_CDC_CLK_DMIC_CTL,
					dmic_clk_en, 0);
		break;
	}
	return 0;
}

static int tabla_codec_enable_anc(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	const char *filename;
	const struct firmware *fw;
	int i;
	int ret;
	int num_anc_slots;
	struct anc_header *anc_head;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	u32 anc_writes_size = 0;
	int anc_size_remaining;
	u32 *anc_ptr;
	u16 reg;
	u8 mask, val, old_val;

	pr_debug("%s %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:

		filename = "wcd9310/wcd9310_anc.bin";

		ret = request_firmware(&fw, filename, codec->dev);
		if (ret != 0) {
			dev_err(codec->dev, "Failed to acquire ANC data: %d\n",
				ret);
			return -ENODEV;
		}

		if (fw->size < sizeof(struct anc_header)) {
			dev_err(codec->dev, "Not enough data\n");
			release_firmware(fw);
			return -ENOMEM;
		}

		/* First number is the number of register writes */
		anc_head = (struct anc_header *)(fw->data);
		anc_ptr = (u32 *)((u32)fw->data + sizeof(struct anc_header));
		anc_size_remaining = fw->size - sizeof(struct anc_header);
		num_anc_slots = anc_head->num_anc_slots;

		if (tabla->anc_slot >= num_anc_slots) {
			dev_err(codec->dev, "Invalid ANC slot selected\n");
			release_firmware(fw);
			return -EINVAL;
		}

		for (i = 0; i < num_anc_slots; i++) {

			if (anc_size_remaining < TABLA_PACKED_REG_SIZE) {
				dev_err(codec->dev, "Invalid register format\n");
				release_firmware(fw);
				return -EINVAL;
			}
			anc_writes_size = (u32)(*anc_ptr);
			anc_size_remaining -= sizeof(u32);
			anc_ptr += 1;

			if (anc_writes_size * TABLA_PACKED_REG_SIZE
				> anc_size_remaining) {
				dev_err(codec->dev, "Invalid register format\n");
				release_firmware(fw);
				return -ENOMEM;
			}

			if (tabla->anc_slot == i)
				break;

			anc_size_remaining -= (anc_writes_size *
				TABLA_PACKED_REG_SIZE);
			anc_ptr += anc_writes_size;
		}
		if (i == num_anc_slots) {
			dev_err(codec->dev, "Selected ANC slot not present\n");
			release_firmware(fw);
			return -ENOMEM;
		}

		for (i = 0; i < anc_writes_size; i++) {
			TABLA_CODEC_UNPACK_ENTRY(anc_ptr[i], reg,
				mask, val);
			old_val = snd_soc_read(codec, reg);
			snd_soc_write(codec, reg, (old_val & ~mask) |
				(val & mask));
		}
		release_firmware(fw);

		break;
	case SND_SOC_DAPM_POST_PMD:
		snd_soc_write(codec, TABLA_A_CDC_CLK_ANC_RESET_CTL, 0xFF);
		snd_soc_write(codec, TABLA_A_CDC_CLK_ANC_CLK_EN_CTL, 0);
		break;
	}
	return 0;
}

static void tabla_codec_update_cfilt_usage(struct snd_soc_codec *codec,
					   u8 cfilt_sel, int inc)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	u32 *cfilt_cnt_ptr = NULL;
	u16 micb_cfilt_reg;

	switch (cfilt_sel) {
	case TABLA_CFILT1_SEL:
		cfilt_cnt_ptr = &tabla->cfilt1_cnt;
		micb_cfilt_reg = TABLA_A_MICB_CFILT_1_CTL;
		break;
	case TABLA_CFILT2_SEL:
		cfilt_cnt_ptr = &tabla->cfilt2_cnt;
		micb_cfilt_reg = TABLA_A_MICB_CFILT_2_CTL;
		break;
	case TABLA_CFILT3_SEL:
		cfilt_cnt_ptr = &tabla->cfilt3_cnt;
		micb_cfilt_reg = TABLA_A_MICB_CFILT_3_CTL;
		break;
	default:
		return; /* should not happen */
	}

	if (inc) {
		if (!(*cfilt_cnt_ptr)++) {
			snd_soc_update_bits(codec, micb_cfilt_reg, 0x80, 0x80);
		}
	} else {
		/* check if count not zero, decrement
		 * then check if zero, go ahead disable cfilter
		 */
		if ((*cfilt_cnt_ptr) && !--(*cfilt_cnt_ptr)) {
			snd_soc_update_bits(codec, micb_cfilt_reg, 0x80, 0);
		}
	}
}

static int tabla_find_k_value(unsigned int ldoh_v, unsigned int cfilt_mv)
{
	int rc = -EINVAL;
	unsigned min_mv, max_mv;

	switch (ldoh_v) {
	case TABLA_LDOH_1P95_V:
		min_mv = 160;
		max_mv = 1800;
		break;
	case TABLA_LDOH_2P35_V:
		min_mv = 200;
		max_mv = 2200;
		break;
	case TABLA_LDOH_2P75_V:
		min_mv = 240;
		max_mv = 2600;
		break;
	case TABLA_LDOH_2P85_V:
		min_mv = 250;
		max_mv = 2700;
		break;
	default:
		goto done;
	}

	if (cfilt_mv < min_mv || cfilt_mv > max_mv)
		goto done;

	for (rc = 4; rc <= 44; rc++) {
		min_mv = max_mv * (rc) / 44;
		if (min_mv >= cfilt_mv) {
			rc -= 4;
			break;
		}
	}
done:
	return rc;
}

static int tabla_codec_enable_micbias(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	u16 micb_int_reg;
	int micb_line;
	u8 cfilt_sel_val = 0;
	char *internal1_text = "Internal1";
	char *internal2_text = "Internal2";
	char *internal3_text = "Internal3";

	pr_debug("%s %d\n", __func__, event);
	switch (w->reg) {
	case TABLA_A_MICB_1_CTL:
		micb_int_reg = TABLA_A_MICB_1_INT_RBIAS;
		cfilt_sel_val = tabla->pdata->micbias.bias1_cfilt_sel;
		micb_line = TABLA_MICBIAS1;
		break;
	case TABLA_A_MICB_2_CTL:
		micb_int_reg = TABLA_A_MICB_2_INT_RBIAS;
		cfilt_sel_val = tabla->pdata->micbias.bias2_cfilt_sel;
		micb_line = TABLA_MICBIAS2;
		break;
	case TABLA_A_MICB_3_CTL:
		micb_int_reg = TABLA_A_MICB_3_INT_RBIAS;
		cfilt_sel_val = tabla->pdata->micbias.bias3_cfilt_sel;
		micb_line = TABLA_MICBIAS3;
		break;
	case TABLA_1_A_MICB_4_CTL:
	case TABLA_2_A_MICB_4_CTL:
		micb_int_reg = tabla->reg_addr.micb_4_int_rbias;
		cfilt_sel_val = tabla->pdata->micbias.bias4_cfilt_sel;
		micb_line = TABLA_MICBIAS4;
		break;
	default:
		pr_err("%s: Error, invalid micbias register\n", __func__);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, w->reg, 0x0E, 0x0A);
		tabla_codec_update_cfilt_usage(codec, cfilt_sel_val, 1);

		if (strnstr(w->name, internal1_text, 30))
			snd_soc_update_bits(codec, micb_int_reg, 0xE0, 0xE0);
		else if (strnstr(w->name, internal2_text, 30))
			snd_soc_update_bits(codec, micb_int_reg, 0x1C, 0x1C);
		else if (strnstr(w->name, internal3_text, 30))
			snd_soc_update_bits(codec, micb_int_reg, 0x3, 0x3);

		break;
	case SND_SOC_DAPM_POST_PMU:

		usleep_range(20000, 20000);
		break;

	case SND_SOC_DAPM_POST_PMD:
		if (strnstr(w->name, internal1_text, 30))
			snd_soc_update_bits(codec, micb_int_reg, 0x80, 0x00);
		else if (strnstr(w->name, internal2_text, 30))
			snd_soc_update_bits(codec, micb_int_reg, 0x10, 0x00);
		else if (strnstr(w->name, internal3_text, 30))
			snd_soc_update_bits(codec, micb_int_reg, 0x2, 0x0);

		tabla_codec_update_cfilt_usage(codec, cfilt_sel_val, 0);
		break;
	}

	return 0;
}


static void tx_hpf_corner_freq_callback(struct work_struct *work)
{
	struct delayed_work *hpf_delayed_work;
	struct hpf_work *hpf_work;
	struct tabla_priv *tabla;
	struct snd_soc_codec *codec;
	u16 tx_mux_ctl_reg;
	u8 hpf_cut_of_freq;

	hpf_delayed_work = to_delayed_work(work);
	hpf_work = container_of(hpf_delayed_work, struct hpf_work, dwork);
	tabla = hpf_work->tabla;
	codec = hpf_work->tabla->codec;
	hpf_cut_of_freq = hpf_work->tx_hpf_cut_of_freq;

	tx_mux_ctl_reg = TABLA_A_CDC_TX1_MUX_CTL +
			(hpf_work->decimator - 1) * 8;

	pr_debug("%s(): decimator %u hpf_cut_of_freq 0x%x\n", __func__,
		hpf_work->decimator, (unsigned int)hpf_cut_of_freq);

	snd_soc_update_bits(codec, tx_mux_ctl_reg, 0x30, hpf_cut_of_freq << 4);
}

#define  TX_MUX_CTL_CUT_OFF_FREQ_MASK	0x30
#define  CF_MIN_3DB_4HZ			0x0
#define  CF_MIN_3DB_75HZ		0x1
#define  CF_MIN_3DB_150HZ		0x2

static int tabla_codec_enable_dec(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	unsigned int decimator;
	char *dec_name = NULL;
	char *widget_name = NULL;
	char *temp;
	int ret = 0;
	u16 dec_reset_reg, tx_vol_ctl_reg, tx_mux_ctl_reg;
	u8 dec_hpf_cut_of_freq;
	int offset;


	pr_debug("%s %d\n", __func__, event);

	widget_name = kstrndup(w->name, 15, GFP_KERNEL);
	if (!widget_name)
		return -ENOMEM;
	temp = widget_name;

	dec_name = strsep(&widget_name, " ");
	widget_name = temp;
	if (!dec_name) {
		pr_err("%s: Invalid decimator = %s\n", __func__, w->name);
		ret =  -EINVAL;
		goto out;
	}

	ret = kstrtouint(strpbrk(dec_name, "123456789"), 10, &decimator);
	if (ret < 0) {
		pr_err("%s: Invalid decimator = %s\n", __func__, dec_name);
		ret =  -EINVAL;
		goto out;
	}

	pr_debug("%s(): widget = %s dec_name = %s decimator = %u\n", __func__,
			w->name, dec_name, decimator);

	if (w->reg == TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL) {
		dec_reset_reg = TABLA_A_CDC_CLK_TX_RESET_B1_CTL;
		offset = 0;
	} else if (w->reg == TABLA_A_CDC_CLK_TX_CLK_EN_B2_CTL) {
		dec_reset_reg = TABLA_A_CDC_CLK_TX_RESET_B2_CTL;
		offset = 8;
	} else {
		pr_err("%s: Error, incorrect dec\n", __func__);
		return -EINVAL;
	}

	tx_vol_ctl_reg = TABLA_A_CDC_TX1_VOL_CTL_CFG + 8 * (decimator -1);
	tx_mux_ctl_reg = TABLA_A_CDC_TX1_MUX_CTL + 8 * (decimator - 1);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:

		// Enableable TX digital mute */
		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x01, 0x01);

		snd_soc_update_bits(codec, dec_reset_reg, 1 << w->shift,
			1 << w->shift);
		snd_soc_update_bits(codec, dec_reset_reg, 1 << w->shift, 0x0);

		dec_hpf_cut_of_freq = snd_soc_read(codec, tx_mux_ctl_reg);

		dec_hpf_cut_of_freq = (dec_hpf_cut_of_freq & 0x30) >> 4;

		tx_hpf_work[decimator - 1].tx_hpf_cut_of_freq =
			dec_hpf_cut_of_freq;

		if ((dec_hpf_cut_of_freq != CF_MIN_3DB_150HZ)) {

			/* set cut of freq to CF_MIN_3DB_150HZ (0x1); */
			snd_soc_update_bits(codec, tx_mux_ctl_reg, 0x30,
					    CF_MIN_3DB_150HZ << 4);
		}

		/* enable HPF */
		snd_soc_update_bits(codec, tx_mux_ctl_reg , 0x08, 0x00);

		break;

	case SND_SOC_DAPM_POST_PMU:

		/* Disable TX digital mute */
		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x01, 0x00);

		if (tx_hpf_work[decimator - 1].tx_hpf_cut_of_freq !=
				CF_MIN_3DB_150HZ) {

			schedule_delayed_work(&tx_hpf_work[decimator - 1].dwork,
					msecs_to_jiffies(300));
		}
		/* apply the digital gain after the decimator is enabled*/
		if ((w->shift) < ARRAY_SIZE(rx_digital_gain_reg))
			snd_soc_write(codec,
				  tx_digital_gain_reg[w->shift + offset],
				  snd_soc_read(codec,
				  tx_digital_gain_reg[w->shift + offset])
				  );

		break;

	case SND_SOC_DAPM_PRE_PMD:

		snd_soc_update_bits(codec, tx_vol_ctl_reg, 0x01, 0x01);
		cancel_delayed_work_sync(&tx_hpf_work[decimator - 1].dwork);
		break;

	case SND_SOC_DAPM_POST_PMD:

		snd_soc_update_bits(codec, tx_mux_ctl_reg, 0x08, 0x08);
		snd_soc_update_bits(codec, tx_mux_ctl_reg, 0x30,
			(tx_hpf_work[decimator - 1].tx_hpf_cut_of_freq) << 4);

		break;
	}
out:
	kfree(widget_name);
	return ret;
}

static int tabla_codec_reset_interpolator(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d %s\n", __func__, event, w->name);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_RX_RESET_CTL,
			1 << w->shift, 1 << w->shift);
		snd_soc_update_bits(codec, TABLA_A_CDC_CLK_RX_RESET_CTL,
			1 << w->shift, 0x0);
		break;
	case SND_SOC_DAPM_POST_PMU:
		/* apply the digital gain after the interpolator is enabled*/
		if ((w->shift) < ARRAY_SIZE(rx_digital_gain_reg))
			snd_soc_write(codec,
				  rx_digital_gain_reg[w->shift],
				  snd_soc_read(codec,
				  rx_digital_gain_reg[w->shift])
				  );
		break;
	}
	return 0;
}

static int tabla_codec_enable_ldo_h(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
	case SND_SOC_DAPM_POST_PMD:
		usleep_range(1000, 1000);
		break;
	}
	return 0;
}

static int tabla_codec_enable_rx_bias(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %d\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		tabla_enable_rx_bias(codec, 1);
		break;
	case SND_SOC_DAPM_POST_PMD:
		tabla_enable_rx_bias(codec, 0);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget tabla_dapm_i2s_widgets[] = {
	SND_SOC_DAPM_SUPPLY("RX_I2S_CLK", TABLA_A_CDC_CLK_RX_I2S_CTL,
	4, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("TX_I2S_CLK", TABLA_A_CDC_CLK_TX_I2S_CTL, 4,
	0, NULL, 0),
};

static int tabla_lineout_dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	pr_debug("%s %s %d\n", __func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, w->reg, 0x40, 0x40);
		break;

	case SND_SOC_DAPM_POST_PMD:
		snd_soc_update_bits(codec, w->reg, 0x40, 0x00);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget tabla_1_x_dapm_widgets[] = {
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS4 External", TABLA_1_A_MICB_4_CTL, 7,
				0, tabla_codec_enable_micbias,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_widget tabla_2_higher_dapm_widgets[] = {
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS4 External", TABLA_2_A_MICB_4_CTL, 7,
				0, tabla_codec_enable_micbias,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
				SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route audio_i2s_map[] = {
	{"RX_I2S_CLK", NULL, "CDC_CONN"},
	{"SLIM RX1", NULL, "RX_I2S_CLK"},
	{"SLIM RX2", NULL, "RX_I2S_CLK"},
	{"SLIM RX3", NULL, "RX_I2S_CLK"},
	{"SLIM RX4", NULL, "RX_I2S_CLK"},

	{"SLIM TX7", NULL, "TX_I2S_CLK"},
	{"SLIM TX8", NULL, "TX_I2S_CLK"},
	{"SLIM TX9", NULL, "TX_I2S_CLK"},
	{"SLIM TX10", NULL, "TX_I2S_CLK"},
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* SLIMBUS Connections */

	{"SLIM TX1", NULL, "SLIM TX1 MUX"},
	{"SLIM TX1 MUX", "DEC1", "DEC1 MUX"},

	{"SLIM TX2", NULL, "SLIM TX2 MUX"},
	{"SLIM TX2 MUX", "DEC2", "DEC2 MUX"},

	{"SLIM TX3", NULL, "SLIM TX3 MUX"},
	{"SLIM TX3 MUX", "DEC3", "DEC3 MUX"},
	{"SLIM TX3 MUX", "RMIX1", "RX1 MIX1"},
	{"SLIM TX3 MUX", "RMIX2", "RX2 MIX1"},
	{"SLIM TX3 MUX", "RMIX3", "RX3 MIX1"},
	{"SLIM TX3 MUX", "RMIX4", "RX4 MIX1"},
	{"SLIM TX3 MUX", "RMIX5", "RX5 MIX1"},
	{"SLIM TX3 MUX", "RMIX6", "RX6 MIX1"},
	{"SLIM TX3 MUX", "RMIX7", "RX7 MIX1"},

	{"SLIM TX4", NULL, "SLIM TX4 MUX"},
	{"SLIM TX4 MUX", "DEC4", "DEC4 MUX"},

	{"SLIM TX5", NULL, "SLIM TX5 MUX"},
	{"SLIM TX5 MUX", "DEC5", "DEC5 MUX"},
	{"SLIM TX5 MUX", "RMIX1", "RX1 MIX1"},
	{"SLIM TX5 MUX", "RMIX2", "RX2 MIX1"},
	{"SLIM TX5 MUX", "RMIX3", "RX3 MIX1"},
	{"SLIM TX5 MUX", "RMIX4", "RX4 MIX1"},
	{"SLIM TX5 MUX", "RMIX5", "RX5 MIX1"},
	{"SLIM TX5 MUX", "RMIX6", "RX6 MIX1"},
	{"SLIM TX5 MUX", "RMIX7", "RX7 MIX1"},

	{"SLIM TX6", NULL, "SLIM TX6 MUX"},
	{"SLIM TX6 MUX", "DEC6", "DEC6 MUX"},

	{"SLIM TX7", NULL, "SLIM TX7 MUX"},
	{"SLIM TX7 MUX", "DEC1", "DEC1 MUX"},
	{"SLIM TX7 MUX", "DEC2", "DEC2 MUX"},
	{"SLIM TX7 MUX", "DEC3", "DEC3 MUX"},
	{"SLIM TX7 MUX", "DEC4", "DEC4 MUX"},
	{"SLIM TX7 MUX", "DEC5", "DEC5 MUX"},
	{"SLIM TX7 MUX", "DEC6", "DEC6 MUX"},
	{"SLIM TX7 MUX", "DEC7", "DEC7 MUX"},
	{"SLIM TX7 MUX", "DEC8", "DEC8 MUX"},
	{"SLIM TX7 MUX", "DEC9", "DEC9 MUX"},
	{"SLIM TX7 MUX", "DEC10", "DEC10 MUX"},
	{"SLIM TX7 MUX", "RMIX1", "RX1 MIX1"},
	{"SLIM TX7 MUX", "RMIX2", "RX2 MIX1"},
	{"SLIM TX7 MUX", "RMIX3", "RX3 MIX1"},
	{"SLIM TX7 MUX", "RMIX4", "RX4 MIX1"},
	{"SLIM TX7 MUX", "RMIX5", "RX5 MIX1"},
	{"SLIM TX7 MUX", "RMIX6", "RX6 MIX1"},
	{"SLIM TX7 MUX", "RMIX7", "RX7 MIX1"},

	{"SLIM TX8", NULL, "SLIM TX8 MUX"},
	{"SLIM TX8 MUX", "DEC1", "DEC1 MUX"},
	{"SLIM TX8 MUX", "DEC2", "DEC2 MUX"},
	{"SLIM TX8 MUX", "DEC3", "DEC3 MUX"},
	{"SLIM TX8 MUX", "DEC4", "DEC4 MUX"},
	{"SLIM TX8 MUX", "DEC5", "DEC5 MUX"},
	{"SLIM TX8 MUX", "DEC6", "DEC6 MUX"},
	{"SLIM TX8 MUX", "DEC7", "DEC7 MUX"},
	{"SLIM TX8 MUX", "DEC8", "DEC8 MUX"},
	{"SLIM TX8 MUX", "DEC9", "DEC9 MUX"},
	{"SLIM TX8 MUX", "DEC10", "DEC10 MUX"},

	{"SLIM TX9", NULL, "SLIM TX9 MUX"},
	{"SLIM TX9 MUX", "DEC1", "DEC1 MUX"},
	{"SLIM TX9 MUX", "DEC2", "DEC2 MUX"},
	{"SLIM TX9 MUX", "DEC3", "DEC3 MUX"},
	{"SLIM TX9 MUX", "DEC4", "DEC4 MUX"},
	{"SLIM TX9 MUX", "DEC5", "DEC5 MUX"},
	{"SLIM TX9 MUX", "DEC6", "DEC6 MUX"},
	{"SLIM TX9 MUX", "DEC7", "DEC7 MUX"},
	{"SLIM TX9 MUX", "DEC8", "DEC8 MUX"},
	{"SLIM TX9 MUX", "DEC9", "DEC9 MUX"},
	{"SLIM TX9 MUX", "DEC10", "DEC10 MUX"},

	{"SLIM TX10", NULL, "SLIM TX10 MUX"},
	{"SLIM TX10 MUX", "DEC1", "DEC1 MUX"},
	{"SLIM TX10 MUX", "DEC2", "DEC2 MUX"},
	{"SLIM TX10 MUX", "DEC3", "DEC3 MUX"},
	{"SLIM TX10 MUX", "DEC4", "DEC4 MUX"},
	{"SLIM TX10 MUX", "DEC5", "DEC5 MUX"},
	{"SLIM TX10 MUX", "DEC6", "DEC6 MUX"},
	{"SLIM TX10 MUX", "DEC7", "DEC7 MUX"},
	{"SLIM TX10 MUX", "DEC8", "DEC8 MUX"},
	{"SLIM TX10 MUX", "DEC9", "DEC9 MUX"},
	{"SLIM TX10 MUX", "DEC10", "DEC10 MUX"},

	/* Earpiece (RX MIX1) */
	{"EAR", NULL, "EAR PA"},
	{"EAR PA", NULL, "EAR_PA_MIXER"},
	{"EAR_PA_MIXER", NULL, "DAC1"},
	{"DAC1", NULL, "CP"},

	{"ANC1 FB MUX", "EAR_HPH_L", "RX1 MIX2"},
	{"ANC1 FB MUX", "EAR_LINE_1", "RX2 MIX2"},
	{"ANC", NULL, "ANC1 FB MUX"},

	/* Headset (RX MIX1 and RX MIX2) */
	{"HEADPHONE", NULL, "HPHL"},
	{"HEADPHONE", NULL, "HPHR"},

	{"HPHL", NULL, "HPHL_PA_MIXER"},
	{"HPHL_PA_MIXER", NULL, "HPHL DAC"},

	{"HPHR", NULL, "HPHR_PA_MIXER"},
	{"HPHR_PA_MIXER", NULL, "HPHR DAC"},

	{"HPHL DAC", NULL, "CP"},
	{"HPHR DAC", NULL, "CP"},

	{"ANC", NULL, "ANC1 MUX"},
	{"ANC", NULL, "ANC2 MUX"},
	{"ANC1 MUX", "ADC1", "ADC1"},
	{"ANC1 MUX", "ADC2", "ADC2"},
	{"ANC1 MUX", "ADC3", "ADC3"},
	{"ANC1 MUX", "ADC4", "ADC4"},
	{"ANC2 MUX", "ADC1", "ADC1"},
	{"ANC2 MUX", "ADC2", "ADC2"},
	{"ANC2 MUX", "ADC3", "ADC3"},
	{"ANC2 MUX", "ADC4", "ADC4"},

	{"ANC", NULL, "CDC_CONN"},

	{"DAC1", "Switch", "RX1 CHAIN"},
	{"HPHL DAC", "Switch", "RX1 CHAIN"},
	{"HPHR DAC", NULL, "RX2 CHAIN"},

	{"LINEOUT1", NULL, "LINEOUT1 PA"},
	{"LINEOUT2", NULL, "LINEOUT2 PA"},
	{"LINEOUT3", NULL, "LINEOUT3 PA"},
	{"LINEOUT4", NULL, "LINEOUT4 PA"},
	{"LINEOUT5", NULL, "LINEOUT5 PA"},

	{"LINEOUT1 PA", NULL, "LINEOUT1_PA_MIXER"},
	{"LINEOUT1_PA_MIXER", NULL, "LINEOUT1 DAC"},
	{"LINEOUT2 PA", NULL, "LINEOUT2_PA_MIXER"},
	{"LINEOUT2_PA_MIXER", NULL, "LINEOUT2 DAC"},
	{"LINEOUT3 PA", NULL, "LINEOUT3_PA_MIXER"},
	{"LINEOUT3_PA_MIXER", NULL, "LINEOUT3 DAC"},
	{"LINEOUT4 PA", NULL, "LINEOUT4_PA_MIXER"},
	{"LINEOUT4_PA_MIXER", NULL, "LINEOUT4 DAC"},
	{"LINEOUT5 PA", NULL, "LINEOUT5_PA_MIXER"},
	{"LINEOUT5_PA_MIXER", NULL, "LINEOUT5 DAC"},

	{"LINEOUT1 DAC", NULL, "RX3 MIX2"},
	{"LINEOUT5 DAC", NULL, "RX7 MIX1"},

	{"RX1 CHAIN", NULL, "RX1 MIX2"},
	{"RX2 CHAIN", NULL, "RX2 MIX2"},
	{"RX1 CHAIN", NULL, "ANC"},
	{"RX2 CHAIN", NULL, "ANC"},

	{"CP", NULL, "RX_BIAS"},
	{"LINEOUT1 DAC", NULL, "RX_BIAS"},
	{"LINEOUT2 DAC", NULL, "RX_BIAS"},
	{"LINEOUT3 DAC", NULL, "RX_BIAS"},
	{"LINEOUT4 DAC", NULL, "RX_BIAS"},
	{"LINEOUT5 DAC", NULL, "RX_BIAS"},

	{"RX1 MIX1", NULL, "COMP1_CLK"},
	{"RX2 MIX1", NULL, "COMP1_CLK"},
	{"RX3 MIX1", NULL, "COMP2_CLK"},
	{"RX5 MIX1", NULL, "COMP2_CLK"},


	{"RX1 MIX1", NULL, "RX1 MIX1 INP1"},
	{"RX1 MIX1", NULL, "RX1 MIX1 INP2"},
	{"RX1 MIX1", NULL, "RX1 MIX1 INP3"},
	{"RX2 MIX1", NULL, "RX2 MIX1 INP1"},
	{"RX2 MIX1", NULL, "RX2 MIX1 INP2"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP1"},
	{"RX3 MIX1", NULL, "RX3 MIX1 INP2"},
	{"RX4 MIX1", NULL, "RX4 MIX1 INP1"},
	{"RX4 MIX1", NULL, "RX4 MIX1 INP2"},
	{"RX5 MIX1", NULL, "RX5 MIX1 INP1"},
	{"RX5 MIX1", NULL, "RX5 MIX1 INP2"},
	{"RX6 MIX1", NULL, "RX6 MIX1 INP1"},
	{"RX6 MIX1", NULL, "RX6 MIX1 INP2"},
	{"RX7 MIX1", NULL, "RX7 MIX1 INP1"},
	{"RX7 MIX1", NULL, "RX7 MIX1 INP2"},
	{"RX1 MIX2", NULL, "RX1 MIX1"},
	{"RX1 MIX2", NULL, "RX1 MIX2 INP1"},
	{"RX1 MIX2", NULL, "RX1 MIX2 INP2"},
	{"RX2 MIX2", NULL, "RX2 MIX1"},
	{"RX2 MIX2", NULL, "RX2 MIX2 INP1"},
	{"RX2 MIX2", NULL, "RX2 MIX2 INP2"},
	{"RX3 MIX2", NULL, "RX3 MIX1"},
	{"RX3 MIX2", NULL, "RX3 MIX2 INP1"},
	{"RX3 MIX2", NULL, "RX3 MIX2 INP2"},

	{"RX1 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX1 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX1 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX1 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX1 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX1 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX1 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX1 MIX1 INP1", "IIR1", "IIR1"},
	{"RX1 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX1 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX1 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX1 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX1 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX1 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX1 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX1 MIX1 INP2", "IIR1", "IIR1"},
	{"RX1 MIX1 INP3", "RX1", "SLIM RX1"},
	{"RX1 MIX1 INP3", "RX2", "SLIM RX2"},
	{"RX1 MIX1 INP3", "RX3", "SLIM RX3"},
	{"RX1 MIX1 INP3", "RX4", "SLIM RX4"},
	{"RX1 MIX1 INP3", "RX5", "SLIM RX5"},
	{"RX1 MIX1 INP3", "RX6", "SLIM RX6"},
	{"RX1 MIX1 INP3", "RX7", "SLIM RX7"},
	{"RX2 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX2 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX2 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX2 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX2 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX2 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX2 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX2 MIX1 INP1", "IIR1", "IIR1"},
	{"RX2 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX2 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX2 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX2 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX2 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX2 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX2 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX2 MIX1 INP2", "IIR1", "IIR1"},
	{"RX3 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX3 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX3 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX3 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX3 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX3 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX3 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX3 MIX1 INP1", "IIR1", "IIR1"},
	{"RX3 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX3 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX3 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX3 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX3 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX3 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX3 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX3 MIX1 INP2", "IIR1", "IIR1"},
	{"RX4 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX4 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX4 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX4 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX4 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX4 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX4 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX4 MIX1 INP1", "IIR1", "IIR1"},
	{"RX4 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX4 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX4 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX4 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX4 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX4 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX4 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX4 MIX1 INP2", "IIR1", "IIR1"},
	{"RX5 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX5 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX5 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX5 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX5 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX5 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX5 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX5 MIX1 INP1", "IIR1", "IIR1"},
	{"RX5 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX5 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX5 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX5 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX5 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX5 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX5 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX5 MIX1 INP2", "IIR1", "IIR1"},
	{"RX6 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX6 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX6 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX6 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX6 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX6 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX6 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX6 MIX1 INP1", "IIR1", "IIR1"},
	{"RX6 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX6 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX6 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX6 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX6 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX6 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX6 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX6 MIX1 INP2", "IIR1", "IIR1"},
	{"RX7 MIX1 INP1", "RX1", "SLIM RX1"},
	{"RX7 MIX1 INP1", "RX2", "SLIM RX2"},
	{"RX7 MIX1 INP1", "RX3", "SLIM RX3"},
	{"RX7 MIX1 INP1", "RX4", "SLIM RX4"},
	{"RX7 MIX1 INP1", "RX5", "SLIM RX5"},
	{"RX7 MIX1 INP1", "RX6", "SLIM RX6"},
	{"RX7 MIX1 INP1", "RX7", "SLIM RX7"},
	{"RX7 MIX1 INP1", "IIR1", "IIR1"},
	{"RX7 MIX1 INP2", "RX1", "SLIM RX1"},
	{"RX7 MIX1 INP2", "RX2", "SLIM RX2"},
	{"RX7 MIX1 INP2", "RX3", "SLIM RX3"},
	{"RX7 MIX1 INP2", "RX4", "SLIM RX4"},
	{"RX7 MIX1 INP2", "RX5", "SLIM RX5"},
	{"RX7 MIX1 INP2", "RX6", "SLIM RX6"},
	{"RX7 MIX1 INP2", "RX7", "SLIM RX7"},
	{"RX7 MIX1 INP2", "IIR1", "IIR1"},
	{"RX1 MIX2 INP1", "IIR1", "IIR1"},
	{"RX1 MIX2 INP2", "IIR1", "IIR1"},
	{"RX2 MIX2 INP1", "IIR1", "IIR1"},
	{"RX2 MIX2 INP2", "IIR1", "IIR1"},
	{"RX3 MIX2 INP1", "IIR1", "IIR1"},
	{"RX3 MIX2 INP2", "IIR1", "IIR1"},

	/* Decimator Inputs */
	{"DEC1 MUX", "DMIC1", "DMIC1"},
	{"DEC1 MUX", "ADC6", "ADC6"},
	{"DEC1 MUX", NULL, "CDC_CONN"},
	{"DEC2 MUX", "DMIC2", "DMIC2"},
	{"DEC2 MUX", "ADC5", "ADC5"},
	{"DEC2 MUX", NULL, "CDC_CONN"},
	{"DEC3 MUX", "DMIC3", "DMIC3"},
	{"DEC3 MUX", "ADC4", "ADC4"},
	{"DEC3 MUX", NULL, "CDC_CONN"},
	{"DEC4 MUX", "DMIC4", "DMIC4"},
	{"DEC4 MUX", "ADC3", "ADC3"},
	{"DEC4 MUX", NULL, "CDC_CONN"},
	{"DEC5 MUX", "DMIC5", "DMIC5"},
	{"DEC5 MUX", "ADC2", "ADC2"},
	{"DEC5 MUX", NULL, "CDC_CONN"},
	{"DEC6 MUX", "DMIC6", "DMIC6"},
	{"DEC6 MUX", "ADC1", "ADC1"},
	{"DEC6 MUX", NULL, "CDC_CONN"},
	{"DEC7 MUX", "DMIC1", "DMIC1"},
	{"DEC7 MUX", "DMIC6", "DMIC6"},
	{"DEC7 MUX", "ADC1", "ADC1"},
	{"DEC7 MUX", "ADC6", "ADC6"},
	{"DEC7 MUX", NULL, "CDC_CONN"},
	{"DEC8 MUX", "DMIC2", "DMIC2"},
	{"DEC8 MUX", "DMIC5", "DMIC5"},
	{"DEC8 MUX", "ADC2", "ADC2"},
	{"DEC8 MUX", "ADC5", "ADC5"},
	{"DEC8 MUX", NULL, "CDC_CONN"},
	{"DEC9 MUX", "DMIC4", "DMIC4"},
	{"DEC9 MUX", "DMIC5", "DMIC5"},
	{"DEC9 MUX", "ADC2", "ADC2"},
	{"DEC9 MUX", "ADC3", "ADC3"},
	{"DEC9 MUX", NULL, "CDC_CONN"},
	{"DEC10 MUX", "DMIC3", "DMIC3"},
	{"DEC10 MUX", "DMIC6", "DMIC6"},
	{"DEC10 MUX", "ADC1", "ADC1"},
	{"DEC10 MUX", "ADC4", "ADC4"},
	{"DEC10 MUX", NULL, "CDC_CONN"},

	/* ADC Connections */
	{"ADC1", NULL, "AMIC1"},
	{"ADC2", NULL, "AMIC2"},
	{"ADC3", NULL, "AMIC3"},
	{"ADC4", NULL, "AMIC4"},
	{"ADC5", NULL, "AMIC5"},
	{"ADC6", NULL, "AMIC6"},

	/* AUX PGA Connections */
	{"HPHL_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"HPHL_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"HPHL_PA_MIXER", "AUX_PGA_L_INV Switch", "AUX_PGA_Left"},
	{"HPHL_PA_MIXER", "AUX_PGA_R_INV Switch", "AUX_PGA_Right"},
	{"HPHR_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"HPHR_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"HPHR_PA_MIXER", "AUX_PGA_L_INV Switch", "AUX_PGA_Left"},
	{"HPHR_PA_MIXER", "AUX_PGA_R_INV Switch", "AUX_PGA_Right"},
	{"LINEOUT1_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"LINEOUT1_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"LINEOUT1_PA_MIXER", "AUX_PGA_L_INV Switch", "AUX_PGA_Left"},
	{"LINEOUT1_PA_MIXER", "AUX_PGA_R_INV Switch", "AUX_PGA_Right"},
	{"LINEOUT2_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"LINEOUT2_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"LINEOUT2_PA_MIXER", "AUX_PGA_L_INV Switch", "AUX_PGA_Left"},
	{"LINEOUT2_PA_MIXER", "AUX_PGA_R_INV Switch", "AUX_PGA_Right"},
	{"LINEOUT3_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"LINEOUT3_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"LINEOUT3_PA_MIXER", "AUX_PGA_L_INV Switch", "AUX_PGA_Left"},
	{"LINEOUT3_PA_MIXER", "AUX_PGA_R_INV Switch", "AUX_PGA_Right"},
	{"LINEOUT4_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"LINEOUT4_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"LINEOUT4_PA_MIXER", "AUX_PGA_L_INV Switch", "AUX_PGA_Left"},
	{"LINEOUT4_PA_MIXER", "AUX_PGA_R_INV Switch", "AUX_PGA_Right"},
	{"LINEOUT5_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"LINEOUT5_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"LINEOUT5_PA_MIXER", "AUX_PGA_L_INV Switch", "AUX_PGA_Left"},
	{"LINEOUT5_PA_MIXER", "AUX_PGA_R_INV Switch", "AUX_PGA_Right"},
	{"EAR_PA_MIXER", "AUX_PGA_L Switch", "AUX_PGA_Left"},
	{"EAR_PA_MIXER", "AUX_PGA_R Switch", "AUX_PGA_Right"},
	{"EAR_PA_MIXER", "AUX_PGA_L_INV Switch", "AUX_PGA_Left"},
	{"EAR_PA_MIXER", "AUX_PGA_R_INV Switch", "AUX_PGA_Right"},
	{"AUX_PGA_Left", NULL, "AMIC5"},
	{"AUX_PGA_Right", NULL, "AMIC6"},

	{"IIR1", NULL, "IIR1 INP1 MUX"},
	{"IIR1 INP1 MUX", "DEC1", "DEC1 MUX"},
	{"IIR1 INP1 MUX", "DEC2", "DEC2 MUX"},
	{"IIR1 INP1 MUX", "DEC3", "DEC3 MUX"},
	{"IIR1 INP1 MUX", "DEC4", "DEC4 MUX"},
	{"IIR1 INP1 MUX", "DEC5", "DEC5 MUX"},
	{"IIR1 INP1 MUX", "DEC6", "DEC6 MUX"},
	{"IIR1 INP1 MUX", "DEC7", "DEC7 MUX"},
	{"IIR1 INP1 MUX", "DEC8", "DEC8 MUX"},
	{"IIR1 INP1 MUX", "DEC9", "DEC9 MUX"},
	{"IIR1 INP1 MUX", "DEC10", "DEC10 MUX"},

	{"MIC BIAS1 Internal1", NULL, "LDO_H"},
	{"MIC BIAS1 Internal2", NULL, "LDO_H"},
	{"MIC BIAS1 External", NULL, "LDO_H"},
	{"MIC BIAS2 Internal1", NULL, "LDO_H"},
	{"MIC BIAS2 Internal2", NULL, "LDO_H"},
	{"MIC BIAS2 Internal3", NULL, "LDO_H"},
	{"MIC BIAS2 External", NULL, "LDO_H"},
	{"MIC BIAS3 Internal1", NULL, "LDO_H"},
	{"MIC BIAS3 Internal2", NULL, "LDO_H"},
	{"MIC BIAS3 External", NULL, "LDO_H"},
	{"MIC BIAS4 External", NULL, "LDO_H"},
};

static const struct snd_soc_dapm_route tabla_1_x_lineout_2_to_4_map[] = {

	{"RX4 DSM MUX", "DSM_INV", "RX3 MIX2"},
	{"RX4 DSM MUX", "CIC_OUT", "RX4 MIX1"},

	{"LINEOUT2 DAC", NULL, "RX4 DSM MUX"},

	{"LINEOUT3 DAC", NULL, "RX5 MIX1"},
	{"LINEOUT3 DAC GROUND", "Switch", "RX3 MIX2"},
	{"LINEOUT3 DAC", NULL, "LINEOUT3 DAC GROUND"},

	{"RX6 DSM MUX", "DSM_INV", "RX5 MIX1"},
	{"RX6 DSM MUX", "CIC_OUT", "RX6 MIX1"},

	{"LINEOUT4 DAC", NULL, "RX6 DSM MUX"},
	{"LINEOUT4 DAC GROUND", "Switch", "RX4 DSM MUX"},
	{"LINEOUT4 DAC", NULL, "LINEOUT4 DAC GROUND"},
};


static const struct snd_soc_dapm_route tabla_2_x_lineout_2_to_4_map[] = {

	{"RX4 DSM MUX", "DSM_INV", "RX3 MIX2"},
	{"RX4 DSM MUX", "CIC_OUT", "RX4 MIX1"},

	{"LINEOUT3 DAC", NULL, "RX4 DSM MUX"},

	{"LINEOUT2 DAC", NULL, "RX5 MIX1"},

	{"RX6 DSM MUX", "DSM_INV", "RX5 MIX1"},
	{"RX6 DSM MUX", "CIC_OUT", "RX6 MIX1"},

	{"LINEOUT4 DAC", NULL, "RX6 DSM MUX"},
};

static int tabla_readable(struct snd_soc_codec *ssc, unsigned int reg)
{
	int i;
	struct wcd9xxx *tabla_core = dev_get_drvdata(ssc->dev->parent);

	if (TABLA_IS_1_X(tabla_core->version)) {
		for (i = 0; i < ARRAY_SIZE(tabla_1_reg_readable); i++) {
			if (tabla_1_reg_readable[i] == reg)
				return 1;
		}
	} else {
		for (i = 0; i < ARRAY_SIZE(tabla_2_reg_readable); i++) {
			if (tabla_2_reg_readable[i] == reg)
				return 1;
		}
	}

	return tabla_reg_readable[reg];
}
static bool tabla_is_digital_gain_register(unsigned int reg)
{
	bool rtn = false;
	switch (reg) {
	case TABLA_A_CDC_RX1_VOL_CTL_B2_CTL:
	case TABLA_A_CDC_RX2_VOL_CTL_B2_CTL:
	case TABLA_A_CDC_RX3_VOL_CTL_B2_CTL:
	case TABLA_A_CDC_RX4_VOL_CTL_B2_CTL:
	case TABLA_A_CDC_RX5_VOL_CTL_B2_CTL:
	case TABLA_A_CDC_RX6_VOL_CTL_B2_CTL:
	case TABLA_A_CDC_RX7_VOL_CTL_B2_CTL:
	case TABLA_A_CDC_TX1_VOL_CTL_GAIN:
	case TABLA_A_CDC_TX2_VOL_CTL_GAIN:
	case TABLA_A_CDC_TX3_VOL_CTL_GAIN:
	case TABLA_A_CDC_TX4_VOL_CTL_GAIN:
	case TABLA_A_CDC_TX5_VOL_CTL_GAIN:
	case TABLA_A_CDC_TX6_VOL_CTL_GAIN:
	case TABLA_A_CDC_TX7_VOL_CTL_GAIN:
	case TABLA_A_CDC_TX8_VOL_CTL_GAIN:
	case TABLA_A_CDC_TX9_VOL_CTL_GAIN:
	case TABLA_A_CDC_TX10_VOL_CTL_GAIN:
		rtn = true;
		break;
	default:
		break;
	}
	return rtn;
}
static int tabla_volatile(struct snd_soc_codec *ssc, unsigned int reg)
{
	/* Registers lower than 0x100 are top level registers which can be
	 * written by the Tabla core driver.
	 */

	if ((reg >= TABLA_A_CDC_MBHC_EN_CTL) || (reg < 0x100))
		return 1;

	/* IIR Coeff registers are not cacheable */
	if ((reg >= TABLA_A_CDC_IIR1_COEF_B1_CTL) &&
		(reg <= TABLA_A_CDC_IIR2_COEF_B5_CTL))
		return 1;

	/* Digital gain register is not cacheable so we have to write
	 * the setting even it is the same
	 */
	if (tabla_is_digital_gain_register(reg))
		return 1;

	/* HPH status registers */
	if (reg == TABLA_A_RX_HPH_L_STATUS || reg == TABLA_A_RX_HPH_R_STATUS)
		return 1;

	return 0;
}

#define TABLA_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)
static int tabla_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	int ret;
	BUG_ON(reg > TABLA_MAX_REGISTER);

	if (!tabla_volatile(codec, reg)) {
		ret = snd_soc_cache_write(codec, reg, value);
		if (ret != 0)
			dev_err(codec->dev, "Cache write to %x failed: %d\n",
				reg, ret);
	}

	return wcd9xxx_reg_write(codec->control_data, reg, value);
}
static unsigned int tabla_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	unsigned int val;
	int ret;

	BUG_ON(reg > TABLA_MAX_REGISTER);

	if (!tabla_volatile(codec, reg) && tabla_readable(codec, reg) &&
		reg < codec->driver->reg_cache_size) {
		ret = snd_soc_cache_read(codec, reg, &val);
		if (ret >= 0) {
			return val;
		} else
			dev_err(codec->dev, "Cache read from %x failed: %d\n",
				reg, ret);
	}

	val = wcd9xxx_reg_read(codec->control_data, reg);
	return val;
}

static int tabla_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct wcd9xxx *tabla_core = dev_get_drvdata(dai->codec->dev->parent);
	pr_debug("%s(): substream = %s  stream = %d\n" , __func__,
		 substream->name, substream->stream);
	if ((tabla_core != NULL) &&
	    (tabla_core->dev != NULL) &&
	    (tabla_core->dev->parent != NULL))
		pm_runtime_get_sync(tabla_core->dev->parent);

	return 0;
}

int tabla_mclk_enable(struct snd_soc_codec *codec, int mclk_enable, bool dapm)
{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);

	pr_debug("%s: mclk_enable = %u, dapm = %d\n", __func__, mclk_enable,
		 dapm);

	if (mclk_enable) {
		tabla->mclk_enabled = true;

			tabla_codec_disable_clock_block(codec);
			tabla_codec_enable_bandgap(codec,
						   TABLA_BANDGAP_AUDIO_MODE);
			tabla_codec_enable_clock_block(codec, 0);
	} else {

		if (!tabla->mclk_enabled) {
			pr_err("Error, MCLK already diabled\n");
			return -EINVAL;
		}
		tabla->mclk_enabled = false;

			tabla_codec_disable_clock_block(codec);
			tabla_codec_enable_bandgap(codec,
						   TABLA_BANDGAP_OFF);
		}

	return 0;
}

static int tabla_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	pr_debug("%s\n", __func__);
	return 0;
}

static int tabla_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	u8 val = 0;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(dai->codec);

	pr_debug("%s\n", __func__);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* CPU is master */
		if (tabla->intf_type == WCD9XXX_INTERFACE_TYPE_I2C) {
			if (dai->id == AIF1_CAP)
				snd_soc_update_bits(dai->codec,
					TABLA_A_CDC_CLK_TX_I2S_CTL,
					TABLA_I2S_MASTER_MODE_MASK, 0);
			else if (dai->id == AIF1_PB)
				snd_soc_update_bits(dai->codec,
					TABLA_A_CDC_CLK_RX_I2S_CTL,
					TABLA_I2S_MASTER_MODE_MASK, 0);
		}
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
	/* CPU is slave */
		if (tabla->intf_type == WCD9XXX_INTERFACE_TYPE_I2C) {
			val = TABLA_I2S_MASTER_MODE_MASK;
			if (dai->id == AIF1_CAP)
				snd_soc_update_bits(dai->codec,
					TABLA_A_CDC_CLK_TX_I2S_CTL, val, val);
			else if (dai->id == AIF1_PB)
				snd_soc_update_bits(dai->codec,
					TABLA_A_CDC_CLK_RX_I2S_CTL, val, val);
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int tabla_set_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)

{
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(dai->codec);
	u32 i = 0;
	if (!tx_slot && !rx_slot) {
		pr_err("%s: Invalid\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s(): dai_name = %s DAI-ID %x tx_ch %d rx_ch %d\n",
			__func__, dai->name, dai->id, tx_num, rx_num);

	if (dai->id == AIF1_PB || dai->id == AIF2_PB || dai->id == AIF3_PB) {
		for (i = 0; i < rx_num; i++) {
			tabla->dai[dai->id - 1].ch_num[i]  = rx_slot[i];
			tabla->dai[dai->id - 1].ch_act = 0;
			tabla->dai[dai->id - 1].ch_tot = rx_num;
		}
	} else if (dai->id == AIF1_CAP || dai->id == AIF2_CAP ||
		   dai->id == AIF3_CAP) {
		tabla->dai[dai->id - 1].ch_tot = tx_num;
		/* All channels are already active.
		 * do not reset ch_act flag
		 */
		if ((tabla->dai[dai->id - 1].ch_tot != 0)
			&& (tabla->dai[dai->id - 1].ch_act ==
			tabla->dai[dai->id - 1].ch_tot)) {
			pr_info("%s: ch_act = %d, ch_tot = %d\n", __func__,
				tabla->dai[dai->id - 1].ch_act,
				tabla->dai[dai->id - 1].ch_tot);
			return 0;
		}

		tabla->dai[dai->id - 1].ch_act = 0;
		for (i = 0; i < tx_num; i++)
			tabla->dai[dai->id - 1].ch_num[i]  = tx_slot[i];
	}
	return 0;
}

static int tabla_get_channel_map(struct snd_soc_dai *dai,
				unsigned int *tx_num, unsigned int *tx_slot,
				unsigned int *rx_num, unsigned int *rx_slot)

{
	struct wcd9xxx *tabla = dev_get_drvdata(dai->codec->control_data);

	u32 cnt = 0;
	u32 tx_ch[SLIM_MAX_TX_PORTS];
	u32 rx_ch[SLIM_MAX_RX_PORTS];

	if (!rx_slot && !tx_slot) {
		pr_err("%s: Invalid\n", __func__);
		return -EINVAL;
	}

	/* for virtual port, codec driver needs to do
	 * housekeeping, for now should be ok
	 */
	wcd9xxx_get_channel(tabla, rx_ch, tx_ch);
	if (dai->id == AIF1_PB) {
		*rx_num = tabla_dai[dai->id - 1].playback.channels_max;
		while (cnt < *rx_num) {
			rx_slot[cnt] = rx_ch[cnt];
			cnt++;
		}
	} else if (dai->id == AIF1_CAP) {
		*tx_num = tabla_dai[dai->id - 1].capture.channels_max;
		while (cnt < *tx_num) {
			tx_slot[cnt] = tx_ch[6 + cnt];
			cnt++;
		}
	} else if (dai->id == AIF2_PB) {
		*rx_num = tabla_dai[dai->id - 1].playback.channels_max;
		while (cnt < *rx_num) {
			rx_slot[cnt] = rx_ch[5 + cnt];
			cnt++;
		}
	} else if (dai->id == AIF2_CAP) {
		*tx_num = tabla_dai[dai->id - 1].capture.channels_max;
		tx_slot[0] = tx_ch[cnt];
		tx_slot[1] = tx_ch[1 + cnt];
		tx_slot[2] = tx_ch[5 + cnt];
		tx_slot[3] = tx_ch[3 + cnt];

	} else if (dai->id == AIF3_PB) {
		*rx_num = tabla_dai[dai->id - 1].playback.channels_max;
		rx_slot[0] = rx_ch[3];
		rx_slot[1] = rx_ch[4];

	} else if (dai->id == AIF3_CAP) {
		*tx_num = tabla_dai[dai->id - 1].capture.channels_max;
		tx_slot[cnt] = tx_ch[2 + cnt];
		tx_slot[cnt + 1] = tx_ch[4 + cnt];
	}
	pr_debug("%s(): dai_name = %s DAI-ID %x tx_ch %d rx_ch %d\n",
			__func__, dai->name, dai->id, *tx_num, *rx_num);


	return 0;
}


static struct snd_soc_dapm_widget tabla_dapm_aif_in_widgets[] = {

	SND_SOC_DAPM_AIF_IN_E("SLIM RX1", "AIF1 Playback", 0, SND_SOC_NOPM, 1,
				0, tabla_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("SLIM RX2", "AIF1 Playback", 0, SND_SOC_NOPM, 2,
				0, tabla_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("SLIM RX3", "AIF1 Playback", 0, SND_SOC_NOPM, 3,
				0, tabla_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("SLIM RX4", "AIF3 Playback", 0, SND_SOC_NOPM, 4,
				0, tabla_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("SLIM RX5", "AIF3 Playback", 0, SND_SOC_NOPM, 5,
				0, tabla_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("SLIM RX6", "AIF2 Playback", 0, SND_SOC_NOPM, 6,
				0, tabla_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_IN_E("SLIM RX7", "AIF2 Playback", 0, SND_SOC_NOPM, 7,
				0, tabla_codec_enable_slimrx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
};

static struct snd_soc_dapm_widget tabla_dapm_aif_out_widgets[] = {

	SND_SOC_DAPM_AIF_OUT_E("SLIM TX1", "AIF2 Capture", 0, SND_SOC_NOPM, 1,
				0, tabla_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("SLIM TX2", "AIF2 Capture", 0, SND_SOC_NOPM, 2,
				0, tabla_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("SLIM TX3", "AIF3 Capture", 0, SND_SOC_NOPM, 3,
				0, tabla_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("SLIM TX4", "AIF2 Capture", 0, SND_SOC_NOPM, 4,
				0, tabla_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("SLIM TX5", "AIF3 Capture", 0, SND_SOC_NOPM, 5,
				0, tabla_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("SLIM TX6", "AIF2 Capture", 0, SND_SOC_NOPM, 6,
				0, tabla_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("SLIM TX7", "AIF1 Capture", 0, SND_SOC_NOPM, 7,
				0, tabla_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("SLIM TX8", "AIF1 Capture", 0, SND_SOC_NOPM, 8,
				0, tabla_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("SLIM TX9", "AIF1 Capture", 0, SND_SOC_NOPM, 9,
				0, tabla_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("SLIM TX10", "AIF1 Capture", 0, SND_SOC_NOPM, 10,
				0, tabla_codec_enable_slimtx,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
};

static int tabla_set_interpolator_rate(struct snd_soc_dai *dai,
	u8 rx_fs_rate_reg_val, u32 compander_fs, u32 sample_rate)
{
	u32 i, j;
	u8 rx_mix1_inp;
	u16 rx_mix_1_reg_1, rx_mix_1_reg_2;
	u16 rx_fs_reg;
	u8 rx_mix_1_reg_1_val, rx_mix_1_reg_2_val;
	struct snd_soc_codec *codec = dai->codec;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_widget *w = tabla_dapm_aif_in_widgets;

	for (i = 0; i < ARRAY_SIZE(tabla_dapm_aif_in_widgets); i++) {

		if (strncmp(dai->driver->playback.stream_name, w[i].sname, 13))
			continue;

		rx_mix1_inp = w[i].shift + 4;

		if ((rx_mix1_inp < 0x5) || (rx_mix1_inp > 0xB)) {

			pr_err("%s: Invalid SLIM RX%u port.  widget = %s\n",
				__func__,  rx_mix1_inp  - 4 , w[i].name);
			return -EINVAL;
		}

		rx_mix_1_reg_1 = TABLA_A_CDC_CONN_RX1_B1_CTL;

		for (j = 0; j < NUM_INTERPOLATORS; j++) {

			rx_mix_1_reg_2 = rx_mix_1_reg_1 + 1;

			rx_mix_1_reg_1_val = snd_soc_read(codec,
					rx_mix_1_reg_1);
			rx_mix_1_reg_2_val = snd_soc_read(codec,
					rx_mix_1_reg_2);

			if (((rx_mix_1_reg_1_val & 0x0F) == rx_mix1_inp) ||
			   (((rx_mix_1_reg_1_val >> 4) & 0x0F) == rx_mix1_inp)
			   || ((rx_mix_1_reg_2_val & 0x0F) == rx_mix1_inp)) {

				rx_fs_reg = TABLA_A_CDC_RX1_B5_CTL + 8 * j;

				pr_debug("%s: %s connected to RX%u\n", __func__,
					w[i].name, j + 1);

				pr_debug("%s: set RX%u sample rate to %u\n",
					__func__, j + 1, sample_rate);

				snd_soc_update_bits(codec, rx_fs_reg,
						0xE0, rx_fs_rate_reg_val);

				if (comp_rx_path[j] < COMPANDER_MAX)
					tabla->comp_fs[comp_rx_path[j]]
					= compander_fs;
			}
			if (j <= 2)
				rx_mix_1_reg_1 += 3;
			else
				rx_mix_1_reg_1 += 2;
		}
	}
	return 0;
}

static int tabla_set_decimator_rate(struct snd_soc_dai *dai,
	u8 tx_fs_rate_reg_val, u32 sample_rate)
{
	struct snd_soc_codec *codec = dai->codec;
	struct snd_soc_dapm_widget *w = tabla_dapm_aif_out_widgets;

	u32 i, tx_port;
	u16 tx_port_reg, tx_fs_reg;
	u8 tx_port_reg_val;
	s8 decimator;

	for (i = 0; i < ARRAY_SIZE(tabla_dapm_aif_out_widgets); i++) {

		if (strncmp(dai->driver->capture.stream_name, w[i].sname, 12))
			continue;

		tx_port = w[i].shift;

		if ((tx_port < 1) || (tx_port > NUM_DECIMATORS)) {
			pr_err("%s: Invalid SLIM TX%u port.  widget = %s\n",
				__func__, tx_port, w[i].name);
			return -EINVAL;
		}

		tx_port_reg = TABLA_A_CDC_CONN_TX_SB_B1_CTL + (tx_port - 1);
		tx_port_reg_val =  snd_soc_read(codec, tx_port_reg);

		decimator = 0;

		if ((tx_port >= 1) && (tx_port <= 6)) {

			tx_port_reg_val =  tx_port_reg_val & 0x0F;
			if (tx_port_reg_val == 0x8)
				decimator = tx_port;

		} else if ((tx_port >= 7) && (tx_port <= NUM_DECIMATORS)) {

			tx_port_reg_val =  tx_port_reg_val & 0x1F;

			if ((tx_port_reg_val >= 0x8) &&
			    (tx_port_reg_val <= 0x11)) {

				decimator = (tx_port_reg_val - 0x8) + 1;
			}
		}

		if (decimator) { /* SLIM_TX port has a DEC as input */

			tx_fs_reg = TABLA_A_CDC_TX1_CLK_FS_CTL +
				8 * (decimator - 1);

			pr_debug("%s: set DEC%u (-> SLIM_TX%u) rate to %u\n",
				__func__, decimator, tx_port, sample_rate);

			snd_soc_update_bits(codec, tx_fs_reg, 0x07,
					tx_fs_rate_reg_val);

		} else {
			if ((tx_port_reg_val >= 0x1) &&
					(tx_port_reg_val <= 0x7)) {

				pr_debug("%s: RMIX%u going to SLIM TX%u\n",
					__func__, tx_port_reg_val, tx_port);

			} else if  ((tx_port_reg_val >= 0x8) &&
					(tx_port_reg_val <= 0x11)) {

				pr_err("%s: ERROR: Should not be here\n",
						__func__);
				pr_err("%s: ERROR: DEC connected to SLIM TX%u\n"
						, __func__, tx_port);
				return -EINVAL;

			} else if (tx_port_reg_val == 0) {
				pr_debug("%s: no signal to SLIM TX%u\n",
						__func__, tx_port);
			} else {
				pr_err("%s: ERROR: wrong signal to SLIM TX%u\n"
						, __func__, tx_port);
				pr_err("%s: ERROR: wrong signal = %u\n"
						, __func__, tx_port_reg_val);
				return -EINVAL;
			}
		}
	}
	return 0;
}

static int tabla_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(dai->codec);
	u8 tx_fs_rate_reg_val, rx_fs_rate_reg_val;
	u32 compander_fs;
	int ret;

	pr_debug("%s: dai_name = %s DAI-ID %x rate %d num_ch %d\n", __func__,
			dai->name, dai->id, params_rate(params),
			params_channels(params));

	switch (params_rate(params)) {
	case 8000:
		tx_fs_rate_reg_val = 0x00;
		rx_fs_rate_reg_val = 0x00;
		compander_fs = COMPANDER_FS_8KHZ;
		break;
	case 16000:
		tx_fs_rate_reg_val = 0x01;
		rx_fs_rate_reg_val = 0x20;
		compander_fs = COMPANDER_FS_16KHZ;
		break;
	case 32000:
		tx_fs_rate_reg_val = 0x02;
		rx_fs_rate_reg_val = 0x40;
		compander_fs = COMPANDER_FS_32KHZ;
		break;
	case 48000:
		tx_fs_rate_reg_val = 0x03;
		rx_fs_rate_reg_val = 0x60;
		compander_fs = COMPANDER_FS_48KHZ;
		break;
	case 96000:
		tx_fs_rate_reg_val = 0x04;
		rx_fs_rate_reg_val = 0x80;
		compander_fs = COMPANDER_FS_96KHZ;
		break;
	case 192000:
		tx_fs_rate_reg_val = 0x05;
		rx_fs_rate_reg_val = 0xA0;
		compander_fs = COMPANDER_FS_192KHZ;
		break;
	default:
		pr_err("%s: Invalid sampling rate %d\n", __func__,
				params_rate(params));
		return -EINVAL;
	}

	switch (substream->stream) {
	case SNDRV_PCM_STREAM_CAPTURE:

		ret = tabla_set_decimator_rate(dai, tx_fs_rate_reg_val,
				params_rate(params));
		if (ret < 0) {
			pr_err("%s: set decimator rate failed %d\n", __func__,
					ret);
			return ret;
		}

		if (tabla->intf_type == WCD9XXX_INTERFACE_TYPE_I2C) {
			switch (params_format(params)) {
			case SNDRV_PCM_FORMAT_S16_LE:
				snd_soc_update_bits(codec,
					TABLA_A_CDC_CLK_TX_I2S_CTL, 0x20, 0x20);
				break;
			case SNDRV_PCM_FORMAT_S32_LE:
				snd_soc_update_bits(codec,
					TABLA_A_CDC_CLK_TX_I2S_CTL, 0x20, 0x00);
				break;
			default:
				pr_err("%s: invalid TX format %u\n", __func__,
						params_format(params));
				return -EINVAL;
			}
			snd_soc_update_bits(codec, TABLA_A_CDC_CLK_TX_I2S_CTL,
					0x07, tx_fs_rate_reg_val);
		} else {
			tabla->dai[dai->id - 1].rate   = params_rate(params);
		}
		break;

	case SNDRV_PCM_STREAM_PLAYBACK:

		ret = tabla_set_interpolator_rate(dai, rx_fs_rate_reg_val,
				compander_fs, params_rate(params));
		if (ret < 0) {
			pr_err("%s: set decimator rate failed %d\n", __func__,
					ret);
			return ret;
		}

		if (tabla->intf_type == WCD9XXX_INTERFACE_TYPE_I2C) {
			switch (params_format(params)) {
			case SNDRV_PCM_FORMAT_S16_LE:
				snd_soc_update_bits(codec,
					TABLA_A_CDC_CLK_RX_I2S_CTL, 0x20, 0x20);
				break;
			case SNDRV_PCM_FORMAT_S32_LE:
				snd_soc_update_bits(codec,
					TABLA_A_CDC_CLK_RX_I2S_CTL, 0x20, 0x00);
				break;
			default:
				pr_err("%s: invalid RX format %u\n", __func__,
						params_format(params));
				return -EINVAL;
			}
			snd_soc_update_bits(codec, TABLA_A_CDC_CLK_RX_I2S_CTL,
					0x03, (rx_fs_rate_reg_val >> 0x05));
		} else {
			tabla->dai[dai->id - 1].rate   = params_rate(params);
		}
		break;

	default:
		pr_err("%s: Invalid stream type %d\n", __func__,
				substream->stream);
		return -EINVAL;
	}
	return 0;
}

static struct snd_soc_dai_ops tabla_dai_ops = {
	.startup = tabla_startup,
	.hw_params = tabla_hw_params,
	.set_sysclk = tabla_set_dai_sysclk,
	.set_fmt = tabla_set_dai_fmt,
	.set_channel_map = tabla_set_channel_map,
	.get_channel_map = tabla_get_channel_map,
};

static struct snd_soc_dai_driver tabla_dai[] = {
	{
		.name = "tabla_rx1",
		.id = AIF1_PB,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = WCD9310_RATES,
			.formats = TABLA_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &tabla_dai_ops,
	},
	{
		.name = "tabla_tx1",
		.id = AIF1_CAP,
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = WCD9310_RATES,
			.formats = TABLA_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &tabla_dai_ops,
	},
	{
		.name = "tabla_rx2",
		.id = AIF2_PB,
		.playback = {
			.stream_name = "AIF2 Playback",
			.rates = WCD9310_RATES,
			.formats = TABLA_FORMATS,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &tabla_dai_ops,
	},
	{
		.name = "tabla_tx2",
		.id = AIF2_CAP,
		.capture = {
			.stream_name = "AIF2 Capture",
			.rates = WCD9310_RATES,
			.formats = TABLA_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &tabla_dai_ops,
	},
	{
		.name = "tabla_tx3",
		.id = AIF3_CAP,
		.capture = {
			.stream_name = "AIF3 Capture",
			.rates = WCD9310_RATES,
			.formats = TABLA_FORMATS,
			.rate_max = 48000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &tabla_dai_ops,
	},
	{
		.name = "tabla_rx3",
		.id = AIF3_PB,
		.playback = {
			.stream_name = "AIF3 Playback",
			.rates = WCD9310_RATES,
			.formats = TABLA_FORMATS,
			.rate_min = 8000,
			.rate_max = 192000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &tabla_dai_ops,
	},
};

static struct snd_soc_dai_driver tabla_i2s_dai[] = {
	{
		.name = "tabla_i2s_rx1",
		.id = 1,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = WCD9310_RATES,
			.formats = TABLA_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &tabla_dai_ops,
	},
	{
		.name = "tabla_i2s_tx1",
		.id = 2,
		.capture = {
			.stream_name = "AIF1 Capture",
			.rates = WCD9310_RATES,
			.formats = TABLA_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 4,
		},
		.ops = &tabla_dai_ops,
	},
};

static int tabla_codec_enable_chmask(struct tabla_priv *tabla_p,
	int event, int index)
{
	int  ret = 0;
	u32 k = 0;
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		for (k = 0; k < tabla_p->dai[index].ch_tot; k++) {
			ret = wcd9xxx_get_slave_port(
					tabla_p->dai[index].ch_num[k]);
			if (ret < 0) {
				pr_err("%s: Invalid slave port ID: %d\n",
					__func__, ret);
				ret = -EINVAL;
				break;
			}
			tabla_p->dai[index].ch_mask |= 1 << ret;
		}
		ret = 0;
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = wait_event_timeout(tabla_p->dai[index].dai_wait,
					(tabla_p->dai[index].ch_mask == 0),
				msecs_to_jiffies(SLIM_CLOSE_TIMEOUT));
		if (!ret) {
			pr_err("%s: Slim close tx/rx wait timeout\n",
				__func__);
			ret = -EINVAL;
		} else          //Qualcomm++
		    ret = 0;
		break;
	}
	return ret;
}

static int tabla_codec_enable_slimrx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct wcd9xxx *tabla;
	struct snd_soc_codec *codec = w->codec;
	struct tabla_priv *tabla_p = snd_soc_codec_get_drvdata(codec);
	u32  j = 0;
	int  ret = 0;   //Qualcomm++
	codec->control_data = dev_get_drvdata(codec->dev->parent);
	tabla = codec->control_data;

	/* Execute the callback only if interface type is slimbus */
    if (tabla_p->intf_type != WCD9XXX_INTERFACE_TYPE_SLIMBUS) {
        if (event == SND_SOC_DAPM_POST_PMD && (tabla != NULL) &&
            (tabla->dev != NULL) &&
            (tabla->dev->parent != NULL)) {
            pm_runtime_mark_last_busy(tabla->dev->parent);
            pm_runtime_put(tabla->dev->parent);
        }
        return 0;
    }

	pr_debug("%s: %s %d\n", __func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		for (j = 0; j < ARRAY_SIZE(tabla_dai); j++) {
			if ((tabla_dai[j].id == AIF1_CAP) ||
			    (tabla_dai[j].id == AIF2_CAP) ||
			    (tabla_dai[j].id == AIF3_CAP))
				continue;
			if (!strncmp(w->sname,
				tabla_dai[j].playback.stream_name, 13)) {
				++tabla_p->dai[j].ch_act;
				break;
			}
		}
		if (tabla_p->dai[j].ch_act == tabla_p->dai[j].ch_tot) {
			ret = tabla_codec_enable_chmask(tabla_p,
							SND_SOC_DAPM_POST_PMU,
							j);
			ret = wcd9xxx_cfg_slim_sch_rx(tabla,
					tabla_p->dai[j].ch_num,
					tabla_p->dai[j].ch_tot,
					tabla_p->dai[j].rate);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		for (j = 0; j < ARRAY_SIZE(tabla_dai); j++) {
			if ((tabla_dai[j].id == AIF1_CAP) ||
			    (tabla_dai[j].id == AIF2_CAP) ||
			    (tabla_dai[j].id == AIF3_CAP))
				continue;
			if (!strncmp(w->sname,
				tabla_dai[j].playback.stream_name, 13)) {
				--tabla_p->dai[j].ch_act;
				break;
			}
		}
		if (!tabla_p->dai[j].ch_act) {
			ret = wcd9xxx_close_slim_sch_rx(tabla,
						tabla_p->dai[j].ch_num,
						tabla_p->dai[j].ch_tot);
//Qualcomm++
            ret = tabla_codec_enable_chmask(tabla_p,
                            SND_SOC_DAPM_POST_PMD,
                            j);
            if (ret < 0) {
                ret = wcd9xxx_disconnect_port(tabla,
                            tabla_p->dai[j].ch_num,
                            tabla_p->dai[j].ch_tot,
                            1);
                pr_info("%s: Disconnect RX port ret = %d\n",
                    __func__, ret);
            }
//Qualcomm++
			tabla_p->dai[j].rate = 0;
			memset(tabla_p->dai[j].ch_num, 0, (sizeof(u32)*
					tabla_p->dai[j].ch_tot));
			tabla_p->dai[j].ch_tot = 0;

            if ((tabla != NULL) &&
                (tabla->dev != NULL) &&
                (tabla->dev->parent != NULL)) {
                pm_runtime_mark_last_busy(tabla->dev->parent);
                pm_runtime_put(tabla->dev->parent);
            }
		}
	}
	return ret;
}

static int tabla_codec_enable_slimtx(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct wcd9xxx *tabla;
	struct snd_soc_codec *codec = w->codec;
	struct tabla_priv *tabla_p = snd_soc_codec_get_drvdata(codec);
	/* index to the DAI ID, for now hardcoding */
	u32  j = 0;
	int  ret = 0;   //Qualcomm++

	codec->control_data = dev_get_drvdata(codec->dev->parent);
	tabla = codec->control_data;

	/* Execute the callback only if interface type is slimbus */
    if (tabla_p->intf_type != WCD9XXX_INTERFACE_TYPE_SLIMBUS) {
        if (event == SND_SOC_DAPM_POST_PMD && (tabla != NULL) &&
            (tabla->dev != NULL) &&
            (tabla->dev->parent != NULL)) {
            pm_runtime_mark_last_busy(tabla->dev->parent);
            pm_runtime_put(tabla->dev->parent);
        }
        return 0;
    }

	pr_debug("%s(): %s %d\n", __func__, w->name, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		for (j = 0; j < ARRAY_SIZE(tabla_dai); j++) {
			if (tabla_dai[j].id == AIF1_PB ||
				tabla_dai[j].id == AIF2_PB ||
				tabla_dai[j].id == AIF3_PB)
				continue;
			if (!strncmp(w->sname,
				tabla_dai[j].capture.stream_name, 13)) {
				++tabla_p->dai[j].ch_act;
				break;
			}
		}
		if (tabla_p->dai[j].ch_act == tabla_p->dai[j].ch_tot) {
			ret = tabla_codec_enable_chmask(tabla_p,
							SND_SOC_DAPM_POST_PMU,
							j);
			ret = wcd9xxx_cfg_slim_sch_tx(tabla,
						tabla_p->dai[j].ch_num,
						tabla_p->dai[j].ch_tot,
						tabla_p->dai[j].rate);
		}
		break;
	case SND_SOC_DAPM_POST_PMD:
		for (j = 0; j < ARRAY_SIZE(tabla_dai); j++) {
			if (tabla_dai[j].id == AIF1_PB ||
				tabla_dai[j].id == AIF2_PB ||
				tabla_dai[j].id == AIF3_PB)
				continue;
			if (!strncmp(w->sname,
				tabla_dai[j].capture.stream_name, 13)) {
				--tabla_p->dai[j].ch_act;
				break;
			}
		}
		if (!tabla_p->dai[j].ch_act) {
			ret = wcd9xxx_close_slim_sch_tx(tabla,
						tabla_p->dai[j].ch_num,
						tabla_p->dai[j].ch_tot);
//Qualcomm++
            ret = tabla_codec_enable_chmask(tabla_p,
                        SND_SOC_DAPM_POST_PMD,
                        j);
            if (ret < 0) {
                ret = wcd9xxx_disconnect_port(tabla,
                        tabla_p->dai[j].ch_num,
                        tabla_p->dai[j].ch_tot, 0);
                pr_info("%s: Disconnect TX port, ret = %d\n",
                    __func__, ret);
            }
//Qualcomm++

			tabla_p->dai[j].rate = 0;
			memset(tabla_p->dai[j].ch_num, 0, (sizeof(u32)*
					tabla_p->dai[j].ch_tot));
			tabla_p->dai[j].ch_tot = 0;
//Qualcomm++
#if 0
			ret = tabla_codec_enable_chmask(tabla_p,
							SND_SOC_DAPM_POST_PMD,
							j);
#endif
//Qualcomm++
            if ((tabla != NULL) &&
                (tabla->dev != NULL) &&
                (tabla->dev->parent != NULL)) {
                pm_runtime_mark_last_busy(tabla->dev->parent);
                pm_runtime_put(tabla->dev->parent);
            }
		}
	}
	return ret;
}

/* Todo: Have seperate dapm widgets for I2S and Slimbus.
 * Might Need to have callbacks registered only for slimbus
 */
static const struct snd_soc_dapm_widget tabla_dapm_widgets[] = {
	/*RX stuff */
	SND_SOC_DAPM_OUTPUT("EAR"),

	SND_SOC_DAPM_PGA("EAR PA", TABLA_A_RX_EAR_EN, 4, 0, NULL, 0),

	SND_SOC_DAPM_MIXER("DAC1", TABLA_A_RX_EAR_EN, 6, 0, dac1_switch,
		ARRAY_SIZE(dac1_switch)),

	/* Headphone */
	SND_SOC_DAPM_OUTPUT("HEADPHONE"),
	SND_SOC_DAPM_PGA_E("HPHL", TABLA_A_RX_HPH_CNP_EN, 5, 0, NULL, 0,
		tabla_hph_pa_event, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MIXER("HPHL DAC", TABLA_A_RX_HPH_L_DAC_CTL, 7, 0,
		hphl_switch, ARRAY_SIZE(hphl_switch)),

	SND_SOC_DAPM_PGA_E("HPHR", TABLA_A_RX_HPH_CNP_EN, 4, 0, NULL, 0,
		tabla_hph_pa_event, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("HPHR DAC", NULL, TABLA_A_RX_HPH_R_DAC_CTL, 7, 0,
		tabla_hphr_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	/* Speaker */
	SND_SOC_DAPM_OUTPUT("LINEOUT1"),
	SND_SOC_DAPM_OUTPUT("LINEOUT2"),
	SND_SOC_DAPM_OUTPUT("LINEOUT3"),
	SND_SOC_DAPM_OUTPUT("LINEOUT4"),
	SND_SOC_DAPM_OUTPUT("LINEOUT5"),

	SND_SOC_DAPM_PGA_E("LINEOUT1 PA", TABLA_A_RX_LINE_CNP_EN, 0, 0, NULL,
			0, tabla_codec_enable_lineout, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT2 PA", TABLA_A_RX_LINE_CNP_EN, 1, 0, NULL,
			0, tabla_codec_enable_lineout, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT3 PA", TABLA_A_RX_LINE_CNP_EN, 2, 0, NULL,
			0, tabla_codec_enable_lineout, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT4 PA", TABLA_A_RX_LINE_CNP_EN, 3, 0, NULL,
			0, tabla_codec_enable_lineout, SND_SOC_DAPM_PRE_PMU |
			SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("LINEOUT5 PA", TABLA_A_RX_LINE_CNP_EN, 4, 0, NULL, 0,
		tabla_codec_enable_lineout, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("LINEOUT1 DAC", NULL, TABLA_A_RX_LINE_1_DAC_CTL, 7, 0
		, tabla_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("LINEOUT2 DAC", NULL, TABLA_A_RX_LINE_2_DAC_CTL, 7, 0
		, tabla_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_DAC_E("LINEOUT3 DAC", NULL, TABLA_A_RX_LINE_3_DAC_CTL, 7, 0
		, tabla_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SWITCH("LINEOUT3 DAC GROUND", SND_SOC_NOPM, 0, 0,
				&lineout3_ground_switch),
	SND_SOC_DAPM_DAC_E("LINEOUT4 DAC", NULL, TABLA_A_RX_LINE_4_DAC_CTL, 7, 0
		, tabla_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SWITCH("LINEOUT4 DAC GROUND", SND_SOC_NOPM, 0, 0,
				&lineout4_ground_switch),
	SND_SOC_DAPM_DAC_E("LINEOUT5 DAC", NULL, TABLA_A_RX_LINE_5_DAC_CTL, 7, 0
		, tabla_lineout_dac_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MIXER_E("RX1 MIX2", TABLA_A_CDC_CLK_RX_B1_CTL, 0, 0, NULL,
		0, tabla_codec_reset_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX2 MIX2", TABLA_A_CDC_CLK_RX_B1_CTL, 1, 0, NULL,
		0, tabla_codec_reset_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX3 MIX2", TABLA_A_CDC_CLK_RX_B1_CTL, 2, 0, NULL,
		0, tabla_codec_reset_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX4 MIX1", TABLA_A_CDC_CLK_RX_B1_CTL, 3, 0, NULL,
		0, tabla_codec_reset_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX5 MIX1", TABLA_A_CDC_CLK_RX_B1_CTL, 4, 0, NULL,
		0, tabla_codec_reset_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX6 MIX1", TABLA_A_CDC_CLK_RX_B1_CTL, 5, 0, NULL,
		0, tabla_codec_reset_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("RX7 MIX1", TABLA_A_CDC_CLK_RX_B1_CTL, 6, 0, NULL,
		0, tabla_codec_reset_interpolator, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MIXER("RX1 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX3 MIX1", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_MUX_E("RX4 DSM MUX", TABLA_A_CDC_CLK_RX_B1_CTL, 3, 0,
		&rx4_dsm_mux, tabla_codec_reset_interpolator,
		SND_SOC_DAPM_PRE_PMU),

	SND_SOC_DAPM_MUX_E("RX6 DSM MUX", TABLA_A_CDC_CLK_RX_B1_CTL, 5, 0,
		&rx6_dsm_mux, tabla_codec_reset_interpolator,
		SND_SOC_DAPM_PRE_PMU),

	SND_SOC_DAPM_MIXER("RX1 CHAIN", TABLA_A_CDC_RX1_B6_CTL, 5, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("RX2 CHAIN", TABLA_A_CDC_RX2_B6_CTL, 5, 0, NULL, 0),

	SND_SOC_DAPM_MUX("RX1 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX1 MIX1 INP3", SND_SOC_NOPM, 0, 0,
		&rx_mix1_inp3_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx2_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx2_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx3_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX3 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx3_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX4 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx4_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX4 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx4_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX5 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx5_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX5 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx5_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX6 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx6_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX6 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx6_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX7 MIX1 INP1", SND_SOC_NOPM, 0, 0,
		&rx7_mix1_inp1_mux),
	SND_SOC_DAPM_MUX("RX7 MIX1 INP2", SND_SOC_NOPM, 0, 0,
		&rx7_mix1_inp2_mux),
	SND_SOC_DAPM_MUX("RX1 MIX2 INP1", SND_SOC_NOPM, 0, 0,
		&rx1_mix2_inp1_mux),
	SND_SOC_DAPM_MUX("RX1 MIX2 INP2", SND_SOC_NOPM, 0, 0,
		&rx1_mix2_inp2_mux),
	SND_SOC_DAPM_MUX("RX2 MIX2 INP1", SND_SOC_NOPM, 0, 0,
		&rx2_mix2_inp1_mux),
	SND_SOC_DAPM_MUX("RX2 MIX2 INP2", SND_SOC_NOPM, 0, 0,
		&rx2_mix2_inp2_mux),
	SND_SOC_DAPM_MUX("RX3 MIX2 INP1", SND_SOC_NOPM, 0, 0,
		&rx3_mix2_inp1_mux),
	SND_SOC_DAPM_MUX("RX3 MIX2 INP2", SND_SOC_NOPM, 0, 0,
		&rx3_mix2_inp2_mux),

	SND_SOC_DAPM_SUPPLY("CP", TABLA_A_CP_EN, 0, 0,
		tabla_codec_enable_charge_pump, SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_SUPPLY("RX_BIAS", SND_SOC_NOPM, 0, 0,
		tabla_codec_enable_rx_bias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	/* TX */

	SND_SOC_DAPM_SUPPLY("CDC_CONN", TABLA_A_CDC_CLK_OTHR_CTL, 2, 0, NULL,
		0),

	SND_SOC_DAPM_SUPPLY("LDO_H", TABLA_A_LDO_H_MODE_1, 7, 0,
		tabla_codec_enable_ldo_h, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_SUPPLY("COMP1_CLK", SND_SOC_NOPM, 0, 0,
		tabla_config_compander, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_POST_PMD),
	SND_SOC_DAPM_SUPPLY("COMP2_CLK", SND_SOC_NOPM, 1, 0,
		tabla_config_compander, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_POST_PMD),

	SND_SOC_DAPM_INPUT("AMIC1"),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS1 External", TABLA_A_MICB_1_CTL, 7, 0,
		tabla_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS1 Internal1", TABLA_A_MICB_1_CTL, 7, 0,
		tabla_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS1 Internal2", TABLA_A_MICB_1_CTL, 7, 0,
		tabla_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC1", NULL, TABLA_A_TX_1_2_EN, 7, 0,
		tabla_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_INPUT("AMIC3"),
	SND_SOC_DAPM_ADC_E("ADC3", NULL, TABLA_A_TX_3_4_EN, 7, 0,
		tabla_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_INPUT("AMIC4"),
	SND_SOC_DAPM_ADC_E("ADC4", NULL, TABLA_A_TX_3_4_EN, 3, 0,
		tabla_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_INPUT("AMIC5"),
	SND_SOC_DAPM_ADC_E("ADC5", NULL, TABLA_A_TX_5_6_EN, 7, 0,
		tabla_codec_enable_adc, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_INPUT("AMIC6"),
	SND_SOC_DAPM_ADC_E("ADC6", NULL, TABLA_A_TX_5_6_EN, 3, 0,
		tabla_codec_enable_adc, SND_SOC_DAPM_POST_PMU),

	SND_SOC_DAPM_MUX_E("DEC1 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL, 0, 0,
		&dec1_mux, tabla_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC2 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL, 1, 0,
		&dec2_mux, tabla_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC3 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL, 2, 0,
		&dec3_mux, tabla_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC4 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL, 3, 0,
		&dec4_mux, tabla_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC5 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL, 4, 0,
		&dec5_mux, tabla_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC6 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL, 5, 0,
		&dec6_mux, tabla_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC7 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL, 6, 0,
		&dec7_mux, tabla_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC8 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B1_CTL, 7, 0,
		&dec8_mux, tabla_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC9 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B2_CTL, 0, 0,
		&dec9_mux, tabla_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX_E("DEC10 MUX", TABLA_A_CDC_CLK_TX_CLK_EN_B2_CTL, 1, 0,
		&dec10_mux, tabla_codec_enable_dec,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU |
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("ANC1 MUX", SND_SOC_NOPM, 0, 0, &anc1_mux),
	SND_SOC_DAPM_MUX("ANC2 MUX", SND_SOC_NOPM, 0, 0, &anc2_mux),

	SND_SOC_DAPM_MIXER_E("ANC", SND_SOC_NOPM, 0, 0, NULL, 0,
		tabla_codec_enable_anc, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("ANC1 FB MUX", SND_SOC_NOPM, 0, 0, &anc1_fb_mux),

	SND_SOC_DAPM_INPUT("AMIC2"),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS2 External", TABLA_A_MICB_2_CTL, 7, 0,
		tabla_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU |	SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS2 Internal1", TABLA_A_MICB_2_CTL, 7, 0,
		tabla_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS2 Internal2", TABLA_A_MICB_2_CTL, 7, 0,
		tabla_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS2 Internal3", TABLA_A_MICB_2_CTL, 7, 0,
		tabla_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS3 External", TABLA_A_MICB_3_CTL, 7, 0,
		tabla_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS3 Internal1", TABLA_A_MICB_3_CTL, 7, 0,
		tabla_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_MICBIAS_E("MIC BIAS3 Internal2", TABLA_A_MICB_3_CTL, 7, 0,
		tabla_codec_enable_micbias, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("ADC2", NULL, TABLA_A_TX_1_2_EN, 3, 0,
		tabla_codec_enable_adc, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_MUX("SLIM TX1 MUX", SND_SOC_NOPM, 0, 0, &sb_tx1_mux),
	SND_SOC_DAPM_MUX("SLIM TX2 MUX", SND_SOC_NOPM, 0, 0, &sb_tx2_mux),
	SND_SOC_DAPM_MUX("SLIM TX3 MUX", SND_SOC_NOPM, 0, 0, &sb_tx3_mux),
	SND_SOC_DAPM_MUX("SLIM TX4 MUX", SND_SOC_NOPM, 0, 0, &sb_tx4_mux),
	SND_SOC_DAPM_MUX("SLIM TX5 MUX", SND_SOC_NOPM, 0, 0, &sb_tx5_mux),
	SND_SOC_DAPM_MUX("SLIM TX6 MUX", SND_SOC_NOPM, 0, 0, &sb_tx6_mux),
	SND_SOC_DAPM_MUX("SLIM TX7 MUX", SND_SOC_NOPM, 0, 0, &sb_tx7_mux),
	SND_SOC_DAPM_MUX("SLIM TX8 MUX", SND_SOC_NOPM, 0, 0, &sb_tx8_mux),
	SND_SOC_DAPM_MUX("SLIM TX9 MUX", SND_SOC_NOPM, 0, 0, &sb_tx9_mux),
	SND_SOC_DAPM_MUX("SLIM TX10 MUX", SND_SOC_NOPM, 0, 0, &sb_tx10_mux),

	/* Digital Mic Inputs */
	SND_SOC_DAPM_ADC_E("DMIC1", NULL, SND_SOC_NOPM, 0, 0,
		tabla_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC2", NULL, SND_SOC_NOPM, 0, 0,
		tabla_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC3", NULL, SND_SOC_NOPM, 0, 0,
		tabla_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC4", NULL, SND_SOC_NOPM, 0, 0,
		tabla_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("DMIC5", NULL, SND_SOC_NOPM, 0, 0,
		tabla_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_ADC_E("DMIC6", NULL, SND_SOC_NOPM, 0, 0,
		tabla_codec_enable_dmic, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMD),

	/* Sidetone */
	SND_SOC_DAPM_MUX("IIR1 INP1 MUX", SND_SOC_NOPM, 0, 0, &iir1_inp1_mux),
	SND_SOC_DAPM_PGA("IIR1", TABLA_A_CDC_CLK_SD_CTL, 0, 0, NULL, 0),

	/* AUX PGA */
	SND_SOC_DAPM_ADC_E("AUX_PGA_Left", NULL, TABLA_A_AUX_L_EN, 7, 0,
		tabla_codec_enable_aux_pga, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU |	SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_ADC_E("AUX_PGA_Right", NULL, TABLA_A_AUX_R_EN, 7, 0,
		tabla_codec_enable_aux_pga, SND_SOC_DAPM_PRE_PMU |
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMD),

	/* Lineout, ear and HPH PA Mixers */
	SND_SOC_DAPM_MIXER("HPHL_PA_MIXER", SND_SOC_NOPM, 0, 0,
		hphl_pa_mix, ARRAY_SIZE(hphl_pa_mix)),

	SND_SOC_DAPM_MIXER("HPHR_PA_MIXER", SND_SOC_NOPM, 0, 0,
		hphr_pa_mix, ARRAY_SIZE(hphr_pa_mix)),

	SND_SOC_DAPM_MIXER("LINEOUT1_PA_MIXER", SND_SOC_NOPM, 0, 0,
		lineout1_pa_mix, ARRAY_SIZE(lineout1_pa_mix)),

	SND_SOC_DAPM_MIXER("LINEOUT2_PA_MIXER", SND_SOC_NOPM, 0, 0,
		lineout2_pa_mix, ARRAY_SIZE(lineout2_pa_mix)),

	SND_SOC_DAPM_MIXER("LINEOUT3_PA_MIXER", SND_SOC_NOPM, 0, 0,
		lineout3_pa_mix, ARRAY_SIZE(lineout3_pa_mix)),

	SND_SOC_DAPM_MIXER("LINEOUT4_PA_MIXER", SND_SOC_NOPM, 0, 0,
		lineout4_pa_mix, ARRAY_SIZE(lineout4_pa_mix)),

	SND_SOC_DAPM_MIXER("LINEOUT5_PA_MIXER", SND_SOC_NOPM, 0, 0,
		lineout5_pa_mix, ARRAY_SIZE(lineout5_pa_mix)),

	SND_SOC_DAPM_MIXER("EAR_PA_MIXER", SND_SOC_NOPM, 0, 0,
		ear_pa_mix, ARRAY_SIZE(ear_pa_mix)),
};

int tabla_hs_detect(struct snd_soc_codec *codec,
		    const struct tabla_mbhc_config *cfg)
{
	struct tabla_priv *tabla;

	if (!codec) {
		pr_err("Error: no codec or calibration\n");
		return -EINVAL;
	}

	tabla = snd_soc_codec_get_drvdata(codec);
	tabla->mbhc_cfg = *cfg;

	return 0;
}
EXPORT_SYMBOL_GPL(tabla_hs_detect);

static irqreturn_t tabla_slimbus_irq(int irq, void *data)
{
	struct tabla_priv *priv = data;
	struct snd_soc_codec *codec = priv->codec;
	struct tabla_priv *tabla_p = snd_soc_codec_get_drvdata(codec);
	int i, j, port_id, k, ch_mask_temp;
	unsigned long slimbus_value;
	u8 val;

	for (i = 0; i < WCD9XXX_SLIM_NUM_PORT_REG; i++) {
		slimbus_value = wcd9xxx_interface_reg_read(codec->control_data,
			TABLA_SLIM_PGD_PORT_INT_STATUS0 + i);
		for_each_set_bit(j, &slimbus_value, BITS_PER_BYTE) {
			val = wcd9xxx_interface_reg_read(codec->control_data,
				TABLA_SLIM_PGD_PORT_INT_SOURCE0 + i*8 + j);
			if (val & 0x1)
				pr_err_ratelimited("overflow error on port %x,"
					" value %x\n", i*8 + j, val);
			if (val & 0x2)
				pr_err_ratelimited("underflow error on port %x,"
					" value %x\n", i*8 + j, val);
			if (val & 0x4) {
				pr_debug("%s: port %x disconnect value %x\n",
					__func__, i*8 + j, val);
				port_id = i*8 + j;
				for (k = 0; k < ARRAY_SIZE(tabla_dai); k++) {
					ch_mask_temp = 1 << port_id;
					if (ch_mask_temp &
						tabla_p->dai[k].ch_mask) {
						tabla_p->dai[k].ch_mask &=
								~ch_mask_temp;
					if (!tabla_p->dai[k].ch_mask)
							wake_up(
						&tabla_p->dai[k].dai_wait);
					}
				}
			}
		}
		wcd9xxx_interface_reg_write(codec->control_data,
			TABLA_SLIM_PGD_PORT_INT_CLR0 + i, slimbus_value);
		val = 0x0;
	}

	return IRQ_HANDLED;
}

static int tabla_handle_pdata(struct tabla_priv *tabla)
{
	struct snd_soc_codec *codec = tabla->codec;
	struct wcd9xxx_pdata *pdata = tabla->pdata;
	int k1, k2, k3, rc = 0;
	u8 leg_mode = pdata->amic_settings.legacy_mode;
	u8 txfe_bypass = pdata->amic_settings.txfe_enable;
	u8 txfe_buff = pdata->amic_settings.txfe_buff;
	u8 flag = pdata->amic_settings.use_pdata;
	u8 i = 0, j = 0;
	u8 val_txfe = 0, value = 0;

	if (!pdata) {
		rc = -ENODEV;
		goto done;
	}

	/* Make sure settings are correct */
	if ((pdata->micbias.ldoh_v > TABLA_LDOH_2P85_V) ||
	    (pdata->micbias.bias1_cfilt_sel > TABLA_CFILT3_SEL) ||
	    (pdata->micbias.bias2_cfilt_sel > TABLA_CFILT3_SEL) ||
	    (pdata->micbias.bias3_cfilt_sel > TABLA_CFILT3_SEL) ||
	    (pdata->micbias.bias4_cfilt_sel > TABLA_CFILT3_SEL)) {
		rc = -EINVAL;
		goto done;
	}

	/* figure out k value */
	k1 = tabla_find_k_value(pdata->micbias.ldoh_v,
		pdata->micbias.cfilt1_mv);
	k2 = tabla_find_k_value(pdata->micbias.ldoh_v,
		pdata->micbias.cfilt2_mv);
	k3 = tabla_find_k_value(pdata->micbias.ldoh_v,
		pdata->micbias.cfilt3_mv);

	if (IS_ERR_VALUE(k1) || IS_ERR_VALUE(k2) || IS_ERR_VALUE(k3)) {
		rc = -EINVAL;
		goto done;
	}

	/* Set voltage level and always use LDO */
	snd_soc_update_bits(codec, TABLA_A_LDO_H_MODE_1, 0x0C,
		(pdata->micbias.ldoh_v << 2));

	snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_1_VAL, 0xFC,
		(k1 << 2));
	snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_2_VAL, 0xFC,
		(k2 << 2));
	snd_soc_update_bits(codec, TABLA_A_MICB_CFILT_3_VAL, 0xFC,
		(k3 << 2));

	snd_soc_update_bits(codec, TABLA_A_MICB_1_CTL, 0x60,
		(pdata->micbias.bias1_cfilt_sel << 5));
	snd_soc_update_bits(codec, TABLA_A_MICB_2_CTL, 0x60,
		(pdata->micbias.bias2_cfilt_sel << 5));
	snd_soc_update_bits(codec, TABLA_A_MICB_3_CTL, 0x60,
		(pdata->micbias.bias3_cfilt_sel << 5));
	snd_soc_update_bits(codec, tabla->reg_addr.micb_4_ctl, 0x60,
			    (pdata->micbias.bias4_cfilt_sel << 5));

	for (i = 0; i < 6; j++, i += 2) {
		if (flag & (0x01 << i)) {
			value = (leg_mode & (0x01 << i)) ? 0x10 : 0x00;
			val_txfe = (txfe_bypass & (0x01 << i)) ? 0x20 : 0x00;
			val_txfe = val_txfe |
				((txfe_buff & (0x01 << i)) ? 0x10 : 0x00);
			snd_soc_update_bits(codec, TABLA_A_TX_1_2_EN + j * 10,
				0x10, value);
			snd_soc_update_bits(codec,
				TABLA_A_TX_1_2_TEST_EN + j * 10,
				0x30, val_txfe);
		}
		if (flag & (0x01 << (i + 1))) {
			value = (leg_mode & (0x01 << (i + 1))) ? 0x01 : 0x00;
			val_txfe = (txfe_bypass &
					(0x01 << (i + 1))) ? 0x02 : 0x00;
			val_txfe |= (txfe_buff &
					(0x01 << (i + 1))) ? 0x01 : 0x00;
			snd_soc_update_bits(codec, TABLA_A_TX_1_2_EN + j * 10,
				0x01, value);
			snd_soc_update_bits(codec,
				TABLA_A_TX_1_2_TEST_EN + j * 10,
				0x03, val_txfe);
		}
	}
	if (flag & 0x40) {
		value = (leg_mode & 0x40) ? 0x10 : 0x00;
		value = value | ((txfe_bypass & 0x40) ? 0x02 : 0x00);
		value = value | ((txfe_buff & 0x40) ? 0x01 : 0x00);
		snd_soc_update_bits(codec, TABLA_A_TX_7_MBHC_EN,
			0x13, value);
	}

	if (pdata->ocp.use_pdata) {
		/* not defined in CODEC specification */
		if (pdata->ocp.hph_ocp_limit == 1 ||
			pdata->ocp.hph_ocp_limit == 5) {
			rc = -EINVAL;
			goto done;
		}
		snd_soc_update_bits(codec, TABLA_A_RX_COM_OCP_CTL,
			0x0F, pdata->ocp.num_attempts);
		snd_soc_write(codec, TABLA_A_RX_COM_OCP_COUNT,
			((pdata->ocp.run_time << 4) | pdata->ocp.wait_time));
		snd_soc_update_bits(codec, TABLA_A_RX_HPH_OCP_CTL,
			0xE0, (pdata->ocp.hph_ocp_limit << 5));
	}

	for (i = 0; i < ARRAY_SIZE(pdata->regulator); i++) {
		if (!strncmp(pdata->regulator[i].name, "CDC_VDDA_RX", 11)) {
			if (pdata->regulator[i].min_uV == 1800000 &&
			    pdata->regulator[i].max_uV == 1800000) {
				snd_soc_write(codec, TABLA_A_BIAS_REF_CTL,
					      0x1C);
			} else if (pdata->regulator[i].min_uV == 2200000 &&
				   pdata->regulator[i].max_uV == 2200000) {
				snd_soc_write(codec, TABLA_A_BIAS_REF_CTL,
					      0x1E);
			} else {
				pr_err("%s: unsupported CDC_VDDA_RX voltage "
				       "min %d, max %d\n", __func__,
				       pdata->regulator[i].min_uV,
				       pdata->regulator[i].max_uV);
				rc = -EINVAL;
			}
			break;
		}
	}
done:
	return rc;
}

static const struct tabla_reg_mask_val tabla_1_1_reg_defaults[] = {

	/* Tabla 1.1 MICBIAS changes */
	TABLA_REG_VAL(TABLA_A_MICB_1_INT_RBIAS, 0x24),
	TABLA_REG_VAL(TABLA_A_MICB_2_INT_RBIAS, 0x24),
	TABLA_REG_VAL(TABLA_A_MICB_3_INT_RBIAS, 0x24),

	/* Tabla 1.1 HPH changes */
	TABLA_REG_VAL(TABLA_A_RX_HPH_BIAS_PA, 0x57),
	TABLA_REG_VAL(TABLA_A_RX_HPH_BIAS_LDO, 0x56),

	/* Tabla 1.1 EAR PA changes */
	TABLA_REG_VAL(TABLA_A_RX_EAR_BIAS_PA, 0xA6),
	TABLA_REG_VAL(TABLA_A_RX_EAR_GAIN, 0x02),
	TABLA_REG_VAL(TABLA_A_RX_EAR_VCM, 0x03),

	/* Tabla 1.1 Lineout_5 Changes */
	TABLA_REG_VAL(TABLA_A_RX_LINE_5_GAIN, 0x10),

	/* Tabla 1.1 RX Changes */
	TABLA_REG_VAL(TABLA_A_CDC_RX1_B5_CTL, 0x78),
	TABLA_REG_VAL(TABLA_A_CDC_RX2_B5_CTL, 0x78),
	TABLA_REG_VAL(TABLA_A_CDC_RX3_B5_CTL, 0x78),
	TABLA_REG_VAL(TABLA_A_CDC_RX4_B5_CTL, 0x78),
	TABLA_REG_VAL(TABLA_A_CDC_RX5_B5_CTL, 0x78),
	TABLA_REG_VAL(TABLA_A_CDC_RX6_B5_CTL, 0x78),
	TABLA_REG_VAL(TABLA_A_CDC_RX7_B5_CTL, 0x78),

	/* Tabla 1.1 RX1 and RX2 Changes */
	TABLA_REG_VAL(TABLA_A_CDC_RX1_B6_CTL, 0xA0),
	TABLA_REG_VAL(TABLA_A_CDC_RX2_B6_CTL, 0xA0),

	/* Tabla 1.1 RX3 to RX7 Changes */
	TABLA_REG_VAL(TABLA_A_CDC_RX3_B6_CTL, 0x80),
	TABLA_REG_VAL(TABLA_A_CDC_RX4_B6_CTL, 0x80),
	TABLA_REG_VAL(TABLA_A_CDC_RX5_B6_CTL, 0x80),
	TABLA_REG_VAL(TABLA_A_CDC_RX6_B6_CTL, 0x80),
	TABLA_REG_VAL(TABLA_A_CDC_RX7_B6_CTL, 0x80),

	/* Tabla 1.1 CLASSG Changes */
	TABLA_REG_VAL(TABLA_A_CDC_CLSG_FREQ_THRESH_B3_CTL, 0x1B),
};

static const struct tabla_reg_mask_val tabla_2_0_reg_defaults[] = {
	/* Tabla 2.0 MICBIAS changes */
	TABLA_REG_VAL(TABLA_A_MICB_2_MBHC, 0x02),
};

static const struct tabla_reg_mask_val tabla_1_x_only_reg_2_0_defaults[] = {
	TABLA_REG_VAL(TABLA_1_A_MICB_4_INT_RBIAS, 0x24),
};

static const struct tabla_reg_mask_val tabla_2_only_reg_2_0_defaults[] = {
	TABLA_REG_VAL(TABLA_2_A_MICB_4_INT_RBIAS, 0x24),
};

static void tabla_update_reg_defaults(struct snd_soc_codec *codec)
{
	u32 i;
	struct wcd9xxx *tabla_core = dev_get_drvdata(codec->dev->parent);

	for (i = 0; i < ARRAY_SIZE(tabla_1_1_reg_defaults); i++)
		snd_soc_write(codec, tabla_1_1_reg_defaults[i].reg,
				tabla_1_1_reg_defaults[i].val);

	for (i = 0; i < ARRAY_SIZE(tabla_2_0_reg_defaults); i++)
		snd_soc_write(codec, tabla_2_0_reg_defaults[i].reg,
				tabla_2_0_reg_defaults[i].val);

	if (TABLA_IS_1_X(tabla_core->version)) {
		for (i = 0; i < ARRAY_SIZE(tabla_1_x_only_reg_2_0_defaults);
		     i++)
			snd_soc_write(codec,
				      tabla_1_x_only_reg_2_0_defaults[i].reg,
				      tabla_1_x_only_reg_2_0_defaults[i].val);
	} else {
		for (i = 0; i < ARRAY_SIZE(tabla_2_only_reg_2_0_defaults); i++)
			snd_soc_write(codec,
				      tabla_2_only_reg_2_0_defaults[i].reg,
				      tabla_2_only_reg_2_0_defaults[i].val);
	}
}

static const struct tabla_reg_mask_val tabla_codec_reg_init_val[] = {
	/* Initialize current threshold to 350MA
	 * number of wait and run cycles to 4096
	 */
	{TABLA_A_RX_HPH_OCP_CTL, 0xE0, 0x60},
	{TABLA_A_RX_COM_OCP_COUNT, 0xFF, 0xFF},

	{TABLA_A_QFUSE_CTL, 0xFF, 0x03},

	/* Initialize gain registers to use register gain */
	{TABLA_A_RX_HPH_L_GAIN, 0x10, 0x10},
	{TABLA_A_RX_HPH_R_GAIN, 0x10, 0x10},
	{TABLA_A_RX_LINE_1_GAIN, 0x10, 0x10},
	{TABLA_A_RX_LINE_2_GAIN, 0x10, 0x10},
	{TABLA_A_RX_LINE_3_GAIN, 0x10, 0x10},
	{TABLA_A_RX_LINE_4_GAIN, 0x10, 0x10},

    /* Set the MICBIAS default output as pull down*/
    {TABLA_A_MICB_1_CTL, 0x01, 0x01},
    {TABLA_A_MICB_2_CTL, 0x01, 0x01},
    {TABLA_A_MICB_3_CTL, 0x01, 0x01},

	/* Initialize mic biases to differential mode */
	{TABLA_A_MICB_1_INT_RBIAS, 0x24, 0x24},
	{TABLA_A_MICB_2_INT_RBIAS, 0x24, 0x24},
	{TABLA_A_MICB_3_INT_RBIAS, 0x24, 0x24},

	{TABLA_A_CDC_CONN_CLSG_CTL, 0x3C, 0x14},

	/* Use 16 bit sample size for TX1 to TX6 */
	{TABLA_A_CDC_CONN_TX_SB_B1_CTL, 0x30, 0x20},
	{TABLA_A_CDC_CONN_TX_SB_B2_CTL, 0x30, 0x20},
	{TABLA_A_CDC_CONN_TX_SB_B3_CTL, 0x30, 0x20},
	{TABLA_A_CDC_CONN_TX_SB_B4_CTL, 0x30, 0x20},
	{TABLA_A_CDC_CONN_TX_SB_B5_CTL, 0x30, 0x20},
	{TABLA_A_CDC_CONN_TX_SB_B6_CTL, 0x30, 0x20},

	/* Use 16 bit sample size for TX7 to TX10 */
	{TABLA_A_CDC_CONN_TX_SB_B7_CTL, 0x60, 0x40},
	{TABLA_A_CDC_CONN_TX_SB_B8_CTL, 0x60, 0x40},
	{TABLA_A_CDC_CONN_TX_SB_B9_CTL, 0x60, 0x40},
	{TABLA_A_CDC_CONN_TX_SB_B10_CTL, 0x60, 0x40},

	/* Use 16 bit sample size for RX */
	{TABLA_A_CDC_CONN_RX_SB_B1_CTL, 0xFF, 0xAA},
	{TABLA_A_CDC_CONN_RX_SB_B2_CTL, 0xFF, 0xAA},

	/*enable HPF filter for TX paths */
	{TABLA_A_CDC_TX1_MUX_CTL, 0x8, 0x0},
	{TABLA_A_CDC_TX2_MUX_CTL, 0x8, 0x0},
	{TABLA_A_CDC_TX3_MUX_CTL, 0x8, 0x0},
	{TABLA_A_CDC_TX4_MUX_CTL, 0x8, 0x0},
	{TABLA_A_CDC_TX5_MUX_CTL, 0x8, 0x0},
	{TABLA_A_CDC_TX6_MUX_CTL, 0x8, 0x0},
	{TABLA_A_CDC_TX7_MUX_CTL, 0x8, 0x0},
	{TABLA_A_CDC_TX8_MUX_CTL, 0x8, 0x0},
	{TABLA_A_CDC_TX9_MUX_CTL, 0x8, 0x0},
	{TABLA_A_CDC_TX10_MUX_CTL, 0x8, 0x0},

	/* config Decimator for DMIC CLK_MODE_1(3.072Mhz@12.88Mhz mclk) */
	{TABLA_A_CDC_TX1_DMIC_CTL, 0x1, 0x1},
	{TABLA_A_CDC_TX2_DMIC_CTL, 0x1, 0x1},
	{TABLA_A_CDC_TX3_DMIC_CTL, 0x1, 0x1},
	{TABLA_A_CDC_TX4_DMIC_CTL, 0x1, 0x1},
	{TABLA_A_CDC_TX5_DMIC_CTL, 0x1, 0x1},
	{TABLA_A_CDC_TX6_DMIC_CTL, 0x1, 0x1},
	{TABLA_A_CDC_TX7_DMIC_CTL, 0x1, 0x1},
	{TABLA_A_CDC_TX8_DMIC_CTL, 0x1, 0x1},
	{TABLA_A_CDC_TX9_DMIC_CTL, 0x1, 0x1},
	{TABLA_A_CDC_TX10_DMIC_CTL, 0x1, 0x1},

	/* config DMIC clk to CLK_MODE_1 (3.072Mhz@12.88Mhz mclk) */
	{TABLA_A_CDC_CLK_DMIC_CTL, 0x2A, 0x2A},

};

static const struct tabla_reg_mask_val tabla_1_x_codec_reg_init_val[] = {
    /* Set the MICBIAS default output as pull down*/
    {TABLA_1_A_MICB_4_CTL, 0x01, 0x01},
	/* Initialize mic biases to differential mode */
	{TABLA_1_A_MICB_4_INT_RBIAS, 0x24, 0x24},
};

static const struct tabla_reg_mask_val tabla_2_higher_codec_reg_init_val[] = {
    /* Set the MICBIAS default output as pull down*/
    {TABLA_2_A_MICB_4_CTL, 0x01, 0x01},
	/* Initialize mic biases to differential mode */
	{TABLA_2_A_MICB_4_INT_RBIAS, 0x24, 0x24},
};

static void tabla_codec_init_reg(struct snd_soc_codec *codec)
{
	u32 i;
	struct wcd9xxx *tabla_core = dev_get_drvdata(codec->dev->parent);

	for (i = 0; i < ARRAY_SIZE(tabla_codec_reg_init_val); i++)
		snd_soc_update_bits(codec, tabla_codec_reg_init_val[i].reg,
				tabla_codec_reg_init_val[i].mask,
				tabla_codec_reg_init_val[i].val);
	if (TABLA_IS_1_X(tabla_core->version)) {
		for (i = 0; i < ARRAY_SIZE(tabla_1_x_codec_reg_init_val); i++)
			snd_soc_update_bits(codec,
					   tabla_1_x_codec_reg_init_val[i].reg,
					   tabla_1_x_codec_reg_init_val[i].mask,
					   tabla_1_x_codec_reg_init_val[i].val);
	} else {
		for (i = 0; i < ARRAY_SIZE(tabla_2_higher_codec_reg_init_val);
		     i++)
			snd_soc_update_bits(codec,
				      tabla_2_higher_codec_reg_init_val[i].reg,
				      tabla_2_higher_codec_reg_init_val[i].mask,
				      tabla_2_higher_codec_reg_init_val[i].val);
	}
}

static void tabla_update_reg_address(struct tabla_priv *priv)
{
	struct wcd9xxx *tabla_core = dev_get_drvdata(priv->codec->dev->parent);
	struct tabla_reg_address *reg_addr = &priv->reg_addr;

	if (TABLA_IS_1_X(tabla_core->version)) {
		reg_addr->micb_4_mbhc = TABLA_1_A_MICB_4_MBHC;
		reg_addr->micb_4_int_rbias = TABLA_1_A_MICB_4_INT_RBIAS;
		reg_addr->micb_4_ctl = TABLA_1_A_MICB_4_CTL;
	} else if (TABLA_IS_2_0(tabla_core->version)) {
		reg_addr->micb_4_mbhc = TABLA_2_A_MICB_4_MBHC;
		reg_addr->micb_4_int_rbias = TABLA_2_A_MICB_4_INT_RBIAS;
		reg_addr->micb_4_ctl = TABLA_2_A_MICB_4_CTL;
	}
}

//ASUS HANS++
void report_press_event(struct work_struct *work)
{
	button_jiffies = jiffies;
	
	msleep(100);
	if (gpio_get_value(JACK_IN_DET)){
// 		printk("--%s--timeout return---\n",__func__);
		return;
	}
    	else {
		if (g_tabla->mbhc_cfg.button_jack){
			
			if(hs_jiffies == 0){
				printk("--%s--hs_jiffies = 0--\n",__func__);
				return;
			}
			else if(time_is_before_jiffies(hs_jiffies + 2*HZ)){
				snd_soc_jack_report_no_dapm(g_tabla->mbhc_cfg.button_jack,
						SND_JACK_BTN_0,SND_JACK_BTN_0);
				printk("----------[%s] press----------\n",__func__);		
			}
			else{
			printk("---%s---in debounce time---\n",__func__);
				if (g_tabla->button_press == 0){
					g_tabla->button_press += 1;
					printk("----------[%s] press----------\n",__func__);
				}
				else{
					g_tabla->button_press = 0;
					printk("----------[%s] maybe is fake PRESS report----------\n",__func__);
				}
			}
		}
// 		else{
// 			printk("----------[%s] no define g_tabla->mbhc_cfg----------\n",__func__);
// 		}
    }

}

void report_release_event(struct work_struct *work)
{
	msleep(100);
	if (gpio_get_value(JACK_IN_DET))
        	return;
	else {
        	if (g_tabla->mbhc_cfg.button_jack){
			if(time_is_before_jiffies(hs_jiffies + 2*HZ)){
				snd_soc_jack_report_no_dapm(g_tabla->mbhc_cfg.button_jack,0,SND_JACK_BTN_0);
				printk("----------[%s] release----------\n",__func__);
			}
			else{
				if(g_tabla->button_press == 1){	
					g_tabla->button_press -= 1;
					snd_soc_jack_report_no_dapm(g_tabla->mbhc_cfg.button_jack,
							SND_JACK_BTN_0,SND_JACK_BTN_0);
					snd_soc_jack_report_no_dapm(g_tabla->mbhc_cfg.button_jack
								,0,SND_JACK_BTN_0);
					printk("----------[%s] release----------\n",__func__);
				}
				else{
					g_tabla->button_press = 0;
					printk("----------[%s] maybe is fake RELEASE report----------\n",__func__);
				}
			}
        	}
//         	else{
// 			printk("----------[%s] no define g_tabla->mbhc_cfg----------\n",__func__);
// 		}
    	}
}

static void button_timer_fun(unsigned long _data)
{
	int state;
	if (time_is_before_jiffies(hs_jiffies + HZ)){
		state = gpio_get_value(HS_HOOK_DET);
		if (state==0) 
			schedule_work(&button_release_work);
		else 
			schedule_work(&button_press_work);
	}
}

int gGPIO_JACK_IN = 0;
EXPORT_SYMBOL(gGPIO_JACK_IN);

void report_hs_event(struct work_struct *work)
{
	int state;
	g_tabla->button_press = 0;
	
	state = !gpio_get_value(JACK_IN_DET);     
	if (state == 0) {
		switch_set_state(&g_tabla->headset_jack->sdev, state);
	
		gpio_direction_output(PM8921_GPIO_PM_TO_SYS(19), 0);
		gpio_direction_output(PM8921_GPIO_PM_TO_SYS(20), 0);//enable uart log, disable audio
		button_jiffies = hs_jiffies = 0;
		gGPIO_JACK_IN = 0;		

		printk("----------[%s]HS detect removal!!! ----------\n",__func__);
	}
	else {
		gpio_direction_output(PM8921_GPIO_PM_TO_SYS(19), 1);
		gpio_direction_output(PM8921_GPIO_PM_TO_SYS(20), 1);////disable uart log, enable audio
		msleep(50);
		hs_jiffies = jiffies;
		gGPIO_JACK_IN = 1;	

		while(hook_irq_balance >= 1){
			enable_irq(MSM_GPIO_TO_INT(HS_HOOK_DET));
			hook_irq_balance -= 1;
		}
			
		if (!gpio_get_value(HS_HOOK_DET)) {  //headset
			switch_set_state(&g_tabla->headset_jack->sdev, 1);
			printk("----------[%s]HS insertion-(1) ----------\n",__func__);
		}
		else {  //headphone
			switch_set_state(&g_tabla->headset_jack->sdev, 2);
			printk("----------[%s]HP insertion-(2) ----------\n",__func__);
		}
	}
}

static void hs_timer_fun(unsigned long _data)
{

	schedule_work(&report_hs_event_work);
}

static irqreturn_t tabla_hs_detect_irq(int irq, void *data)
{
	disable_irq(MSM_GPIO_TO_INT(HS_HOOK_DET));
	hook_irq_balance += 1;	

//ASUS BSP HANS++
	if (g_flag_csvoice_fe_connected || FMStatus)
		wake_lock_timeout(&jack_in_wake_lock, 3 * HZ);
//ASUS BSP HANS--

	mod_timer(&hs_timer,jiffies + msecs_to_jiffies(1000));
	return IRQ_HANDLED;
}

static irqreturn_t tabla_button_detect_irq(int irq, void *data)
{
	mod_timer(&button_timer,jiffies + msecs_to_jiffies(50));
	return IRQ_HANDLED;
}
//ASUS HANS--

//Bruno++ Audio debug mode
#ifdef  CONFIG_PROC_FS
#define Audio_debug_PROC_FILE  "driver/audio_debug"
static struct proc_dir_entry *audio_debug_proc_file;

#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/file.h>
static mm_segment_t oldfs;
static void initKernelEnv(void)
{
    oldfs = get_fs();
    set_fs(KERNEL_DS);
}

static void deinitKernelEnv(void)
{
    set_fs(oldfs);
}

u32 bMaxxOn = 0;
extern int IsA68Spk;
// extern int g_flag_csvoice_fe_connected;
extern int gSKYPE_state;
extern int gRingtone_state;
extern int gGarmin_state;
void ApplyA68SPKGain(void)
{
    u32 lineout1, lineout3;
    lineout1 = wcd9xxx_reg_read(g_tabla->codec->control_data, TABLA_A_RX_LINE_1_GAIN);
    lineout3 = wcd9xxx_reg_read(g_tabla->codec->control_data, TABLA_A_RX_LINE_3_GAIN);
    printk("[Audio][MaxxAudio] lineout1:0x%x lineout3:0x%x, IsA68Spk:%d gGarmin_state=%d\n", lineout1, lineout3, IsA68Spk,gGarmin_state);

    if ((((lineout1 & 0x0F) != 12) && ((lineout3 & 0x0F) != 12)) || (IsA68Spk)) {
        if ((g_flag_csvoice_fe_connected) || (gSKYPE_state) || (gRingtone_state) || (gGarmin_state)) {
            wcd9xxx_reg_write(g_tabla->codec->control_data, TABLA_A_RX_LINE_1_GAIN, ((lineout1 & 0xF0)|4));     //-6db
            wcd9xxx_reg_write(g_tabla->codec->control_data, TABLA_A_RX_LINE_3_GAIN, ((lineout3 & 0xF0)|4));     //-6db       
        } else {
            if (bMaxxOn) {
                wcd9xxx_reg_write(g_tabla->codec->control_data, TABLA_A_RX_LINE_1_GAIN, ((lineout1 & 0xF0)|4)); //-6db
                wcd9xxx_reg_write(g_tabla->codec->control_data, TABLA_A_RX_LINE_3_GAIN, ((lineout3 & 0xF0)|4)); //-6db
            } else {
                wcd9xxx_reg_write(g_tabla->codec->control_data, TABLA_A_RX_LINE_1_GAIN, ((lineout1 & 0xF0)|8)); //-12db
                wcd9xxx_reg_write(g_tabla->codec->control_data, TABLA_A_RX_LINE_3_GAIN, ((lineout3 & 0xF0)|8)); //-12db
            }
        }
    }
    lineout1 = wcd9xxx_reg_read(g_tabla->codec->control_data, TABLA_A_RX_LINE_1_GAIN);
    lineout3 = wcd9xxx_reg_read(g_tabla->codec->control_data, TABLA_A_RX_LINE_3_GAIN);
    printk("[Audio][MaxxAudio] lineout1:0x%x lineout3:0x%x\n", lineout1, lineout3);
}
EXPORT_SYMBOL_GPL(ApplyA68SPKGain);

static ssize_t audio_debug_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    char messages[256];
    memset(messages, 0, sizeof(messages));

    printk("[Audio Debug] audio_debug_proc_write\n");
    if (len > 256)
    {
        len = 256;
    }
    if (copy_from_user(messages, buff, len))
    {
        return -EFAULT;
    }
    
    initKernelEnv();

    if(strncmp(messages, "1", 1) == 0)
    {
        //ASUS HANS+++
        disable_irq(MSM_GPIO_TO_INT(JACK_IN_DET));
        jack_irq_balance +=1;
        
        disable_irq(MSM_GPIO_TO_INT(HS_HOOK_DET));
        hook_irq_balance += 1;
        
        switch_set_state(&g_tabla->headset_jack->sdev, 0);
        
        gpio_direction_output(PM8921_GPIO_PM_TO_SYS(19), 0);
        gpio_direction_output(PM8921_GPIO_PM_TO_SYS(20), 0);//enable uart log, disable audio
        //ASUS HANS---

        g_bDebugMode = 1;
        printk("Audio Debug Mode!!!\n");
    }
    else if(strncmp(messages, "0", 1) == 0)
    {
        //ASUS HANS+++
		while(jack_irq_balance){
			enable_irq(MSM_GPIO_TO_INT(JACK_IN_DET));
			jack_irq_balance -=1;
		}

		schedule_work(&report_hs_event_work);
        //ASUS HANS---

        printk("Audio Headset Normal Mode!!!\n");

        //TIM-switch audio output between headset and speaker--
        g_bDebugMode = 0;            
    }

    //read register
    else if(strncmp(messages, "read", strlen("read")) == 0)
    {
        u32 val, reg_val;
        sscanf(messages + 5, "%x", &reg_val);
        val = wcd9xxx_reg_read(g_tabla->codec->control_data, reg_val);
        printk("[Audio][wcd9310] read register reg[%x]=[%x]\n", reg_val, val);        
    }

    //write register
    else if(strncmp(messages, "write", strlen("write")) == 0)
    {
        u32 val, reg_val;
        sscanf(messages + 6, "%x %x", &reg_val, &val);        
        wcd9xxx_reg_write(g_tabla->codec->control_data, reg_val, val);

        val = wcd9xxx_reg_read(g_tabla->codec->control_data, reg_val);
        printk("[Audio][wcd9310] write register reg[%x]=[%x]\n", reg_val, val);        
    }

    //maxxaudio
    else if(strncmp(messages, "maxxaudio", strlen("maxxaudio")) == 0)
    {
        sscanf(messages + 10, "%x", &bMaxxOn);
        printk("[Audio][MaxAudio] bMaxxOn = %d\n", bMaxxOn);
        ApplyA68SPKGain();
    }

    deinitKernelEnv(); 
    return len;
}

static struct file_operations audio_debug_proc_ops = {
    //.read = audio_debug_proc_read,
    .write = audio_debug_proc_write,
};

static void create_audio_debug_proc_file(void)
{
    printk("[Audio] create_audio_debug_proc_file\n");
    audio_debug_proc_file = create_proc_entry(Audio_debug_PROC_FILE, 0666, NULL);
    if (audio_debug_proc_file) {
        audio_debug_proc_file->proc_fops = &audio_debug_proc_ops;
    } 
}

static void remove_audio_debug_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    printk("[Audio] remove_audio_debug_proc_file\n");   
    remove_proc_entry(Audio_debug_PROC_FILE, &proc_root);
}
#endif //#ifdef CONFIG_PROC_FS
//Bruno++ Audio debug mode

static int tabla_codec_probe(struct snd_soc_codec *codec)
{
	struct wcd9xxx *control;
	struct tabla_priv *tabla;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret = 0;
	int i;
	int ch_cnt;

//ASUS HANS++
	struct gpio_switch_data *switch_data;
	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;
	
	switch_data->sdev.name = "h2w";
	switch_data->sdev.state = 0;
	switch_data->gpio = JACK_IN_DET;
	
	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		printk("fail to register switch\n");
	else
		printk("success to register switch\n");

	wake_lock_init(&jack_in_wake_lock, WAKE_LOCK_SUSPEND, "jack_in_lock");
	printk(KERN_INFO "[PM]Initialize a wakelock of jack_in\r\n");
//ASUS HANS--

	codec->control_data = dev_get_drvdata(codec->dev->parent);
	control = codec->control_data;

	tabla = kzalloc(sizeof(struct tabla_priv), GFP_KERNEL);
	if (!tabla) {
		dev_err(codec->dev, "Failed to allocate private data\n");
		return -ENOMEM;
	}
	for (i = 0 ; i < NUM_DECIMATORS; i++) {
		tx_hpf_work[i].tabla = tabla;
		tx_hpf_work[i].decimator = i + 1;
		INIT_DELAYED_WORK(&tx_hpf_work[i].dwork,
			tx_hpf_corner_freq_callback);
	}

	snd_soc_codec_set_drvdata(codec, tabla);

	tabla->mclk_enabled = false;
	tabla->bandgap_type = TABLA_BANDGAP_OFF;
	tabla->clock_active = false;
	tabla->config_mode_active = false;
	tabla->codec = codec;
	for (i = 0; i < COMPANDER_MAX; i++) {
		tabla->comp_enabled[i] = 0;
		tabla->comp_fs[i] = COMPANDER_FS_48KHZ;
	}
	tabla->pdata = dev_get_platdata(codec->dev->parent);
	tabla->intf_type = wcd9xxx_get_intf_type();
	tabla->aux_pga_cnt = 0;
	tabla->aux_l_gain = 0x1F;
	tabla->aux_r_gain = 0x1F;
	tabla_update_reg_address(tabla);
	tabla_update_reg_defaults(codec);
	tabla_codec_init_reg(codec);
	ret = tabla_handle_pdata(tabla);
	if (IS_ERR_VALUE(ret)) {
		pr_err("%s: bad pdata\n", __func__);
		goto err_pdata;
	}

	snd_soc_add_controls(codec, tabla_snd_controls,
			     ARRAY_SIZE(tabla_snd_controls));
	if (TABLA_IS_1_X(control->version))
		snd_soc_add_controls(codec, tabla_1_x_snd_controls,
				     ARRAY_SIZE(tabla_1_x_snd_controls));
	else
		snd_soc_add_controls(codec, tabla_2_higher_snd_controls,
				     ARRAY_SIZE(tabla_2_higher_snd_controls));

	snd_soc_dapm_new_controls(dapm, tabla_dapm_widgets,
				  ARRAY_SIZE(tabla_dapm_widgets));

	snd_soc_dapm_new_controls(dapm, tabla_dapm_aif_in_widgets,
				  ARRAY_SIZE(tabla_dapm_aif_in_widgets));

	snd_soc_dapm_new_controls(dapm, tabla_dapm_aif_out_widgets,
				  ARRAY_SIZE(tabla_dapm_aif_out_widgets));

	if (TABLA_IS_1_X(control->version))
		snd_soc_dapm_new_controls(dapm, tabla_1_x_dapm_widgets,
					  ARRAY_SIZE(tabla_1_x_dapm_widgets));
	else
		snd_soc_dapm_new_controls(dapm, tabla_2_higher_dapm_widgets,
				    ARRAY_SIZE(tabla_2_higher_dapm_widgets));

	if (tabla->intf_type == WCD9XXX_INTERFACE_TYPE_I2C) {
		snd_soc_dapm_new_controls(dapm, tabla_dapm_i2s_widgets,
			ARRAY_SIZE(tabla_dapm_i2s_widgets));
		snd_soc_dapm_add_routes(dapm, audio_i2s_map,
			ARRAY_SIZE(audio_i2s_map));
	}
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	if (TABLA_IS_1_X(control->version)) {
		snd_soc_dapm_add_routes(dapm, tabla_1_x_lineout_2_to_4_map,
				      ARRAY_SIZE(tabla_1_x_lineout_2_to_4_map));
	} else if (TABLA_IS_2_0(control->version)) {
		snd_soc_dapm_add_routes(dapm, tabla_2_x_lineout_2_to_4_map,
				      ARRAY_SIZE(tabla_2_x_lineout_2_to_4_map));
	} else  {
		pr_err("%s : ERROR.  Unsupported Tabla version 0x%2x\n",
			__func__, control->version);
		goto err_pdata;
	}

	snd_soc_dapm_sync(dapm);

	ret = wcd9xxx_request_irq(codec->control_data, TABLA_IRQ_SLIMBUS,
		tabla_slimbus_irq, "SLIMBUS Slave", tabla);
	if (ret) {
		pr_err("%s: Failed to request irq %d\n", __func__,
			TABLA_IRQ_SLIMBUS);
		goto err_slimbus_irq;
	}

	for (i = 0; i < WCD9XXX_SLIM_NUM_PORT_REG; i++)
		wcd9xxx_interface_reg_write(codec->control_data,
			TABLA_SLIM_PGD_PORT_INT_EN0 + i, 0xFF);

	for (i = 0; i < ARRAY_SIZE(tabla_dai); i++) {
		switch (tabla_dai[i].id) {
		case AIF1_PB:
			ch_cnt = tabla_dai[i].playback.channels_max;
			break;
		case AIF1_CAP:
			ch_cnt = tabla_dai[i].capture.channels_max;
			break;
		case AIF2_PB:
			ch_cnt = tabla_dai[i].playback.channels_max;
			break;
		case AIF2_CAP:
			ch_cnt = tabla_dai[i].capture.channels_max;
			break;
		case AIF3_PB:
			ch_cnt = tabla_dai[i].playback.channels_max;
			break;
		case AIF3_CAP:
			ch_cnt = tabla_dai[i].capture.channels_max;
			break;
		default:
			continue;
		}
		tabla->dai[i].ch_num = kzalloc((sizeof(unsigned int)*
					ch_cnt), GFP_KERNEL);
		init_waitqueue_head(&tabla->dai[i].dai_wait);
	}

//ASUS HANS++
	tabla->headset_jack = switch_data;

	ret = request_irq(MSM_GPIO_TO_INT(JACK_IN_DET), tabla_hs_detect_irq, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "JACK_IN Status IRQ", tabla);
	if (ret) {
		pr_err("%s: Failed to request irq JACK_IN\n", __func__);
	}
		
	ret = request_irq(MSM_GPIO_TO_INT(HS_HOOK_DET), tabla_button_detect_irq, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "HS_HOOK Status IRQ", tabla);
	if (ret) {
		pr_err("%s: Failed to request irq HS_HOOK\n", __func__);
	}
	disable_irq(MSM_GPIO_TO_INT(HS_HOOK_DET));

	g_tabla = tabla;

	setup_timer(&hs_timer, hs_timer_fun, (unsigned long)tabla);
	setup_timer(&button_timer, button_timer_fun, (unsigned long)tabla);

	INIT_WORK(&button_press_work, report_press_event);
	INIT_WORK(&button_release_work, report_release_event);
	INIT_WORK(&report_hs_event_work, report_hs_event);

	if(!gpio_get_value(JACK_IN_DET)){
		schedule_work(&report_hs_event_work);
	}
//ASUS HANS--

#ifdef  CONFIG_PROC_FS
    create_audio_debug_proc_file();
#endif

	return ret;

err_slimbus_irq:
err_pdata:
	kfree(tabla);
	return ret;
}
static int tabla_codec_remove(struct snd_soc_codec *codec)
{
	int i;
	struct tabla_priv *tabla = snd_soc_codec_get_drvdata(codec);
	wcd9xxx_free_irq(codec->control_data, TABLA_IRQ_SLIMBUS, tabla);
	tabla_codec_disable_clock_block(codec);
	tabla_codec_enable_bandgap(codec, TABLA_BANDGAP_OFF);
	for (i = 0; i < ARRAY_SIZE(tabla_dai); i++)
		kfree(tabla->dai[i].ch_num);

	kfree(tabla);
//Bruno++	
#ifdef  CONFIG_PROC_FS
    remove_audio_debug_proc_file();
#endif

	return 0;
}
static struct snd_soc_codec_driver soc_codec_dev_tabla = {
	.probe	= tabla_codec_probe,
	.remove	= tabla_codec_remove,
	.read = tabla_read,
	.write = tabla_write,

	.readable_register = tabla_readable,
	.volatile_register = tabla_volatile,

	.reg_cache_size = TABLA_CACHE_SIZE,
	.reg_cache_default = tabla_reg_defaults,
	.reg_word_size = 1,
};

#ifdef CONFIG_PM
// extern int g_flag_csvoice_fe_connected;
// extern int FMStatus;
static int a68_wake;

static int tabla_suspend(struct device *dev)
{
	dev_dbg(dev, "%s: system suspend\n", __func__);

	while(jack_irq_balance){
		enable_irq(MSM_GPIO_TO_INT(JACK_IN_DET));
		jack_irq_balance -=1;
	}

	while(hook_irq_balance >= 1){
		enable_irq(MSM_GPIO_TO_INT(HS_HOOK_DET));
		hook_irq_balance -= 1;
	}


	//ASUS BSP HANS++
	if (g_flag_csvoice_fe_connected || FMStatus){

		enable_irq_wake(MSM_GPIO_TO_INT(6));
		enable_irq_wake(MSM_GPIO_TO_INT(62));
        	a68_wake = 1;
	}
	//ASUS BSP HANS--


	return 0;
}

static int tabla_resume(struct device *dev)
{
	dev_dbg(dev, "%s: system resume\n", __func__);

	//ASUS BSP HANS+++
	if (a68_wake){
        	disable_irq_wake(gpio_to_irq(6));
        	disable_irq_wake(gpio_to_irq(62));
        	a68_wake = 0;
	}
	//ASUS BSP HANS---

	return 0;
}

static const struct dev_pm_ops tabla_pm_ops = {
	.suspend	= tabla_suspend,
	.resume		= tabla_resume,
};
#endif

static int __devinit tabla_probe(struct platform_device *pdev)
{
	int ret = 0;
	if (wcd9xxx_get_intf_type() == WCD9XXX_INTERFACE_TYPE_SLIMBUS)
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_tabla,
			tabla_dai, ARRAY_SIZE(tabla_dai));
	else if (wcd9xxx_get_intf_type() == WCD9XXX_INTERFACE_TYPE_I2C)
		ret = snd_soc_register_codec(&pdev->dev, &soc_codec_dev_tabla,
			tabla_i2s_dai, ARRAY_SIZE(tabla_i2s_dai));
	return ret;
}
static int __devexit tabla_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}
static struct platform_driver tabla_codec_driver = {
	.probe = tabla_probe,
	.remove = tabla_remove,
	.driver = {
		.name = "tabla_codec",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &tabla_pm_ops,
#endif
	},
};

static struct platform_driver tabla1x_codec_driver = {
	.probe = tabla_probe,
	.remove = tabla_remove,
	.driver = {
		.name = "tabla1x_codec",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &tabla_pm_ops,
#endif
	},
};

static int __init tabla_codec_init(void)
{
	int rtn = platform_driver_register(&tabla_codec_driver);
	if (rtn == 0) {
		rtn = platform_driver_register(&tabla1x_codec_driver);
		if (rtn != 0)
			platform_driver_unregister(&tabla_codec_driver);
	}
	return rtn;
}

static void __exit tabla_codec_exit(void)
{
	platform_driver_unregister(&tabla1x_codec_driver);
	platform_driver_unregister(&tabla_codec_driver);
}

module_init(tabla_codec_init);
module_exit(tabla_codec_exit);

MODULE_DESCRIPTION("Tabla codec driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
