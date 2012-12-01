/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
//Bruno++ replace P01 key for debug
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
//Bruno++ replace P01 key for debug
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
//Ledger ++
#include <linux/wakelock.h>
//Ledger --
//ASUS BSP TIM-2011.10.04++
#include <linux/microp_notify.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
static struct input_dev *g_input_dev;
//ASUS BSP TIM-2011.10.04--
//Bruno++ replace P01 key for debug
#define P01_KEY_VOLUP   417
#define P01_KEY_VOLDOWN 416
#define P01_KEY_POWER   418
#define power_key_for_test 30
static int pad_pwr_key_press_state = 0;
//Ledger ++
static struct wake_lock pwr_key_wake_lock;
static bool g_bResume=1,g_bpwr_key_lock_sts=0;
extern int g_flag_csvoice_fe_connected;
extern int FMStatus;
//Ledger --

struct P01_button_code {
    int vol_up;
    int vol_down;
    int power_key;
};
//Bruno++ replace P01 key for debug

//ASUS BSP HANS++
static int pwk_state = 0;
static int pwk_wake = 0;
//ASUS BSP HANS--


// ASUS_BSP SinaChou ++
extern int pm_new_state;
// ASUS_BSP SinaChou --
struct gpio_button_data {
	struct gpio_keys_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	int timer_debounce;	/* in msecs */
	bool disabled;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned int n_buttons;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
    struct P01_button_code p01button_code;	
	struct gpio_button_data data[0];
};


//jack for debug slow
//#include "../../../sound/soc/codecs/wcd9310.h"
static  struct work_struct __wait_for_two_keys_work;

void wait_for_two_keys_work(struct work_struct *work)
{
    static int one_instance_running = 0;
    int i, volume_up_key, volume_down_key, power_key;

    volume_up_key = 53;
    volume_down_key = 54;
    power_key = 26;

    //printk("wait_for_two_keys_work++\n");
    if(!one_instance_running)
    { 
        if(gpio_get_value_cansleep(power_key) != 0 || gpio_get_value_cansleep(volume_up_key) != 0 || gpio_get_value_cansleep(volume_down_key) != 0)
        {
            //printk("wait_for_two_keys_work one of the keys is not pressed wait_for_two_keys_work--\n");
            return;
        }
        one_instance_running = 1;
        
        for(i = 0; i < 20; i++)
        {
            if(gpio_get_value_cansleep(volume_up_key) == 0 && gpio_get_value_cansleep(volume_down_key) == 0 && gpio_get_value_cansleep(power_key) == 0 )   
            {
                msleep(100);
            }         
            else
                break;
        }
        if(i == 20)
        {
            printk("start to gi chk\n");
            save_all_thread_info();
            
            msleep(5 * 1000);
            
            printk("start to gi delta\n");
            delta_all_thread_info();
            save_phone_hang_log();
            //Dump_wcd9310_reg();     //Bruno++    
            //printk_lcd("slow log captured\n");
        }
        else
        {
            //printk("wait_for_two_keys_work one of the keys is not pressed\n");
        }
        one_instance_running = 0;
    }
    //else
    //    printk("wait_for_two_keys_work already running\n");
    //printk("wait_for_two_keys_work--\n");
            
    
}
/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

/**
 * gpio_keys_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and possible debouncing timer.
		 */
		disable_irq(gpio_to_irq(bdata->button->gpio));
		if (bdata->timer_debounce)
			del_timer_sync(&bdata->timer);

		bdata->disabled = true;
	}
}

/**
 * gpio_keys_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(gpio_to_irq(bdata->button->gpio));
		bdata->disabled = false;
	}
}

/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
					  char *buf, unsigned int type,
					  bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(bdata->button->code, bits);
	}

	ret = bitmap_scnlistprintf(buf, PAGE_SIZE - 2, bits, n_events);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);

	return ret;
}

/**
 * gpio_keys_attr_store_helper() - enable/disable buttons based on given bitmap
 * @ddata: pointer to drvdata
 * @buf: buffer from userspace that contains stringified bitmap
 * @type: button type (%EV_KEY, %EV_SW)
 *
 * This function parses stringified bitmap from @buf and disables/enables
 * GPIO buttons accordinly. Returns 0 on success and negative error
 * on failure.
 */
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
					   const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	/* First validate */
	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits) &&
		    !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
static ssize_t gpio_keys_show_##name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
									\
	return gpio_keys_attr_show_helper(ddata, buf,			\
					  type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/keys [ro]
 * /sys/devices/platform/gpio-keys/switches [ro]
 */
static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)					\
static ssize_t gpio_keys_store_##name(struct device *dev,		\
				      struct device_attribute *attr,	\
				      const char *buf,			\
				      size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
									\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
		return error;						\
									\
	return count;							\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/disabled_keys [rw]
 * /sys/devices/platform/gpio-keys/disables_switches [rw]
 */
static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};
static unsigned int count_start = 0;  
static unsigned int count = 0;  
extern void set_dload_mode(int on);
extern void resetdevice(void);
#include <linux/reboot.h>
#include <asm/cacheflush.h>
#include <linux/asus_global.h>
extern struct _asus_global asus_global;

static void gpio_keys_report_event(struct gpio_button_data *bdata)
{
	struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;
   	int volume_up_key, volume_down_key;
	volume_up_key = 53;
	volume_down_key = 54;

//ASUS BSP TIM-2011.09.25++
    printk("key code=%d  state=%s\n",button->code,state ? "press" : "release");
//ASUS BSP TIM-2011.09.25--

	if (gpio_get_value_cansleep(volume_up_key) == 0){
		count_start = 1;
	}
	else{
		count_start = 0;
	}
	if (count_start)
	{
		if (gpio_get_value_cansleep(volume_down_key) == 0)
		{
			count++;
			if (count == 10)
			{
				printk("Kernel alive...\r\n");
				
				set_dload_mode(0);
				asus_global.ramdump_enable_magic = 0;
				printk(KERN_CRIT "asus_global.ramdump_enable_magic = 0x%x\n",asus_global.ramdump_enable_magic);
				flush_cache_all();	
				//reset device	
				resetdevice();				
			}		
		}
	}
	else
	{
		count = 0;
	}
	if (type == EV_ABS) {
		if (state)
			input_event(input, type, button->code, button->value);
	} else {
		//ASUS BSP HANS++
		if(button->code == KEY_POWER){
			if(state){
				pwk_state = 1;
				pwk_wake = 1;
			}
			else if(!pwk_state && pwk_wake) {
//Ledger ++
//                                printk("[PM]%s- pwr fake-key:%x,pm sts:%x,keylock sts:%x\r\n",__func__,state,g_bResume,g_bpwr_key_lock_sts);
                                if (g_bResume) {
                                        wake_lock_timeout(&pwr_key_wake_lock, 3 * HZ);
                                        g_bpwr_key_lock_sts=1;
                                        g_bResume=0;
                                        printk(KERN_INFO "[PM]Wakelock 3 sec for PWR fake-key on A6x\r\n");
                                }
                                else if(g_bpwr_key_lock_sts) {
                                        wake_unlock(&pwr_key_wake_lock);
                                        g_bpwr_key_lock_sts=0;
                                        printk(KERN_INFO "[PM]Unlock 3 sec for PWR fake-key on A6x\r\n");
                                }
//Ledger --
				pwk_state = 1;
				pwk_wake = 0;
				printk("send fake power key press event! +++++++\n");
				input_event(input, type, button->code, !!!state);
				input_sync(input);
				msleep(200);
			}else{
				pwk_state = 0;
			}
		}
		//ASUS BSP HANS--
//Ledger ++
                if (state){     //press
                        if (button->code == KEY_POWER) {
//                                printk("[PM]%s- pwr key:%x,pm sts:%x,keylock sts:%x\r\n",__func__,state,g_bResume,g_bpwr_key_lock_sts);
                                if (g_bResume) {
                                        wake_lock_timeout(&pwr_key_wake_lock, 3 * HZ);
                                        g_bpwr_key_lock_sts=1;
                                        g_bResume=0;
                                        printk(KERN_INFO "[PM]Wakelock 3 sec for PWR key on A6x\r\n");
                                }
                                else if(g_bpwr_key_lock_sts) {
                                        wake_unlock(&pwr_key_wake_lock);
                                        g_bpwr_key_lock_sts=0;
                                        printk(KERN_INFO "[PM]Unlock 3 sec for PWR key on A6x\r\n");
                                }
                        }
                        else if ( ((button->code == KEY_VOLUMEUP) || (button->code == KEY_VOLUMEDOWN)) 
                                && (g_flag_csvoice_fe_connected || FMStatus) ) {
//                                printk("[PM]%s- vol key:%x,pm sts:%x,keylock sts:%x\r\n",__func__,state,g_bResume,g_bpwr_key_lock_sts);
                                if (g_bResume) {
                                        wake_lock_timeout(&pwr_key_wake_lock, 3 * HZ);
                                        g_bpwr_key_lock_sts=1;
                                        g_bResume=0;
                                        printk(KERN_INFO "[PM]Wakelock 3 sec for VOL key on A6x\r\n");
                                }
                        }
                }
//Ledger ++
		input_event(input, type, button->code, !!state);
	}
	input_sync(input);
}

//ASUS BSP TIM-2011.10.04++
static void Pad_keys_report_event(int button_code, int press)
{
    printk("PAD key code=%d  state=%s\n",button_code, press ? "press" : "release");
//Ledger ++
        if (button_code==P01_KEY_POWER && press){     //press
                printk("[PM]%s- p03 pwr key:%x,pm sts:%x,keylock sts:%x\r\n",__func__,press,g_bResume,g_bpwr_key_lock_sts);
                if (g_bResume) {
                        wake_lock_timeout(&pwr_key_wake_lock, 3 * HZ);
                        g_bpwr_key_lock_sts=1;
                        g_bResume=0;
                        printk(KERN_INFO "[PM]Wakelock 3 sec for PWR key on P0x\r\n");
                }
                else if(g_bpwr_key_lock_sts) {
                        wake_unlock(&pwr_key_wake_lock);
                        g_bpwr_key_lock_sts=0;
                        printk(KERN_INFO "[PM]Unlock 3 sec for PWR key on P0x\r\n");
                }
        }
//Ledger --
    input_event(g_input_dev, EV_KEY, button_code, press);
    input_sync(g_input_dev);
}
static void gpio_keys_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);
    //added by jack for slow log
    schedule_work(&__wait_for_two_keys_work);
	gpio_keys_report_event(bdata);
}

static void gpio_keys_timer(unsigned long _data)
{
	struct gpio_button_data *data = (struct gpio_button_data *)_data;

	schedule_work(&data->work);
}

//ASUS_BSP +++ Peter_lu "For fastboot mode"
#ifdef CONFIG_FASTBOOT
#include <linux/fastboot.h>
//#include <linux/wakelock.h>
#include <linux/spinlock.h>
//#include <linux/mutex.h>
static DEFINE_SPINLOCK(handler_lock);

enum PWR_KEY_STATE
{
    PWR_KEY_STATE_NOT_HANDLED_YET = 0,
    PWR_KEY_STATE_WAITTING_FOR_DEBUNCING_TIMEOUT,
    PWR_KEY_STATE_WAITING_FOR_KEY_RELEASE,
    PWR_KEY_STATE_COUNT,
};
enum PWR_KEY_DEBOUNCING_LEVEL
{
    PWR_KEY_DEBOUNCING_LEVEL_NO = 0,
    PWR_KEY_DEBOUNCING_LEVEL_SHORT,
    PWR_KEY_DEBOUNCING_LEVEL_LONG,
    PWR_KEY_DEBOUNCING_LEVEL_COUNT,
};
struct power_key_context{
    void(*transit)(struct power_key_context * context, enum PWR_KEY_STATE  newstate);
    void(*startDebouncing)(struct power_key_context * context, enum PWR_KEY_DEBOUNCING_LEVEL level);
    void(*stopDebouncing)(struct power_key_context * context);
    bool(*isDebouncing)(struct power_key_context * context);
    void (*reportKey)(struct power_key_context * context, bool keyPressed);
};
struct power_key_state {
    enum PWR_KEY_STATE state;
    const char *name;
    struct power_key_context *context;
    void (*onPressed)(struct power_key_state *state);
    void (*onReleased)(struct power_key_state *state);
    void (*onDebouncingTimeout)(struct power_key_state *state);
    void (*setContext)(struct power_key_state *state, struct power_key_context *context);
};
//the member functions of power_key_state
static void power_key_state_setContext(struct power_key_state *this ,struct power_key_context *context)
{
    BUG_ON(this == NULL); 

    this->context = context;
}
//the member functions of not_handled_yet_state
static void not_handled_yet_state_onPressed(struct power_key_state *this)
{
    BUG_ON(this == NULL); 

    pr_debug("not_handled_yet_state_onPressed+++\n");

    if(this->context->isDebouncing(this->context)){

        return;
    }
   
    if(is_fastboot_enable()){
    
        this->context->startDebouncing(this->context, PWR_KEY_DEBOUNCING_LEVEL_LONG);

    }else{
    
        this->context->startDebouncing(this->context,PWR_KEY_DEBOUNCING_LEVEL_SHORT);//todo :should be defined in boardxxx.c

    }

    this->context->transit(this->context, PWR_KEY_STATE_WAITTING_FOR_DEBUNCING_TIMEOUT);

}
static void not_handled_yet_state_onReleased(struct power_key_state *this)
{
    return;
/*
    if(this->context->isDebouncing(this->context)){

        return;
    }
    
    //this->context->stopDebouncing(this->context);

    this->context->reportKey(this->context, false);
*/    
}
static void not_handled_yet_state_onDebouncingTimeout(struct power_key_state *this)
{
    return;
/*
    if(is_power_key_pressed()){
        
        this->context->reportKey(this->context,true);

    }
    */
}
//the member functions of waiting_for_debuncing_timeout_state
static void waiting_for_debuncing_timeout_state_onPressed(struct power_key_state *this)
{
    BUG_ON(this == NULL); 
    
    //just ignore it...
}
static void waiting_for_debuncing_timeout_state_onReleased(struct power_key_state *this)
{
    this->context->stopDebouncing(this->context);

    this->context->transit(this->context, PWR_KEY_STATE_NOT_HANDLED_YET);
    
}
static void waiting_for_debuncing_timeout_state_onDebouncingTimeout(struct power_key_state *this)
{
    this->context->reportKey(this->context,true);

    ready_to_wake_up_in_fastboot();

    this->context->transit(this->context, PWR_KEY_STATE_WAITING_FOR_KEY_RELEASE);
}
//the member functions of waiting_for_key_release_state
static void waiting_for_key_release_state_onPressed(struct power_key_state *this)
{
    BUG_ON(this == NULL); 
    
    //just ignore it...
}
static void waiting_for_key_release_state_onReleased(struct power_key_state *this)
{
    this->context->reportKey(this->context,false);

    this->context->transit(this->context, PWR_KEY_STATE_NOT_HANDLED_YET);
    
}
static void waiting_for_key_release_state_onDebouncingTimeout(struct power_key_state *this)
{
    BUG_ON(this == NULL); 
    
    //just ignore it...
}
struct power_key_handler {
    bool isInited;
    int debounce_interval_in_normal_mode;
    int keyCode;
    struct power_key_state *stateList;
    enum PWR_KEY_STATE currentState;
    struct timer_list timer; // for handle timeout...
    struct mutex lock;
    struct power_key_context context;
    struct wake_lock release_wake_lock;
    void (*init)(struct power_key_handler  *handler, int debounce_interval_in_normal_mode);    
    void (*deInit)(struct power_key_handler  *handler);
    void (*handle)(struct power_key_handler  *handler,int key_pressed);
    void (*time_expired)(unsigned long _data);
};
//the member functions of power_key_handler
static void power_key_handler_transit(struct power_key_context * context, enum PWR_KEY_STATE  newstate)
{
//    BUG_ON(NULL == context);

    struct power_key_handler  *this=
        container_of(context, struct power_key_handler, context);

    pr_debug("power_key_handler transit...\n");

    BUG_ON(this->currentState == newstate);

    pr_debug("old state:%s\n",this->stateList[this->currentState].name);

    this->currentState = newstate;

    pr_debug("new state:%s\n",this->stateList[this->currentState].name);

}
static void power_key_handler_startDebouncing(struct power_key_context * context, enum PWR_KEY_DEBOUNCING_LEVEL level)
{
   // BUG_ON(NULL == context);

    struct power_key_handler  *this=
        container_of(context, struct power_key_handler, context);

    int expires;

    //mutex_lock(&this->lock);
    switch(level){

        case PWR_KEY_DEBOUNCING_LEVEL_LONG:
            expires =    TIME_FOR_POWERKEY_LONGPRESS;
            break;
        case PWR_KEY_DEBOUNCING_LEVEL_NO:
        case PWR_KEY_DEBOUNCING_LEVEL_SHORT:
        default:
            expires = this->debounce_interval_in_normal_mode;
            break;
    }

    pr_debug("power_key_handler startDebouncing %d msec\n",expires);
    
    if (!timer_pending(&this->timer))
    	mod_timer(&this->timer, jiffies + msecs_to_jiffies(expires));

    //mutex_unlock(&this->lock);
}
static void power_key_handler_stopDebouncing(struct power_key_context * context)
{
    //BUG_ON(NULL == context);

    struct power_key_handler  *this=
        container_of(context, struct power_key_handler, context);

    //mutex_lock(&this->lock);
    pr_debug("power_key_handler stopDebouncing\n");

    if (timer_pending(&this->timer))
    	del_timer(&this->timer);

    //mutex_unlock(&this->lock);

}

static bool power_key_handler_isDebouncing(struct power_key_context * context)
{
    //BUG_ON(NULL == context);

    struct power_key_handler  *this=
        container_of(context, struct power_key_handler, context);

    return (timer_pending(&this->timer));
}
//the power key and P01 power key will be translated to the same keycode for framework,....so just send power key event ....
//public for all state to use.
static void power_key_handler_reportKey(struct power_key_context * context, bool keyPressed)
{
    struct power_key_handler  *this=
        container_of(context, struct power_key_handler, context);

    printk("power keys state=%s,%d\n", keyPressed ? "press" : "release",this->currentState);
       
    input_event(g_input_dev, EV_KEY, KEY_POWER, (int)keyPressed);
    input_sync(g_input_dev);
}
void send_fake_power_key_event(bool keyPressed)
{
    input_event(g_input_dev, EV_KEY, KEY_POWER, (int)keyPressed);
    input_sync(g_input_dev);    
}
static void power_key_handler_time_expired(unsigned long _data)
{
    unsigned long flags;

    struct power_key_handler *this = (struct power_key_handler *)_data;
    
    spin_lock_irqsave(&handler_lock, flags);

    pr_debug("power_key_handler Debouncing timeout\n");  

    this->stateList[this->currentState].onDebouncingTimeout(&this->stateList[this->currentState]);

    spin_unlock_irqrestore(&handler_lock, flags);

}
static void power_key_handler_init(struct power_key_handler  *this, int debounce_interval_in_normal_mode)
{
    unsigned long flags;

    BUG_ON(this == NULL);

    BUG_ON(this->isInited == true);

    spin_lock_irqsave(&handler_lock, flags);
 
    //[+++] This is a workaround to make sure PWR key sent
//  wake_lock_init(&pwr_key_wake_lock, WAKE_LOCK_SUSPEND, "pwr_key_temp");
//  printk(KERN_INFO "[PM]Initialize a wakelock of PWR key\r\n");
    //[---] This is a workaround to make sure PWR key sent
    
    if(false == this->isInited){

        enum PWR_KEY_STATE state_index;

        this->currentState = PWR_KEY_STATE_NOT_HANDLED_YET;

        for(state_index = PWR_KEY_STATE_NOT_HANDLED_YET; state_index < PWR_KEY_STATE_COUNT ; state_index++){

                this->stateList[state_index].setContext(&this->stateList[state_index], &this->context);
        }
        
        wake_lock_init(&this->release_wake_lock, WAKE_LOCK_SUSPEND, "power_key_press");
        
        //mutex_init(&this->timer_lock);

        setup_timer(&this->timer, this->time_expired, (unsigned long)this);

        this->debounce_interval_in_normal_mode = debounce_interval_in_normal_mode;

        this->isInited = true;
    }

    spin_unlock_irqrestore(&handler_lock, flags);    
}
static void power_key_handler_deInit(struct power_key_handler  *this)
{
    BUG_ON(this == NULL);

    BUG_ON(this->isInited == false);

    if(true == this->isInited){

        del_timer_sync(&this->timer);

        this->isInited = false;

    }

}
static void power_key_handler_handle(struct power_key_handler  *this,int key_pressed)
{
    unsigned long flags;

    spin_lock_irqsave(&handler_lock, flags);

    BUG_ON(this == NULL);

    BUG_ON(this->isInited == false);

    pr_debug("power_key_handler handle+++, now state:%d\n",this->currentState);

    if(key_pressed){

        this->stateList[this->currentState].onPressed(&this->stateList[this->currentState]);

        if(!wake_lock_active(&this->release_wake_lock)){

            wake_lock(&this->release_wake_lock);

        }
        
        pr_debug(KERN_INFO "[PM]power key release wakelock, to prevent entering suspend\r\n");

    }else{

        this->stateList[this->currentState].onReleased(&this->stateList[this->currentState]);

        if(wake_lock_active(&this->release_wake_lock)){

            wake_unlock(&this->release_wake_lock);

        }

        pr_debug(KERN_INFO "[PM]power key wakelock release\r\n");

    }

    spin_unlock_irqrestore(&handler_lock, flags);
   
}
static struct power_key_state a6x_power_key_state[PWR_KEY_STATE_COUNT] = {
    {
        .state           = PWR_KEY_STATE_NOT_HANDLED_YET,
        .name           = "not_handled_yet_state",
        .context         = NULL,
        .onPressed           = not_handled_yet_state_onPressed,
        .onReleased     = not_handled_yet_state_onReleased, 
        .onDebouncingTimeout = not_handled_yet_state_onDebouncingTimeout,
        .setContext = power_key_state_setContext,
},
{
        .state           = PWR_KEY_STATE_WAITTING_FOR_DEBUNCING_TIMEOUT,
        .name           = "waiting_for_debuncing_timeout_state",
        .context         = NULL,
        .onPressed           = waiting_for_debuncing_timeout_state_onPressed,
        .onReleased     = waiting_for_debuncing_timeout_state_onReleased, 
        .onDebouncingTimeout = waiting_for_debuncing_timeout_state_onDebouncingTimeout,
        .setContext = power_key_state_setContext,
},
 {
        .state           = PWR_KEY_STATE_WAITING_FOR_KEY_RELEASE,
        .name           = "waiting_for_key_release_state",
        .context         = NULL,
        .onPressed           = waiting_for_key_release_state_onPressed,
        .onReleased     = waiting_for_key_release_state_onReleased, 
        .onDebouncingTimeout = waiting_for_key_release_state_onDebouncingTimeout,
        .setContext = power_key_state_setContext,
},
};

static struct power_key_handler g_power_key_handler = {
    .isInited = false,
    .keyCode = KEY_POWER,
    .stateList =a6x_power_key_state,
    .currentState = PWR_KEY_STATE_NOT_HANDLED_YET,
    .context = {
        .transit = power_key_handler_transit,
        .startDebouncing = power_key_handler_startDebouncing,
        .stopDebouncing = power_key_handler_stopDebouncing,
        .isDebouncing = power_key_handler_isDebouncing,
        .reportKey =power_key_handler_reportKey,
    },
    .init = power_key_handler_init,
    .deInit = power_key_handler_deInit,
    .handle = power_key_handler_handle,
    .time_expired = power_key_handler_time_expired,
};
bool isPowerKeyHandled(bool pressed)
{
    static bool is_power_key_handling_by_fastboot = false;

    printk("power_key_isr,state=%s\n",pressed ? "press" : "release");
    
    if(pressed){//press
    
        if(is_fastboot_enable()){
    
            g_power_key_handler.handle(&g_power_key_handler, pressed);
    
            is_power_key_handling_by_fastboot = true;
    
            return true;
            
        }
    
    }else if(true == is_power_key_handling_by_fastboot){//release
    
            g_power_key_handler.handle(&g_power_key_handler, pressed);
    
            is_power_key_handling_by_fastboot = false;
    
            return true;
    }

    return false;

}
#endif //#ifdef CONFIG_FASTBOOT
//ASUS_BSP --- Peter_lu "For fastboot mode"   

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	struct gpio_keys_button *button = bdata->button;

	BUG_ON(irq != gpio_to_irq(button->gpio));

//ASUS_BSP +++ Peter_lu "For fastboot mode"
#ifdef CONFIG_FASTBOOT
    //ignore all key code when in fastboot mode except power key
    if(button->code ==KEY_POWER ){

        int state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;

        if(isPowerKeyHandled(state)){

            return IRQ_HANDLED;
        }
            
    }
#endif //#ifdef CONFIG_FASTBOOT
//ASUS_BSP --- Peter_lu "For fastboot mode"

	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
	else
		schedule_work(&bdata->work);

	return IRQ_HANDLED;
}

static int __devinit gpio_keys_setup_key(struct platform_device *pdev,
					 struct gpio_button_data *bdata,
					 struct gpio_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	unsigned long irqflags;
	int irq, error;

	setup_timer(&bdata->timer, gpio_keys_timer, (unsigned long)bdata);
	INIT_WORK(&bdata->work, gpio_keys_work_func);

	error = gpio_request(button->gpio, desc);
	if (error < 0) {
		dev_err(dev, "failed to request GPIO %d, error %d\n",
			button->gpio, error);
		goto fail2;
	}

//ASUS BSP HANS+++
// 	error = gpio_direction_input(button->gpio);
// 	if (error < 0) {
// 		dev_err(dev, "failed to configure"
// 			" direction for GPIO %d, error %d\n",
// 			button->gpio, error);
// 		goto fail3;
// 	}
//ASUS BSP HANS---

	if (button->debounce_interval) {
		error = gpio_set_debounce(button->gpio,
					  button->debounce_interval * 1000);
		/* use timer if gpiolib doesn't provide debounce */
		if (error < 0)
			bdata->timer_debounce = button->debounce_interval;
	}
//ASUS BSP TIM-2011.08.16++
//  irq = gpio_to_irq(button->gpio);
    irq = MSM_GPIO_TO_INT(button->gpio);
//ASUS BSP TIM-2011.08.16--
	if (irq < 0) {
		error = irq;
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
			button->gpio, error);
		goto fail3;
	}

	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(irq, gpio_keys_isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			irq, error);
		goto fail3;
	}

	return 0;

fail3:
	gpio_free(button->gpio);
fail2:
	return error;
}

//Bruno++ replace P01 key for debug
#ifdef  CONFIG_PROC_FS
#define P01_debug_KEY_PROC_FILE  "driver/P01_debug_key"
static struct proc_dir_entry *p01_debug_key_proc_file;

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
    
static ssize_t p01_debug_key_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{    
    char messages[256];
    struct gpio_keys_drvdata *ddata = input_get_drvdata(g_input_dev);

    memset(messages, 0, sizeof(messages));
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
        
        ddata->p01button_code.power_key = power_key_for_test;
	ddata->p01button_code.vol_up = P01_KEY_VOLUP;
	ddata->p01button_code.vol_down = P01_KEY_VOLDOWN;

        ddata->data[2].button->code = power_key_for_test;
        ddata->data[0].button->code = KEY_VOLUMEUP;
	ddata->data[1].button->code = KEY_VOLUMEDOWN;
        printk("[KeyPad] P01 Debug Mode!!!\n");
    }
    else if(strncmp(messages, "0", 1) == 0)
    {

        ddata->p01button_code.power_key = P01_KEY_POWER;
	ddata->p01button_code.vol_up = P01_KEY_VOLUP;
	ddata->p01button_code.vol_down = P01_KEY_VOLDOWN;

        ddata->data[2].button->code = KEY_POWER;
	ddata->data[0].button->code = KEY_VOLUMEUP;
	ddata->data[1].button->code = KEY_VOLUMEDOWN;
        printk("[KeyPad] P01 Normal Mode!!!\n");
    }
    else if(strncmp(messages, "2", 1) == 0){

       	ddata->p01button_code.power_key = P01_KEY_POWER;
	ddata->p01button_code.vol_up = 139;
	ddata->p01button_code.vol_down = 158;
            	
	ddata->data[2].button->code = KEY_POWER;
	ddata->data[0].button->code = 139;
    	ddata->data[1].button->code = 158;
       	printk("[KeyPad] BACK/MENU Mode!!!\n");
    }
 
    deinitKernelEnv(); 
    return len;
}

static struct file_operations p01_debug_key_proc_ops = {
    //.read = audio_debug_proc_read,
    .write = p01_debug_key_proc_write,
};

static void create_p01_debug_key_proc_file(void)
{
    struct gpio_keys_drvdata *ddata = input_get_drvdata(g_input_dev);

    printk("[KeyPad] create_p01_debug_key_proc_file\n");
    p01_debug_key_proc_file = create_proc_entry(P01_debug_KEY_PROC_FILE, 0666, NULL);
    if (p01_debug_key_proc_file) {
        p01_debug_key_proc_file->proc_fops = &p01_debug_key_proc_ops;
    }

    //init
    ddata->p01button_code.vol_up = P01_KEY_VOLUP;
    ddata->p01button_code.vol_down = P01_KEY_VOLDOWN; 
    ddata->p01button_code.power_key = P01_KEY_POWER;
}

static void remove_p01_debug_key_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    printk("[KeyPad] remove_p01_debug_key_proc_file\n");   
    remove_proc_entry(P01_debug_KEY_PROC_FILE, &proc_root);
}
#endif //#ifdef CONFIG_PROC_FS
//Bruno++ replace P01 key for debug
static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	return ddata->enable ? ddata->enable(input->dev.parent) : 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	if (ddata->disable)
		ddata->disable(input->dev.parent);
}

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

    //jack for debug slow
    INIT_WORK(&__wait_for_two_keys_work, wait_for_two_keys_work);

//Ledger ++
        wake_lock_init(&pwr_key_wake_lock, WAKE_LOCK_SUSPEND, "pwr_key_lock");
        printk(KERN_INFO "[PM]Initialize a wakelock of PWR key\r\n");
//Ledger --

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->input = input;
	ddata->n_buttons = pdata->nbuttons;
	ddata->enable = pdata->enable;
	ddata->disable = pdata->disable;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
    	g_input_dev = input;    

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];
		unsigned int type = button->type ?: EV_KEY;

		bdata->input = input;
		bdata->button = button;

		error = gpio_keys_setup_key(pdev, bdata, button);
		if (error)
			goto fail2;

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);
		
//ASUS_BSP +++ Peter_lu "For fastboot mode"
#ifdef CONFIG_FASTBOOT                     
             if(button->code == KEY_POWER){

                 g_power_key_handler.init(&g_power_key_handler, button->debounce_interval);
             
             }
#endif //#ifdef CONFIG_FASTBOOT
//ASUS_BSP --- Peter_lu "For fastboot mode"   

	}
    input_set_capability(input, EV_KEY, P01_KEY_VOLUP);
    input_set_capability(input, EV_KEY, P01_KEY_VOLDOWN);
    input_set_capability(input, EV_KEY, P01_KEY_POWER);

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		goto fail2;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail3;
	}

	/* get current state of buttons */
	for (i = 0; i < pdata->nbuttons; i++)
		gpio_keys_report_event(&ddata->data[i]);
	input_sync(input);

	device_init_wakeup(&pdev->dev, wakeup);
//ASUS Tim++
#ifdef  CONFIG_PROC_FS
    create_p01_debug_key_proc_file();
    input_set_capability(input, EV_KEY, KEY_MENU);
    input_set_capability(input, EV_KEY, KEY_BACK);
    input_set_capability(input, EV_KEY, 30);
#endif
//ASUS Tim--
	return 0;

 fail3:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
 fail2:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->buttons[i].gpio), &ddata->data[i]);
		if (ddata->data[i].timer_debounce)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		gpio_free(pdata->buttons[i].gpio);
	}

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, &ddata->data[i]);
		if (ddata->data[i].timer_debounce)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
		gpio_free(pdata->buttons[i].gpio);
//ASUS_BSP +++ Peter_lu "For fastboot mode"
#ifdef CONFIG_FASTBOOT                     
             if(pdata->buttons[i].code == KEY_POWER){

                 g_power_key_handler.deInit(&g_power_key_handler);
             
             }
#endif //#ifdef CONFIG_FASTBOOT
//ASUS_BSP --- Peter_lu "For fastboot mode"
	}

	input_unregister_device(input);

#ifdef  CONFIG_PROC_FS
    remove_p01_debug_key_proc_file();
#endif
	return 0;
}


#ifdef CONFIG_PM
//ASUS BSP HANS++
//extern int g_flag_csvoice_fe_connected;
//extern int FMStatus;
static int a68_wake;
static int phone_in_pad = 0;
//ASUS BSP HANS--

static int gpio_keys_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	if (device_may_wakeup(&pdev->dev)) {
		for (i = 0; i < pdata->nbuttons; i++) {
			struct gpio_keys_button *button = &pdata->buttons[i];
			if (button->wakeup) {

				if(phone_in_pad && (button->gpio == 26))
					continue;

				enable_irq_wake(gpio_to_irq(button->gpio));
			}
		}
	}

	//ASUS BSP HANS++
	if ((g_flag_csvoice_fe_connected || FMStatus) && !phone_in_pad){
        	enable_irq_wake(gpio_to_irq(53));
        	enable_irq_wake(gpio_to_irq(54));
        	a68_wake = 1;
	}
	//ASUS BSP HANS--


	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	//struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;


	for (i = 0; i < pdata->nbuttons; i++) {

		struct gpio_keys_button *button = &pdata->buttons[i];
		if (button->wakeup && device_may_wakeup(&pdev->dev)) {

			if(phone_in_pad && (button->gpio == 26))
				continue;

			disable_irq_wake(gpio_to_irq(button->gpio));
		}

		//gpio_keys_report_event(&ddata->data[i]); //ASUS_BSP HANS
	}
	//input_sync(ddata->input);

	//ASUS BSP HANS+++
	if (a68_wake){
        	disable_irq_wake(gpio_to_irq(53));
        	disable_irq_wake(gpio_to_irq(54));
        	a68_wake = 0;
	}
	//ASUS BSP HANS---

	return 0;
}
//Ledger --
static int gpio_keys_suspend_noirq(struct device *dev)
{
        g_bResume=0;
        printk("[PM]%s:,pm sts:%x,keylock sts:%x\r\n",__func__,g_bResume,g_bpwr_key_lock_sts);

        return 0;
}

static int gpio_keys_resume_noirq(struct device *dev)
{
        g_bResume=1;
        printk("[PM]%s:,pm sts:%x,keylock sts:%x\r\n",__func__,g_bResume,g_bpwr_key_lock_sts);
 
        return 0;
}
//Ledger ++

static const struct dev_pm_ops gpio_keys_pm_ops = {
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
//Ledger ++
        .suspend_noirq  = gpio_keys_suspend_noirq,
        .resume_noirq   = gpio_keys_resume_noirq,
//Ledger --
};
#endif

static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &gpio_keys_pm_ops,
#endif
	}
};

//ASUS BSP TIM-2011.10.04++
static int P01_keys_report(struct notifier_block *this, unsigned long event, void *ptr)
{
        struct gpio_keys_drvdata *ddata = input_get_drvdata(g_input_dev);

        switch (event) {
//ASUS BSP HANS+++
	case P01_ADD:
	{
		int i;

		phone_in_pad = 1;

		mutex_lock(&ddata->disable_lock);
		for (i = 0; i < ddata->n_buttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			gpio_keys_disable_button(bdata);
		}
		mutex_unlock(&ddata->disable_lock);

		for (i = 0; i < ddata->n_buttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			gpio_free(bdata->button->gpio);
		}

 	input_event(g_input_dev, EV_KEY, 115, 0);
 	input_event(g_input_dev, EV_KEY, 114, 0);
 	input_sync(g_input_dev);

		return NOTIFY_DONE;
	}
	case P01_REMOVE:
	{
		int i;
		int error;

		phone_in_pad = 0;

		for (i = 0; i < ddata->n_buttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			const char *desc = bdata->button->desc ? bdata->button->desc : "gpio_keys";
			error = gpio_request(bdata->button->gpio, desc);
			if(error < 0){
				printk("failed to request GPIO %d, error %d\n",
					bdata->button->gpio, error);
			}
		}

		mutex_lock(&ddata->disable_lock);
		for (i = 0; i < ddata->n_buttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			gpio_keys_enable_button(bdata);
		}
		mutex_unlock(&ddata->disable_lock);

		return NOTIFY_DONE;
	}
//ASUS BSP HANS---
        case P01_VOLUP_KEY_PRESSED:
                printk("[P01_KEY] P01 VOLUMEUP PRESS.\r\n");
//Bruno++ replace P01 key for debug
#ifdef  CONFIG_PROC_FS
                Pad_keys_report_event(ddata->p01button_code.vol_up, 1);
#else
                Pad_keys_report_event(P01_KEY_VOLUP, 1);
#endif
                return NOTIFY_DONE;
        case P01_VOLUP_KEY_RELEASED:
                printk("[P01_KEY] P01 VOLUMEUP RELEASE.\r\n");
//Bruno++ replace P01 key for debug
#ifdef  CONFIG_PROC_FS
                Pad_keys_report_event(ddata->p01button_code.vol_up, 0);
#else
                Pad_keys_report_event(P01_KEY_VOLUP, 0);
#endif
//Bruno++ replace P01 key for debug
                return NOTIFY_DONE;
        case P01_VOLDN_KEY_PRESSED:
                printk("[P01_KEY] P01 VOLUMEDOWN PRESS.\r\n");
//Bruno++ replace P01 key for debug
#ifdef  CONFIG_PROC_FS
                Pad_keys_report_event(ddata->p01button_code.vol_down, 1);
#else
                Pad_keys_report_event(P01_KEY_VOLDOWN, 1);
#endif
//Bruno++ replace P01 key for debug
                return NOTIFY_DONE;
        case P01_VOLDN_KEY_RELEASED:
                printk("[P01_KEY] P01 VOLUMEDOWN RELEASE.\r\n");
//Bruno++ replace P01 key for debug
#ifdef  CONFIG_PROC_FS
                Pad_keys_report_event(ddata->p01button_code.vol_down, 0);
#else
                Pad_keys_report_event(P01_KEY_VOLDOWN, 0);
#endif
//Bruno++ replace P01 key for debug
                return NOTIFY_DONE;
        case P01_PWR_KEY_PRESSED:
                printk("[P01_KEY] P01 POWERKEY PRESS.\r\n");

//ASUS_BSP +++ Peter_lu "suspend for fastboot mode"
#ifdef CONFIG_FASTBOOT
                if(isPowerKeyHandled(true))
			return NOTIFY_DONE;
				
#endif //#ifdef CONFIG_FASTBOOT
//ASUS_BSP ---

                Pad_keys_report_event(ddata->p01button_code.power_key, 1);
                pad_pwr_key_press_state=1;
                return NOTIFY_DONE;
        case P01_PWR_KEY_RELEASED:
            printk("[P01_KEY] P01 POWERKEY RELEASE.\r\n");
			
//ASUS_BSP +++ Peter_lu "suspend for fastboot mode"
#ifdef CONFIG_FASTBOOT
                if(isPowerKeyHandled(false))
			return NOTIFY_DONE;
				
#endif //#ifdef CONFIG_FASTBOOT
//ASUS_BSP ---

                if (pad_pwr_key_press_state == 0 && pm_new_state!= 0)  // only get release key, but no press key while current pwr state != ON
                {
                    printk("compensate PAD power key press event\n");
					Pad_keys_report_event(ddata->p01button_code.power_key, 1);
                }
                Pad_keys_report_event(ddata->p01button_code.power_key, 0);
                pad_pwr_key_press_state=0;
                return NOTIFY_DONE;
        default:
                return NOTIFY_DONE;
        }
}
 
static struct notifier_block my_hs_notifier = {
        .notifier_call = P01_keys_report,
        .priority = VIBRATOR_MP_NOTIFY,
};
//ASUS BSP TIM-2011.10.04--
static int __init gpio_keys_init(void)
{
//ASUS BSP TIM-2011.10.04++
    register_microp_notifier(&my_hs_notifier);
//ASUS BSP TIM-2011.10.04--
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
