// SPDX-License-Identifier: GPL-2.0-only
/*
 * LED Heartbeat Trigger
 *
 * Copyright (C) 2006 Atsushi Nemoto <anemo@mba.ocn.ne.jp>
 *
 * 基于Richard Purdie的ledtrig-timer.c和一些架构的CONFIG_HEARTBEAT代码。
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/panic_notifier.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/sched/loadavg.h>
#include <linux/leds.h>
#include <linux/reboot.h>
#include "../leds.h"

static int panic_heartbeats;

/* RUNNING LIGHT ADD: 定义跑马灯模式的常量 */
#define MODE_HEARTBEAT 0      // 原有心跳模式
#define MODE_RUNNING_LIGHT 1  // 跑马灯模式

struct heartbeat_trig_data {
	struct led_classdev *led_cdev;
	unsigned int phase;
	unsigned int period;
	struct timer_list timer;
	unsigned int invert;
	
	/* RUNNING LIGHT ADD: 添加跑马灯相关字段 */
	unsigned int mode;              // 模式选择：心跳模式或跑马灯模式
	unsigned int running_light_pos; // 跑马灯当前位置
	unsigned int running_light_num; // 跑马灯LED数量（默认为3：红绿蓝）
	unsigned int running_light_delay; // 跑马灯延迟时间（jiffies）
};

/* RUNNING LIGHT ADD: 设置跑马灯LED数量和延迟时间的函数 */
static void setup_running_light_params(struct heartbeat_trig_data *heartbeat_data)
{
	heartbeat_data->running_light_num = 3; // 红、绿、蓝三色LED
	heartbeat_data->running_light_delay = msecs_to_jiffies(500); // 500ms延迟
}

/* RUNNING LIGHT ADD: 跑马灯模式处理函数 */
static void led_running_light_function(struct heartbeat_trig_data *heartbeat_data)
{
	struct led_classdev *led_cdev = heartbeat_data->led_cdev;
	unsigned long brightness = LED_OFF;
	
	// 根据跑马灯位置设置亮度
	if (heartbeat_data->running_light_pos == 0) {
		brightness = led_cdev->blink_brightness; // 第一个LED亮起
	} else {
		brightness = LED_OFF; // 其他LED关闭
	}
	
	led_set_brightness_nosleep(led_cdev, brightness);
	
	// 更新跑马灯位置
	heartbeat_data->running_light_pos = (heartbeat_data->running_light_pos + 1) % 
	                                    heartbeat_data->running_light_num;
	
	// 设置定时器下次触发
	mod_timer(&heartbeat_data->timer, jiffies + heartbeat_data->running_light_delay);
}

/* RUNNING LIGHT ADD: 用于控制模式切换的属性 */
static ssize_t led_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct heartbeat_trig_data *heartbeat_data =
		led_trigger_get_drvdata(dev);

	return sprintf(buf, "%u\n", heartbeat_data->mode);
}

/* RUNNING LIGHT ADD: 用于控制模式切换的存储函数 */
static ssize_t led_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct heartbeat_trig_data *heartbeat_data =
		led_trigger_get_drvdata(dev);
	unsigned long mode;
	int ret;

	ret = kstrtoul(buf, 0, &mode);
	if (ret)
		return ret;

	// 验证模式值是否有效
	if (mode != MODE_HEARTBEAT && mode != MODE_RUNNING_LIGHT)
		return -EINVAL;

	/* ORIGINAL HEARTBEAT MODIFIED: 切换模式时重新初始化定时器 */
	heartbeat_data->mode = mode;
	
	// 如果切换到跑马灯模式，需要重置相关参数
	if (mode == MODE_RUNNING_LIGHT) {
		heartbeat_data->running_light_pos = 0;
		setup_running_light_params(heartbeat_data);
	}
	
	// 重新启动定时器以应用新模式
	if (!led_cdev->blink_brightness)
		led_cdev->blink_brightness = led_cdev->max_brightness;
	
	if (mode == MODE_HEARTBEAT) {
		led_heartbeat_function(&heartbeat_data->timer);
	} else {
		led_running_light_function(heartbeat_data);
	}
	
	return size;
}

static DEVICE_ATTR(mode, 0644, led_mode_show, led_mode_store);

static ssize_t led_invert_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct heartbeat_trig_data *heartbeat_data =
		led_trigger_get_drvdata(dev);

	return sprintf(buf, "%u\n", heartbeat_data->invert);
}

static ssize_t led_invert_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct heartbeat_trig_data *heartbeat_data =
		led_trigger_get_drvdata(dev);
	unsigned long state;
	int ret;

	ret = kstrtoul(buf, 0, &state);
	if (ret)
		return ret;

	heartbeat_data->invert = !!state;

	return size;
}

static DEVICE_ATTR(invert, 0644, led_invert_show, led_invert_store);

static struct attribute *heartbeat_trig_attrs[] = {
	&dev_attr_invert.attr,
	&dev_attr_mode.attr,  /* RUNNING LIGHT ADD: 添加模式属性 */
	NULL
};
ATTRIBUTE_GROUPS(heartbeat_trig);

static void led_heartbeat_function(struct timer_list *t)
{
	struct heartbeat_trig_data *heartbeat_data =
		from_timer(heartbeat_data, t, timer);
	struct led_classdev *led_cdev;
	unsigned long brightness = LED_OFF;
	unsigned long delay = 0;

	led_cdev = heartbeat_data->led_cdev;

	if (unlikely(panic_heartbeats)) {
		led_set_brightness_nosleep(led_cdev, LED_OFF);
		return;
	}

	if (test_and_clear_bit(LED_BLINK_BRIGHTNESS_CHANGE, &led_cdev->work_flags))
		led_cdev->blink_brightness = led_cdev->new_blink_brightness;

	/* RUNNING LIGHT ADD: 如果是跑马灯模式，则调用跑马灯处理函数 */
	if (heartbeat_data->mode == MODE_RUNNING_LIGHT) {
		led_running_light_function(heartbeat_data);
		return;
	}

	/* ORIGINAL HEARTBEAT MODIFIED: 以下是原 heartbeat 逻辑 */
	/* acts like an actual heart beat -- ie thump-thump-pause... */
	switch (heartbeat_data->phase) {
	case 0:
		/*
		 * 下面的双曲函数根据当前(1分钟)负载修改心跳周期长度。
		 * 它经过以下点：
		 * f(0)=1260, f(1)=860, f(5)=510, f(inf)->300.
		 */
		heartbeat_data->period = 300 +
			(6720 << FSHIFT) / (5 * avenrun[0] + (7 << FSHIFT));
		heartbeat_data->period =
			msecs_to_jiffies(heartbeat_data->period);
		delay = msecs_to_jiffies(70);
		heartbeat_data->phase++;
		if (!heartbeat_data->invert)
			brightness = led_cdev->blink_brightness;
		break;
	case 1:
		delay = heartbeat_data->period / 4 - msecs_to_jiffies(70);
		heartbeat_data->phase++;
		if (heartbeat_data->invert)
			brightness = led_cdev->blink_brightness;
		break;
	case 2:
		delay = msecs_to_jiffies(70);
		heartbeat_data->phase++;
		if (!heartbeat_data->invert)
			brightness = led_cdev->blink_brightness;
		break;
	default:
		delay = heartbeat_data->period - heartbeat_data->period / 4 -
			msecs_to_jiffies(70);
		heartbeat_data->phase = 0;
		if (heartbeat_data->invert)
			brightness = led_cdev->blink_brightness;
		break;
	}

	led_set_brightness_nosleep(led_cdev, brightness);
	mod_timer(&heartbeat_data->timer, jiffies + delay);
}


static int heartbeat_trig_activate(struct led_classdev *led_cdev)
{
	struct heartbeat_trig_data *heartbeat_data;

	heartbeat_data = kzalloc(sizeof(*heartbeat_data), GFP_KERNEL);
	if (!heartbeat_data)
		return -ENOMEM;

	led_set_trigger_data(led_cdev, heartbeat_data);
	heartbeat_data->led_cdev = led_cdev;

	timer_setup(&heartbeat_data->timer, led_heartbeat_function, 0);
	heartbeat_data->phase = 0;
	
	/* RUNNING LIGHT ADD: 初始化跑马灯相关参数 */
	heartbeat_data->mode = MODE_HEARTBEAT;          // 默认为心跳模式
	heartbeat_data->running_light_pos = 0;          // 跑马灯初始位置
	heartbeat_data->running_light_num = 3;          // 默认3个LED
	heartbeat_data->running_light_delay = msecs_to_jiffies(500); // 500ms延迟
	
	if (!led_cdev->blink_brightness)
		led_cdev->blink_brightness = led_cdev->max_brightness;
	led_heartbeat_function(&heartbeat_data->timer);
	set_bit(LED_BLINK_SW, &led_cdev->work_flags);

	return 0;
}

static void heartbeat_trig_deactivate(struct led_classdev *led_cdev)
{
	struct heartbeat_trig_data *heartbeat_data =
		led_get_trigger_data(led_cdev);

	timer_shutdown_sync(&heartbeat_data->timer);
	kfree(heartbeat_data);
	clear_bit(LED_BLINK_SW, &led_cdev->work_flags);
}

static struct led_trigger heartbeat_led_trigger = {
	.name     = "heartbeat",
	.activate = heartbeat_trig_activate,
	.deactivate = heartbeat_trig_deactivate,
	.groups = heartbeat_trig_groups,
};

static int heartbeat_reboot_notifier(struct notifier_block *nb,
				     unsigned long code, void *unused)
{
	led_trigger_unregister(&heartbeat_led_trigger);
	return NOTIFY_DONE;
}

static int heartbeat_panic_notifier(struct notifier_block *nb,
				     unsigned long code, void *unused)
{
	panic_heartbeats = 1;
	return NOTIFY_DONE;
}

static struct notifier_block heartbeat_reboot_nb = {
	.notifier_call = heartbeat_reboot_notifier,
};

static struct notifier_block heartbeat_panic_nb = {
	.notifier_call = heartbeat_panic_notifier,
};

static int __init heartbeat_trig_init(void)
{
	int rc = led_trigger_register(&heartbeat_led_trigger);

	if (!rc) {
		atomic_notifier_chain_register(&panic_notifier_list,
					       &heartbeat_panic_nb);
		register_reboot_notifier(&heartbeat_reboot_nb);
	}
	return rc;
}

static void __exit heartbeat_trig_exit(void)
{
	unregister_reboot_notifier(&heartbeat_reboot_nb);
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &heartbeat_panic_nb);
	led_trigger_unregister(&heartbeat_led_trigger);
}

module_init(heartbeat_trig_init);
module_exit(heartbeat_trig_exit);

MODULE_AUTHOR("Atsushi Nemoto <anemo@mba.ocn.ne.jp>");
MODULE_DESCRIPTION("Heartbeat LED trigger");
MODULE_LICENSE("GPL v2");