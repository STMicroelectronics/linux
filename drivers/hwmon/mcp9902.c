// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * mcp9902.c - Part of lm_sensors, Linux kernel modules for hardware
 *             monitoring
 *
 * Based on the lm90 driver. The MCP9902 is a sensor chip made by Microchip.
 * It reports up to two temperatures (its own plus up to
 * one external one). Complete datasheet can be
 * obtained from Microchips's website at:
 *   http://ww1.microchip.com/downloads/en/DeviceDoc/20005382C.pdf
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>

static const unsigned short normal_i2c[] = {
	0x1C, 0x4C,  I2C_CLIENT_END };

/*
 * The MCP9902 registers
 */

#define MCP9902_REG_R_CHIP_ID				0xFD
#define MCP9902_REG_R_MAN_ID				0xFE
#define MCP9902_REG_R_REV_ID				0xFF
#define MCP9902_REG_R_CONFIG				0x03
#define MCP9902_REG_W_CONFIG				0x09
#define MCP9902_REG_R_CONVRATE				0x04
#define MCP9902_REG_W_CONVRATE				0x0A
#define MCP9902_REG_R_STATUS				0x02
#define MCP9902_REG_R_LOCAL_TEMP			0x00
#define MCP9902_REG_R_LOCAL_TEMP_FRACTION	0x29
#define MCP9902_REG_R_REMOTE_TEMP			0x01
#define MCP9902_REG_R_REMOTE_TEMP_FRACTION	0x10
#define MCP9902_REG_R_LOCAL_HIGH			0x05
#define MCP9902_REG_W_LOCAL_HIGH			0x0B
#define MCP9902_REG_R_LOCAL_LOW				0x06
#define MCP9902_REG_W_LOCAL_LOW				0x0C
#define MCP9902_REG_R_REMOTE_HIGH			0x07
#define MCP9902_REG_W_REMOTE_HIGH			0x0D
#define MCP9902_REG_R_REMOTE_LOW			0x08
#define MCP9902_REG_W_REMOTE_LOW			0x0E
#define MCP9902_REG_R_REMOTE_CRIT			0x19
#define MCP9902_REG_W_REMOTE_CRIT			0x19
#define MCP9902_REG_R_LOCAL_CRIT			0x20
#define MCP9902_REG_W_LOCAL_CRIT			0x20
#define MCP9902_REG_R_TCRIT_HYST			0x21
#define MCP9902_REG_W_TCRIT_HYST			0x21

/*
 * Conversions
 */

static int temp_from_reg(int val, int fraction)
{
	/* milli-degree C */
	return (((val - 64) * 1000) + (((fraction & 0xE0) >> 5) * 125));
}

static int temp_to_reg(int val)
{
	return (val + 64) / 1000;
}

enum temp_index {
	t_input1 = 0,
	t_input1_fraction,
	t_input2,
	t_input2_fraction,
	t_low1,
	t_high1,
	t_crit1,
	t_low2,
	t_high2,
	t_crit2,
	t_hyst,
	t_num_regs
};

/*
 * Client data (each client gets its own)
 */

struct mcp9902_data {
	struct i2c_client *client;
	struct mutex update_lock;
	char valid; /* zero until following fields are valid */
	unsigned long last_updated; /* in jiffies */

	/* registers values */
	u8 temp[t_num_regs];	/* index with enum temp_index */
	u8 alarms;
};

static const u8 regs_read[t_num_regs] = {
	[t_input1] = MCP9902_REG_R_LOCAL_TEMP,
	[t_input1_fraction] = MCP9902_REG_R_LOCAL_TEMP_FRACTION,
	[t_input2] = MCP9902_REG_R_REMOTE_TEMP,
	[t_input2_fraction] = MCP9902_REG_R_REMOTE_TEMP_FRACTION,
	[t_low1] = MCP9902_REG_R_LOCAL_LOW,
	[t_high1] = MCP9902_REG_R_LOCAL_HIGH,
	[t_crit1] = MCP9902_REG_R_LOCAL_CRIT,
	[t_low2] = MCP9902_REG_R_REMOTE_LOW,
	[t_high2] = MCP9902_REG_R_REMOTE_HIGH,
	[t_crit2] = MCP9902_REG_R_REMOTE_CRIT,
	[t_hyst] = MCP9902_REG_R_TCRIT_HYST,
};

static const u8 regs_write[t_num_regs] = {
	[t_low1] = MCP9902_REG_W_LOCAL_LOW,
	[t_high1] = MCP9902_REG_W_LOCAL_HIGH,
	[t_crit1] = MCP9902_REG_W_LOCAL_CRIT,
	[t_low2] = MCP9902_REG_W_REMOTE_LOW,
	[t_high2] = MCP9902_REG_W_REMOTE_HIGH,
	[t_crit2] = MCP9902_REG_W_REMOTE_CRIT,
	[t_hyst] = MCP9902_REG_W_TCRIT_HYST,
};

static struct mcp9902_data *mcp9902_update_device(struct device *dev)
{
	struct mcp9902_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int i;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + HZ * 2) || !data->valid) {
		dev_dbg(&client->dev, "Updating mcp9902 data.\n");
		for (i = 0; i < t_num_regs; i++)
			data->temp[i] = i2c_smbus_read_byte_data(client,
					regs_read[i]);
		data->alarms = i2c_smbus_read_byte_data(client,
					MCP9902_REG_R_STATUS);

		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);

	return data;
}

/*
 * Sysfs stuff
 */

static ssize_t temp_show(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct mcp9902_data *data = mcp9902_update_device(dev);
	u8 fraction_reg;
	
	switch(attr->index)
	{
		case t_input1:
			fraction_reg = data->temp[t_input1_fraction];
			break;
		case t_input2:
			fraction_reg = data->temp[t_input2_fraction];
			break;
		default:
			fraction_reg = 0;
			break;
	}
	return sprintf(buf, "%d\n", temp_from_reg(data->temp[attr->index], fraction_reg));
}

static ssize_t temp_store(struct device *dev,
			  struct device_attribute *devattr, const char *buf,
			  size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct mcp9902_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	long val;
	int err = kstrtol(buf, 10, &val);
	if (err)
		return err;

	mutex_lock(&data->update_lock);
	data->temp[attr->index] = temp_to_reg(val);
	i2c_smbus_write_byte_data(client, regs_write[attr->index],
				  data->temp[attr->index]);
	mutex_unlock(&data->update_lock);
	return count;
}

static ssize_t alarms_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct mcp9902_data *data = mcp9902_update_device(dev);
	return sprintf(buf, "%d\n", data->alarms);
}

static ssize_t alarm_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int bitnr = to_sensor_dev_attr(attr)->index;
	struct mcp9902_data *data = mcp9902_update_device(dev);
	return sprintf(buf, "%d\n", (data->alarms >> bitnr) & 1);
}

static SENSOR_DEVICE_ATTR_RO(temp1_input, temp, t_input1);
static SENSOR_DEVICE_ATTR_RW(temp1_min, temp, t_low1);
static SENSOR_DEVICE_ATTR_RW(temp1_max, temp, t_high1);
static SENSOR_DEVICE_ATTR_RW(temp1_crit, temp, t_crit1);
static SENSOR_DEVICE_ATTR_RO(temp2_input, temp, t_input2);
static SENSOR_DEVICE_ATTR_RW(temp2_min, temp, t_low2);
static SENSOR_DEVICE_ATTR_RW(temp2_max, temp, t_high2);
static SENSOR_DEVICE_ATTR_RW(temp2_crit, temp, t_crit2);
static SENSOR_DEVICE_ATTR_RW(temp_crit_hyst, temp, t_hyst);

static DEVICE_ATTR_RO(alarms);
static SENSOR_DEVICE_ATTR_RO(temp1_crit_alarm, alarm, 0);
static SENSOR_DEVICE_ATTR_RO(temp2_crit_alarm, alarm, 1);
static SENSOR_DEVICE_ATTR_RO(temp2_fault, alarm, 2);
static SENSOR_DEVICE_ATTR_RO(temp2_min_alarm, alarm, 3);
static SENSOR_DEVICE_ATTR_RO(temp2_max_alarm, alarm, 4);
static SENSOR_DEVICE_ATTR_RO(temp1_min_alarm, alarm, 5);
static SENSOR_DEVICE_ATTR_RO(temp1_max_alarm, alarm, 6);

static struct attribute *mcp9902_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp1_min.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_crit.dev_attr.attr,
	&sensor_dev_attr_temp2_min.dev_attr.attr,
	&sensor_dev_attr_temp2_max.dev_attr.attr,
	&sensor_dev_attr_temp2_crit.dev_attr.attr,
	&sensor_dev_attr_temp_crit_hyst.dev_attr.attr,

	&dev_attr_alarms.attr,
	&sensor_dev_attr_temp1_crit_alarm.dev_attr.attr,
	&sensor_dev_attr_temp2_crit_alarm.dev_attr.attr,
	&sensor_dev_attr_temp2_fault.dev_attr.attr,
	&sensor_dev_attr_temp2_min_alarm.dev_attr.attr,
	&sensor_dev_attr_temp2_max_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_min_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_max_alarm.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(mcp9902);

/* Return 0 if detection is successful, -ENODEV otherwise */
static int mcp9902_detect(struct i2c_client *client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	u8 man_id, chip_id;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	/* identification */
	man_id = i2c_smbus_read_byte_data(client, MCP9902_REG_R_MAN_ID);
	chip_id = i2c_smbus_read_byte_data(client, MCP9902_REG_R_CHIP_ID);
	if (man_id != 0x5D || chip_id != 0x04) {
		dev_info(&adapter->dev,
			 "Unsupported chip (man_id=0x%02X, chip_id=0x%02X).\n",
			 man_id, chip_id);
		return -ENODEV;
	}

	strlcpy(info->type, "mcp9902", I2C_NAME_SIZE);

	return 0;
}

static void mcp9902_init_client(struct i2c_client *client)
{

	/*
	 * Start the conversions.
	 */
	i2c_smbus_write_byte_data(client, MCP9902_REG_W_CONVRATE, 5);	/* 2 Hz */
	i2c_smbus_write_byte_data(client, MCP9902_REG_W_CONFIG, 0x9F);	/* run - extended temp */
}

static int mcp9902_probe(struct i2c_client *new_client,
			 const struct i2c_device_id *id)
{
	struct mcp9902_data *data;
	struct device *hwmon_dev;

	data = devm_kzalloc(&new_client->dev, sizeof(struct mcp9902_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = new_client;
	mutex_init(&data->update_lock);

	/* Initialize the MCP9902 chip */
	mcp9902_init_client(new_client);

	hwmon_dev = devm_hwmon_device_register_with_groups(&new_client->dev,
							   new_client->name,
							   data,
							   mcp9902_groups);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id mcp9902_id[] = {
	{ "mcp9902", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mcp9902_id);

#ifdef CONFIG_OF
static const struct of_device_id mcp9902_of_match[] = {
	{ .compatible = "microchip,mcp9902", },
	{},
};

MODULE_DEVICE_TABLE(of, mcp9902_of_match);
#endif

static struct i2c_driver mcp9902_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "mcp9902",
		.of_match_table = of_match_ptr(mcp9902_of_match),
	},
	.probe		= mcp9902_probe,
	.id_table	= mcp9902_id,
	.detect		= mcp9902_detect,
	.address_list	= normal_i2c,
};

module_i2c_driver(mcp9902_driver);

MODULE_AUTHOR("Mark Carlin <mcarlin@datumsystems.com>");
MODULE_DESCRIPTION("MCP9902 temperature sensor driver");
MODULE_LICENSE("GPL");
