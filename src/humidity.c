/**
 * collectd - src/humidity.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; only version 2.1 of the License is
 * applicable.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Authors:
 *   Tomas Menzl
 **/

#include "collectd.h"
#include "common.h"
#include "utils_cache.h"
#include "plugin.h"

#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <math.h>

/* ------------ BMP085 defines ------------ */
/* I2C address of the HIH6131 sensor - assuming default address */
#define HIH6131_I2C_ADDRESS          0x27

#define HIH6131_CONV_TIME_US         40000

/* ------------------------------------------ */
static const char *config_keys[] =
{
    "Device",
    "Address"
};

static int    config_keys_num     = STATIC_ARRAY_SIZE(config_keys);
                                  
static char * config_device       = NULL;                /**< I2C bus device */
static int    config_address      = HIH6131_I2C_ADDRESS; /**< sensor I2C address */

static _Bool  configured          = 0;     /**< the whole plugin config status */
                                  
static int    i2c_bus_fd          = -1;    /**< I2C bus device FD */
                                  

/* ------------------------ HIH6131 access ------------------------ */

/** 
 * Read sensor measurements
 *
 * @param humidity    measured humidity in %
 * @param temperature measured temperature in C
 *
 * @return Zero when successful
 */
static int HIH6131_read(double * humidity, double * temperature)
{
    char errbuf[1024];
    unsigned char data[4];
    int res;
    unsigned status;
    int rawHumidity;
    int rawTemperature;
    struct i2c_msg msgs[1] = {
        {
            .addr = HIH6131_I2C_ADDRESS,
            .flags = I2C_M_RD,
//          .flags = I2C_M_RD | I2C_M_NOSTART,
            .len = 4,
            .buf = (char*) data,
        }
    };
    
    struct i2c_rdwr_ioctl_data packets = {
        .msgs = msgs,
        .nmsgs = 1,
    };

    memset(data, 0, 4);

    // 0 or 1 ???
    res=write(i2c_bus_fd, data, 0);
    if(res < 0)
    {
        ERROR ("humidity: HIH6131_read - cannot write to sensor: %s",
               sstrerror (errno, errbuf, sizeof (errbuf)));
        return 1;
    }
    
    usleep(HIH6131_CONV_TIME_US);
    
    res = ioctl(i2c_bus_fd, I2C_RDWR, &packets);
    if(res < 0)
    {
        ERROR ("humidity: HIH6131_read - sensor read error: %s",
               sstrerror (errno, errbuf, sizeof (errbuf)));
        return 1;
    }

    status = (data[0] & 0xc0) >> 6;
    rawHumidity = (data[0] & 0x3f)<<8 | data[1];
    rawTemperature = data[2]<<6 | (data[3] & 0xfc);

    DEBUG ("humidity: HIH6131_read: status = %02x, raw humidity=%d, raw temperature=%d",
           status,
           rawHumidity,
           rawTemperature);

    switch(status)
    {
    case 0x00:
        DEBUG ("humidity: HIH6131_read - status OK");
        break;
        
    case 0x01:
        DEBUG ("humidity: HIH6131_read - status \"stale data\"");
        break;
        
    case 0x02:
        DEBUG ("humidity: HIH6131_read - status \"command mode\"");
        break;
        
    default:
        DEBUG ("humidity: HIH6131_read - status \"other\"");
        break;
    }

    *humidity = (double) rawHumidity / 163.82;      // / (2^14 -2) * 100 = / 16382 * 100 = 163.8
    *temperature = (double) rawTemperature * 165.0 / 16382.0 - 40.0;

    DEBUG ("humidity: HIH6131_read - humidity: %lf %%, temperature: %lf C",
           *humidity,
           *temperature);
    
    return 0;
}


/* ------------------------ main plugin callbacks ------------------------ */

/** 
 * Main plugin configuration callback (using simple config)
 * 
 * @param key   configuration key we should process
 * @param value configuration value we should process
 * 
 * @return Zero when successful.
 */
static int collectd_humidity_config (const char *key, const char *value)
{
    DEBUG("humidity: collectd_humidity_config");

    if (strcasecmp (key, "Device") == 0)
    {
        sfree (config_device);
        config_device = strdup (value);
    }
    else if (strcasecmp (key, "Address") == 0)
    {
        long address_tmp = strtol(value, NULL, 0);
        if (address_tmp < 0 || address_tmp > 0x7f)
        {
            WARNING ("humidity: collectd_humidity_config: invalid address: %ld." \
                     " Allowed values are 0x00 to 0x7f.",
                     address_tmp);
            return 1;
        }
        config_address = (int) address_tmp;
    }
    else 
    {
        return -1;
    }

    return 0;
}


/** 
 * Shutdown callback.
 * 
 * Close I2C and delete all the buffers.
 * 
 * @return Zero when successful (at the moment the only possible outcome)
 */
static int collectd_humidity_shutdown(void)
{
    DEBUG ("humidity: collectd_humidity_shutdown");

    if (i2c_bus_fd > 0)
    {
        close (i2c_bus_fd);
        i2c_bus_fd = -1;
        sfree (config_device);
    }

    return 0;
}


/** 
 * Plugin read callback for HIH6131.
 * 
 *  Dispatching will create values:
 *  - <hostname>/humidity-hih6131/humidity
 *  - <hostname>/humidity-hih6131/temperature
 *
 * @return Zero when successful.
 */
static int HIH6131_collectd_humidity_read (void)
{
    int result = 0;

    double humidity        = 0.0;
    double temperature     = 0.0;

    value_list_t vl = VALUE_LIST_INIT;
    value_t      values[1];
    
    DEBUG("humidity: HIH6131_collectd_humidity_read");

    if (!configured)
    {
        return -1;
    }

    result = HIH6131_read(&humidity, &temperature);
    if(result)
        return result;

    sstrncpy (vl.host, hostname_g, sizeof (vl.host));
    sstrncpy (vl.plugin, "humidity", sizeof (vl.plugin));
    sstrncpy (vl.plugin_instance, "hih6131", sizeof (vl.plugin_instance));

    vl.values_len = 1;
    vl.values = values;

    /* dispatch relative humidity */
    sstrncpy (vl.type, "humidity", sizeof (vl.type));
    sstrncpy (vl.type_instance, "", sizeof (vl.type_instance));
    values[0].gauge = humidity;
    plugin_dispatch_values (&vl);

    /* dispatch sensor temperature */
    sstrncpy (vl.type, "temperature", sizeof (vl.type));
    sstrncpy (vl.type_instance, "", sizeof (vl.type_instance));
    values[0].gauge = temperature;
    plugin_dispatch_values (&vl);

    return 0;
}


/** 
 * Initialization callback
 * 
 * Check config, initialize I2C bus access, conversion coefficients and averaging
 * ring buffers
 * 
 * @return Zero when successful.
 */
static int collectd_humidity_init (void)
{
    char errbuf[1024];
    unsigned long funcs; /**< I2C adapter functionality */

    DEBUG ("humidity: collectd_humidity_init");

    if (config_device == NULL)
    {
        ERROR("humidity: collectd_humidity_init I2C bus device not configured");
        return -1;
    }

    i2c_bus_fd = open(config_device, O_RDWR);
    if (i2c_bus_fd < 0)
    {
        ERROR ("humidity: collectd_humidity_init problem opening I2C bus device \"%s\": %s (is loaded mod i2c-dev?)",
               config_device,
               sstrerror (errno, errbuf, sizeof (errbuf)));
        return -1;
    }

    if(ioctl(i2c_bus_fd, I2C_FUNCS, &funcs) < 0)
    {
        ERROR ("humidity: collectd_humidity_init problem checking I2C adapter functionality \"%s\": %s (is loaded mod i2c-dev?)",
               config_device,
               sstrerror (errno, errbuf, sizeof (errbuf)));
        return -1;
    }

    if(!(funcs & I2C_FUNC_I2C))
    {
        ERROR ("humidity: collectd_humidity_init I2C adapter \"%s\" does not support I2C_FUNC_I2C functionality.",
               config_device);
        return -1;
    }

    if (ioctl(i2c_bus_fd, I2C_SLAVE_FORCE, config_address) < 0)
    {
        ERROR("humidity: collectd_humidity_init problem setting i2c slave address to 0x%02X: %s",
              config_address,
              sstrerror (errno, errbuf, sizeof (errbuf)));
        return -1;
    }

    plugin_register_read ("humidity", HIH6131_collectd_humidity_read );

    configured = 1;
    return 0;
}

/* ------------------------ plugin register / entry point ------------------------ */

/** 
 * Plugin "entry" - register all callback.
 * 
 */
void module_register (void)
{
    plugin_register_config ("humidity", 
                            collectd_humidity_config, 
                            config_keys, 
                            config_keys_num);
    plugin_register_init ("humidity", collectd_humidity_init);
    plugin_register_shutdown ("humidity", collectd_humidity_shutdown);
}
