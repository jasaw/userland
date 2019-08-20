#define _GNU_SOURCE

#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/dir.h>
#include <poll.h>
#include <endian.h>
#include <time.h>

#include "iio_utils.h"
#include "ahrs.h"
#include "calib.h"
#include "sample_dump.h"

#include "iio_sensors.h"


struct iio_trigger_info
{
    char *trigger_name;
    int trig_num;
    char *trig_dir_name;
    int assigned;
};

struct iio_sensor_info
{
    char *sensor_name;
    int sampling_frequency;
    int iio_sample_interval_ms;
    char channel_index_to_axis_map[3];
    int invert_axes[3]; // order follows channel_index_to_axis_map
    //const char *sample_out_file;
    struct calibration_data *calibration;
    struct iio_channel_info *channels;
    int num_channels;
    int dev_num;
    int scan_size;
    int dev_fd;
    int read_size;
    char *dev_dir_name;
    char *buf_dir_name;
    char *buffer_access;
    char *data;
    struct iio_trigger_info *trigger;
};


#define BUFFER_LENGTH                   512


static char *barometric_path = "/sys/bus/i2c/drivers/bmp085/1-0077/pressure0_input";
static char *temperature_path = "/sys/bus/i2c/drivers/bmp085/1-0077/temp0_input";
static double magnetic_declination_mrad = 0;
static struct calibration_data accel_calibration =
{
    .x_offset = 0.263798,
    .y_offset = -0.053282,
    .z_offset = -0.103909,
    .x_scale  = 0.992941,
    .y_scale  = 0.995991,
    .z_scale  = 0.993166,
};
static struct calibration_data magn_calibration =
{
// Magnetometer calibration with quad
    .x_offset = 0.055924,
    .y_offset = 0.129528,
    .z_offset = 0.346131,
    .x_scale  = 1.554669,
    .y_scale  = 1.376062,
    .z_scale  = 1.784469,
};
static struct calibration_data gyro_calibration =
{
    .x_scale  = 1.0,
    .y_scale  = 1.0,
    .z_scale  = 1.0,
};
static struct iio_sensor_info accel =
{
    .sensor_name = "lsm303dlhc_accel",
    .sampling_frequency = 25,
    .iio_sample_interval_ms = 40,
    .channel_index_to_axis_map = {'x', 'y', 'z'},
    .calibration = &accel_calibration,
    .invert_axes = {0, 1, 1},
    .dev_fd = -1,
};
static struct iio_sensor_info magn  =
{
    .sensor_name = "lsm303dlhc_magn",
    .sampling_frequency = 30,
    .iio_sample_interval_ms = 33,
    .channel_index_to_axis_map = {'x', 'z', 'y'},
    .calibration = &magn_calibration,
    .invert_axes = {0, 1, 1},
    .dev_fd = -1,
};
static struct iio_sensor_info gyro  =
{
    .sensor_name = "l3gd20",
    .sampling_frequency = 95,
    .iio_sample_interval_ms = 11,
    .channel_index_to_axis_map = {'x', 'y', 'z'},
    .calibration = &gyro_calibration,
    .invert_axes = {0, 1, 1},
    .dev_fd = -1,
};
static struct iio_trigger_info timer[] =
{
    { .trigger_name = "hrtimertrig0" },
    { .trigger_name = "hrtimertrig1" },
    { .trigger_name = "hrtimertrig2" },
};
static const char *calibration_data_file = "/etc/default/rpi-stereo-cam-stream-calib.conf";
static const char *dump_file = NULL;
static FILE *dump_fp = NULL;
static struct sensor_axis_t accel_axis;
static struct sensor_axis_t magn_axis;
static struct sensor_axis_t gyro_axis;


#define min(a,b) ( (a < b) ? a : b )
#define max(a,b) ( (a > b) ? a : b )

static int stop_iio_device(struct iio_sensor_info *info);


static int enable_xyz_scan_channels(const char *device_dir)
{
    DIR *dp = NULL;
    const struct dirent *ent = NULL;
    char *scan_el_dir = NULL;
    int ret = 0;

    if (asprintf(&scan_el_dir, FORMAT_SCAN_ELEMENTS_DIR, device_dir) < 0)
    {
        ret = -ENOMEM;
        goto error_ret;
    }
    dp = opendir(scan_el_dir);
    if (dp == NULL)
    {
        ret = -errno;
        goto error_ret;
    }
    while (ent = readdir(dp), ent != NULL)
    {
        const char *d_name = ent->d_name + strlen(ent->d_name) - strlen("_x_en");
        if ((d_name >= ent->d_name) &&
            ((strcmp(d_name, "_x_en") == 0) ||
             (strcmp(d_name, "_y_en") == 0) ||
             (strcmp(d_name, "_z_en") == 0)))
        {
            ret = write_sysfs_int(ent->d_name, scan_el_dir, 1);
            if (ret < 0)
                goto error_ret;
        }
    }

error_ret:
    if (dp)
	closedir(dp);
    free(scan_el_dir);
    return ret;
}


static void apply_calibration_data(struct iio_channel_info *channels,
                                   int num_channels,
                                   struct calibration_data *calibration,
                                   char *channel_index_to_axis_map,
                                   int *invert_axis)
{
    if (num_channels == 3)
    {
        int i;
        for (i = 0; i < num_channels; i++)
        {
            // Note: Do not use channels[i].name as it is wrong !!!
            switch (channel_index_to_axis_map[i])
            {
                case 'x':
                    channels[i].offset += calibration->x_offset / channels[i].scale;
                    channels[i].scale  *= calibration->x_scale;
                    break;
                case 'y':
                    channels[i].offset += calibration->y_offset / channels[i].scale;
                    channels[i].scale  *= calibration->y_scale;
                    break;
                case 'z':
                    channels[i].offset += calibration->z_offset / channels[i].scale;
                    channels[i].scale  *= calibration->z_scale;
                    break;
                default: return;
            }
            if (invert_axis[i])
                channels[i].scale *= -1;
        }
    }
}


static int setup_iio_device(struct iio_sensor_info *info)
{
    int ret;

    info->dev_num = find_type_by_name(info->sensor_name, "iio:device");
    if (info->dev_num < 0)
    {
        fprintf(stderr, "Failed to find device %s\n", info->sensor_name);
        return info->dev_num;
    }
    fprintf(stderr, "%s IIO device number: %d\n", info->sensor_name, info->dev_num);
    ret = asprintf(&info->dev_dir_name, "%siio:device%d", iio_dir, info->dev_num);
    if (ret < 0)
        return -ENOMEM;
    ret = asprintf(&info->buf_dir_name, "%s/buffer", info->dev_dir_name);
    if (ret < 0)
        return -ENOMEM;
    ret = asprintf(&info->buffer_access, "/dev/iio:device%d", info->dev_num);
    if (ret < 0)
        return -ENOMEM;

    stop_iio_device(info);

    ret = enable_xyz_scan_channels(info->dev_dir_name);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to enable %s scan elements\n", info->sensor_name);
        return ret;
    }
    ret = write_sysfs_int("sampling_frequency", info->dev_dir_name, info->sampling_frequency);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to set %s sampling frequency\n", info->sensor_name);
        return ret;
    }
    ret = build_channel_array(info->dev_dir_name, &info->channels, &info->num_channels);
    if (ret)
    {
        fprintf(stderr, "Problem reading %s scan element information\n", info->sensor_name);
        return ret;
    }
    apply_calibration_data(info->channels,
                           info->num_channels,
                           info->calibration,
                           info->channel_index_to_axis_map,
                           info->invert_axes);

    info->scan_size = size_from_channelarray(info->channels, info->num_channels);
    info->data = malloc(info->scan_size * BUFFER_LENGTH);
    if (!info->data)
        return -ENOMEM;

    // Setup ring buffer parameters
    ret = write_sysfs_int("length", info->buf_dir_name, BUFFER_LENGTH);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to set %s buffer length\n", info->sensor_name);
        return ret;
    }

    return 0;
}


static int create_trigger(int trigger_id)
{
    char *add_trigger_dir = NULL;
    int ret;

    ret = asprintf(&add_trigger_dir, "%siio_hrtimer_trigger", iio_dir);
    if (ret < 0)
        return -ENOMEM;

    ret = write_sysfs_int("add_trigger", add_trigger_dir, trigger_id);
    free(add_trigger_dir);
    if (ret < 0)
        return ret;
    return 0;
}


static int setup_iio_trigger(struct iio_trigger_info *trigger)
{
    int ret;

    trigger->trig_num = find_type_by_name(trigger->trigger_name, "trigger");
    if (trigger->trig_num < 0)
    {
        fprintf(stderr, "Failed to find trigger %s\n", trigger->trigger_name);
        return trigger->trig_num;
    }
    fprintf(stderr, "%s IIO trigger number: %d\n", trigger->trigger_name, trigger->trig_num);
    ret = asprintf(&trigger->trig_dir_name, "%strigger%d", iio_dir, trigger->trig_num);
    if (ret < 0)
        return -ENOMEM;
    return 0;
}


static int assign_trigger(struct iio_sensor_info *sensor, struct iio_trigger_info *trigger)
{
    int ret;

    if (trigger->assigned)
        return -1;

    fprintf(stderr, "%s - %s\n", sensor->dev_dir_name, trigger->trigger_name);
    ret = write_sysfs_string_and_verify("trigger/current_trigger",
                                        sensor->dev_dir_name,
                                        trigger->trigger_name);
    if (ret < 0)
    {
        fprintf(stderr, "Failed to assign trigger %s to device %s\n", sensor->sensor_name, trigger->trigger_name);
        return ret;
    }
    ret = write_sysfs_int("delay_ns", trigger->trig_dir_name, sensor->iio_sample_interval_ms * 1000000);
    if (ret < 0)
        return ret;
    trigger->assigned = 1;
    sensor->trigger = trigger;
    return 0;
}


static int start_iio_device(struct iio_sensor_info *info)
{
    int ret;

    // Enable the buffer
    ret = write_sysfs_int("enable", info->buf_dir_name, 1);
    if (ret < 0)
        return ret;

    // Attempt to open non blocking the access dev
    info->dev_fd = open(info->buffer_access, O_RDONLY | O_NONBLOCK);
    if (info->dev_fd == -1)
    {
        ret = -errno;
        fprintf(stderr, "Failed to open %s\n", info->buffer_access);
        return ret;
    }

    return 0;
}


static int stop_iio_device(struct iio_sensor_info *info)
{
    if (info->buf_dir_name)
        return write_sysfs_int("enable", info->buf_dir_name, 0);
    return -1;
}


static int disconnect_trigger(struct iio_sensor_info *info)
{
    if (info->trigger == NULL)
        return -1;
    info->trigger->assigned = 0;
    info->trigger = NULL;
    if (info->dev_dir_name)
        return write_sysfs_string("trigger/current_trigger", info->dev_dir_name, "NULL");
    return -1;
}


static void clean_up_iio_device(struct iio_sensor_info *info)
{
    close(info->dev_fd);
    info->dev_fd = -1;
    free(info->channels);
    info->channels = NULL;
    free(info->data);
    info->data = NULL;
    free(info->dev_dir_name);
    info->dev_dir_name = NULL;
    free(info->buf_dir_name);
    info->buf_dir_name = NULL;
    free(info->buffer_access);
    info->buffer_access = NULL;
}


static void clean_up_iio_trigger(struct iio_trigger_info *trigger)
{
    free(trigger->trig_dir_name);
    trigger->trig_dir_name = NULL;
}


static int read_sensor_value(char *path)
{
    int fd;
    int value = -1;
    char tmp_buf[16];

    fd = open(path, O_RDONLY | O_NONBLOCK);
    if ((fd != -1) &&
        (read(fd, tmp_buf, sizeof(tmp_buf)) > 0))
        value = atoi(tmp_buf);
    if (fd != -1)
        close(fd);
    return value;
}


//------------------------------------------------------------------------------

static double double2byte(uint16_t input, struct iio_channel_info *info)
{
    /* First swap if incorrect endian */
    if (info->be)
        input = be16toh(input);
    else
        input = le16toh(input);

    /*
     * Shift before conversion to avoid sign extension
     * of left aligned data
     */
    input >>= info->shift;
    input &= info->mask;
    if (info->is_signed) {
        int16_t val = (int16_t)(input << (16 - info->bits_used)) >>
                      (16 - info->bits_used);
        return ((double)val + info->offset) * info->scale;
    } else {
        return ((double)input + info->offset) * info->scale;
    }
}

static double double4byte(uint32_t input, struct iio_channel_info *info)
{
    /* First swap if incorrect endian */
    if (info->be)
        input = be32toh(input);
    else
        input = le32toh(input);

    /*
     * Shift before conversion to avoid sign extension
     * of left aligned data
     */
    input >>= info->shift;
    input &= info->mask;
    if (info->is_signed) {
        int32_t val = (int32_t)(input << (32 - info->bits_used)) >>
                      (32 - info->bits_used);
        return ((double)val + info->offset) * info->scale;
    } else {
        return ((double)input + info->offset) * info->scale;
    }
}

static double double8byte(uint64_t input, struct iio_channel_info *info)
{
    /* First swap if incorrect endian */
    if (info->be)
        input = be64toh(input);
    else
        input = le64toh(input);

    /*
     * Shift before conversion to avoid sign extension
     * of left aligned data
     */
    input >>= info->shift;
    input &= info->mask;
    if (info->is_signed) {
        int64_t val = (int64_t)(input << (64 - info->bits_used)) >>
                      (64 - info->bits_used);
        /* special case for timestamp */
        if (info->scale == 1.0f && info->offset == 0.0f)
            return (double)val;
        else
            return ((double)val + info->offset) * info->scale;
    } else {
        return ((double)input + info->offset) * info->scale;
    }
}


static void populate_sensor_axis(char *data,
		                 struct iio_channel_info *channels,
		                 int num_channels,
                                 char *channel_index_to_axis_map,
                                 struct sensor_axis_t *axis)
{
    int k;
    if (num_channels != 3)
        return;
    for (k = 0; k < num_channels; k++)
    {
        double *a;
        switch (channel_index_to_axis_map[k])
        {
            case 'x': a = &axis->x; break;
            case 'y': a = &axis->y; break;
            case 'z': a = &axis->z; break;
            default: a = NULL;
        }
        if (a == NULL)
            break;
        switch (channels[k].bytes)
        {
            // only a few cases implemented so far
            case 2:
                *a = double2byte(*(uint16_t *)(data + channels[k].location), &channels[k]);
                break;
            case 4:
                *a = double4byte(*(uint32_t *)(data + channels[k].location), &channels[k]);
                break;
            case 8:
                *a = double8byte(*(uint64_t *)(data + channels[k].location), &channels[k]);
                break;
            default:
                break;
        }
    }
}


//------------------------------------------------------------------------------

void iio_sensors_get_ahrs(double *roll,
                          double *pitch,
                          double *magn_heading,
                          double *relative_altitude)
{
    ahrs_get_current(roll, pitch, magn_heading, relative_altitude);
}


void iio_sensors_process(void)
{
    int pressure = read_sensor_value(barometric_path);
    double temperature = (double)(read_sensor_value(temperature_path))/10;
    static struct timespec pressure_sample_time;
    clock_gettime(CLOCK_MONOTONIC, &pressure_sample_time);

    int more_data = 1;
    while (more_data)
    {
        more_data = 0;
        struct pollfd fds[] =
        {
            { .fd = accel.dev_fd, .events = POLLIN },
            { .fd =  magn.dev_fd, .events = POLLIN },
            { .fd =  gyro.dev_fd, .events = POLLIN },
        };
        poll(fds, sizeof(fds)/sizeof(struct pollfd), 0);

        int num_rows = 0;
        int i;
        for (i = 0; i < sizeof(fds)/sizeof(struct pollfd); i++)
        {
            if ((fds[i].revents & POLLIN) != 0)
            {
                more_data = 1;
                struct iio_sensor_info *sensor;
                switch (i)
                {
                    case 0: sensor = &accel; break;
                    case 1: sensor =  &magn; break;
                    case 2: sensor =  &gyro; break;
                    default: continue;
                }
                sensor->read_size = read(sensor->dev_fd, sensor->data, BUFFER_LENGTH*sensor->scan_size);
		if (sensor->read_size < 0)
                {
                    if (errno == EAGAIN)
                        continue;
                    else
                    {
                        more_data = 0;
                        break;
                    }
		}
                num_rows = max(sensor->read_size/sensor->scan_size, num_rows);
            }
        }

        int accel_div = 1;
        int magn_div = 1;
        int gyro_div = 1;
        for (i = 0; i < sizeof(fds)/sizeof(struct pollfd); i++)
        {
            int *div = NULL;
            struct iio_sensor_info *sensor;
            switch (i)
            {
                case 0: sensor = &accel; div = &accel_div; break;
                case 1: sensor =  &magn; div = &magn_div;  break;
                case 2: sensor =  &gyro; div = &gyro_div;  break;
                default: continue;
            }
            if (sensor->read_size)
                *div = num_rows * sensor->scan_size / sensor->read_size;
        }

        // Read barometric and temperature
        struct timespec now;
        if ((clock_gettime(CLOCK_MONOTONIC, &now) == 0) &&
            (now.tv_sec - pressure_sample_time.tv_sec > 1))
        {
            pressure_sample_time = now;
            pressure = read_sensor_value(barometric_path);
            temperature = (double)(read_sensor_value(temperature_path))/10;
        }

        struct sensor_axis_t *axis;
        int accel_count = 0;
        int magn_count = 0;
        int gyro_count = 0;
        int accel_read_idx = 0;
        int magn_read_idx = 0;
        int gyro_read_idx = 0;
        int j;
        for (j = 0; j < num_rows; j++)
        {
            for (i = 0; i < sizeof(fds)/sizeof(struct pollfd); i++)
            {
                int *count = NULL;
                int *div = NULL;
                int *read_idx = NULL;
                struct iio_sensor_info *sensor;
                switch (i)
                {
                    case 0:
                        sensor = &accel;
                        axis = &accel_axis;
                        count = &accel_count;
                        div = &accel_div;
                        read_idx = &accel_read_idx;
                        break;
                    case 1:
                        sensor = &magn;
                        axis = &magn_axis;
                        count = &magn_count;
                        div = &magn_div;
                        read_idx = &magn_read_idx;
                        break;
                    case 2:
                        sensor = &gyro;
                        axis = &gyro_axis;
                        count = &gyro_count;
                        div = &gyro_div;
                        read_idx = &gyro_read_idx;
                        break;
                    default:
                        continue;
                }
                (*count)++;
                if (*count >= *div)
                {
                    *count = 0;
                    if (*read_idx < sensor->read_size)
                    {
                        populate_sensor_axis(sensor->data + sensor->scan_size * (*read_idx),
                                             sensor->channels,
                                             sensor->num_channels,
                                             sensor->channel_index_to_axis_map,
                                             axis);
                        (*read_idx)++;
                    }
                }
            }

            ahrs_update(&accel_axis, &gyro_axis, &magn_axis, pressure);

            if (dump_fp)
            {
                if (write_raw_samples_to_file(dump_fp, &accel_axis, &magn_axis,
                                              &gyro_axis, pressure, temperature))
                {
                    fprintf(stderr, "Failed to write to %s\n", dump_file);
                    fclose(dump_fp);
                    dump_fp = NULL;
                }
            }
        }
    }
}


int iio_sensors_init(const char *calibration_file, const char *dump_sample_file)
{
    int ret = 0;
    struct stat st;

    if (calibration_file)
        calibration_data_file = calibration_file;

    if (read_calibration_from_file(calibration_data_file, &accel_calibration,
                                   &magn_calibration, &gyro_calibration,
                                   &magnetic_declination_mrad))
        fprintf(stderr, "Warning: no calibration data available\n");

    if (stat(barometric_path, &st) != 0)
    {
        fprintf(stderr, "Error: %s does not exist\n", barometric_path);
        ret = -1;
        goto error_ret;
    }

    if (stat(temperature_path, &st) != 0)
    {
        fprintf(stderr, "Error: %s does not exist\n", temperature_path);
        ret = -1;
        goto error_ret;
    }

    if (dump_sample_file)
    {
        dump_file = dump_sample_file;
        dump_fp = fopen(dump_file, "w");
        if (dump_fp == NULL)
        {
            ret = -errno;
            fprintf(stderr, "Error: unable to open %s\n", dump_file);
            goto error_ret;
        }
    }

    ahrs_init(magnetic_declination_mrad, read_sensor_value(barometric_path));

    create_trigger(0);
    create_trigger(1);
    create_trigger(2);

    if ((ret = setup_iio_trigger(&timer[0])) != 0)
        goto error_ret;
    if ((ret = setup_iio_trigger(&timer[1])) != 0)
        goto error_ret;
    if ((ret = setup_iio_trigger(&timer[2])) != 0)
        goto error_ret;

    if (((ret = setup_iio_device(&accel)) != 0) ||
        ((ret = setup_iio_device(&magn)) != 0) ||
        ((ret = setup_iio_device(&gyro)) != 0))
        goto error_ret;

    if (((ret = assign_trigger(&accel, &timer[0])) != 0) ||
        ((ret = assign_trigger(&magn, &timer[1])) != 0) ||
        ((ret = assign_trigger(&gyro, &timer[2])) != 0))
        goto error_ret;

    if (((ret = start_iio_device(&accel)) != 0) ||
        ((ret = start_iio_device(&magn)) != 0) ||
        ((ret = start_iio_device(&gyro)) != 0))
        goto error_ret;

    return 0;

error_ret:
    iio_sensors_clean_up();
    return ret;
}


void iio_sensors_clean_up(void)
{
    stop_iio_device(&accel);
    stop_iio_device(&magn);
    stop_iio_device(&gyro);
    disconnect_trigger(&accel);
    disconnect_trigger(&magn);
    disconnect_trigger(&gyro);

    clean_up_iio_device(&accel);
    clean_up_iio_device(&magn);
    clean_up_iio_device(&gyro);
    clean_up_iio_trigger(&timer[0]);
    clean_up_iio_trigger(&timer[1]);
    clean_up_iio_trigger(&timer[2]);

    if (dump_fp)
    {
        fclose(dump_fp);
        dump_fp = NULL;
    }
}
