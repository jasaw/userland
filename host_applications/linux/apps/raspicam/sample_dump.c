#include "sample_dump.h"


struct file_raw_samples
{
    float accel_x;
    float accel_y;
    float accel_z;
    float magn_x;
    float magn_y;
    float magn_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float pressure;
    float temperature;
};

struct file_ahrs_samples
{
    float roll;
    float pitch;
    float heading;
    float relative_altitude;
    float temperature;
};


int write_raw_samples_to_file(FILE *fp,
                              struct sensor_axis_t *accel_axis,
                              struct sensor_axis_t *magn_axis,
                              struct sensor_axis_t *gyro_axis,
                              int pressure,
                              double temperature)
{
    struct file_raw_samples b;
    b.accel_x = accel_axis->x;
    b.accel_y = accel_axis->y;
    b.accel_z = accel_axis->z;
    b.magn_x  = magn_axis->x;
    b.magn_y  = magn_axis->y;
    b.magn_z  = magn_axis->z;
    b.gyro_x  = gyro_axis->x;
    b.gyro_y  = gyro_axis->y;
    b.gyro_z  = gyro_axis->z;
    b.pressure = pressure;
    b.temperature = temperature;
    return (fwrite(&b, sizeof(struct file_raw_samples), 1, fp) == 1) ? 0 : -1;
}


int write_ahrs_to_file(FILE *fp,
                       double roll,
                       double pitch,
                       double heading,
                       double relative_altitude,
                       double temperature)
{
    struct file_ahrs_samples b;
    b.roll    = roll;
    b.pitch   = pitch;
    b.heading = heading;
    b.relative_altitude = relative_altitude;
    b.temperature = temperature;
    return (fwrite(&b, sizeof(struct file_ahrs_samples), 1, fp) == 1) ? 0 : -1;
}


int read_raw_samples_from_file(FILE *fp,
                               struct sensor_axis_t *accel_axis,
                               struct sensor_axis_t *magn_axis,
                               struct sensor_axis_t *gyro_axis,
                               int *pressure,
                               double *temperature)
{
    struct file_raw_samples b;
    if (fread(&b, sizeof(b), 1, fp) == 1)
    {
        accel_axis->x = b.accel_x;
        accel_axis->y = b.accel_y;
        accel_axis->z = b.accel_z;
        magn_axis->x  = b.magn_x;
        magn_axis->y  = b.magn_y;
        magn_axis->z  = b.magn_z;
        gyro_axis->x  = b.gyro_x;
        gyro_axis->y  = b.gyro_y;
        gyro_axis->z  = b.gyro_z;
        *pressure     = (int)b.pressure;
        *temperature  = b.temperature;
        return 0;
    }
    return -1;
}


int read_ahrs_from_file(FILE *fp,
                        double *roll,
                        double *pitch,
                        double *heading,
                        double *relative_altitude,
                        double *temperature)
{
    struct file_ahrs_samples b;
    if (fread(&b, sizeof(b), 1, fp) == 1)
    {
        *roll = b.roll;
        *pitch = b.pitch;
        *heading = b.heading;
        *relative_altitude = b.relative_altitude;
        *temperature  = b.temperature;
        return 0;
    }
    return -1;
}
