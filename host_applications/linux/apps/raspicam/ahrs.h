#ifndef _AHRS_H_
#define _AHRS_H_


struct sensor_axis_t
{
    double x;
    double y;
    double z;
};


void ahrs_init(double magnetic_declination_mrad, int pressure);
void ahrs_update(struct sensor_axis_t *accel,
                 struct sensor_axis_t *gyro,
                 struct sensor_axis_t *magn,
                 int pressure);
void ahrs_get_current(double *roll,
                      double *pitch,
                      double *magn_heading,
                      double *relative_altitude);


#endif // _AHRS_H_
