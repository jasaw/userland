#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ahrs.h"


struct ahrs_info
{
    double roll;
    double pitch;
    double magn_heading; // magnetic north
    double true_heading; // true north
    double magnetic_declination;
    double initial_pressure;
    double relative_altitude;

    double roll_radian;
    double pitch_radian;
    double magn_heading_radian;
    double true_heading_radian;
    double magnetic_declination_radian;
};
static struct ahrs_info ahrs;


static inline double to_degrees(double radians)
{
    return radians * (180.0 / M_PI);
}


static inline double to_radians(double degrees)
{
    return degrees * (M_PI / 180.0);
}


static void update_pitch_roll(struct sensor_axis_t *accel)
{
    /* roll: Rotation around the longitudinal axis (the plane body, 'X axis'). -90<=roll<=90    */
    /*                                 y                                                        */
    /*             roll = atan(-----------------)                                               */
    /*                          sqrt(x^2 + z^2)                                                 */
    ahrs.roll_radian = atan(-accel->y / sqrt(accel->x * accel->x + accel->z * accel->z));
    ahrs.roll = to_degrees(ahrs.roll_radian);

    /* pitch: Rotation around the lateral axis (the wing span, 'Y axis'). -180<=pitch<=180)     */
    /*                                 x                                                        */
    /*            pitch = atan(-----------------)                                               */
    /*                          sqrt(y^2 + z^2)                                                 */
    ahrs.pitch_radian = atan(accel->x / sqrt(accel->y * accel->y + accel->z * accel->z));
    ahrs.pitch = to_degrees(ahrs.pitch_radian);
}


static void update_heading(struct sensor_axis_t *magn)
{
    /* Sensor rotates around Z-axis */
    double cosRoll = (double)cos(ahrs.roll_radian);
    double sinRoll = (double)sin(ahrs.roll_radian);
    double cosPitch = (double)cos(ahrs.pitch_radian);
    double sinPitch = (double)sin(ahrs.pitch_radian);

    /* The tilt compensation algorithm                            */
    /* Xh = X.cosPitch + Z.sinPitch                               */
    /* Yh = X.sinRoll.sinPitch + Y.cosRoll - Z.sinRoll.cosPitch   */
    double Xh = magn->x * cosPitch + magn->z * sinPitch;
    double Yh = magn->x * sinRoll * sinPitch + magn->y * cosRoll - magn->z * sinRoll * cosPitch;
    ahrs.magn_heading_radian = -atan2(Yh, Xh);
    ahrs.true_heading_radian = ahrs.magn_heading_radian + ahrs.magnetic_declination_radian;
    // Normalize heading
    if (ahrs.magn_heading_radian < 0.0)
        ahrs.magn_heading_radian += 2*M_PI;
    if (ahrs.true_heading_radian < 0.0)
        ahrs.true_heading_radian += 2*M_PI;
    ahrs.magn_heading = to_degrees(ahrs.magn_heading_radian);
    ahrs.true_heading = to_degrees(ahrs.true_heading_radian);
}


static void update_altitude(int current_pressure)
{
    ahrs.relative_altitude = 44330.0 * (1.0 - pow(current_pressure / ahrs.initial_pressure, 0.1903));
}


void ahrs_init(double magnetic_declination_mrad, int pressure)
{
    memset(&ahrs, 0, sizeof(ahrs));
    ahrs.magnetic_declination_radian = magnetic_declination_mrad/1000;
    ahrs.magnetic_declination = to_degrees(ahrs.magnetic_declination_radian);
    ahrs.initial_pressure = pressure;
}


void ahrs_update(struct sensor_axis_t *accel,
                 struct sensor_axis_t *gyro,
                 struct sensor_axis_t *magn,
                 int pressure)
{
    update_pitch_roll(accel);
    update_heading(magn);
    update_altitude(pressure);
}


void ahrs_get_current(double *roll, double *pitch, double *magn_heading, double *relative_altitude)
{
    *roll = ahrs.roll;
    *pitch = ahrs.pitch;
    *magn_heading = ahrs.magn_heading;
    *relative_altitude = ahrs.relative_altitude;
}
