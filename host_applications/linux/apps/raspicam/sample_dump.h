#ifndef _SAMPLE_DUMP_H_
#define _SAMPLE_DUMP_H_

#include <stdio.h>
#include "ahrs.h"


int write_raw_samples_to_file(FILE *fp,
                              struct sensor_axis_t *accel_axis,
                              struct sensor_axis_t *magn_axis,
                              struct sensor_axis_t *gyro_axis,
                              int pressure,
                              double temperature);
int write_ahrs_to_file(FILE *fp,
                       double roll,
                       double pitch,
                       double heading,
                       double relative_altitude,
                       double temperature);

int read_raw_samples_from_file(FILE *fp,
                               struct sensor_axis_t *accel_axis,
                               struct sensor_axis_t *magn_axis,
                               struct sensor_axis_t *gyro_axis,
                               int *pressure,
                               double *temperature);
int read_ahrs_from_file(FILE *fp,
                        double *roll,
                        double *pitch,
                        double *heading,
                        double *relative_altitude,
                        double *temperature);


#endif // _SAMPLE_DUMP_H_
