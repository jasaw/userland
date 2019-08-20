#ifndef _IIO_SENSORS_H_
#define _IIO_SENSORS_H_



int iio_sensors_init(const char *calibration_file, const char *dump_sample_file);
void iio_sensors_clean_up(void);
void iio_sensors_process(void);
void iio_sensors_get_ahrs(double *roll,
                          double *pitch,
                          double *magn_heading,
                          double *relative_altitude);


#endif // _IIO_SENSORS_H_
