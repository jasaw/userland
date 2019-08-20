#ifndef _NAZA_H_
#define _NAZA_H_

#include <stdint.h>


#define GET_SMART_BATTERY_DATA


#define NAZA_MESSAGE_NONE    0x0000
#define NAZA_MESSAGE_MSG1002 0x1002
#define NAZA_MESSAGE_MSG1003 0x1003
#define NAZA_MESSAGE_MSG1009 0x1009
#ifdef GET_SMART_BATTERY_DATA
#define NAZA_MESSAGE_MSG0926 0x0926
#endif


#define NAZA_MESSAGE_COUNT   3


typedef enum
{
    NO_FIX = 0,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_DGPS = 4
} fixType_t;  // GPS fix type

typedef enum
{
    MANUAL = 0,
    GPS = 1,
    FAILSAFE = 2,
    ATTI = 3
} flight_mode_t;  // Flying mode

typedef enum
{
    RC_UNUSED_1 = 0,
    RC_A = 1,
    RC_E = 2,
    RC_R = 3,
    RC_U = 4,
    RC_T = 5,
    RC_UNUSED_2 = 6,
    RC_X1 = 7,
    RC_X2 = 8,
    RC_UNUSED_3 = 9
} rcInChan_t;  // RC channel index

#ifdef GET_SMART_BATTERY_DATA
typedef enum
{
    CELL_1 = 0,
    CELL_2 = 1,
    CELL_3 = 2,
} smartBatteryCell_t;  // Smart battery cell index
#endif // GET_SMART_BATTERY_DATA


typedef struct __attribute__((packed))
{
    uint16_t id;
    uint16_t len;
} naza_msg_header_t;

typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    double  lon;           // longitude (radians)
    double  lat;           // lattitude (radians)
    float   altGps;        // altitude from GPS (meters)
    float   accX;          // accelerometer X axis data (??)
    float   accY;          // accelerometer Y axis data (??)
    float   accZ;          // accelerometer Z axis data (??)
    float   gyrX;          // gyroscope X axis data (??)
    float   gyrY;          // gyroscope Y axis data (??)
    float   gyrZ;          // gyroscope Z axis data (??)
    float   altBaro;       // altitude from barometric sensor (meters)
    float   headCompX;     // compensated heading X component
    float   p2roll;        // Phantom 2 roll
    float   p2pitch;       // Phantom 2 pitch
    float   headCompY;     // compensated heading Y component
    float   unk1[3];       // related to rate of motion
    float   northVelocity; // averaged northward velocity or 0 when less than 5 satellites locked (m/s)
    float   eastVelocity;  // averaged eastward velocity or 0 when less than 5 satellites locked (m/s)
    float   downVelocity;  // downward velocity (barometric) (m/s)
    float   unk2[3];
    int16_t  magCalX;       // calibrated magnetometer X axis data
    int16_t  magCalY;       // calibrated magnetometer Y axis data
    int16_t  magCalZ;       // calibrated magnetometer Y axis data
    uint8_t  unk3[10];
    uint8_t  numSat;        // number of locked satellites
    uint8_t  unk4;
    uint16_t seqNum;        // sequence number - increases with every message
} naza_msg1002_t;

typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    uint32_t dateTime;      // date/time
    uint32_t lon;           // longitude (x10^7, degree decimal)
    uint32_t lat;           // lattitude (x10^7, degree decimal)
    uint32_t altGps;        // altitude from GPS (millimeters)
    uint32_t hae;           // horizontal accuracy estimate (millimeters)
    uint32_t vae;           // vertical accuracy estimate (millimeters)
    uint8_t  unk0[4];
    int32_t  northVelocity; // northward velocity (cm/s)
    int32_t  eastVelocity;  // eastward velocity (cm/s)
    int32_t  downVelocity;  // downward velocity (cm/s)
    uint16_t pdop;          // position DOP (x100)
    uint16_t vdop;          // vertical DOP (see uBlox NAV-DOP message for details)
    uint16_t ndop;          // northing DOP (see uBlox NAV-DOP message for details)
    uint16_t edop;          // easting DOP (see uBlox NAV-DOP message for details)
    uint8_t  numSat;        // number of locked satellites
    uint8_t  unk1;
    uint8_t  fixType;       // fix type (0 - no lock, 2 - 2D lock, 3 - 3D lock, not sure if other values can be expected - see uBlox NAV-SOL message for details)
    uint8_t  unk2;
    uint8_t  fixStatus;     // fix status flags (see uBlox NAV-SOL message for details)
    uint8_t  unk3[3];
    uint16_t seqNum;        // sequence number - increases with every message
} naza_msg1003_t;

typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    uint8_t  unk1[4];
    uint16_t motorOut[8];  // motor output (M1/M2/M3/M4/M5/M6/F1/F2)
    uint8_t  unk2[4];
    int16_t  rcIn[10];     // RC controller input (order: unused/A/E/R/U/T/unused/X1/X2/unused)
    uint8_t  unk3[11];
    uint8_t  flightMode;   // (0 - manual, 1 - GPS, 2 - failsafe, 3 - atti)
    uint8_t  unk4[8];
    double   homeLat;      // home lattitude (radians)
    double   homeLon;      // home longitude (radians)
    float    homeAltBaro;  // home altitude from barometric sensor plus 20m (meters)
    uint16_t seqNum;       // sequence number - increases with every message
    uint8_t  unk5[2];
    float    stabRollIn;   // attitude stabilizer roll input (-1000~1000)
    float    stabPitchIn;  // attitude stabilizer pitch input (-1000~1000)
    float    stabThroIn;   // altitude stabilizer throttle input (-1000~1000)
    uint8_t  unk6[4];
    float    actAileIn;    // actual aileron input, mode and arm state dependent (-1000~1000)
    float    actElevIn;    // actual elevator input, mode and arm state dependent (-1000~1000)
    float    actThroIn;    // actual throttle input, mode and arm state dependent (-1000~1000)
    uint16_t batVolt;      // main battery voltage (milivolts)
    uint16_t becVolt;      // BEC voltage (milivolts)
    uint8_t  unk7[4];
    uint8_t  controlMode;  // (0 - GPS/failsafe, 1 - waypoint mode?, 3 - manual, 6 - atti)
    uint8_t  unk8[5];
    int16_t  gyrScalX;     // ???
    int16_t  gyrScalY;     // ???
    int16_t  gyrScalZ;     // ???
    uint8_t  unk9[32];
    float    downVelocity; // downward velocity (m/s)
    float    altBaro;      // altitude from barometric sensor (meters)
    float    p1roll;       // Phantom 1 roll angle (radians)
    float    p1pitch;      // Phantom 1 pitch angle (radians)
} naza_msg1009_t;

#ifdef GET_SMART_BATTERY_DATA
typedef struct __attribute__((packed))
{
    naza_msg_header_t header;
    uint16_t designCapacity;  // design capacity in mAh
    uint16_t fullCapacity;    // design capacity in mAh
    uint16_t currentCapacity; // design capacity in mAh
    uint16_t voltage;         // battry voltage in mV
    int16_t  current;         // current in mA
    uint8_t  lifePercent;     // percentage of life
    uint8_t  chargePercent;   // percentage of charge
    int16_t temperature;      // temperature in tenths of a degree Celsius
    uint16_t dischargeCount;  // number of discharges
    uint16_t serialNumber;    // serial number
    uint16_t cellVoltage[3];  // individual cell voltage in mV
    uint8_t  unk1[11];
} naza_msg0926_t;
#endif

typedef union
{
    uint8_t           bytes[256]; // Max message size (184) + header size (4) + footer size (4)
    naza_msg_header_t header;
    naza_msg1002_t    msg1002;
    naza_msg1003_t    msg1003;
    naza_msg1009_t    msg1009;
#ifdef GET_SMART_BATTERY_DATA
    naza_msg0926_t    msg0926;
#endif
} naza_msg_t;


struct naza_info_t
{
    double lon;       // longitude in degree decimal
    double lat;       // latitude in degree decimal
    double alt;       // altitude in m (from barometric sensor)
    double gpsAlt;    // altitude in m (from GPS)
    double spd;       // speed in m/s
    fixType_t fix;    // fix type (see fixType_t enum)
    int sat;          // number of satellites
    double heading;   // heading in degrees (titlt compensated)
    double headingNc; // heading in degrees (not titlt compensated)
    double cog;       // course over ground
    double vsi;       // vertical speed indicator (barometric) in m/s (a.k.a. climb speed)
    double hdop;      // horizontal dilution of precision
    double vdop;      // vertical dilution of precision
    double gpsVsi;    // vertical speed indicator (GPS based) in m/s (a.k.a. climb speed)
    float pitchRad;   // pitch in radians
    float rollRad;    // roll in radians
    int pitch;        // pitch in degrees
    int roll;         // roll in degrees
    int year;         // year (minus 2000)
    int month;
    int day;
    int hour;         // hour (for time between 16:00 and 23:59 the hour returned from GPS module is actually 00:00 - 7:59)
    int minute;
    int second;
    int battery;      // battery voltage in mV
    int motorOut[8];  // motor output (M1/M2/M3/M4/M5/M6/F1/F2)
    int rcIn[10];     // RC stick input (-1000~1000), use rcInChan_t enum to index the table
    flight_mode_t mode; // flight mode (see flight_mode_t enum)
#ifdef GET_SMART_BATTERY_DATA
    int batteryPercent; // smart battery charge percentage (0-100%)
    int batteryCell[3]; // smart battery cell voltage in mV, use smartBatteryCell_t enum to index the table
#endif
};


int naza_dump_binary(const char *filename, struct naza_info_t *nz);
void naza_print_ascii(FILE *fp, struct naza_info_t *nz);


#endif // _NAZA_H_
