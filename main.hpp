// main.hpp
#ifndef VC_MAIN_HPP
#define VC_MAIN_HPP

// 1) 기본 타입 정의
typedef double real_T;
typedef double time_T;
typedef unsigned char boolean_T;
typedef int int_T;
typedef unsigned int uint_T;
typedef unsigned long ulong_T;
typedef unsigned long long ulonglong_T;
typedef char char_T;
typedef unsigned char uchar_T;
typedef char_T byte_T;

typedef signed char int8_T;
typedef unsigned char uint8_T;
typedef short int16_T;
typedef unsigned short uint16_T;
typedef int int32_T;
typedef unsigned int uint32_T;
typedef long int64_T;
typedef unsigned long uint64_T;
typedef float real32_T;
typedef double real64_T;

// 2) Time_e 먼저
struct Time_e {
  int32_T hh;
  int32_T mm;
  int32_T ss;
  int32_T ssss;
};

// 3) 의존 없는 struct 먼저
struct Sensor_imu {
  Time_e timestamp_indata;
  Time_e timestamp_outdata;
  real_T gyo_x;
  real_T gyo_y;
  real_T gyo_z;
  real_T acl_x;
  real_T acl_y;
  real_T acl_z;
};

struct Sensor_gps {
  Time_e timestamp_indata;
  Time_e timestamp_outdata;
  int32_T time_UTC;
  real_T longitude;
  real_T latitude;
  real_T altitude;
  real_T speed;
  real_T heading;
  uint8_T satellite_num;
  uint8_T direction_EW;
  uint8_T direction_NS;
  uint8_T quality_mode;
  real_T quality_horizontal;
  real_T quality_vertical;
  real_T quality_3D;
};

struct VCU_info {
  Time_e timestamp_indata;
  Time_e timestamp_outdata;
  real32_T x_position;
  real32_T y_position;
  int32_T x_velocity;
  int32_T y_velocity;
  real32_T yaw;
  real32_T yawrate;
  uint8_T motor_left_cur;
  uint8_T motor_right_cur;
  real32_T velocity_res;
  boolean_T reverse_enable;
  boolean_T emergency_enable;
};

// ★ Object 를 Object_info보다 먼저!
struct Object {
  real32_T distance;
  real32_T lateral;
  real32_T velocity_x;
  real32_T velocity_y;
  uint8_T status;
  uint8_T target_on;
  uint8_T type;
};

struct Object_info {
  Time_e timestamp_indata;
  Time_e timestamp_outdata;
  Object obstacle[10];   // 이제 가능
  int32_T obj_number;
};

struct HMI {
  Time_e timestamp_indata;
  Time_e timestamp_outdata;
  uint8_T current_position_flag;
  uint8_T mode_drive;
  uint8_T mode_roll;
  uint8_T feedback_flag;
  uint8_T workpoint_num;
  uint8_T cancel_flag;
  uint8_T stopover_flag;
};

// 최상위
struct vc_in {
  Sensor_imu  imu_info;
  Sensor_gps  gps_info;
  VCU_info    vcu_info;
  Object_info obj_info;
  HMI         hmi_info;
};

#endif // VC_MAIN_HPP
