#include "main.hpp"
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <iostream>

using namespace std;

constexpr int OBJ_SIZE = 10;

template<typename... Args>
inline void csv(ostream& out, Args&&... args) {
    ((out << std::forward<Args>(args) << ","), ...);
}

static string now_with_ms() {
    using namespace std::chrono;
    const auto now = system_clock::now();
    time_t t = system_clock::to_time_t(now);
    tm lt = *localtime(&t);
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    char buf[64];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
             lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
             lt.tm_hour, lt.tm_min, lt.tm_sec, ms.count());
    return string(buf);
}

static string make_log_filename(const string& base = "vc_log") {
    auto t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    ostringstream oss;
    oss << base << "_" << put_time(&tm, "%Y%m%d_%H%M") << ".csv";
    return oss.str();
}


#define IMU_FIELDS(F) \
    F("imu_in_hh",      d.imu_info.timestamp_indata.hh) \
    F("imu_in_mm",      d.imu_info.timestamp_indata.mm) \
    F("imu_in_ss",      d.imu_info.timestamp_indata.ss) \
    F("imu_in_ssss",    d.imu_info.timestamp_indata.ssss) \
    F("imu_out_hh",     d.imu_info.timestamp_outdata.hh) \
    F("imu_out_mm",     d.imu_info.timestamp_outdata.mm) \
    F("imu_out_ss",     d.imu_info.timestamp_outdata.ss) \
    F("imu_out_ssss",   d.imu_info.timestamp_outdata.ssss) \
    F("imu_gyo_x",      d.imu_info.gyo_x) \
    F("imu_gyo_y",      d.imu_info.gyo_y) \
    F("imu_gyo_z",      d.imu_info.gyo_z) \
    F("imu_acl_x",      d.imu_info.acl_x) \
    F("imu_acl_y",      d.imu_info.acl_y) \
    F("imu_acl_z",      d.imu_info.acl_z)

#define GPS_FIELDS(F) \
    F("gps_in_hh",      d.gps_info.timestamp_indata.hh) \
    F("gps_in_mm",      d.gps_info.timestamp_indata.mm) \
    F("gps_in_ss",      d.gps_info.timestamp_indata.ss) \
    F("gps_in_ssss",    d.gps_info.timestamp_indata.ssss) \
    F("gps_out_hh",     d.gps_info.timestamp_outdata.hh) \
    F("gps_out_mm",     d.gps_info.timestamp_outdata.mm) \
    F("gps_out_ss",     d.gps_info.timestamp_outdata.ss) \
    F("gps_out_ssss",   d.gps_info.timestamp_outdata.ssss) \
    F("gps_timeUTC",    d.gps_info.time_UTC) \
    F("gps_long",       d.gps_info.longitude) \
    F("gps_lat",        d.gps_info.latitude) \
    F("gps_alt",        d.gps_info.altitude) \
    F("gps_speed",      d.gps_info.speed) \
    F("gps_heading",    d.gps_info.heading) \
    F("gps_sat",        static_cast<int>(d.gps_info.satellite_num)) \
    F("gps_EW",         static_cast<int>(d.gps_info.direction_EW)) \
    F("gps_NS",         static_cast<int>(d.gps_info.direction_NS)) \
    F("gps_quality_mode", static_cast<int>(d.gps_info.quality_mode)) \
    F("gps_qh",         d.gps_info.quality_horizontal) \
    F("gps_qv",         d.gps_info.quality_vertical) \
    F("gps_q3d",        d.gps_info.quality_3D)

#define VCU_FIELDS(F) \
    F("vcu_in_hh",      d.vcu_info.timestamp_indata.hh) \
    F("vcu_in_mm",      d.vcu_info.timestamp_indata.mm) \
    F("vcu_in_ss",      d.vcu_info.timestamp_indata.ss) \
    F("vcu_in_ssss",    d.vcu_info.timestamp_indata.ssss) \
    F("vcu_out_hh",     d.vcu_info.timestamp_outdata.hh) \
    F("vcu_out_mm",     d.vcu_info.timestamp_outdata.mm) \
    F("vcu_out_ss",     d.vcu_info.timestamp_outdata.ss) \
    F("vcu_out_ssss",   d.vcu_info.timestamp_outdata.ssss) \
    F("vcu_x",          d.vcu_info.x_position) \
    F("vcu_y",          d.vcu_info.y_position) \
    F("vcu_vx",         d.vcu_info.x_velocity) \
    F("vcu_vy",         d.vcu_info.y_velocity) \
    F("vcu_yaw",        d.vcu_info.yaw) \
    F("vcu_yawrate",    d.vcu_info.yawrate) \
    F("vcu_mL",         static_cast<int>(d.vcu_info.motor_left_cur)) \
    F("vcu_mR",         static_cast<int>(d.vcu_info.motor_right_cur)) \
    F("vcu_vel_res",    d.vcu_info.velocity_res) \
    F("vcu_reverse",    (d.vcu_info.reverse_enable ? 1 : 0)) \
    F("vcu_emergency",  (d.vcu_info.emergency_enable ? 1 : 0))

#define HMI_FIELDS(F) \
    F("hmi_in_hh",      d.hmi_info.timestamp_indata.hh) \
    F("hmi_in_mm",      d.hmi_info.timestamp_indata.mm) \
    F("hmi_in_ss",      d.hmi_info.timestamp_indata.ss) \
    F("hmi_in_ssss",    d.hmi_info.timestamp_indata.ssss) \
    F("hmi_out_hh",     d.hmi_info.timestamp_outdata.hh) \
    F("hmi_out_mm",     d.hmi_info.timestamp_outdata.mm) \
    F("hmi_out_ss",     d.hmi_info.timestamp_outdata.ss) \
    F("hmi_out_ssss",   d.hmi_info.timestamp_outdata.ssss) \
    F("hmi_curpos",     static_cast<int>(d.hmi_info.current_position_flag)) \
    F("hmi_mode_drive", static_cast<int>(d.hmi_info.mode_drive)) \
    F("hmi_mode_roll",  static_cast<int>(d.hmi_info.mode_roll)) \
    F("hmi_feedback",   static_cast<int>(d.hmi_info.feedback_flag)) \
    F("hmi_workpoint",  static_cast<int>(d.hmi_info.workpoint_num)) \
    F("hmi_cancel",     static_cast<int>(d.hmi_info.cancel_flag)) \
    F("hmi_stopover",   static_cast<int>(d.hmi_info.stopover_flag))


static string build_header() {
    ostringstream h;
    h << "timestamp,";
#define HNAME(name, expr) h << name << ",";
    IMU_FIELDS(HNAME)
    GPS_FIELDS(HNAME)
    VCU_FIELDS(HNAME)

    // OBJ(고정 슬롯 헤더)
    h << "obj_in_hh,obj_in_mm,obj_in_ss,obj_in_ssss,"
      << "obj_out_hh,obj_out_mm,obj_out_ss,obj_out_ssss,"
      << "obj_number,";
    for (int i = 0; i < OBJ_SIZE; ++i) {
        h << "obj" << i << "_distance,"
          << "obj" << i << "_lateral,"
          << "obj" << i << "_vx,"
          << "obj" << i << "_vy,"
          << "obj" << i << "_status,"
          << "obj" << i << "_target_on,"
          << "obj" << i << "_type,";
    }

    HMI_FIELDS(HNAME)
#undef HNAME
    h.seekp(-1, ios::cur); // 마지막 콤마 제거(선택)
    h << "\n";
    return h.str();
}

static ofstream open_with_header(const string& filename) {
    ofstream out(filename, ios::app);
    if (!out) return out;
    if (out.tellp() == 0) out << build_header();
    return out;
}

static inline void write_imu(ostream& out, const vc_in& d) {
#define HVAL(name, expr) csv(out, (expr));
    IMU_FIELDS(HVAL)
#undef HVAL
}

static inline void write_gps(ostream& out, const vc_in& d) {
#define HVAL(name, expr) csv(out, (expr));
    GPS_FIELDS(HVAL)
#undef HVAL
}

static inline void write_vcu(ostream& out, const vc_in& d) {
#define HVAL(name, expr) csv(out, (expr));
    VCU_FIELDS(HVAL)
#undef HVAL
}

static inline void write_obj(ostream& out, const vc_in& d) {
    csv(out,
        d.obj_info.timestamp_indata.hh, d.obj_info.timestamp_indata.mm,
        d.obj_info.timestamp_indata.ss, d.obj_info.timestamp_indata.ssss,
        d.obj_info.timestamp_outdata.hh, d.obj_info.timestamp_outdata.mm,
        d.obj_info.timestamp_outdata.ss, d.obj_info.timestamp_outdata.ssss,
        static_cast<int>(d.obj_info.obj_number)
    );
    for (int i = 0; i < OBJ_SIZE; ++i) {
        if (d.obj_info.obstacle != nullptr && i < d.obj_info.obj_number) {
            const auto& o = d.obj_info.obstacle[i];
            csv(out,
                o.distance, o.lateral, o.velocity_x, o.velocity_y,
                static_cast<int>(o.status), (o.target_on ? 1 : 0),
                static_cast<int>(o.type)
            );
        } else {
            out << "NA,NA,NA,NA,NA,NA,NA,";
        }
    }
}

static inline void write_hmi(ostream& out, const vc_in& d) {
#define HVAL(name, expr) csv(out, (expr));
    HMI_FIELDS(HVAL)
#undef HVAL
}

// ---------- 한 줄 쓰기 ----------
void append_vc_csv(const vc_in& d) {
    auto filename = make_log_filename();
    ofstream out = open_with_header(filename);
    if (!out) { cerr << "CSV 열기 실패!\n"; return; }

    out << "\"" << now_with_ms() << "\","; // Excel 날짜 오인 방지 따옴표
    write_imu(out, d);
    write_gps(out, d);
    write_vcu(out, d);
    write_obj(out, d);
    write_hmi(out, d);
    out << "\n";
}

// ---------- 테스트 메인 ----------
int main() {
    vc_in d{};
    for (int i = 0; i < 10; ++i) {
                // 샘플 값 채우기
        // d.imu_info.timestamp_indata.hh = i;
        // d.imu_info.timestamp_outdata.hh = i;
        // d.imu_info.gyo_x = i;

        // d.gps_info.timestamp_indata.hh = i;
        // d.gps_info.timestamp_outdata.hh = i;
        // d.gps_info.time_UTC = i;
        // d.gps_info.longitude = i;
        // d.gps_info.latitude  = i;
        // d.gps_info.donghun = 77;

        // d.vcu_info.timestamp_indata.hh = i;
        // d.vcu_info.timestamp_outdata.hh = i;
        // d.vcu_info.x_position = i;
        // d.vcu_info.reverse_enable = (i % 2);

        // d.obj_info.timestamp_indata.hh = i;
        // d.obj_info.timestamp_outdata.hh = i;
        d.obj_info.obj_number = 10;
        for (int j = 0; j < d.obj_info.obj_number; ++j) {
            d.obj_info.obstacle[j].distance   = j;
            d.obj_info.obstacle[j].lateral    = -j;
            d.obj_info.obstacle[j].velocity_x = j * 0.5f;
            d.obj_info.obstacle[j].velocity_y = -j * 0.5f;
            d.obj_info.obstacle[j].status     = (j % 4);
            d.obj_info.obstacle[j].target_on  = (j % 2);
            d.obj_info.obstacle[j].type       = (j % 3);
        }


        // d.hmi_info.timestamp_indata.hh = i;
        // d.hmi_info.timestamp_outdata.hh = i;
        // d.hmi_info.current_position_flag = (i % 2);
        // d.hmi_info.mode_drive = 1;
        // d.hmi_info.mode_roll  = 0;
        append_vc_csv(d);
    }
    cout << "csv 저장완료\n";
    return 0;
}
