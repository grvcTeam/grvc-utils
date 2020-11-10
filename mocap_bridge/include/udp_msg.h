#ifndef _UDP_MSG_
#define _UDP_MSG_
#include <iostream>
#include <iomanip>

struct {
    float x;
    float y;
    float z;
} typedef vector3_t;

struct {
    float x;
    float y;
    float z;
    float w;
} typedef quaternion_t;

struct {
    vector3_t    position;
    quaternion_t orientation;
} typedef pose_t;

struct {
    pose_t    pose;
    vector3_t twist;
} typedef odom_t;

struct {
    uint32_t sec;
    uint32_t nsec;
} typedef stamp_t;

struct {
    stamp_t stamp;
    float fit_param;
    odom_t ekf;
    odom_t leica;
    odom_t camera;
    odom_t camera_raw;
} typedef msg_t;


std::ostream& operator << (std::ostream& os, const vector3_t& v) {
    os << std::setprecision(9);
    os << "    x: " << v.x << std::endl;
    os << "    y: " << v.y << std::endl;
    os << "    z: " << v.z;
    return os;
}

std::ostream& operator << (std::ostream& os, const quaternion_t& v) {
    os << std::setprecision(9);
    os << "    x: " << v.x << std::endl;
    os << "    y: " << v.y << std::endl;
    os << "    z: " << v.z << std::endl;
    os << "    w: " << v.w;
    return os;
}

std::ostream& operator << (std::ostream& os, const odom_t& v) {
    os << "  position:\n" << v.pose.position << std::endl;
    os << "  orientation:\n"    << v.pose.orientation;
    return os;
}

std::ostream& operator << (std::ostream& os, const stamp_t& v) {
    os << "  secs: "  << v.sec << std::endl;
    os << "  nsecs: " << v.nsec;
    return os;
}

std::ostream& operator << (std::ostream& os, const msg_t& v) {
    os << "stamp:\n" << v.stamp << std::endl;
    os << "fit_param: " << v.fit_param << std::endl;
    os << "estimate:\n" << v.ekf << std::endl;
    os << "---";
    return os;
}

#endif // _UDP_MSG_