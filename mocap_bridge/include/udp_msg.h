#ifndef _UDP_MSG_
#define _UDP_MSG_
#include <iostream>
#include <iomanip>

struct {
    double x;
    double y;
    double z;
} typedef vector3_t;

struct {
    double w;
    double x;
    double y;
    double z;
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

/**
 * @brief Output UDP message with the calculed data
 * 
 */
struct {
    stamp_t stamp;
    odom_t ekf;
    bool sensor_status[10];
    uint8_t error_status[10];
} typedef udp_msg_t;



#endif // _UDP_MSG_