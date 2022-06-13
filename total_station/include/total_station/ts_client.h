//----------------------------------------------------------------------------------------------------------------------
// Total Station Client
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2022 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#ifndef _TS_CLIENT_H_
#define _TS_CLIENT_H_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * @brief Total Station server response message
 * 
 */
struct {
    float x;
    float y;
    float z;
    uint64_t timestamp;
} typedef ts_msg_t;


/**
 * @brief Total Station client class
 * 
 */
class TotalStationClient {
private:
    /// ROS node handler
    ros::NodeHandle n_;

    /// Publisher
    ros::Publisher pub_;

    /// Address to send the data
    struct sockaddr_in remote_addr_;

    /// Socket file descriptor
    int sock_;

    /// Total Station server port
    int ts_server_port_;

    /// Total Station server IP
    std::string ts_server_ip_;

    /// Publishing topic name
    std::string pub_topic_name_;

public:

    TotalStationClient():
        remote_addr_({0}),
        sock_(-1)
    {
        ros::param::param<std::string>("~topic_name", pub_topic_name_, "leica/pose");
        ros::param::param<std::string>("~ts_server_ip", ts_server_ip_, "127.0.0.1");
        ros::param::param<int>("~ts_server_port", ts_server_port_, 8000);
    }


    /**
     * @brief Total Station initialization method
     * 
     */
    void init() {

        ROS_INFO("Waiting for Total Station server");

        
        // Creates address to send the data
        remote_addr_.sin_family         = AF_INET;
        remote_addr_.sin_port           = htons(ts_server_port_);
        remote_addr_.sin_addr.s_addr    = inet_addr(ts_server_ip_.c_str());

        // Creates the socket
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ == -1) {
            ROS_ERROR("Total Station client socket creation error");
            exit(EXIT_FAILURE);
        }

        // Sets socket timeout 500ms
        struct timeval sock_timeo_500ms = {0};
        sock_timeo_500ms.tv_sec  = 1;
        sock_timeo_500ms.tv_usec = 0;
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &sock_timeo_500ms, sizeof(sock_timeo_500ms));


        // Waits until Total Station server is available for 10s
        ts_msg_t msg = {0};
        ssize_t n = -1;
        while (ros::ok()) {
            // Send a message to total station server to start sending us data
            sendto(sock_, "Hello there!", 13, 0, (const struct sockaddr *)&remote_addr_, sizeof(remote_addr_));
            // Waits for response
            if((n = recv(sock_, &msg, sizeof(ts_msg_t), 0)) > 0) break;
            // ROS_WARN("No Total Station server response. Retrying...");
        }

        pub_ = n_.advertise<geometry_msgs::PoseStamped>(pub_topic_name_, 1);
    }



    /**
     * @brief Total Station start method
     * 
     */
    void start() {
        ROS_INFO("Starting Total Station loop");

        ts_msg_t msg = {0};
        uint64_t timestamp_a = 0;

        // Main loop
        while (ros::ok()) {
            ssize_t n = recv(sock_, &msg, sizeof(msg), 0);

            // If not data received sends a message to the server to tell it we are still alive
            if(n < 0) {
                ROS_WARN("No Total Station server response. Retrying...");
                sendto(sock_, "I'm still alive!", 17, 0, (const struct sockaddr *)&remote_addr_, sizeof(remote_addr_));
                continue;
            }

            // Checks if the received data is the same as the previous one or is not
            // valid. When something goes wrong, TS server sends {0,0,0,0}.
            if (msg.timestamp == 0 || msg.timestamp == timestamp_a) continue;
            timestamp_a = msg.timestamp;

            geometry_msgs::PoseStamped data;
            data.header.frame_id = "map";
            data.header.stamp = ros::Time::now();

            data.pose.position.x = msg.x;
            data.pose.position.y = msg.y;
            data.pose.position.z = msg.z;

            data.pose.orientation.w = 1;
            data.pose.orientation.x = 0;
            data.pose.orientation.y = 0;
            data.pose.orientation.z = 0;

            pub_.publish(data);
        }

        close(sock_);
    }
};

#endif // _TS_CLIENT_H_