#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>       // For timeval

#include "udp_msg.h"
#include "terminal.h"

#define PUB_VISION_TOPIC_NAME "/mavros/vision_pose/pose"
#define LOCAL_UDP_PORT 1234

using namespace std;

void wellcome_msg();

const vector<string> sensor_names = {"Leica", "Cam0", "Cam1"};

std::string map_status(const bool status) {
    stringstream ss;
    if(status) {
        ss << ANSI_COLOR_GREEN << ANSI_BOLD << " ON" << ANSI_COLOR_RESET;
    }
    else {
        ss << ANSI_COLOR_RED << ANSI_BOLD << "OFF" << ANSI_COLOR_RESET;
    }
    return ss.str();
}


int main(int argc, char **argv) {
    wellcome_msg();
    ros::init(argc, argv, "udp_2_vision");
    ros::NodeHandle n;

    // Vision publisher
    ros::Publisher pub_vision  = n.advertise<geometry_msgs::PoseStamped>(PUB_VISION_TOPIC_NAME, 1);

    // Gets port
    uint local_udp_port;
    cout << "Enter port to listen to: ";
    cin  >> local_udp_port;


    // Creates local address structure for receiving data
    struct sockaddr_in local_addr = {0};
    local_addr.sin_family         = AF_INET;
    local_addr.sin_port           = htons(local_udp_port);
    local_addr.sin_addr.s_addr    = INADDR_ANY;

    // Creates a socket
    int sock = socket (AF_INET, SOCK_DGRAM, 0);
    if (sock == -1) {
        terminal::ERROR("Socket creation error");
        return EXIT_FAILURE;
    }

    // Binds the socket to local_addr
    if (bind(sock, (struct sockaddr*) &local_addr, sizeof(local_addr)) == -1) {
        terminal::ERROR("Bind error");
        close(sock);
        return EXIT_FAILURE;
    }

    // Sets socket timeout
    struct timeval sock_timeo = {0};
    sock_timeo.tv_sec  = 0;
    sock_timeo.tv_usec = 500000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &sock_timeo, sizeof(sock_timeo));


    stringstream ss;
    ss << "Listening on UDP port " << local_udp_port;
    terminal::INFO(ss.str());

    sleep(1);

    ss.str(std::string());
    ss << "Publishing data to \"" << PUB_VISION_TOPIC_NAME << "\"";
    terminal::INFO(ss.str());

    cout << "---" << endl << "Vision monitor:" << endl;

    while(ros::ok()) {
        udp_msg_t msg_in = {0};
        ssize_t n = recv(sock, &msg_in, sizeof(msg_in), 0);
        
        bool ekf_status = false;
        if (n > 0) {
            ekf_status = true;
            geometry_msgs::PoseStamped msg_out;

            msg_out.header.stamp    = ros::Time::now();
            msg_out.header.frame_id = "map";

            msg_out.pose.position.x = msg_in.ekf.pose.position.x;
            msg_out.pose.position.y = msg_in.ekf.pose.position.y;
            msg_out.pose.position.z = msg_in.ekf.pose.position.z;

            msg_out.pose.orientation.x = msg_in.ekf.pose.orientation.x;
            msg_out.pose.orientation.y = msg_in.ekf.pose.orientation.y;
            msg_out.pose.orientation.z = msg_in.ekf.pose.orientation.z;
            msg_out.pose.orientation.w = msg_in.ekf.pose.orientation.w;

            pub_vision.publish(msg_out);
        }

        char s[1000] = {0};
        sprintf(s, "\r");
        sprintf(s, "%sEstimator - [%s] ", s, map_status(ekf_status).c_str());
        for(uint k = 0; k < sensor_names.size(); k++) {
            sprintf(s, "%s| %s - [%s] ", s, sensor_names[k].c_str(), map_status(msg_in.sensor_status[k]).c_str());
        }
        write(STDOUT_FILENO, s, sizeof(s));
        fflush(stdout);
    }

    close(sock);
    cout << endl;
    terminal::INFO("Ending");
    return 0;
}

void wellcome_msg() {

cout << " --------------------------------------------------" << endl;
cout << "\
            _      ____        _     _             \n\
  _   _  __| |_ __|___ \\__   _(_)___(_) ___  _ __  \n\
 | | | |/ _` | '_ \\ __) \\ \\ / / / __| |/ _ \\| '_ \\ \n\
 | |_| | (_| | |_) / __/ \\ V /| \\__ \\ | (_) | | | |\n\
  \\__,_|\\__,_| .__/_____| \\_/ |_|___/_|\\___/|_| |_|\n\
             |_|                                   " << endl;
cout << "              UDP to ROS Vision Pose              " << endl;
cout << "           Developed by Diego B. Gayango           " << endl;
cout << " --------------------------------------------------" << endl;
cout << endl;

}