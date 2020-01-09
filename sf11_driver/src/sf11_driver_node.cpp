//----------------------------------------------------------------------------------------------------------------------
// GRVC SF11_DRIVER
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2020 GRVC University of Seville
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
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <handy_tools/serial_port.h>

class SF11Range {
public:

    SF11Range() {
        // Constant data
        data_.header.frame_id = "sf11";
        data_.radiation_type = sensor_msgs::Range::INFRARED;  // not really...
        data_.field_of_view = 0.0035;  // [rad]
        data_.min_range = 0.2;  // [m]
        data_.max_range = 120;  // [m]
    }

    bool has_new_data() { return has_new_data_; }

    sensor_msgs::Range get() {
        // TODO: Use strength_/voltage_?
        data_.header.stamp = ros::Time::now();
        data_.range = distance_;
        has_new_data_ = false;
        return data_;
    }

	void parse(const char *rx_buffer, size_t rx_size) {
		// Look for distance packets, each is separated by a newline
		for (size_t i = 0; i < rx_size; i++) {
			char rx = rx_buffer[i];
            // ROS_DEBUG("rx: [%c]", rx);

			// TODO: switch?
			if (rx == '\n') {
				packet_buffer_[packet_index_] = 0;
				// ROS_DEBUG("Packet: [%s]", packet_buffer_);

				if (sscanf(packet_buffer_, "%f m %f V %f", &distance_, &voltage_, &strength_) == 3) {
                    has_new_data_ = true;
					// ROS_DEBUG("Distance: %0.2f m", distance_);
				}
				packet_index_ = 0;

			} else if (rx == '\r') {
				// Ignore
			} else {
				if (packet_index_ < sizeof(packet_buffer_) - 1) {
					packet_buffer_[packet_index_++] = rx;
				}
			}
		}
	}

protected:
    sensor_msgs::Range data_;
	float distance_ = 0;
	float voltage_ = 0;
	float strength_ = 0;
    bool has_new_data_ = false;

	char packet_buffer_[64];
	int packet_index_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sf11_driver_node");
    ros::NodeHandle n;

    std::string serial_path;
    int serial_baudrate;
    ros::param::param<std::string>("~serial_path", serial_path, "/dev/ttyUSB0");
    ros::param::param<int>("~serial_baudrate", serial_baudrate, 115200);

    int serial_port = open(serial_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port == -1) {
        char e_buffer[256];
        snprintf(e_buffer, sizeof(e_buffer), "Error %i calling open: %s", errno, strerror(errno));
        throw std::runtime_error(e_buffer);
    }

    serial_port::configure(serial_port, serial_baudrate);
    serial_port::lock(serial_port);

	SF11Range sf11_range;
    ros::Publisher sf11_range_pub = n.advertise<sensor_msgs::Range>("sf11", 10);
    char rx_buffer[64];

    ros::Rate rate(20);  // [Hz]
    while (ros::ok()) {

        size_t rx_bytes = read(serial_port, &rx_buffer, sizeof(rx_buffer));
        if (rx_bytes < 0) {
            ROS_ERROR("Error reading: %s", strerror(errno));
        } else if (rx_bytes > 0) {
			sf11_range.parse(rx_buffer, rx_bytes);
        }

        if (sf11_range.has_new_data()) {
            sf11_range_pub.publish(sf11_range.get());
        }

        ros::spinOnce();
        rate.sleep();
    }

    close(serial_port);
    return 0;
}
