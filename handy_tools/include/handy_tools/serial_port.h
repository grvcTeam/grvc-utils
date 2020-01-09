// A couple of functions to simplify serial port handling, without OOP interference...
// Thanks mbedded.ninja! [https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/]
// freal@us.es
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <stdio.h>
#include <string.h>
#include <fcntl.h>    // Contains file controls like O_RDWR
#include <errno.h>    // Error integer and strerror() function
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()
#include <sys/file.h>
#include <stdexcept>

namespace serial_port {

void configure(int fd, speed_t baudrate = B9600, cc_t vmin = 0, cc_t vtime = 10) {
    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    // Read in existing settings, and handle any error
    if(tcgetattr(fd, &tty) != 0) {
        char e_buffer[256];
        snprintf(e_buffer, sizeof(e_buffer), "Error %i from tcgetattr: %s", errno, strerror(errno));
        throw std::runtime_error(e_buffer);
    }

    tty.c_cflag &= ~PARENB;   // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;   // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8;       // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;  // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;    // Disable echo
    tty.c_lflag &= ~ECHOE;   // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);  // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR;  // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS;  // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT;  // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = vtime;  // Wait for up to vtime deciseconds, returning as soon as any data is received.
    tty.c_cc[VMIN] = vmin;

    // Set in/out baud rate
    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        char e_buffer[256];
        snprintf(e_buffer, sizeof(e_buffer), "Error %i from tcsetattr: %s", errno, strerror(errno));
        throw std::runtime_error(e_buffer);
    }
}

void lock(int fd) {
    // Acquire non-blocking exclusive lock
    if(flock(fd, LOCK_EX | LOCK_NB) == -1) {
        char e_buffer[256];
        snprintf(e_buffer, sizeof(e_buffer), "Serial port with file descriptor %d is already locked by another process.", fd);
        throw std::runtime_error(e_buffer);
    }
}

}
#endif  // SERIAL_PORT_H
