#include <string>
#include <chrono>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <unistd.h>
#include "terminal.h"

#define NOW std::chrono::system_clock::now().time_since_epoch().count()*1e-9

using namespace std;

std::stringstream ss;

namespace terminal {
    mutex m_print;
    void PRINT_LN(std::string msg) {
        ss << msg << endl;
        stringstream tss;
        tss << ANSI_CLEAR_SCREEN << ss.str();
        write(STDOUT_FILENO, tss.str().c_str(), tss.str().size());
        fflush(stdout);
    }

    void PRINT(std::string msg) {
        ss << msg;
        stringstream tss;
        tss << ANSI_CLEAR_SCREEN << ss.str();
        write(STDOUT_FILENO, tss.str().c_str(), tss.str().size());
        fflush(stdout);
    }

    void PRINT_STATUS(std::string msg) {
        stringstream tss;
        tss << ANSI_CLEAR_SCREEN << ss.str() << msg;
        write(STDOUT_FILENO, tss.str().c_str(), tss.str().size());
        fflush(stdout);
    }

    void INFO(std::string msg) {
        m_print.lock();
        ss << ANSI_COLOR_CYAN;
        ss << fixed << setprecision(9);
        ss << "[ INFO] [" << NOW << "]: " << msg << endl;
        ss << ANSI_COLOR_RESET;
        PRINT_STATUS("");
        m_print.unlock();
        // cout << "[ INFO] [" << NOW << "]: " << msg << endl;
    }

    void WARN(std::string msg) {
        m_print.lock();
        ss << ANSI_COLOR_YELLOW;
        ss << fixed << setprecision(9);
        ss << "[ WARN] [" << NOW << "]: " << msg << endl;
        ss << ANSI_COLOR_RESET;
        PRINT_STATUS("");
        m_print.unlock();
    }

    void ERROR(std::string msg, const char deterr[]) {
        m_print.lock();
        cerr << ANSI_COLOR_RED;
        cerr << fixed << setprecision(9);
        cerr << "[ERROR] [" << NOW << "]: " << msg << ": " << deterr << endl;
        cerr << ANSI_COLOR_RESET;
        //std::cout.flush();
        m_print.unlock();
    }

    void ERROR(std::string msg, std::string deterr) {
        m_print.lock();
        cerr << ANSI_COLOR_RED;
        cerr << fixed << setprecision(9);
        cerr << "[ERROR] [" << NOW << "]: " << msg << ": " << deterr << endl;
        cerr << ANSI_COLOR_RESET;
        //std::cout.flush();
        m_print.unlock();
    }

    void ERROR(std::string msg) {
        m_print.lock();
        cerr << ANSI_COLOR_RED;
        cerr << fixed << setprecision(9);
        cerr << "[ERROR] [" << NOW << "]: " << msg << ": " << strerror(errno) << endl;
        cerr << ANSI_COLOR_RESET;
        //std::cout.flush();
        m_print.unlock();
    }
}