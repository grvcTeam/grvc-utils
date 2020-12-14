#include <string>
#include <chrono>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <cerrno>
#include <cstring>
#include "terminal.h"

#define NOW std::chrono::system_clock::now().time_since_epoch().count()*1e-9

using namespace std;

namespace terminal {
    mutex m_print;
    void INFO(std::string msg) {
        m_print.lock();
        cout << ANSI_COLOR_CYAN;
        cout << fixed << setprecision(9);
        cout << "[ INFO] [" << NOW << "]: " << msg << endl;
        cout << ANSI_COLOR_RESET;
        std::cout.flush();
        m_print.unlock();
        // cout << "[ INFO] [" << NOW << "]: " << msg << endl;
    }

    void WARN(std::string msg) {
        m_print.lock();
        cout << ANSI_COLOR_YELLOW;
        cout << fixed << setprecision(9);
        cout << "[ WARN] [" << NOW << "]: " << msg << endl;
        cout << ANSI_COLOR_RESET;
        std::cout.flush();
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