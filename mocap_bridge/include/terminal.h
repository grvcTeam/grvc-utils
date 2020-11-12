#include <string>
#include <iostream>

namespace terminal {
    void INFO(std::string);
    void WARN(std::string);
    void ERROR(std::string, const char *);
    void ERROR(std::string);
}