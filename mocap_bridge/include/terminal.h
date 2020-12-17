#include <string>
#include <iostream>

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"
#define ANSI_BOLD          "\x1b[1m"
#define ANSI_CLEAR_SCREEN  "\e[1;1H\e[2J"

/**
 * @brief Prints formatted messages
 * 
 */
namespace terminal {
    void PRINT(std::string msg);
    void PRINT_LN(std::string msg);
    void PRINT_STATUS(std::string msg);

    /**
     * @brief Prints information message to the standard output
     * 
     * @param msg Message to print
     */
    void INFO(std::string msg);

    /**
     * @brief Prints warning message to the standard output
     * 
     * @param msg Message to print
     */
    void WARN(std::string msg);

    /**
     * @brief Prints error message to the standard error
     * 
     * @param msg Message to print
     * @param deterr Detailed error message
     */
    void ERROR(std::string msg, const char deterr[]);

    /**
     * @brief Prints error message to the standard error
     * 
     * @param msg Message to print
     * @param deterr Detailed error message
     */
    void ERROR(std::string msg, std::string deterr);

    /**
     * @brief Prints error message to the standard error
     * 
     * @param msg Message to print
     */
    void ERROR(std::string msg);
}