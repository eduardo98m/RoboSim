#pragma once
#include <iostream>
#include <sstream>
#include <chrono>



// Function to get the current time as a string
std::string GetCurrentTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t time = std::chrono::system_clock::to_time_t(now);
    char timestamp[100];
    std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", std::localtime(&time));
    return timestamp;
}

// ANSI escape codes for text colors
namespace TextColor {
    const std::string Reset = "\033[0m";
    const std::string Red = "\033[31m";
    const std::string Green = "\033[32m";
    const std::string Yellow = "\033[33m";
}

// Improved MSG function with timestamps using <chrono>
void MSG(const std::string& message, const std::string& color = TextColor::Reset) {
    try {
        std::cout << color << "[" << GetCurrentTime() << "] " << message << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

void FATAL_IF(bool condition, const std::string& message) {
    if (condition) {
        MSG(message, TextColor::Red);
        exit(EXIT_FAILURE);
    }
}

void FATAL(const std::string& message) {
    MSG(message, TextColor::Red);
    exit(EXIT_FAILURE);
}






