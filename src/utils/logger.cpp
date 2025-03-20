#include "neural_racer/utils/logger.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>

namespace neural_racer {
namespace utils {

// Initialize static members
std::mutex Logger::mutex;
std::vector<std::shared_ptr<LogSink>> Logger::sinks;
LogLevel Logger::logLevel = LogLevel::Info;
bool Logger::initialized = false;

// ConsoleSink implementation
void ConsoleSink::write(const LogMessage& message) {
    // Set color based on log level
    switch (message.level) {
        case LogLevel::Trace:
            std::cout << "\033[90m"; // Dark gray
            break;
        case LogLevel::Debug:
            std::cout << "\033[37m"; // Light gray
            break;
        case LogLevel::Info:
            std::cout << "\033[0m";  // Default color
            break;
        case LogLevel::Warning:
            std::cout << "\033[33m"; // Yellow
            break;
        case LogLevel::Error:
            std::cout << "\033[31m"; // Red
            break;
        case LogLevel::Critical:
            std::cout << "\033[1;31m"; // Bright red
            break;
    }
    
    // Format timestamp
    std::string timestamp = Logger::formatTimestamp(message.timestamp);
    
    // Write message
    std::cout << "[" << timestamp << "] [" 
              << Logger::levelToString(message.level) << "] [" 
              << message.module << "] " 
              << message.message;
    
    // Reset color
    std::cout << "\033[0m" << std::endl;
}

void ConsoleSink::flush() {
    std::cout.flush();
}

// FileSink implementation
FileSink::FileSink(const std::string& filename, bool append) {
    if (append) {
        file.open(filename, std::ios::out | std::ios::app);
    } else {
        file.open(filename, std::ios::out | std::ios::trunc);
    }
    
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open log file: " + filename);
    }
}

FileSink::~FileSink() {
    if (file.is_open()) {
        file.close();
    }
}

void FileSink::write(const LogMessage& message) {
    std::lock_guard<std::mutex> lock(mutex);
    
    if (!file.is_open()) {
        return;
    }
    
    // Format timestamp
    std::string timestamp = Logger::formatTimestamp(message.timestamp);
    
    // Write message
    file << "[" << timestamp << "] [" 
         << Logger::levelToString(message.level) << "] [" 
         << message.module << "] " 
         << message.message << std::endl;
}

void FileSink::flush() {
    std::lock_guard<std::mutex> lock(mutex);
    
    if (file.is_open()) {
        file.flush();
    }
}

// Logger implementation
bool Logger::initialize(bool consoleOutput, bool fileOutput, const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex);
    
    if (initialized) {
        return true;  // Already initialized
    }
    
    // Clear existing sinks
    sinks.clear();
    
    // Add console sink if requested
    if (consoleOutput) {
        sinks.push_back(std::make_shared<ConsoleSink>());
    }
    
    // Add file sink if requested
    if (fileOutput) {
        try {
            sinks.push_back(std::make_shared<FileSink>(filename));
        } catch (const std::exception& e) {
            std::cerr << "Failed to create file sink: " << e.what() << std::endl;
            return false;
        }
    }
    
    initialized = true;
    
    // Log initialization message
    info("Logger", "Logging system initialized");
    
    return true;
}

void Logger::shutdown() {
    std::lock_guard<std::mutex> lock(mutex);
    
    if (!initialized) {
        return;
    }
    
    // Log shutdown message
    info("Logger", "Logging system shutting down");
    
    // Flush all sinks
    for (auto& sink : sinks) {
        sink->flush();
    }
    
    // Clear sinks
    sinks.clear();
    
    initialized = false;
}

bool Logger::isInitialized() {
    return initialized;
}

void Logger::setLogLevel(LogLevel level) {
    std::lock_guard<std::mutex> lock(mutex);
    logLevel = level;
    
    // Log level change
    if (initialized) {
        info("Logger", "Log level set to " + levelToString(level));
    }
}

LogLevel Logger::getLogLevel() {
    return logLevel;
}

void Logger::addSink(std::shared_ptr<LogSink> sink) {
    std::lock_guard<std::mutex> lock(mutex);
    
    if (sink) {
        sinks.push_back(sink);
    }
}

void Logger::log(LogLevel level, const std::string& module, const std::string& message) {
    // Early return if level is below threshold
    if (level < logLevel) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex);
    
    if (!initialized || sinks.empty()) {
        // Not initialized or no sinks, write to stderr
        std::cerr << "[" << levelToString(level) << "] [" 
                 << module << "] " << message << std::endl;
        return;
    }
    
    // Create log message
    LogMessage logMessage(level, module, message);
    
    // Write to all sinks
    for (auto& sink : sinks) {
        sink->write(logMessage);
    }
}

void Logger::trace(const std::string& module, const std::string& message) {
    log(LogLevel::Trace, module, message);
}

void Logger::debug(const std::string& module, const std::string& message) {
    log(LogLevel::Debug, module, message);
}

void Logger::info(const std::string& module, const std::string& message) {
    log(LogLevel::Info, module, message);
}

void Logger::warning(const std::string& module, const std::string& message) {
    log(LogLevel::Warning, module, message);
}

void Logger::error(const std::string& module, const std::string& message) {
    log(LogLevel::Error, module, message);
}

void Logger::critical(const std::string& module, const std::string& message) {
    log(LogLevel::Critical, module, message);
}

std::string Logger::levelToString(LogLevel level) {
    switch (level) {
        case LogLevel::Trace:    return "TRACE";
        case LogLevel::Debug:    return "DEBUG";
        case LogLevel::Info:     return "INFO";
        case LogLevel::Warning:  return "WARN";
        case LogLevel::Error:    return "ERROR";
        case LogLevel::Critical: return "CRIT";
        default:                 return "UNKNOWN";
    }
}

std::string Logger::formatTimestamp(const std::chrono::system_clock::time_point& timestamp) {
    auto time = std::chrono::system_clock::to_time_t(timestamp);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        timestamp.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    
    return ss.str();
}

} // namespace utils
} // namespace neural_racer