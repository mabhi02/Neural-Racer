#pragma once

#include <string>
#include <mutex>
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>
#include <chrono>

namespace neural_racer {
namespace utils {

/**
 * @brief Log severity levels
 */
enum class LogLevel {
    Trace,      ///< Detailed trace information
    Debug,      ///< Debugging information
    Info,       ///< Informational messages
    Warning,    ///< Warning messages
    Error,      ///< Error messages
    Critical    ///< Critical errors that may cause program termination
};

/**
 * @brief Log message structure
 */
struct LogMessage {
    std::chrono::system_clock::time_point timestamp;  ///< Message timestamp
    LogLevel level;                                   ///< Log level
    std::string module;                               ///< Source module name
    std::string message;                              ///< Log message content
    
    LogMessage() : level(LogLevel::Info) {}
    
    LogMessage(LogLevel level, const std::string& module, const std::string& message) :
        timestamp(std::chrono::system_clock::now()),
        level(level),
        module(module),
        message(message) {}
};

/**
 * @brief Interface for log sinks (log output destinations)
 */
class LogSink {
public:
    virtual ~LogSink() = default;
    
    /**
     * @brief Write a log message to the sink
     * 
     * @param message Log message to write
     */
    virtual void write(const LogMessage& message) = 0;
    
    /**
     * @brief Flush log sink buffers
     */
    virtual void flush() = 0;
};

/**
 * @brief Console log sink
 */
class ConsoleSink : public LogSink {
public:
    void write(const LogMessage& message) override;
    void flush() override;
};

/**
 * @brief File log sink
 */
class FileSink : public LogSink {
public:
    /**
     * @brief Construct a new file sink
     * 
     * @param filename Log file name
     * @param append True to append to existing file, false to overwrite
     */
    FileSink(const std::string& filename, bool append = true);
    
    ~FileSink();
    
    void write(const LogMessage& message) override;
    void flush() override;

private:
    std::ofstream file;
    std::mutex mutex;
};

/**
 * @brief Central logging system
 * 
 * This class provides a thread-safe logging system with multiple
 * output sinks, log levels, and message formatting.
 */
class Logger {
public:
    /**
     * @brief Initialize the logging system
     * 
     * @param consoleOutput True to enable console output
     * @param fileOutput True to enable file output
     * @param filename Log file name (if file output enabled)
     * @return true if initialization was successful, false otherwise
     */
    static bool initialize(bool consoleOutput = true, bool fileOutput = false, 
                          const std::string& filename = "neural_racer.log");
    
    /**
     * @brief Shutdown the logging system
     */
    static void shutdown();
    
    /**
     * @brief Check if logger is initialized
     * 
     * @return true if initialized, false otherwise
     */
    static bool isInitialized();
    
    /**
     * @brief Set the global log level
     * 
     * @param level Minimum log level to record
     */
    static void setLogLevel(LogLevel level);
    
    /**
     * @brief Get the current global log level
     * 
     * @return LogLevel Current log level
     */
    static LogLevel getLogLevel();
    
    /**
     * @brief Add a log sink
     * 
     * @param sink Shared pointer to log sink
     */
    static void addSink(std::shared_ptr<LogSink> sink);
    
    /**
     * @brief Log a message
     * 
     * @param level Log level
     * @param module Source module name
     * @param message Log message
     */
    static void log(LogLevel level, const std::string& module, const std::string& message);
    
    /**
     * @brief Log a trace message
     * 
     * @param module Source module name
     * @param message Log message
     */
    static void trace(const std::string& module, const std::string& message);
    
    /**
     * @brief Log a debug message
     * 
     * @param module Source module name
     * @param message Log message
     */
    static void debug(const std::string& module, const std::string& message);
    
    /**
     * @brief Log an info message
     * 
     * @param module Source module name
     * @param message Log message
     */
    static void info(const std::string& module, const std::string& message);
    
    /**
     * @brief Log a warning message
     * 
     * @param module Source module name
     * @param message Log message
     */
    static void warning(const std::string& module, const std::string& message);
    
    /**
     * @brief Log an error message
     * 
     * @param module Source module name
     * @param message Log message
     */
    static void error(const std::string& module, const std::string& message);
    
    /**
     * @brief Log a critical message
     * 
     * @param module Source module name
     * @param message Log message
     */
    static void critical(const std::string& module, const std::string& message);
    
    /**
     * @brief Convert log level to string
     * 
     * @param level Log level
     * @return std::string String representation of log level
     */
    static std::string levelToString(LogLevel level);

private:
    static std::mutex mutex;
    static std::vector<std::shared_ptr<LogSink>> sinks;
    static LogLevel logLevel;
    static bool initialized;
    
    // Format a timestamp
    static std::string formatTimestamp(const std::chrono::system_clock::time_point& timestamp);
};

} // namespace utils
} // namespace neural_racer