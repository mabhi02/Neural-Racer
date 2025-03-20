#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <mutex>
#include <memory>
#include <functional>

namespace neural_racer {
namespace utils {

/**
 * @brief Performance profiling event
 */
struct ProfileEvent {
    std::string name;                                 ///< Event name
    std::chrono::high_resolution_clock::time_point start; ///< Event start time
    std::chrono::high_resolution_clock::time_point end;   ///< Event end time
    std::chrono::nanoseconds duration;                   ///< Event duration
    
    ProfileEvent() = default;
    
    ProfileEvent(const std::string& name) :
        name(name),
        start(std::chrono::high_resolution_clock::now()) {}
    
    void complete() {
        end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    }
    
    float getDurationMs() const {
        return duration.count() / 1000000.0f;
    }
};

/**
 * @brief Profiling session statistics
 */
struct ProfileStats {
    std::string eventName;        ///< Event name
    int64_t callCount;            ///< Number of times the event was recorded
    float totalTimeMs;            ///< Total time spent in this event (ms)
    float minTimeMs;              ///< Minimum event duration (ms)
    float maxTimeMs;              ///< Maximum event duration (ms)
    float avgTimeMs;              ///< Average event duration (ms)
    
    ProfileStats() : 
        callCount(0),
        totalTimeMs(0.0f),
        minTimeMs(std::numeric_limits<float>::max()),
        maxTimeMs(0.0f),
        avgTimeMs(0.0f) {}
    
    void update(float durationMs) {
        callCount++;
        totalTimeMs += durationMs;
        minTimeMs = std::min(minTimeMs, durationMs);
        maxTimeMs = std::max(maxTimeMs, durationMs);
        avgTimeMs = totalTimeMs / callCount;
    }
};

/**
 * @brief Scope-based profiling timer
 * 
 * This class automatically measures execution time for a code block
 * and reports the results when it goes out of scope.
 */
class ScopedTimer {
public:
    /**
     * @brief Construct a new scoped timer
     * 
     * @param name Timer name
     */
    explicit ScopedTimer(const std::string& name);
    
    /**
     * @brief Destroy the scoped timer and record the elapsed time
     */
    ~ScopedTimer();

private:
    ProfileEvent event;
};

/**
 * @brief Performance profiler
 * 
 * This class provides performance profiling capabilities for
 * measuring execution time of code blocks and functions.
 */
class Profiler {
public:
    /**
     * @brief Initialize the profiler
     * 
     * @param enabled True to enable profiling, false to disable
     * @return true if initialization was successful
     */
    static bool initialize(bool enabled = true);
    
    /**
     * @brief Shutdown the profiler
     */
    static void shutdown();
    
    /**
     * @brief Check if profiler is initialized
     * 
     * @return true if initialized, false otherwise
     */
    static bool isInitialized();
    
    /**
     * @brief Enable or disable profiling
     * 
     * @param enabled True to enable profiling, false to disable
     */
    static void setEnabled(bool enabled);
    
    /**
     * @brief Check if profiling is enabled
     * 
     * @return true if enabled, false otherwise
     */
    static bool isEnabled();
    
    /**
     * @brief Start a profiling event
     * 
     * @param name Event name
     * @return Pointer to the created event, or nullptr if profiling is disabled
     */
    static ProfileEvent* beginEvent(const std::string& name);
    
    /**
     * @brief End a profiling event
     * 
     * @param event Pointer to the event to end
     */
    static void endEvent(ProfileEvent* event);
    
    /**
     * @brief Record a complete profiling event with a specified duration
     * 
     * @param name Event name
     * @param durationMs Event duration in milliseconds
     */
    static void recordEvent(const std::string& name, float durationMs);
    
    /**
     * @brief Get statistics for all profiled events
     * 
     * @return std::vector<ProfileStats> Vector of event statistics
     */
    static std::vector<ProfileStats> getStats();
    
    /**
     * @brief Get statistics for a specific event
     * 
     * @param name Event name
     * @return ProfileStats Event statistics
     */
    static ProfileStats getEventStats(const std::string& name);
    
    /**
     * @brief Clear all profiling data
     */
    static void clear();
    
    /**
     * @brief Save profiling data to file
     * 
     * @param filename Output filename
     * @return true if file was saved successfully, false otherwise
     */
    static bool saveToFile(const std::string& filename);

private:
    static std::mutex mutex;
    static std::unordered_map<std::string, ProfileStats> stats;
    static bool initialized;
    static bool enabled;
};

/**
 * @brief Macro for easy scoped timing
 * 
 * Usage: PROFILE_SCOPE("ScopeName");
 */
#define PROFILE_SCOPE(name) neural_racer::utils::ScopedTimer timer##__LINE__(name)

/**
 * @brief Macro for timing a function
 * 
 * Usage: PROFILE_FUNCTION();
 */
#define PROFILE_FUNCTION() PROFILE_SCOPE(__FUNCTION__)

} // namespace utils
} // namespace neural_racer