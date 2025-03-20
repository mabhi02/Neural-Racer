#include "neural_racer/utils/profiler.hpp"
#include "neural_racer/utils/logger.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace neural_racer {
namespace utils {

// Initialize static members
std::mutex Profiler::mutex;
std::unordered_map<std::string, ProfileStats> Profiler::stats;
bool Profiler::initialized = false;
bool Profiler::enabled = false;

// ScopedTimer implementation
ScopedTimer::ScopedTimer(const std::string& name) : event(name) {
    if (Profiler::isEnabled()) {
        event.start = std::chrono::high_resolution_clock::now();
    }
}

ScopedTimer::~ScopedTimer() {
    if (Profiler::isEnabled()) {
        event.complete();
        Profiler::recordEvent(event.name, event.getDurationMs());
    }
}

// Profiler implementation
bool Profiler::initialize(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex);
    
    if (initialized) {
        return true;  // Already initialized
    }
    
    // Clear existing stats
    stats.clear();
    
    Profiler::enabled = enabled;
    initialized = true;
    
    Logger::info("Profiler", "Profiling system initialized (enabled: " + 
                std::string(enabled ? "true" : "false") + ")");
    
    return true;
}

void Profiler::shutdown() {
    std::lock_guard<std::mutex> lock(mutex);
    
    if (!initialized) {
        return;
    }
    
    Logger::info("Profiler", "Profiling system shutting down");
    
    // Clear stats
    stats.clear();
    
    initialized = false;
    enabled = false;
}

bool Profiler::isInitialized() {
    return initialized;
}

void Profiler::setEnabled(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex);
    Profiler::enabled = enabled;
    
    Logger::info("Profiler", "Profiling " + std::string(enabled ? "enabled" : "disabled"));
}

bool Profiler::isEnabled() {
    return initialized && enabled;
}

ProfileEvent* Profiler::beginEvent(const std::string& name) {
    if (!isEnabled()) {
        return nullptr;
    }
    
    ProfileEvent* event = new ProfileEvent(name);
    return event;
}

void Profiler::endEvent(ProfileEvent* event) {
    if (!isEnabled() || !event) {
        return;
    }
    
    event->complete();
    recordEvent(event->name, event->getDurationMs());
    
    delete event;
}

void Profiler::recordEvent(const std::string& name, float durationMs) {
    if (!isEnabled()) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex);
    
    // Update statistics for this event
    ProfileStats& eventStats = stats[name];
    if (eventStats.eventName.empty()) {
        eventStats.eventName = name;
    }
    
    eventStats.update(durationMs);
}

std::vector<ProfileStats> Profiler::getStats() {
    std::lock_guard<std::mutex> lock(mutex);
    
    std::vector<ProfileStats> result;
    result.reserve(stats.size());
    
    for (const auto& pair : stats) {
        result.push_back(pair.second);
    }
    
    // Sort by total time (descending)
    std::sort(result.begin(), result.end(), 
             [](const ProfileStats& a, const ProfileStats& b) {
                 return a.totalTimeMs > b.totalTimeMs;
             });
    
    return result;
}

ProfileStats Profiler::getEventStats(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex);
    
    auto it = stats.find(name);
    if (it != stats.end()) {
        return it->second;
    }
    
    // Return empty stats if not found
    ProfileStats emptyStats;
    emptyStats.eventName = name;
    return emptyStats;
}

void Profiler::clear() {
    std::lock_guard<std::mutex> lock(mutex);
    stats.clear();
    
    Logger::info("Profiler", "Profiling data cleared");
}

bool Profiler::saveToFile(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex);
    
    try {
        std::ofstream file(filename);
        if (!file.is_open()) {
            Logger::error("Profiler", "Failed to open file for writing: " + filename);
            return false;
        }
        
        // Write header
        file << "# Neural Racer Performance Profile\n";
        file << "# Event,Calls,Total(ms),Min(ms),Max(ms),Avg(ms)\n";
        
        // Get sorted stats
        auto sortedStats = getStats();
        
        // Write stats
        for (const auto& stat : sortedStats) {
            file << stat.eventName << ","
                 << stat.callCount << ","
                 << std::fixed << std::setprecision(3) << stat.totalTimeMs << ","
                 << std::fixed << std::setprecision(3) << stat.minTimeMs << ","
                 << std::fixed << std::setprecision(3) << stat.maxTimeMs << ","
                 << std::fixed << std::setprecision(3) << stat.avgTimeMs << "\n";
        }
        
        // Write summary
        file << "\n# Summary\n";
        file << "# Total Events: " << sortedStats.size() << "\n";
        
        float totalTime = 0.0f;
        int64_t totalCalls = 0;
        
        for (const auto& stat : sortedStats) {
            totalTime += stat.totalTimeMs;
            totalCalls += stat.callCount;
        }
        
        file << "# Total Time: " << std::fixed << std::setprecision(3) << totalTime << " ms\n";
        file << "# Total Calls: " << totalCalls << "\n";
        
        Logger::info("Profiler", "Performance data saved to file: " + filename);
        return true;
    } catch (const std::exception& e) {
        Logger::error("Profiler", "Failed to save performance data: " + std::string(e.what()));
        return false;
    }
}

} // namespace utils
} // namespace neural_racer