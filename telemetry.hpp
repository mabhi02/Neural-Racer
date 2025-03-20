#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <functional>
#include <chrono>
#include <deque>
#include <mutex>
#include <fstream>

// Forward declarations
namespace neural_racer {
namespace simulation {
    class Race;
}
namespace physics {
    class Vehicle;
}
namespace ai {
    class Driver;
}
}

namespace neural_racer {
namespace simulation {

/**
 * @brief Telemetry data point types
 */
enum class TelemetryType {
    Float,          ///< Single precision float
    Int,            ///< Integer
    Bool,           ///< Boolean
    String,         ///< String
    Vector2,        ///< 2D vector (two floats)
    Vector3,        ///< 3D vector (three floats)
    Quaternion      ///< Quaternion (four floats)
};

/**
 * @brief Telemetry data sample
 */
struct TelemetrySample {
    union Value {
        float floatValue;
        int intValue;
        bool boolValue;
        float vec2Value[2];
        float vec3Value[3];
        float quatValue[4];
        
        Value() : floatValue(0.0f) {}
    };
    
    float timestamp;          ///< Sample timestamp in seconds
    Value value;              ///< Sample value
    std::string stringValue;  ///< String value (used for string type)
    
    TelemetrySample() : timestamp(0.0f) {}
};

/**
 * @brief Telemetry channel definition
 */
struct TelemetryChannel {
    std::string name;                  ///< Channel name
    std::string units;                 ///< Units of measurement
    TelemetryType type;                ///< Data type
    std::deque<TelemetrySample> data;  ///< Sample data
    size_t maxSamples;                 ///< Maximum number of samples to keep
    
    TelemetryChannel() : 
        type(TelemetryType::Float),
        maxSamples(1000) {}
    
    TelemetryChannel(const std::string& name, const std::string& units, TelemetryType type, size_t maxSamples = 1000) :
        name(name),
        units(units),
        type(type),
        maxSamples(maxSamples) {}
};

/**
 * @brief Lap timing statistics
 */
struct LapStatistics {
    int currentLap;               ///< Current lap number
    float bestLapTimeSeconds;     ///< Best lap time in seconds
    float lastLapTimeSeconds;     ///< Last lap time in seconds
    float currentLapTimeSeconds;  ///< Current lap time in seconds
    std::vector<float> sectorTimes; ///< Current lap sector times
    float averageSpeedKmh;        ///< Average speed in km/h
    float topSpeedKmh;            ///< Top speed in km/h
    
    LapStatistics() :
        currentLap(0),
        bestLapTimeSeconds(0.0f),
        lastLapTimeSeconds(0.0f),
        currentLapTimeSeconds(0.0f),
        averageSpeedKmh(0.0f),
        topSpeedKmh(0.0f) {}
};

/**
 * @brief Telemetry listener interface
 */
class TelemetryListener {
public:
    virtual ~TelemetryListener() = default;
    
    /**
     * @brief Called when new telemetry data is available
     * 
     * @param channelName Channel name
     * @param sample Telemetry sample
     */
    virtual void onTelemetryData(const std::string& channelName, const TelemetrySample& sample) = 0;
    
    /**
     * @brief Called when a lap is completed
     * 
     * @param driverName Driver name
     * @param lapTime Lap time in seconds
     * @param lapNumber Lap number
     */
    virtual void onLapCompleted(const std::string& driverName, float lapTime, int lapNumber) = 0;
};

/**
 * @brief Telemetry system for collecting and analyzing race data
 * 
 * This class collects telemetry data from vehicles and drivers,
 * and provides tools for analysis and visualization.
 */
class Telemetry {
public:
    /**
     * @brief Construct a telemetry system
     * 
     * @param maxSamplesPerChannel Maximum number of samples to keep per channel
     */
    explicit Telemetry(size_t maxSamplesPerChannel = 1000);
    
    /**
     * @brief Destroy the telemetry system
     */
    ~Telemetry();
    
    /**
     * @brief Initialize the telemetry system
     * 
     * @return true if initialization was successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Shutdown the telemetry system
     */
    void shutdown();
    
    /**
     * @brief Add a telemetry channel
     * 
     * @param name Channel name
     * @param units Units of measurement
     * @param type Data type
     * @return true if channel was added, false if already exists
     */
    bool addChannel(const std::string& name, const std::string& units, TelemetryType type);
    
    /**
     * @brief Add a telemetry sample
     * 
     * @param channelName Channel name
     * @param timestamp Sample timestamp
     * @param value Sample value
     * @return true if sample was added, false otherwise
     */
    bool addSample(const std::string& channelName, float timestamp, float value);
    
    /**
     * @brief Add a telemetry sample
     * 
     * @param channelName Channel name
     * @param timestamp Sample timestamp
     * @param value Sample value
     * @return true if sample was added, false otherwise
     */
    bool addSample(const std::string& channelName, float timestamp, int value);
    
    /**
     * @brief Add a telemetry sample
     * 
     * @param channelName Channel name
     * @param timestamp Sample timestamp
     * @param value Sample value
     * @return true if sample was added, false otherwise
     */
    bool addSample(const std::string& channelName, float timestamp, bool value);
    
    /**
     * @brief Add a telemetry sample
     * 
     * @param channelName Channel name
     * @param timestamp Sample timestamp
     * @param value Sample value
     * @return true if sample was added, false otherwise
     */
    bool addSample(const std::string& channelName, float timestamp, const std::string& value);
    
    /**
     * @brief Add a telemetry sample
     * 
     * @param channelName Channel name
     * @param timestamp Sample timestamp
     * @param x X component
     * @param y Y component
     * @return true if sample was added, false otherwise
     */
    bool addSample(const std::string& channelName, float timestamp, float x, float y);
    
    /**
     * @brief Add a telemetry sample
     * 
     * @param channelName Channel name
     * @param timestamp Sample timestamp
     * @param x X component
     * @param y Y component
     * @param z Z component
     * @return true if sample was added, false otherwise
     */
    bool addSample(const std::string& channelName, float timestamp, float x, float y, float z);
    
    /**
     * @brief Add a telemetry sample
     * 
     * @param channelName Channel name
     * @param timestamp Sample timestamp
     * @param w W component
     * @param x X component
     * @param y Y component
     * @param z Z component
     * @return true if sample was added, false otherwise
     */
    bool addSample(const std::string& channelName, float timestamp, float w, float x, float y, float z);
    
    /**
     * @brief Register a telemetry listener
     * 
     * @param listener Listener to register
     */
    void registerListener(TelemetryListener* listener);
    
    /**
     * @brief Unregister a telemetry listener
     * 
     * @param listener Listener to unregister
     */
    void unregisterListener(TelemetryListener* listener);
    
    /**
     * @brief Record telemetry for a vehicle
     * 
     * @param vehicle Vehicle to record
     * @param prefix Channel name prefix
     */
    void recordVehicle(std::shared_ptr<physics::Vehicle> vehicle, const std::string& prefix = "vehicle.");
    
    /**
     * @brief Record telemetry for a driver
     * 
     * @param driver Driver to record
     * @param prefix Channel name prefix
     */
    void recordDriver(std::shared_ptr<ai::Driver> driver, const std::string& prefix = "driver.");
    
    /**
     * @brief Update telemetry system
     * 
     * @param race Race to update from
     */
    void update(std::shared_ptr<Race> race);
    
    /**
     * @brief Get lap statistics
     * 
     * @return const LapStatistics& Lap statistics
     */
    const LapStatistics& getLapStatistics() const;
    
    /**
     * @brief Get channel names
     * 
     * @return std::vector<std::string> Vector of channel names
     */
    std::vector<std::string> getChannelNames() const;
    
    /**
     * @brief Get channel info
     * 
     * @param channelName Channel name
     * @return const TelemetryChannel& Channel info
     */
    const TelemetryChannel& getChannelInfo(const std::string& channelName) const;
    
    /**
     * @brief Get channel data
     * 
     * @param channelName Channel name
     * @param startTime Start time for data range
     * @param endTime End time for data range
     * @return std::vector<TelemetrySample> Vector of samples in the time range
     */
    std::vector<TelemetrySample> getChannelData(const std::string& channelName, 
                                               float startTime, float endTime) const;
    
    /**
     * @brief Save telemetry data to a file
     * 
     * @param filename File to save to
     * @return true if data was saved successfully, false otherwise
     */
    bool saveToFile(const std::string& filename) const;
    
    /**
     * @brief Load telemetry data from a file
     * 
     * @param filename File to load from
     * @return true if data was loaded successfully, false otherwise
     */
    bool loadFromFile(const std::string& filename);
    
    /**
     * @brief Export telemetry data to CSV
     * 
     * @param filename CSV file to export to
     * @return true if data was exported successfully, false otherwise
     */
    bool exportToCsv(const std::string& filename) const;
    
    /**
     * @brief Clear all telemetry data
     */
    void clear();

private:
    std::unordered_map<std::string, TelemetryChannel> channels;
    std::vector<TelemetryListener*> listeners;
    size_t maxSamplesPerChannel;
    LapStatistics lapStats;
    mutable std::mutex telemetryMutex;
    
    // Vehicle and driver recording
    std::vector<std::weak_ptr<physics::Vehicle>> recordedVehicles;
    std::vector<std::weak_ptr<ai::Driver>> recordedDrivers;
    std::unordered_map<std::string, std::string> channelPrefixes;
    
    // Last recorded vehicle states for delta calculations
    struct VehicleState {
        float lastRecordTime;
        float lastSpeed;
        float topSpeed;
        float totalDistance;
    };
    std::unordered_map<std::shared_ptr<physics::Vehicle>, VehicleState> vehicleStates;
    
    // Helper methods
    void notifyListeners(const std::string& channelName, const TelemetrySample& sample);
    void recordVehicleState(std::shared_ptr<physics::Vehicle> vehicle, float timestamp);
    void recordDriverState(std::shared_ptr<ai::Driver> driver, float timestamp);
    void updateLapStatistics(const Race& race);
};

} // namespace simulation
} // namespace neural_racer