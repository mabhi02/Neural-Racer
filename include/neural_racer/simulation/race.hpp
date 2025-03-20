#pragma once

#define _USE_MATH_DEFINES  // For M_PI and other math constants
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <functional>
#include <mutex>
#include <random>   // For random number generation
#include <cmath>    // For math functions
#include "../ai/driver.hpp"
#include "../physics/vehicle.hpp"
#include "track.hpp"

namespace neural_racer {
namespace simulation {

// Forward declaration
class Telemetry;

/**
 * @brief Weather conditions for the race
 */
struct WeatherConditions {
    enum class Type {
        Clear,
        Cloudy,
        LightRain,
        HeavyRain,
        Storm
    };
    
    Type type;                     ///< Weather type
    float trackTemperature;        ///< Track temperature in Celsius
    float airTemperature;          ///< Air temperature in Celsius
    float windSpeed;               ///< Wind speed in m/s
    float windDirection;           ///< Wind direction in radians (0 = North)
    float precipitationIntensity;  ///< Precipitation intensity (0.0-1.0)
    
    WeatherConditions() :
        type(Type::Clear),
        trackTemperature(25.0f),
        airTemperature(20.0f),
        windSpeed(0.0f),
        windDirection(0.0f),
        precipitationIntensity(0.0f) {}
};

/**
 * @brief Race event types
 */
enum class RaceEventType {
    Start,
    Finish,
    LapCompleted,
    Collision,
    Overtake,
    PitStop,
    FlagYellow,
    FlagRed,
    FlagBlue,
    FlagChequered,
    // Added missing event types from the error list
    WeatherChanged,   // Weather condition changes
    Paused,           // Race paused
    Resumed,          // Race resumed
    SectorChanged     // Driver entered new sector
};

/**
 * @brief Race event data
 */
struct RaceEvent {
    RaceEventType type;            ///< Event type
    float timeStamp;               ///< Time of event in seconds
    std::string driverName;        ///< Driver involved
    std::vector<std::string> data; ///< Additional event data
};

/**
 * @brief Driver position and timing
 */
struct DriverPosition {
    std::string driverName;        ///< Driver name
    int position;                  ///< Current position
    int lap;                       ///< Current lap
    float lapTime;                 ///< Current lap time
    float bestLapTime;             ///< Best lap time
    float gapToLeader;             ///< Gap to leader in seconds
    float gapToNext;               ///< Gap to next driver in seconds
    int sector;                    ///< Current track sector
    int pitstops;                  ///< Number of pit stops
    float _totalDistance;          ///< Internal: total distance traveled
    
    DriverPosition() :
        position(0),
        lap(0),
        lapTime(0.0f),
        bestLapTime(0.0f),
        gapToLeader(0.0f),
        gapToNext(0.0f),
        sector(1),
        pitstops(0),
        _totalDistance(0.0f) {}
};

/**
 * @brief Race configuration
 */
struct RaceConfig {
    int laps;                      ///< Number of laps
    bool standing_start;           ///< Standing start vs rolling start
    bool enableWeatherChanges;     ///< Enable dynamic weather changes
    bool enableDamage;             ///< Enable vehicle damage
    bool enableFuelConsumption;    ///< Enable fuel consumption
    bool enableTireWear;           ///< Enable tire wear
    float timeScale;               ///< Simulation time scale (1.0 = real-time)
    float timeLimit;               ///< Time limit in seconds (0 = no limit)
    
    RaceConfig() :
        laps(5),
        standing_start(true),
        enableWeatherChanges(false),
        enableDamage(false),
        enableFuelConsumption(false),
        enableTireWear(false),
        timeScale(1.0f),
        timeLimit(0.0f) {}
};

/**
 * @brief Race state
 */
enum class RaceState {
    Initialized,
    Countdown,
    Running,
    YellowFlag,
    RedFlag,
    Finished,
    Cancelled
};

/**
 * @brief Race simulation
 * 
 * This class simulates a complete race with multiple AI drivers,
 * track conditions, and race events.
 */
class Race : public std::enable_shared_from_this<Race> {
public:
    /**
     * @brief Race event callback function type
     */
    using EventCallback = std::function<void(const RaceEvent&)>;
    
    /**
     * @brief Construct a race on the given track
     * 
     * @param track Track to race on
     */
    explicit Race(std::shared_ptr<Track> track);
    
    /**
     * @brief Initialize the race
     * 
     * @param config Race configuration
     * @return true if initialization was successful, false otherwise
     */
    bool initialize(const RaceConfig& config = RaceConfig());
    
    /**
     * @brief Shutdown the race and release resources
     */
    void shutdown();
    
    /**
     * @brief Add a driver to the race
     * 
     * @param driver Driver to add
     * @return true if driver was added successfully, false otherwise
     */
    bool addDriver(std::shared_ptr<ai::Driver> driver);
    
    /**
     * @brief Set race configuration
     * 
     * @param config Race configuration
     */
    void setConfig(const RaceConfig& config);
    
    /**
     * @brief Get race configuration
     * 
     * @return const RaceConfig& Race configuration
     */
    const RaceConfig& getConfig() const;
    
    /**
     * @brief Set weather conditions
     * 
     * @param weather Weather conditions
     */
    void setWeather(const WeatherConditions& weather);
    
    /**
     * @brief Get current weather conditions
     * 
     * @return const WeatherConditions& Current weather conditions
     */
    const WeatherConditions& getWeather() const;
    
    /**
     * @brief Start the race
     * 
     * @return true if race was started successfully, false otherwise
     */
    bool start();
    
    /**
     * @brief Pause the race
     */
    void pause();
    
    /**
     * @brief Resume the race
     */
    void resume();
    
    /**
     * @brief Stop the race
     */
    void stop();
    
    /**
     * @brief Update the race simulation
     * 
     * @param deltaTime Time step in seconds
     */
    void update(float deltaTime);
    
    /**
     * @brief Register a callback for race events
     * 
     * @param callback Event callback function
     * @return int Callback ID for unregistering
     */
    int registerEventCallback(EventCallback callback);
    
    /**
     * @brief Unregister an event callback
     * 
     * @param callbackId Callback ID returned from registerEventCallback
     */
    void unregisterEventCallback(int callbackId);
    
    /**
     * @brief Get driver positions
     * 
     * @return std::vector<DriverPosition> Vector of driver positions
     */
    std::vector<DriverPosition> getDriverPositions() const;
    
    /**
     * @brief Get race state
     * 
     * @return RaceState Current race state
     */
    RaceState getState() const;
    
    /**
     * @brief Get race elapsed time
     * 
     * @return float Race elapsed time in seconds
     */
    float getElapsedTime() const;
    
    /**
     * @brief Get race remaining time
     * 
     * @return float Race remaining time in seconds, or -1 if no time limit
     */
    float getRemainingTime() const;
    
    /**
     * @brief Get the race leader
     * 
     * @return std::string Name of the race leader
     */
    std::string getLeader() const;
    
    /**
     * @brief Get the number of completed laps by the leader
     * 
     * @return int Number of completed laps
     */
    int getCompletedLaps() const;
    
    /**
     * @brief Simulate a complete race
     * 
     * @param totalTime Total simulation time in seconds
     * @param telemetry Optional telemetry system to record data
     * @return true if simulation completed successfully, false otherwise
     */
    bool simulate(float totalTime, std::shared_ptr<Telemetry> telemetry = nullptr);

private:
    std::shared_ptr<Track> track;
    RaceConfig config;
    WeatherConditions weather;
    RaceState state;
    float elapsedTime;
    
    std::vector<std::shared_ptr<ai::Driver>> drivers;
    std::unordered_map<std::string, DriverPosition> driverPositions;
    
    struct CallbackInfo {
        int id;
        EventCallback callback;
    };
    std::vector<CallbackInfo> eventCallbacks;
    int nextCallbackId;
    
    mutable std::mutex raceMutex;
    bool paused;
    
    // For random number generation
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist;
    
    // Race state management
    void updateDriverPositions();
    void checkLapCompletion();
    void checkRaceCompletion();
    void updateWeather(float deltaTime);
    
    // Event handling
    void fireEvent(RaceEventType type, const std::string& driverName, 
                   const std::vector<std::string>& data = {});
    
    // Collision detection
    void detectCollisions();
    
    // Track sector management
    void updateSectors();
    
    // Helper methods
    int findDriverPosition(const std::string& driverName);
};

} // namespace simulation
} // namespace neural_racer