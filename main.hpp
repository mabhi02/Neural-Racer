#pragma once

#include <memory>
#include <string>
#include <functional>

// Forward declarations of main components
namespace neural_racer {
namespace hardware {
    class GPUAccelerator;
    class SystemInfo;
}
namespace ai {
    class InferenceEngine;
    class Driver;
    class DriverModel;
}
namespace physics {
    class Engine;
    class Vehicle;
}
namespace simulation {
    class Race;
    class Track;
    class Telemetry;
}
namespace utils {
    class Logger;
    class Profiler;
    class Config;
}
}

namespace neural_racer {

/**
 * @brief Main application class
 * 
 * This class manages the lifecycle of the Neural Racer application
 * and orchestrates the various components.
 */
class NeuralRacer {
public:
    /**
     * @brief Construct a new Neural Racer application
     */
    NeuralRacer();
    
    /**
     * @brief Destroy the Neural Racer application
     */
    ~NeuralRacer();
    
    /**
     * @brief Initialize the application
     * 
     * @param configFile Optional path to configuration file
     * @return true if initialization was successful, false otherwise
     */
    bool initialize(const std::string& configFile = "");
    
    /**
     * @brief Run the application
     * 
     * @return int Exit code (0 for success)
     */
    int run();
    
    /**
     * @brief Shutdown the application
     */
    void shutdown();
    
    /**
     * @brief Check if the application is running
     * 
     * @return true if the application is running, false otherwise
     */
    bool isRunning() const;
    
    /**
     * @brief Load a track
     * 
     * @param trackName Name of the track to load
     * @return true if track was loaded successfully, false otherwise
     */
    bool loadTrack(const std::string& trackName);
    
    /**
     * @brief Load a vehicle
     * 
     * @param vehicleName Name of the vehicle to load
     * @return true if vehicle was loaded successfully, false otherwise
     */
    bool loadVehicle(const std::string& vehicleName);
    
    /**
     * @brief Create an AI driver
     * 
     * @param skillLevel Skill level of the driver (0.0-1.0)
     * @param name Driver name
     * @return true if driver was created successfully, false otherwise
     */
    bool createDriver(float skillLevel, const std::string& name = "AI Driver");
    
    /**
     * @brief Start a race
     * 
     * @param laps Number of laps
     * @return true if race was started successfully, false otherwise
     */
    bool startRace(int laps = 5);
    
    /**
     * @brief Pause the race
     */
    void pauseRace();
    
    /**
     * @brief Resume the race
     */
    void resumeRace();
    
    /**
     * @brief Stop the race
     */
    void stopRace();
    
    /**
     * @brief Register a callback for telemetry data
     * 
     * @param callback Callback function that takes a channel name and value
     * @return int Callback ID for unregistering
     */
    int registerTelemetryCallback(std::function<void(const std::string&, float)> callback);
    
    /**
     * @brief Unregister a telemetry callback
     * 
     * @param callbackId Callback ID returned from registerTelemetryCallback
     */
    void unregisterTelemetryCallback(int callbackId);
    
    /**
     * @brief Run a demonstration of hardware capabilities
     * 
     * @return true if demonstration was successful, false otherwise
     */
    bool runHardwareDemo();
    
    /**
     * @brief Run a demonstration of AI capabilities
     * 
     * @return true if demonstration was successful, false otherwise
     */
    bool runAIDemo();
    
    /**
     * @brief Run a full simulation demo
     * 
     * @return true if demonstration was successful, false otherwise
     */
    bool runSimulationDemo();

private:
    // Application state
    bool initialized;
    bool running;
    
    // Core components
    std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator;
    std::shared_ptr<physics::Engine> physicsEngine;
    std::shared_ptr<simulation::Track> track;
    std::shared_ptr<simulation::Race> race;
    std::shared_ptr<simulation::Telemetry> telemetry;
    
    // Vehicles and drivers
    std::vector<std::shared_ptr<physics::Vehicle>> vehicles;
    std::vector<std::shared_ptr<ai::Driver>> drivers;
    
    // Utility components
    std::shared_ptr<utils::Logger> logger;
    std::shared_ptr<utils::Profiler> profiler;
    std::shared_ptr<utils::Config> config;
    
    // Callback management
    struct TelemetryCallbackInfo {
        int id;
        std::function<void(const std::string&, float)> callback;
    };
    std::vector<TelemetryCallbackInfo> telemetryCallbacks;
    int nextCallbackId;
    
    // Update loop
    void updateLoop();
    
    // Helper methods
    void initializeComponents();
    void shutdownComponents();
    void updateTelemetry();
    void loadDefaultConfig();
};

} // namespace neural_racer