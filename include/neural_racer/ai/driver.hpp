#pragma once

#define _USE_MATH_DEFINES  // For M_PI and other math constants
#include <memory>
#include <string>
#include <vector>
#include <cmath>   // For math functions
#include "inference.hpp"
#include "../physics/vehicle.hpp"
#include "../simulation/track.hpp"

namespace neural_racer {
namespace ai {

/**
 * @brief Driver behavior parameters
 */
struct DriverParameters {
    float aggression;         ///< Aggression factor (0.0-1.0)
    float consistency;        ///< Consistency factor (0.0-1.0)
    float adaptability;       ///< Adaptability to changing conditions (0.0-1.0)
    float defensiveness;      ///< Defensive driving tendency (0.0-1.0)
    float tireManagement;     ///< Tire wear management (0.0-1.0)
    float fuelManagement;     ///< Fuel consumption management (0.0-1.0)
    float wetSkill;           ///< Skill in wet conditions (0.0-1.0)
    float racecraft;          ///< Ability to race in traffic (0.0-1.0)
    
    DriverParameters() :
        aggression(0.5f),
        consistency(0.5f),
        adaptability(0.5f),
        defensiveness(0.5f),
        tireManagement(0.5f),
        fuelManagement(0.5f),
        wetSkill(0.5f),
        racecraft(0.5f) {}
};

/**
 * @brief Driver statistics
 */
struct DriverStats {
    int racesCompleted;       ///< Number of races completed
    int racesWon;             ///< Number of races won
    int podiumFinishes;       ///< Number of podium finishes
    int fastestLaps;          ///< Number of fastest laps
    float bestLapTime;        ///< Best lap time in seconds
    float avgLapTime;         ///< Average lap time in seconds
    int overtakesMade;        ///< Number of successful overtakes
    int mistakesMade;         ///< Number of driving mistakes
    
    DriverStats() :
        racesCompleted(0),
        racesWon(0),
        podiumFinishes(0),
        fastestLaps(0),
        bestLapTime(0.0f),
        avgLapTime(0.0f),
        overtakesMade(0),
        mistakesMade(0) {}
};

/**
 * @brief Racing line point
 */
struct RacingLinePoint {
    float x;                  ///< X position
    float y;                  ///< Y position
    float targetSpeed;        ///< Target speed at this point
    float brakePoint;         ///< Braking point (0.0-1.0)
    float apex;               ///< Apex point (0.0-1.0)
    float exitPoint;          ///< Exit point (0.0-1.0)
};

/**
 * @brief Race strategy
 */
enum class Strategy {
    Aggressive,              ///< Aggressive strategy (faster pace, higher tire wear)
    Balanced,                ///< Balanced strategy (moderate pace and tire wear)
    Conservative,            ///< Conservative strategy (fuel/tire saving)
    WetWeather,              ///< Strategy for wet conditions
    Qualifying               ///< Qualifying strategy (maximum pace)
};

/**
 * @brief AI driver that controls a vehicle
 * 
 * This class implements an AI driver that uses a neural network model
 * to control a vehicle in a racing simulation.
 */
class Driver : public std::enable_shared_from_this<Driver> {
public:
    /**
     * @brief Construct a new Driver
     * 
     * @param driverModel Driver AI model
     * @param vehicle Vehicle to control
     * @param name Driver name
     */
    Driver(std::shared_ptr<DriverModel> driverModel, 
           std::shared_ptr<physics::Vehicle> vehicle,
           const std::string& name = "AI Driver");
    
    /**
     * @brief Initialize the driver
     * 
     * @return true if initialization was successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Update the driver's control of the vehicle
     * 
     * @param deltaTime Time step in seconds
     */
    void update(float deltaTime);
    
    /**
     * @brief Set the track for the driver
     * 
     * @param track Track to race on
     */
    void setTrack(std::shared_ptr<simulation::Track> track);
    
    /**
     * @brief Set driver parameters
     * 
     * @param parameters Driver behavior parameters
     */
    void setParameters(const DriverParameters& parameters);
    
    /**
     * @brief Get driver parameters
     * 
     * @return const DriverParameters& Driver behavior parameters
     */
    const DriverParameters& getParameters() const;
    
    /**
     * @brief Get driver statistics
     * 
     * @return const DriverStats& Driver statistics
     */
    const DriverStats& getStats() const;
    
    /**
     * @brief Get driver model
     * 
     * @return const DriverModel& Driver AI model
     */
    const DriverModel& getModel() const;
    
    /**
     * @brief Get driver name
     * 
     * @return const std::string& Driver name
     */
    const std::string& getName() const;
    
    /**
     * @brief Get driver's vehicle
     * 
     * @return std::shared_ptr<physics::Vehicle> The driver's vehicle
     */
    std::shared_ptr<physics::Vehicle> getVehicle() const {
        return vehicle;
    }
    
    /**
     * @brief Set race strategy
     * 
     * @param strategy Race strategy to use
     */
    void setStrategy(Strategy strategy);
    
    /**
     * @brief Get current race strategy
     * 
     * @return Strategy Current race strategy
     */
    Strategy getStrategy() const;
    
    /**
     * @brief Set current racing line
     * 
     * @param racingLine Vector of racing line points
     */
    void setRacingLine(const std::vector<RacingLinePoint>& racingLine);
    
    /**
     * @brief Get current racing line
     * 
     * @return const std::vector<RacingLinePoint>& Current racing line
     */
    const std::vector<RacingLinePoint>& getRacingLine() const;
    
    /**
     * @brief Set current lap
     * 
     * @param lap Current lap number
     */
    void setCurrentLap(int lap);
    
    /**
     * @brief Get current lap
     * 
     * @return int Current lap number
     */
    int getCurrentLap() const;
    
    /**
     * @brief Update statistics after completing a lap
     * 
     * @param lapTime Lap time in seconds
     * @param mistakes Number of mistakes made
     * @param overtakes Number of overtakes made
     */
    void updateLapStats(float lapTime, int mistakes, int overtakes);

private:
    std::string name;
    std::shared_ptr<DriverModel> driverModel;
    std::shared_ptr<physics::Vehicle> vehicle;
    std::shared_ptr<simulation::Track> track;
    DriverParameters parameters;
    DriverStats stats;
    
    Strategy currentStrategy;
    std::vector<RacingLinePoint> racingLine;
    int currentLap;
    int currentRacingLineIndex;
    
    // Sensor data for AI
    DriverModel::SensorInputs sensorInputs;
    
    // Update sensor data based on vehicle and track state
    void updateSensorInputs();
    
    // Calculate racing line based on track
    void calculateRacingLine();
    
    // Find closest racing line point
    size_t findClosestRacingLinePoint();
    
    // Adapt behavior based on conditions
    void adaptToConditions();
    
    // Process driver model outputs and apply to vehicle
    void processControls(const DriverModel::ControlOutputs& controls);
};

/**
 * @brief Factory for creating AI drivers
 */
class DriverFactory {
public:
    /**
     * @brief Create a driver with specified skill level
     * 
     * @param skillLevel Skill level (0.0-1.0, higher is more skilled)
     * @param vehicle Vehicle to control
     * @param gpuAccelerator Optional GPU accelerator for hardware acceleration
     * @param name Driver name
     * @return std::shared_ptr<Driver> Shared pointer to the created driver
     */
    static std::shared_ptr<Driver> createWithSkillLevel(
        float skillLevel,
        std::shared_ptr<physics::Vehicle> vehicle,
        std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator = nullptr,
        const std::string& name = "AI Driver");
    
    /**
     * @brief Create a driver with a specific personality profile
     * 
     * @param parameters Driver behavior parameters
     * @param vehicle Vehicle to control
     * @param gpuAccelerator Optional GPU accelerator for hardware acceleration
     * @param name Driver name
     * @return std::shared_ptr<Driver> Shared pointer to the created driver
     */
    static std::shared_ptr<Driver> createWithPersonality(
        const DriverParameters& parameters,
        std::shared_ptr<physics::Vehicle> vehicle,
        std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator = nullptr,
        const std::string& name = "AI Driver");
    
    /**
     * @brief Create a driver based on a real driver's style
     * 
     * @param driverStyle Real driver style identifier (e.g., "aggressive", "smooth")
     * @param vehicle Vehicle to control
     * @param gpuAccelerator Optional GPU accelerator for hardware acceleration
     * @param name Driver name
     * @return std::shared_ptr<Driver> Shared pointer to the created driver
     */
    static std::shared_ptr<Driver> createFromRealDriverStyle(
        const std::string& driverStyle,
        std::shared_ptr<physics::Vehicle> vehicle,
        std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator = nullptr,
        const std::string& name = "AI Driver");
};

} // namespace ai
} // namespace neural_racer