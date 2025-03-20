#include "neural_racer/ai/driver.hpp"
#include "neural_racer/utils/logger.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace neural_racer {
namespace ai {

Driver::Driver(std::shared_ptr<DriverModel> driverModel, 
               std::shared_ptr<physics::Vehicle> vehicle,
               const std::string& name)
    : name(name), driverModel(driverModel), vehicle(vehicle), currentLap(1), currentRacingLineIndex(0) {
    
    // Initialize with default parameters
    parameters = DriverParameters();
    stats = DriverStats();
    currentStrategy = Strategy::Balanced;
    
    utils::Logger::info("Driver", "Created AI driver: " + name);
}

bool Driver::initialize() {
    if (!driverModel) {
        utils::Logger::error("Driver", "Driver model not set");
        return false;
    }
    
    if (!vehicle) {
        utils::Logger::error("Driver", "Vehicle not set");
        return false;
    }
    
    // Initialize driver model
    if (!driverModel->initialize()) {
        utils::Logger::error("Driver", "Failed to initialize driver model");
        return false;
    }
    
    utils::Logger::info("Driver", name + " initialized successfully");
    return true;
}

void Driver::update(float deltaTime) {
    if (!driverModel || !vehicle) {
        return;
    }
    
    // Update sensor inputs based on vehicle and track state
    updateSensorInputs();
    
    // Process sensor inputs and generate control outputs
    auto controls = driverModel->process(sensorInputs);
    
    // Apply controls to vehicle
    processControls(controls);
    
    // Adapt behavior based on conditions
    adaptToConditions();
    
    // Find closest racing line point
    currentRacingLineIndex = findClosestRacingLinePoint();
}

void Driver::setTrack(std::shared_ptr<simulation::Track> track) {
    this->track = track;
    
    if (track) {
        // Calculate racing line based on track
        calculateRacingLine();
        utils::Logger::info("Driver", name + " assigned to track: " + track->getName());
    }
}

void Driver::setParameters(const DriverParameters& parameters) {
    this->parameters = parameters;
    
    // Apply parameters to driver model
    if (driverModel) {
        driverModel->setAggressivenessFactor(parameters.aggression);
        driverModel->enableAdaptiveBehavior(parameters.adaptability > 0.5f);
    }
    
    utils::Logger::info("Driver", name + " parameters updated");
}

const DriverParameters& Driver::getParameters() const {
    return parameters;
}

const DriverStats& Driver::getStats() const {
    return stats;
}

const DriverModel& Driver::getModel() const {
    return *driverModel;
}

const std::string& Driver::getName() const {
    return name;
}

void Driver::setStrategy(Strategy strategy) {
    this->currentStrategy = strategy;
    
    // Apply strategy-specific parameters
    switch (strategy) {
        case Strategy::Aggressive:
            parameters.aggression = 0.9f;
            parameters.tireManagement = 0.3f;
            parameters.fuelManagement = 0.3f;
            break;
            
        case Strategy::Balanced:
            parameters.aggression = 0.5f;
            parameters.tireManagement = 0.5f;
            parameters.fuelManagement = 0.5f;
            break;
            
        case Strategy::Conservative:
            parameters.aggression = 0.3f;
            parameters.tireManagement = 0.8f;
            parameters.fuelManagement = 0.8f;
            break;
            
        case Strategy::WetWeather:
            parameters.aggression = 0.3f;
            parameters.wetSkill = 0.9f;
            break;
            
        case Strategy::Qualifying:
            parameters.aggression = 1.0f;
            parameters.tireManagement = 0.1f;
            parameters.fuelManagement = 0.1f;
            break;
    }
    
    // Apply parameters to driver model
    if (driverModel) {
        driverModel->setAggressivenessFactor(parameters.aggression);
    }
    
    utils::Logger::info("Driver", name + " switched to " + 
                      std::to_string(static_cast<int>(strategy)) + " strategy");
}

Strategy Driver::getStrategy() const {
    return currentStrategy;
}

void Driver::setRacingLine(const std::vector<RacingLinePoint>& racingLine) {
    this->racingLine = racingLine;
    utils::Logger::debug("Driver", name + " racing line updated with " + 
                       std::to_string(racingLine.size()) + " points");
}

const std::vector<RacingLinePoint>& Driver::getRacingLine() const {
    return racingLine;
}

void Driver::setCurrentLap(int lap) {
    currentLap = lap;
}

int Driver::getCurrentLap() const {
    return currentLap;
}

void Driver::updateLapStats(float lapTime, int mistakes, int overtakes) {
    // Update lap statistics
    stats.racesCompleted++;
    
    // Update best lap time
    if (stats.bestLapTime == 0.0f || lapTime < stats.bestLapTime) {
        stats.bestLapTime = lapTime;
    }
    
    // Update average lap time
    if (stats.avgLapTime == 0.0f) {
        stats.avgLapTime = lapTime;
    } else {
        stats.avgLapTime = (stats.avgLapTime * (stats.racesCompleted - 1) + lapTime) / stats.racesCompleted;
    }
    
    // Update mistakes and overtakes
    stats.mistakesMade += mistakes;
    stats.overtakesMade += overtakes;
    
    utils::Logger::info("Driver", name + " completed lap in " + 
                      std::to_string(lapTime) + " seconds, with " + 
                      std::to_string(mistakes) + " mistakes and " + 
                      std::to_string(overtakes) + " overtakes");
}

void Driver::updateSensorInputs() {
    if (!vehicle || !track) {
        return;
    }
    
    // Get vehicle state
    auto vehicleState = vehicle->getState();
    
    // Basic inputs
    sensorInputs.speed = vehicleState.velocity;
    sensorInputs.acceleration = vehicleState.acceleration;
    
    // Track position calculation requires finding nearest point on track
    // and determining lateral offset
    if (!track->getWaypoints().empty()) {
        auto nearestWaypoint = track->findNearestWaypoint(
            vehicleState.positionX, vehicleState.positionY);
        
        // Calculate track position (-1 to 1, where 0 is center)
        // This is a simplified calculation
        float dx = vehicleState.positionX - nearestWaypoint.x;
        float dy = vehicleState.positionY - nearestWaypoint.y;
        
        // Project the vector (dx, dy) onto the track normal
        float nx = -std::sin(nearestWaypoint.heading);
        float ny = std::cos(nearestWaypoint.heading);
        
        float trackOffset = dx * nx + dy * ny;
        sensorInputs.trackPosition = (2.0f * trackOffset) / nearestWaypoint.width;
        
        // Clamp to valid range
        sensorInputs.trackPosition = std::max(-1.0f, std::min(1.0f, sensorInputs.trackPosition));
        
        // Calculate track angle (difference between vehicle heading and track heading)
        sensorInputs.trackAngle = vehicleState.heading - nearestWaypoint.heading;
        
        // Normalize to -π to π
        while (sensorInputs.trackAngle > M_PI) sensorInputs.trackAngle -= 2.0f * M_PI;
        while (sensorInputs.trackAngle < -M_PI) sensorInputs.trackAngle += 2.0f * M_PI;
        
        // Get upcoming track curvature
        // For this, we need to look ahead a bit
        size_t aheadIndex = 0;
        for (size_t i = 0; i < track->getWaypoints().size(); i++) {
            if (track->getWaypoints()[i].distanceFromStart > nearestWaypoint.distanceFromStart + 20.0f) {
                aheadIndex = i;
                break;
            }
        }
        
        if (aheadIndex > 0 && aheadIndex < track->getWaypoints().size()) {
            // Get segment curvature
            int segmentIdx = track->getWaypoints()[aheadIndex].segmentIndex;
            if (segmentIdx >= 0 && segmentIdx < static_cast<int>(track->getSegments().size())) {
                sensorInputs.trackCurvature = track->getSegments()[segmentIdx].curvature;
                
                // Apply sign based on turn direction
                if (track->getSegments()[segmentIdx].type == simulation::SegmentType::RightTurn) {
                    sensorInputs.trackCurvature = -sensorInputs.trackCurvature;
                }
            }
        }
    }
    
    // Range sensors (simplified simulation)
    // These would be measurements of distance to track edges or other vehicles
    sensorInputs.rangeSensors.clear();
    
    // Front sensor
    sensorInputs.rangeSensors.push_back(100.0f); // 100m (or until track edge/vehicle)
    
    // Front-left sensor
    sensorInputs.rangeSensors.push_back(100.0f);
    
    // Front-right sensor
    sensorInputs.rangeSensors.push_back(100.0f);
    
    // Left sensor
    sensorInputs.rangeSensors.push_back(100.0f);
    
    // Right sensor
    sensorInputs.rangeSensors.push_back(100.0f);
    
    // In a full implementation, we would raytrace to find actual distances
    // to track edges and other vehicles
}

void Driver::calculateRacingLine() {
    if (!track) {
        return;
    }
    
    // Get optimal racing line from track
    auto optimalLine = track->calculateRacingLine();
    
    // Convert to racing line points
    racingLine.clear();
    for (const auto& wp : optimalLine) {
        RacingLinePoint point;
        point.x = wp.x;
        point.y = wp.y;
        point.targetSpeed = wp.optimalSpeed;
        
        // These would be calculated more accurately in a full implementation
        point.brakePoint = 0.0f;
        point.apex = 0.0f;
        point.exitPoint = 0.0f;
        
        racingLine.push_back(point);
    }
    
    utils::Logger::debug("Driver", name + " calculated racing line with " + 
                       std::to_string(racingLine.size()) + " points");
}

size_t Driver::findClosestRacingLinePoint() {
    if (racingLine.empty() || !vehicle) {
        return 0;
    }
    
    auto state = vehicle->getState();
    
    // Find closest point on racing line
    float minDist = std::numeric_limits<float>::max();
    size_t closestIdx = 0;
    
    for (size_t i = 0; i < racingLine.size(); i++) {
        float dx = state.positionX - racingLine[i].x;
        float dy = state.positionY - racingLine[i].y;
        float dist = dx * dx + dy * dy; // Squared distance is enough for comparison
        
        if (dist < minDist) {
            minDist = dist;
            closestIdx = i;
        }
    }
    
    return closestIdx;
}

void Driver::adaptToConditions() {
    // Adapt behavior based on various conditions
    
    // Example: Rain adaptation
    if (track) {
        auto weather = track->getEnvironment();
        if (weather.rainProbability > 0.5f) {
            // Adapt to wet conditions
            float wetAdaptation = parameters.wetSkill * weather.rainProbability;
            
            // Apply wet skill to lower aggression and increase consistency
            parameters.aggression *= (1.0f - 0.5f * wetAdaptation);
            parameters.consistency *= (1.0f + 0.3f * wetAdaptation);
            
            if (driverModel) {
                driverModel->setAggressivenessFactor(parameters.aggression);
            }
        }
    }
    
    // Example: Tire wear adaptation
    // This would come from the vehicle's tire model in a full implementation
    float tireWear = 0.0f; // 0 = new, 1 = worn out
    
    if (tireWear > 0.7f) {
        // Reduce aggression with worn tires
        float tireAdaptation = parameters.tireManagement * (tireWear - 0.7f) / 0.3f;
        parameters.aggression *= (1.0f - tireAdaptation);
        
        if (driverModel) {
            driverModel->setAggressivenessFactor(parameters.aggression);
        }
    }
}

void Driver::processControls(const DriverModel::ControlOutputs& controls) {
    if (!vehicle) {
        return;
    }
    
    // Apply controls to vehicle
    vehicle->setControls(controls.throttle, controls.brake, controls.steering);
    
    // Log control values at debug level
    utils::Logger::trace("Driver", name + " controls - Throttle: " + 
                       std::to_string(controls.throttle) + ", Brake: " + 
                       std::to_string(controls.brake) + ", Steering: " + 
                       std::to_string(controls.steering));
}

// DriverFactory implementation
std::shared_ptr<Driver> DriverFactory::createWithSkillLevel(
    float skillLevel,
    std::shared_ptr<physics::Vehicle> vehicle,
    std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator,
    const std::string& name) {
    
    // Create driver model based on skill level
    auto driverModel = DriverModelFactory::createWithSkillLevel(skillLevel, gpuAccelerator);
    
    // Create driver
    auto driver = std::make_shared<Driver>(driverModel, vehicle, name);
    
    // Set driver parameters based on skill level
    DriverParameters params;
    params.aggression = 0.3f + skillLevel * 0.7f;
    params.consistency = 0.3f + skillLevel * 0.7f;
    params.adaptability = 0.3f + skillLevel * 0.7f;
    params.defensiveness = 0.3f + skillLevel * 0.4f; // High skill doesn't necessarily mean less defensive
    params.tireManagement = 0.3f + skillLevel * 0.7f;
    params.fuelManagement = 0.3f + skillLevel * 0.7f;
    params.wetSkill = 0.3f + skillLevel * 0.7f;
    params.racecraft = 0.3f + skillLevel * 0.7f;
    
    driver->setParameters(params);
    
    // Initialize driver
    driver->initialize();
    
    return driver;
}

std::shared_ptr<Driver> DriverFactory::createWithPersonality(
    const DriverParameters& parameters,
    std::shared_ptr<physics::Vehicle> vehicle,
    std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator,
    const std::string& name) {
    
    // Calculate average skill level from parameters
    float skillLevel = (parameters.consistency + 
                       parameters.adaptability + 
                       parameters.tireManagement + 
                       parameters.fuelManagement + 
                       parameters.wetSkill + 
                       parameters.racecraft) / 6.0f;
    
    // Create driver model based on skill level
    auto driverModel = DriverModelFactory::createWithSkillLevel(skillLevel, gpuAccelerator);
    
    // Set aggressiveness
    driverModel->setAggressivenessFactor(parameters.aggression);
    
    // Create driver
    auto driver = std::make_shared<Driver>(driverModel, vehicle, name);
    
    // Set parameters
    driver->setParameters(parameters);
    
    // Initialize driver
    driver->initialize();
    
    return driver;
}

std::shared_ptr<Driver> DriverFactory::createFromRealDriverStyle(
    const std::string& driverStyle,
    std::shared_ptr<physics::Vehicle> vehicle,
    std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator,
    const std::string& name) {
    
    // Create preset parameter profiles for different driving styles
    DriverParameters params;
    
    if (driverStyle == "aggressive") {
        params.aggression = 0.9f;
        params.consistency = 0.7f;
        params.adaptability = 0.6f;
        params.defensiveness = 0.3f;
        params.tireManagement = 0.4f;
        params.fuelManagement = 0.4f;
        params.wetSkill = 0.6f;
        params.racecraft = 0.8f;
    } 
    else if (driverStyle == "smooth") {
        params.aggression = 0.4f;
        params.consistency = 0.9f;
        params.adaptability = 0.7f;
        params.defensiveness = 0.6f;
        params.tireManagement = 0.9f;
        params.fuelManagement = 0.9f;
        params.wetSkill = 0.8f;
        params.racecraft = 0.7f;
    } 
    else if (driverStyle == "defensive") {
        params.aggression = 0.3f;
        params.consistency = 0.8f;
        params.adaptability = 0.6f;
        params.defensiveness = 0.9f;
        params.tireManagement = 0.7f;
        params.fuelManagement = 0.7f;
        params.wetSkill = 0.6f;
        params.racecraft = 0.6f;
    } 
    else if (driverStyle == "balanced") {
        params.aggression = 0.6f;
        params.consistency = 0.7f;
        params.adaptability = 0.7f;
        params.defensiveness = 0.6f;
        params.tireManagement = 0.7f;
        params.fuelManagement = 0.7f;
        params.wetSkill = 0.7f;
        params.racecraft = 0.7f;
    }
    else if (driverStyle == "rookie") {
        params.aggression = 0.5f;
        params.consistency = 0.4f;
        params.adaptability = 0.3f;
        params.defensiveness = 0.5f;
        params.tireManagement = 0.3f;
        params.fuelManagement = 0.3f;
        params.wetSkill = 0.3f;
        params.racecraft = 0.3f;
    }
    else {
        // Default to balanced
        params.aggression = 0.6f;
        params.consistency = 0.7f;
        params.adaptability = 0.7f;
        params.defensiveness = 0.6f;
        params.tireManagement = 0.7f;
        params.fuelManagement = 0.7f;
        params.wetSkill = 0.7f;
        params.racecraft = 0.7f;
    }
    
    // Create driver with the selected personality
    return createWithPersonality(params, vehicle, gpuAccelerator, name);
}

} // namespace ai
} // namespace neural_racer