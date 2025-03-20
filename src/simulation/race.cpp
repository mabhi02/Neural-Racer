#include "neural_racer/simulation/race.hpp"
#include "neural_racer/simulation/telemetry.hpp"
#include "neural_racer/utils/logger.hpp"
#include "neural_racer/utils/profiler.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>

namespace neural_racer {
namespace simulation {

Race::Race(std::shared_ptr<Track> track)
    : track(track), 
      state(RaceState::Initialized), 
      elapsedTime(0.0f), 
      paused(false), 
      nextCallbackId(1) {
    
    // Initialize with default configuration
    config = RaceConfig();
    weather = WeatherConditions();
    
    if (track) {
        utils::Logger::info("Race", "Race created on track: " + track->getName());
    } else {
        utils::Logger::warning("Race", "Race created without a track");
    }
}

bool Race::initialize(const RaceConfig& config) {
    // Lock mutex to prevent concurrent initialization
    std::lock_guard<std::mutex> lock(raceMutex);
    
    if (state != RaceState::Initialized) {
        utils::Logger::warning("Race", "Cannot initialize race: already initialized");
        return false;
    }
    
    utils::Logger::info("Race", "Initializing race");
    
    // Store configuration
    this->config = config;
    
    // Reset race state
    elapsedTime = 0.0f;
    paused = false;
    
    // Clear driver positions
    driverPositions.clear();
    
    // Initialize driver positions
    for (size_t i = 0; i < drivers.size(); i++) {
        DriverPosition position;
        position.driverName = drivers[i]->getName();
        position.position = static_cast<int>(i + 1); // Starting grid position
        position.lap = 0;
        position.lapTime = 0.0f;
        position.bestLapTime = 0.0f;
        position.gapToLeader = 0.0f;
        position.gapToNext = 0.0f;
        position.sector = 1;
        position.pitstops = 0;
        
        driverPositions[position.driverName] = position;
    }
    
    utils::Logger::info("Race", "Race initialized with " + std::to_string(drivers.size()) + " drivers");
    return true;
}

void Race::shutdown() {
    // Lock mutex to prevent concurrent shutdown
    std::lock_guard<std::mutex> lock(raceMutex);
    
    if (state == RaceState::Running || state == RaceState::Countdown) {
        // Stop the race
        stop();
    }
    
    // Clear driver positions
    driverPositions.clear();
    
    // Clear all callbacks
    eventCallbacks.clear();
    
    utils::Logger::info("Race", "Race shut down");
}

bool Race::addDriver(std::shared_ptr<ai::Driver> driver) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(raceMutex);
    
    if (state != RaceState::Initialized) {
        utils::Logger::warning("Race", "Cannot add driver: race already started");
        return false;
    }
    
    // Check if driver is already added
    auto it = std::find(drivers.begin(), drivers.end(), driver);
    if (it != drivers.end()) {
        utils::Logger::warning("Race", "Driver already added: " + driver->getName());
        return false;
    }
    
    // Add driver to list
    drivers.push_back(driver);
    
    // Assign track to driver
    if (track) {
        driver->setTrack(track);
    }
    
    // Initialize driver position
    DriverPosition position;
    position.driverName = driver->getName();
    position.position = static_cast<int>(drivers.size()); // Starting grid position
    position.lap = 0;
    position.lapTime = 0.0f;
    position.bestLapTime = 0.0f;
    position.gapToLeader = 0.0f;
    position.gapToNext = 0.0f;
    position.sector = 1;
    position.pitstops = 0;
    
    driverPositions[position.driverName] = position;
    
    utils::Logger::info("Race", "Driver added to race: " + driver->getName());
    return true;
}

void Race::setConfig(const RaceConfig& config) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(raceMutex);
    
    this->config = config;
    utils::Logger::info("Race", "Race configuration updated");
}

const RaceConfig& Race::getConfig() const {
    return config;
}

void Race::setWeather(const WeatherConditions& weather) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(raceMutex);
    
    this->weather = weather;
    utils::Logger::info("Race", "Weather conditions updated");
    
    // Fire weather change event
    fireEvent(RaceEventType::WeatherChanged, "", {"Weather changed"});
}

const WeatherConditions& Race::getWeather() const {
    return weather;
}

bool Race::start() {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(raceMutex);
    
    if (state == RaceState::Running || state == RaceState::Countdown) {
        utils::Logger::warning("Race", "Race already started");
        return false;
    }
    
    if (drivers.empty()) {
        utils::Logger::warning("Race", "Cannot start race: no drivers");
        return false;
    }
    
    utils::Logger::info("Race", "Starting race with " + std::to_string(drivers.size()) + " drivers");
    
    // Reset race state
    elapsedTime = 0.0f;
    paused = false;
    
    // Reset driver positions
    for (auto& pair : driverPositions) {
        pair.second.lap = 0;
        pair.second.lapTime = 0.0f;
        pair.second.bestLapTime = 0.0f;
        pair.second.gapToLeader = 0.0f;
        pair.second.gapToNext = 0.0f;
        pair.second.sector = 1;
        pair.second.pitstops = 0;
    }
    
    // Set race state to countdown or running
    if (config.standing_start) {
        state = RaceState::Countdown;
        utils::Logger::info("Race", "Race countdown started");
    } else {
        state = RaceState::Running;
        utils::Logger::info("Race", "Race started (rolling start)");
        
        // Fire race start event
        fireEvent(RaceEventType::Start, "", {"Race started", "Rolling start"});
    }
    
    return true;
}

void Race::pause() {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(raceMutex);
    
    if (state != RaceState::Running && state != RaceState::Countdown) {
        return;
    }
    
    paused = true;
    utils::Logger::info("Race", "Race paused");
    
    // Fire race pause event
    fireEvent(RaceEventType::Paused, "", {"Race paused"});
}

void Race::resume() {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(raceMutex);
    
    if (state != RaceState::Running && state != RaceState::Countdown) {
        return;
    }
    
    paused = false;
    utils::Logger::info("Race", "Race resumed");
    
    // Fire race resume event
    fireEvent(RaceEventType::Resumed, "", {"Race resumed"});
}

void Race::stop() {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(raceMutex);
    
    if (state != RaceState::Running && state != RaceState::Countdown) {
        return;
    }
    
    state = RaceState::Finished;
    utils::Logger::info("Race", "Race stopped");
    
    // Fire race finish event
    fireEvent(RaceEventType::Finish, "", {"Race stopped"});
}

void Race::update(float deltaTime) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(raceMutex);
    
    PROFILE_SCOPE("Race::update");
    
    if (state != RaceState::Running && state != RaceState::Countdown) {
        return;
    }
    
    if (paused) {
        return;
    }
    
    // Apply time scale
    deltaTime *= config.timeScale;
    
    // Update elapsed time
    elapsedTime += deltaTime;
    
    // Handle countdown
    if (state == RaceState::Countdown) {
        // Countdown is 3 seconds
        if (elapsedTime >= 3.0f) {
            state = RaceState::Running;
            elapsedTime = 0.0f;
            utils::Logger::info("Race", "Race started");
            
            // Fire race start event
            fireEvent(RaceEventType::Start, "", {"Race started", "Standing start"});
        }
        return;
    }
    
    // Check if race has reached time limit
    if (config.timeLimit > 0.0f && elapsedTime >= config.timeLimit) {
        utils::Logger::info("Race", "Race finished: time limit reached");
        
        // Set state to finished
        state = RaceState::Finished;
        
        // Fire race finish event
        fireEvent(RaceEventType::Finish, "", {"Race finished", "Time limit reached"});
        return;
    }
    
    // Update all drivers
    for (auto& driver : drivers) {
        driver->update(deltaTime);
    }
    
    // Update driver positions
    updateDriverPositions();
    
    // Check lap completion
    checkLapCompletion();
    
    // Check race completion
    checkRaceCompletion();
    
    // Update weather if enabled
    if (config.enableWeatherChanges) {
        updateWeather(deltaTime);
    }
    
    // Detect collisions between vehicles
    detectCollisions();
    
    // Update sectors
    updateSectors();
}

int Race::registerEventCallback(EventCallback callback) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(raceMutex);
    
    // Create callback info
    CallbackInfo info{nextCallbackId, callback};
    
    // Add to list
    eventCallbacks.push_back(info);
    
    // Increment callback ID for next registration
    nextCallbackId++;
    
    utils::Logger::debug("Race", "Event callback registered with ID " + std::to_string(info.id));
    return info.id;
}

void Race::unregisterEventCallback(int callbackId) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(raceMutex);
    
    // Find and remove callback
    auto it = std::find_if(eventCallbacks.begin(), eventCallbacks.end(),
                          [callbackId](const CallbackInfo& info) { return info.id == callbackId; });
    
    if (it != eventCallbacks.end()) {
        eventCallbacks.erase(it);
        utils::Logger::debug("Race", "Event callback unregistered: " + std::to_string(callbackId));
    }
}

std::vector<DriverPosition> Race::getDriverPositions() const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(raceMutex);
    
    // Convert map to vector
    std::vector<DriverPosition> positions;
    for (const auto& pair : driverPositions) {
        positions.push_back(pair.second);
    }
    
    // Sort by position
    std::sort(positions.begin(), positions.end(),
             [](const DriverPosition& a, const DriverPosition& b) {
                 return a.position < b.position;
             });
    
    return positions;
}

RaceState Race::getState() const {
    return state;
}

float Race::getElapsedTime() const {
    return elapsedTime;
}

float Race::getRemainingTime() const {
    if (config.timeLimit <= 0.0f) {
        return -1.0f; // No time limit
    }
    
    return std::max(0.0f, config.timeLimit - elapsedTime);
}

std::string Race::getLeader() const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(raceMutex);
    
    // Find driver in first position
    for (const auto& pair : driverPositions) {
        if (pair.second.position == 1) {
            return pair.first;
        }
    }
    
    return ""; // No leader found
}

int Race::getCompletedLaps() const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(raceMutex);
    
    // Find driver in first position
    for (const auto& pair : driverPositions) {
        if (pair.second.position == 1) {
            return pair.second.lap;
        }
    }
    
    return 0; // No leader found
}

bool Race::simulate(float totalTime, std::shared_ptr<Telemetry> telemetry) {
    // Check if race is ready
    if (drivers.empty()) {
        utils::Logger::warning("Race", "Cannot simulate race: no drivers");
        return false;
    }
    
    utils::Logger::info("Race", "Simulating race for " + std::to_string(totalTime) + " seconds");
    
    // Start the race
    if (!start()) {
        utils::Logger::error("Race", "Failed to start race for simulation");
        return false;
    }
    
    // Set up telemetry if provided
    if (telemetry) {
        // Record all drivers and vehicles
        for (auto& driver : drivers) {
            telemetry->recordDriver(driver, "driver." + driver->getName() + ".");
            telemetry->recordVehicle(driver->getModel().getPhysicsVehicle(), "vehicle." + driver->getName() + ".");
        }
    }
    
    // Track simulation time
    float simulatedTime = 0.0f;
    
    // Choose a suitable time step (60 fps)
    const float timeStep = 1.0f / 60.0f;
    
    // Main simulation loop
    while (simulatedTime < totalTime && state == RaceState::Running) {
        // Update race
        update(timeStep);
        
        // Update telemetry
        if (telemetry) {
            telemetry->update(shared_from_this());
        }
        
        // Update simulated time
        simulatedTime += timeStep;
    }
    
    utils::Logger::info("Race", "Simulation completed: " + std::to_string(simulatedTime) + " seconds simulated");
    
    // Print results
    auto positions = getDriverPositions();
    for (const auto& pos : positions) {
        utils::Logger::info("Race", "P" + std::to_string(pos.position) + 
                          ": " + pos.driverName + 
                          " (Lap: " + std::to_string(pos.lap) + 
                          ", Best Lap: " + std::to_string(pos.bestLapTime) + "s)");
    }
    
    return true;
}

void Race::updateDriverPositions() {
    PROFILE_SCOPE("Race::updateDriverPositions");
    
    if (drivers.empty() || !track || track->getCheckpoints().empty()) {
        return;
    }
    
    // Update driver distance from start and lap information
    for (const auto& driver : drivers) {
        // Get driver name
        const std::string& driverName = driver->getName();
        
        // Skip if not in positions map
        if (driverPositions.find(driverName) == driverPositions.end()) {
            continue;
        }
        
        // Get vehicle position
        auto vehicleState = driver->getVehicle()->getState();
        float posX = vehicleState.positionX;
        float posY = vehicleState.positionY;
        
        // Find nearest checkpoint
        float minDist = std::numeric_limits<float>::max();
        size_t nearestCheckpointIdx = 0;
        
        for (size_t i = 0; i < track->getCheckpoints().size(); i++) {
            const auto& checkpoint = track->getCheckpoints()[i];
            float dx = posX - checkpoint.x;
            float dy = posY - checkpoint.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist < minDist) {
                minDist = dist;
                nearestCheckpointIdx = i;
            }
        }
        
        // Update driver's lap time
        driverPositions[driverName].lapTime += 0.016f; // Assuming 60 fps
        
        // Calculate total distance
        int lap = driverPositions[driverName].lap;
        float totalDistance = (float)lap * track->getInfo().length;
        
        // Add distance from start of current lap
        totalDistance += track->getCheckpoints()[nearestCheckpointIdx].distanceFromStart;
        
        // Store information in driver state for sorting
        driverPositions[driverName]._totalDistance = totalDistance;
    }
    
    // Sort drivers by total distance
    std::vector<std::string> driverNames;
    for (const auto& pair : driverPositions) {
        driverNames.push_back(pair.first);
    }
    
    std::sort(driverNames.begin(), driverNames.end(),
             [this](const std::string& a, const std::string& b) {
                 return driverPositions[a]._totalDistance > driverPositions[b]._totalDistance;
             });
    
    // Update positions
    for (size_t i = 0; i < driverNames.size(); i++) {
        const std::string& driverName = driverNames[i];
        driverPositions[driverName].position = static_cast<int>(i + 1);
    }
    
    // Update gaps
    if (!driverNames.empty()) {
        // Leader has no gap
        driverPositions[driverNames[0]].gapToLeader = 0.0f;
        
        // Calculate gaps for others
        for (size_t i = 1; i < driverNames.size(); i++) {
            const std::string& driverName = driverNames[i];
            const std::string& prevDriverName = driverNames[i - 1];
            
            // Gap to leader
            float gapToLeader = driverPositions[driverNames[0]]._totalDistance - driverPositions[driverName]._totalDistance;
            // Convert distance gap to time gap (assuming 20 m/s average speed)
            driverPositions[driverName].gapToLeader = gapToLeader / 20.0f;
            
            // Gap to previous driver
            float gapToNext = driverPositions[prevDriverName]._totalDistance - driverPositions[driverName]._totalDistance;
            // Convert distance gap to time gap (assuming 20 m/s average speed)
            driverPositions[driverName].gapToNext = gapToNext / 20.0f;
        }
    }
}

void Race::checkLapCompletion() {
    PROFILE_SCOPE("Race::checkLapCompletion");
    
    if (drivers.empty() || !track || track->getCheckpoints().empty()) {
        return;
    }
    
    // Get start/finish checkpoint (first checkpoint)
    const auto& startFinish = track->getCheckpoints()[0];
    
    // Check if any driver has crossed the start/finish line
    for (const auto& driver : drivers) {
        // Get driver name
        const std::string& driverName = driver->getName();
        
        // Skip if not in positions map
        if (driverPositions.find(driverName) == driverPositions.end()) {
            continue;
        }
        
        // Get vehicle position
        auto vehicleState = driver->getVehicle()->getState();
        float posX = vehicleState.positionX;
        float posY = vehicleState.positionY;
        
        // Check if vehicle has crossed the start/finish line
        float dx = posX - startFinish.x;
        float dy = posY - startFinish.y;
        float dist = std::sqrt(dx * dx + dy * dy);
        
        // If close enough to start/finish and heading in the correct direction
        if (dist < 5.0f) {
            // Calculate dot product between checkpoint normal and vehicle velocity
            float checkpointNormalX = std::cos(startFinish.heading);
            float checkpointNormalY = std::sin(startFinish.heading);
            
            float vehicleVelocityX = std::cos(vehicleState.heading) * vehicleState.velocity;
            float vehicleVelocityY = std::sin(vehicleState.heading) * vehicleState.velocity;
            
            float dotProduct = checkpointNormalX * vehicleVelocityX + checkpointNormalY * vehicleVelocityY;
            
            // Crossed in the correct direction
            if (dotProduct > 0.0f) {
                // Store current lap time
                float lapTime = driverPositions[driverName].lapTime;
                
                // Reset lap time for next lap
                driverPositions[driverName].lapTime = 0.0f;
                
                // Increment lap counter
                driverPositions[driverName].lap++;
                
                // Update best lap time
                if (driverPositions[driverName].bestLapTime == 0.0f || 
                    lapTime < driverPositions[driverName].bestLapTime) {
                    driverPositions[driverName].bestLapTime = lapTime;
                }
                
                // Update driver's lap count
                driver->setCurrentLap(driverPositions[driverName].lap);
                
                // Fire lap completed event
                fireEvent(RaceEventType::LapCompleted, driverName, 
                         {std::to_string(driverPositions[driverName].lap),
                          std::to_string(lapTime)});
                
                utils::Logger::info("Race", driverName + " completed lap " + 
                                  std::to_string(driverPositions[driverName].lap) + 
                                  " in " + std::to_string(lapTime) + " seconds");
                
                // Update driver statistics
                driver->updateLapStats(lapTime, 0, 0); // TODO: Track mistakes and overtakes
            }
        }
    }
}

void Race::checkRaceCompletion() {
    PROFILE_SCOPE("Race::checkRaceCompletion");
    
    if (drivers.empty() || config.laps <= 0) {
        return;
    }
    
    // Check if leader has completed all laps
    for (const auto& pair : driverPositions) {
        if (pair.second.position == 1 && pair.second.lap >= config.laps) {
            utils::Logger::info("Race", "Race finished: all laps completed");
            
            // Set state to finished
            state = RaceState::Finished;
            
            // Fire race finish event
            fireEvent(RaceEventType::Finish, pair.first, {"Race finished", "All laps completed"});
            
            // Fire checkered flag event
            fireEvent(RaceEventType::FlagChequered, "", {"Checkered flag"});
            
            break;
        }
    }
}

void Race::updateWeather(float deltaTime) {
    PROFILE_SCOPE("Race::updateWeather");
    
    // Simplified weather model
    // In a real implementation, this would be more complex
    
    // Small random changes to weather over time
    static std::mt19937 rng(static_cast<unsigned int>(std::time(nullptr)));
    std::uniform_real_distribution<float> dist(-0.1f, 0.1f);
    
    // Update weather conditions
    bool weatherChanged = false;
    
    // Temperature changes
    weather.airTemperature += dist(rng) * deltaTime;
    weatherChanged = true;
    
    // Wind changes
    weather.windSpeed += dist(rng) * deltaTime;
    weather.windSpeed = std::max(0.0f, weather.windSpeed);
    weather.windDirection += dist(rng) * deltaTime;
    weatherChanged = true;
    
    // Precipitation changes
    if (dist(rng) > 0.9f) {
        // Small chance of significant weather change
        weather.precipitationIntensity += dist(rng) * 5.0f * deltaTime;
        weather.precipitationIntensity = std::max(0.0f, std::min(1.0f, weather.precipitationIntensity));
        
        // Update weather type based on precipitation
        if (weather.precipitationIntensity > 0.7f) {
            weather.type = WeatherConditions::Type::HeavyRain;
        } else if (weather.precipitationIntensity > 0.3f) {
            weather.type = WeatherConditions::Type::LightRain;
        } else if (weather.precipitationIntensity > 0.1f) {
            weather.type = WeatherConditions::Type::Cloudy;
        } else {
            weather.type = WeatherConditions::Type::Clear;
        }
        
        weatherChanged = true;
    }
    
    // If weather changed significantly, fire event
    if (weatherChanged) {
        // Fire weather changed event
        fireEvent(RaceEventType::WeatherChanged, "", 
                 {std::to_string(static_cast<int>(weather.type)),
                  std::to_string(weather.precipitationIntensity)});
    }
}

void Race::detectCollisions() {
    PROFILE_SCOPE("Race::detectCollisions");
    
    // Simplified collision detection between vehicles
    // In a real implementation, this would use a proper physics engine
    
    // Check all pairs of drivers
    for (size_t i = 0; i < drivers.size(); i++) {
        for (size_t j = i + 1; j < drivers.size(); j++) {
            auto& driver1 = drivers[i];
            auto& driver2 = drivers[j];
            
            // Get vehicle positions
            auto state1 = driver1->getVehicle()->getState();
            auto state2 = driver2->getVehicle()->getState();
            
            // Calculate distance between vehicles
            float dx = state1.positionX - state2.positionX;
            float dy = state1.positionY - state2.positionY;
            float distSq = dx * dx + dy * dy;
            
            // Check collision (assuming vehicle radius of 1.5m)
            if (distSq < 9.0f) { // 3.0m collision distance squared
                // Fire collision event
                fireEvent(RaceEventType::Collision, driver1->getName(), {driver2->getName()});
                
                utils::Logger::info("Race", "Collision between " + driver1->getName() + 
                                  " and " + driver2->getName());
            }
        }
    }
}

void Race::updateSectors() {
    PROFILE_SCOPE("Race::updateSectors");
    
    if (drivers.empty() || !track || track->getSectors().empty()) {
        return;
    }
    
    // Check for sector changes
    for (const auto& driver : drivers) {
        // Get driver name
        const std::string& driverName = driver->getName();
        
        // Skip if not in positions map
        if (driverPositions.find(driverName) == driverPositions.end()) {
            continue;
        }
        
        // Get vehicle position
        auto vehicleState = driver->getVehicle()->getState();
        float posX = vehicleState.positionX;
        float posY = vehicleState.positionY;
        
        // Find current position on track
        auto nearestWaypoint = track->findNearestWaypoint(posX, posY);
        
        // Calculate distance from start of lap
        float distanceFromStart = nearestWaypoint.distanceFromStart;
        
        // Calculate sector based on this distance
        int currentSector = track->getSectorAtDistance(distanceFromStart);
        
        // If sector changed
        if (currentSector > 0 && currentSector != driverPositions[driverName].sector) {
            // Update sector
            driverPositions[driverName].sector = currentSector;
            
            // Fire sector changed event
            fireEvent(RaceEventType::SectorChanged, driverName, 
                     {std::to_string(currentSector),
                      track->getSectors()[currentSector - 1].name});
            
            utils::Logger::debug("Race", driverName + " entered sector " + 
                               std::to_string(currentSector));
        }
    }
}

void Race::fireEvent(RaceEventType type, const std::string& driverName, 
                    const std::vector<std::string>& data) {
    // Create event
    RaceEvent event;
    event.type = type;
    event.timeStamp = elapsedTime;
    event.driverName = driverName;
    event.data = data;
    
    // Notify all registered callbacks
    for (const auto& callbackInfo : eventCallbacks) {
        callbackInfo.callback(event);
    }
}

int Race::findDriverPosition(const std::string& driverName) {
    // Find driver's position
    auto it = driverPositions.find(driverName);
    if (it != driverPositions.end()) {
        return it->second.position;
    }
    
    return 0; // Not found
}

} // namespace simulation
} // namespace neural_racer