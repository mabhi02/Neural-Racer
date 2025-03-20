#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <iomanip>

#include "neural_racer/hardware/device_driver.hpp"
#include "neural_racer/ai/driver.hpp"
#include "neural_racer/physics/vehicle.hpp"
#include "neural_racer/physics/engine.hpp"
#include "neural_racer/simulation/track.hpp"
#include "neural_racer/simulation/race.hpp"
#include "neural_racer/simulation/telemetry.hpp"
#include "neural_racer/utils/logger.hpp"
#include "neural_racer/utils/profiler.hpp"

using namespace neural_racer;

// Callback function for race events
void onRaceEvent(const simulation::RaceEvent& event) {
    switch (event.type) {
        case simulation::RaceEventType::Start:
            std::cout << "Race started!" << std::endl;
            break;
            
        case simulation::RaceEventType::Finish:
            std::cout << "Race finished!" << std::endl;
            break;
            
        case simulation::RaceEventType::LapCompleted:
            std::cout << "Lap completed by " << event.driverName;
            if (!event.data.empty()) {
                std::cout << " (Lap " << event.data[0] << ", Time: " << event.data[1] << "s)";
            }
            std::cout << std::endl;
            break;
            
        case simulation::RaceEventType::Collision:
            std::cout << "Collision involving " << event.driverName;
            if (!event.data.empty()) {
                std::cout << " and " << event.data[0];
            }
            std::cout << std::endl;
            break;
            
        default:
            // Other events
            break;
    }
}

// Telemetry listener
class TelemetryDisplay : public simulation::TelemetryListener {
public:
    void onTelemetryData(const std::string& channelName, const simulation::TelemetrySample& sample) override {
        // Filter to only display certain channels
        if (channelName == "vehicle.leader.velocity" || 
            channelName == "vehicle.leader.engine_rpm" ||
            channelName == "vehicle.leader.position.x" ||
            channelName == "vehicle.leader.position.y") {
            
            float value = 0.0f;
            
            // Extract value based on channel type
            if (channelName.find("velocity") != std::string::npos || 
                channelName.find("engine_rpm") != std::string::npos || 
                channelName.find("position.x") != std::string::npos || 
                channelName.find("position.y") != std::string::npos) {
                value = sample.value.floatValue;
            } else {
                // Other channels not handled in this example
                return;
            }
            
            // Display value
            std::cout << "\r" << channelName << ": " << std::fixed << std::setprecision(2) << value << "             ";
            std::cout.flush();
        }
    }
    
    void onLapCompleted(const std::string& driverName, float lapTime, int lapNumber) override {
        std::cout << "\nDriver " << driverName << " completed lap " << lapNumber 
                  << " in " << std::fixed << std::setprecision(3) << lapTime << "s" << std::endl;
    }
};

int main(int argc, char* argv[]) {
    // Initialize logging
    utils::Logger::initialize();
    utils::Logger::setLogLevel(utils::LogLevel::Info);
    
    // Initialize profiling
    utils::Profiler::initialize(true);
    
    try {
        std::cout << "Neural Racer - Basic Race Example" << std::endl;
        std::cout << "=================================" << std::endl;
        
        // Create GPU accelerator
        auto gpuAccelerator = std::make_shared<hardware::GPUAccelerator>();
        gpuAccelerator->initialize();
        
        // Create physics engine
        auto physicsEngine = std::make_shared<physics::Engine>(gpuAccelerator);
        physicsEngine->initialize();
        
        // Create track (using a preset)
        auto track = simulation::TrackFactory::createFromPreset("Monza");
        
        // Create vehicles
        physics::VehicleSpec vehicleSpec1;
        vehicleSpec1.name = "Race Car 1";
        vehicleSpec1.mass = 720.0f;  // kg
        vehicleSpec1.power = 735.0f;  // kW (~ 1000 HP)
        vehicleSpec1.drag = 0.35f;
        vehicleSpec1.frontGrip = 1.5f;
        vehicleSpec1.rearGrip = 1.4f;
        auto vehicle1 = std::make_shared<physics::Vehicle>(vehicleSpec1, physicsEngine);
        
        physics::VehicleSpec vehicleSpec2;
        vehicleSpec2.name = "Race Car 2";
        vehicleSpec2.mass = 740.0f;  // kg
        vehicleSpec2.power = 730.0f;  // kW
        vehicleSpec2.drag = 0.36f;
        vehicleSpec2.frontGrip = 1.45f;
        vehicleSpec2.rearGrip = 1.35f;
        auto vehicle2 = std::make_shared<physics::Vehicle>(vehicleSpec2, physicsEngine);
        
        // Create AI drivers
        auto driver1 = ai::DriverFactory::createWithSkillLevel(0.85f, vehicle1, gpuAccelerator, "Driver 1");
        auto driver2 = ai::DriverFactory::createWithSkillLevel(0.80f, vehicle2, gpuAccelerator, "Driver 2");
        
        // Create race simulation
        auto race = std::make_shared<simulation::Race>(track);
        
        // Register race event callback
        race->registerEventCallback(onRaceEvent);
        
        // Create and configure race
        simulation::RaceConfig raceConfig;
        raceConfig.laps = 3;
        raceConfig.standing_start = true;
        raceConfig.enableWeatherChanges = true;
        raceConfig.enableTireWear = true;
        raceConfig.enableFuelConsumption = true;
        race->setConfig(raceConfig);
        
        // Add drivers to race
        race->addDriver(driver1);
        race->addDriver(driver2);
        
        // Create telemetry system
        auto telemetry = std::make_shared<simulation::Telemetry>();
        telemetry->initialize();
        
        // Register telemetry listener
        TelemetryDisplay telemetryDisplay;
        telemetry->registerListener(&telemetryDisplay);
        
        // Record telemetry for all vehicles and drivers
        telemetry->recordVehicle(vehicle1, "vehicle.driver1.");
        telemetry->recordVehicle(vehicle2, "vehicle.driver2.");
        telemetry->recordDriver(driver1, "driver.driver1.");
        telemetry->recordDriver(driver2, "driver.driver2.");
        
        // Start race
        std::cout << "Starting race..." << std::endl;
        race->start();
        
        // Simulation loop
        const float timeStep = 1.0f / 60.0f;  // 60 FPS
        float elapsedTime = 0.0f;
        int lastDisplayedSecond = -1;
        
        while (race->getState() == simulation::RaceState::Running ||
               race->getState() == simulation::RaceState::Countdown) {
            
            // Profile the update
            utils::ScopedTimer timer("RaceUpdate");
            
            // Update race simulation
            race->update(timeStep);
            
            // Update telemetry
            telemetry->update(race);
            
            // Update elapsed time
            elapsedTime += timeStep;
            
            // Display driver positions every second
            int currentSecond = static_cast<int>(elapsedTime);
            if (currentSecond != lastDisplayedSecond) {
                lastDisplayedSecond = currentSecond;
                
                std::cout << "\nDriver positions:" << std::endl;
                auto positions = race->getDriverPositions();
                for (const auto& pos : positions) {
                    std::cout << "P" << pos.position << ": " << pos.driverName
                              << " (Lap " << pos.lap << ", Gap: " << std::fixed << std::setprecision(2) << pos.gapToLeader << "s)" << std::endl;
                }
            }
            
            // Sleep to simulate real-time
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
        }
        
        // Display final results
        std::cout << "\nFinal Results:" << std::endl;
        std::cout << "=============" << std::endl;
        
        auto positions = race->getDriverPositions();
        for (const auto& pos : positions) {
            std::cout << "P" << pos.position << ": " << pos.driverName
                      << " (Best Lap: " << std::fixed << std::setprecision(3) << pos.bestLapTime << "s)" << std::endl;
        }
        
        // Save telemetry data
        telemetry->saveToFile("race_telemetry.dat");
        telemetry->exportToCsv("race_telemetry.csv");
        
        // Save profiling data
        utils::Profiler::saveToFile("race_profile.csv");
        
        // Display profiling statistics
        std::cout << "\nPerformance Profile:" << std::endl;
        std::cout << "===================" << std::endl;
        
        auto profileStats = utils::Profiler::getStats();
        for (const auto& stat : profileStats) {
            std::cout << stat.eventName << ": " 
                      << std::fixed << std::setprecision(2) << stat.totalTimeMs << " ms total, "
                      << std::fixed << std::setprecision(2) << stat.avgTimeMs << " ms avg ("
                      << stat