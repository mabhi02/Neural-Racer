#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>

#include "neural_racer/simulation/track.hpp"
#include "neural_racer/simulation/race.hpp"
#include "neural_racer/simulation/telemetry.hpp"
#include "neural_racer/physics/vehicle.hpp"
#include "neural_racer/physics/engine.hpp"
#include "neural_racer/ai/driver.hpp"
#include "neural_racer/utils/logger.hpp"

namespace neural_racer {
namespace testing {

// Initialize logging for tests
class SimulationTest : public ::testing::Test {
protected:
    void SetUp() override {
        utils::Logger::initialize(false);  // No console output for tests
    }
    
    void TearDown() override {
        utils::Logger::shutdown();
    }
};

// Test the Track class
TEST_F(SimulationTest, TrackTest) {
    // Create a track
    auto track = std::make_shared<simulation::Track>("Test Track");
    
    // Check track name
    EXPECT_EQ(track->getName(), "Test Track");
    
    // Set track info
    simulation::TrackInfo info;
    info.name = "Test Track";
    info.location = "Test Location";
    info.length = 5000.0f;
    info.segmentCount = 10;
    info.sectorCount = 3;
    info.checkpointCount = 5;
    info.recordLapTime = 90.0f;
    info.recordHolder = "Test Driver";
    
    track->setInfo(info);
    
    // Check track info
    const auto& retrievedInfo = track->getInfo();
    EXPECT_EQ(retrievedInfo.name, "Test Track");
    EXPECT_EQ(retrievedInfo.location, "Test Location");
    EXPECT_FLOAT_EQ(retrievedInfo.length, 5000.0f);
    EXPECT_EQ(retrievedInfo.recordHolder, "Test Driver");
    
    // Set track environment
    simulation::TrackEnvironment environment;
    environment.timeOfDay = simulation::TrackEnvironment::TimeOfDay::Afternoon;
    environment.baseTemperature = 25.0f;
    environment.humidity = 0.6f;
    environment.windBaseSpeed = 3.0f;
    environment.windVariability = 1.0f;
    environment.rainProbability = 0.2f;
    
    track->setEnvironment(environment);
    
    // Check track environment
    const auto& retrievedEnv = track->getEnvironment();
    EXPECT_EQ(retrievedEnv.timeOfDay, simulation::TrackEnvironment::TimeOfDay::Afternoon);
    EXPECT_FLOAT_EQ(retrievedEnv.baseTemperature, 25.0f);
    EXPECT_FLOAT_EQ(retrievedEnv.humidity, 0.6f);
    
    // Add segments
    simulation::TrackSegment straightSegment;
    straightSegment.type = simulation::SegmentType::Straight;
    straightSegment.length = 1000.0f;
    straightSegment.width = 15.0f;
    track->addSegment(straightSegment);
    
    simulation::TrackSegment turnSegment;
    turnSegment.type = simulation::SegmentType::LeftTurn;
    turnSegment.length = 200.0f;
    turnSegment.width = 12.0f;
    turnSegment.curvature = 0.01f;
    track->addSegment(turnSegment);
    
    // Check segments
    const auto& segments = track->getSegments();
    EXPECT_EQ(segments.size(), 2);
    EXPECT_EQ(segments[0].type, simulation::SegmentType::Straight);
    EXPECT_EQ(segments[1].type, simulation::SegmentType::LeftTurn);
    
    // Build waypoints
    track->buildWaypoints(20.0f);  // 20m spacing
    
    // Check waypoints
    const auto& waypoints = track->getWaypoints();
    EXPECT_GT(waypoints.size(), 0);
    
    // Add checkpoint
    simulation::Checkpoint checkpoint;
    checkpoint.x = 100.0f;
    checkpoint.y = 0.0f;
    checkpoint.width = 15.0f;
    checkpoint.heading = 1.57f;
    checkpoint.distanceFromStart = 100.0f;
    checkpoint.name = "Start/Finish";
    track->addCheckpoint(checkpoint);
    
    // Check checkpoint
    const auto& checkpoints = track->getCheckpoints();
    EXPECT_EQ(checkpoints.size(), 1);
    EXPECT_EQ(checkpoints[0].name, "Start/Finish");
    
    // Add sectors
    simulation::Sector sector1;
    sector1.startDistance = 0.0f;
    sector1.endDistance = 400.0f;
    sector1.name = "Sector 1";
    track->addSector(sector1);
    
    simulation::Sector sector2;
    sector2.startDistance = 400.0f;
    sector2.endDistance = 800.0f;
    sector2.name = "Sector 2";
    track->addSector(sector2);
    
    // Check sectors
    const auto& sectors = track->getSectors();
    EXPECT_EQ(sectors.size(), 2);
    EXPECT_EQ(sectors[0].name, "Sector 1");
    EXPECT_EQ(sectors[1].name, "Sector 2");
    
    // Test sector lookup
    int sectorAt300 = track->getSectorAtDistance(300.0f);
    EXPECT_EQ(sectorAt300, 1);  // Sector 1
    
    int sectorAt500 = track->getSectorAtDistance(500.0f);
    EXPECT_EQ(sectorAt500, 2);  // Sector 2
    
    // Test racing line calculation
    auto racingLine = track->calculateRacingLine();
    EXPECT_GT(racingLine.size(), 0);
}

// Test the TrackFactory
TEST_F(SimulationTest, TrackFactoryTest) {
    // Create tracks using different factory methods
    auto presetTrack = simulation::TrackFactory::createFromPreset("Monaco");
    EXPECT_NE(presetTrack, nullptr);
    EXPECT_EQ(presetTrack->getName(), "Monaco");
    
    auto randomTrack = simulation::TrackFactory::createRandom(8, 3000.0f, 0.5f);
    EXPECT_NE(randomTrack, nullptr);
    EXPECT_GT(randomTrack->getSegments().size(), 0);
    
    // Check random track generation with different parameters
    auto easyTrack = simulation::TrackFactory::createRandom(4, 2000.0f, 0.2f);
    auto hardTrack = simulation::TrackFactory::createRandom(12, 4000.0f, 0.8f);
    
    // Hard track should have more segments and be longer
    EXPECT_GT(hardTrack->getSegments().size(), easyTrack->getSegments().size());
    
    // Check waypoints
    EXPECT_GT(hardTrack->getWaypoints().size(), 0);
    EXPECT_GT(easyTrack->getWaypoints().size(), 0);
}

// Test the Race class
TEST_F(SimulationTest, RaceTest) {
    // Create track
    auto track = simulation::TrackFactory::createRandom(6, 2000.0f, 0.5f);
    
    // Create race
    auto race = std::make_shared<simulation::Race>(track);
    
    // Configure race
    simulation::RaceConfig config;
    config.laps = 3;
    config.standing_start = true;
    config.enableWeatherChanges = true;
    config.enableTireWear = true;
    config.enableFuelConsumption = true;
    race->setConfig(config);
    
    // Check configuration
    const auto& retrievedConfig = race->getConfig();
    EXPECT_EQ(retrievedConfig.laps, 3);
    EXPECT_TRUE(retrievedConfig.standing_start);
    EXPECT_TRUE(retrievedConfig.enableWeatherChanges);
    
    // Set weather conditions
    simulation::WeatherConditions weather;
    weather.type = simulation::WeatherConditions::Type::Cloudy;
    weather.trackTemperature = 30.0f;
    weather.airTemperature = 25.0f;
    weather.windSpeed = 2.0f;
    weather.windDirection = 0.3f;
    weather.precipitationIntensity = 0.0f;
    race->setWeather(weather);
    
    // Check weather conditions
    const auto& retrievedWeather = race->getWeather();
    EXPECT_EQ(retrievedWeather.type, simulation::WeatherConditions::Type::Cloudy);
    EXPECT_FLOAT_EQ(retrievedWeather.trackTemperature, 30.0f);
    
    // Create physics engine
    auto physicsEngine = std::make_shared<physics::Engine>();
    physicsEngine->initialize();
    
    // Create vehicles
    physics::VehicleSpec spec1;
    spec1.name = "Car 1";
    spec1.mass = 720.0f;
    spec1.power = 500.0f;
    auto vehicle1 = std::make_shared<physics::Vehicle>(spec1, physicsEngine);
    
    physics::VehicleSpec spec2;
    spec2.name = "Car 2";
    spec2.mass = 700.0f;
    spec2.power = 490.0f;
    auto vehicle2 = std::make_shared<physics::Vehicle>(spec2, physicsEngine);
    
    // Create drivers
    auto driver1 = ai::DriverFactory::createWithSkillLevel(0.8f, vehicle1, nullptr, "Driver 1");
    auto driver2 = ai::DriverFactory::createWithSkillLevel(0.7f, vehicle2, nullptr, "Driver 2");
    
    // Add drivers to race
    EXPECT_TRUE(race->addDriver(driver1));
    EXPECT_TRUE(race->addDriver(driver2));
    
    // Test event callbacks
    bool startCalled = false;
    bool lapCompletedCalled = false;
    
    auto eventCallback = [&](const simulation::RaceEvent& event) {
        if (event.type == simulation::RaceEventType::Start) {
            startCalled = true;
        } else if (event.type == simulation::RaceEventType::LapCompleted) {
            lapCompletedCalled = true;
        }
    };
    
    int callbackId = race->registerEventCallback(eventCallback);
    EXPECT_GT(callbackId, 0);
    
    // Initialize race
    EXPECT_TRUE(race->initialize());
    
    // Start race
    EXPECT_TRUE(race->start());
    
    // Race should be in countdown or running state
    EXPECT_TRUE(race->getState() == simulation::RaceState::Countdown ||
                race->getState() == simulation::RaceState::Running);
    
    // Update race for a few steps
    for (int i = 0; i < 10; i++) {
        race->update(0.1f);
    }
    
    // Check that start event was fired
    EXPECT_TRUE(startCalled);
    
    // Get driver positions
    auto positions = race->getDriverPositions();
    EXPECT_EQ(positions.size(), 2);
    
    // Test pause and resume
    race->pause();
    float pausedTime = race->getElapsedTime();
    
    // Update should not change elapsed time when paused
    race->update(0.1f);
    EXPECT_FLOAT_EQ(race->getElapsedTime(), pausedTime);
    
    // Resume
    race->resume();
    
    // Update should now change elapsed time
    race->update(0.1f);
    EXPECT_GT(race->getElapsedTime(), pausedTime);
    
    // Test stopping the race
    race->stop();
    EXPECT_EQ(race->getState(), simulation::RaceState::Finished);
    
    // Test race shutdown
    race->shutdown();
}

// Test race simulation with telemetry
TEST_F(SimulationTest, RaceSimulationTest) {
    // Create track
    auto track = simulation::TrackFactory::createFromPreset("Monza");
    
    // Create race
    auto race = std::make_shared<simulation::Race>(track);
    
    // Configure race
    simulation::RaceConfig config;
    config.laps = 1;  // Short race for testing
    config.timeScale = 2.0f;  // Run at 2x speed
    race->setConfig(config);
    
    // Create physics engine
    auto physicsEngine = std::make_shared<physics::Engine>();
    physicsEngine->initialize();
    
    // Create vehicle
    physics::VehicleSpec spec;
    spec.name = "Test Car";
    spec.mass = 720.0f;
    spec.power = 500.0f;
    auto vehicle = std::make_shared<physics::Vehicle>(spec, physicsEngine);
    
    // Create driver
    auto driver = ai::DriverFactory::createWithSkillLevel(0.8f, vehicle, nullptr, "Test Driver");
    
    // Add driver to race
    race->addDriver(driver);
    
    // Create telemetry system
    auto telemetry = std::make_shared<simulation::Telemetry>();
    telemetry->initialize();
    
    // Record telemetry
    telemetry->recordVehicle(vehicle, "vehicle.");
    telemetry->recordDriver(driver, "driver.");
    
    // Run simulation
    bool success = race->simulate(5.0f, telemetry);  // Simulate for 5 seconds
    EXPECT_TRUE(success);
    
    // Export telemetry data
    success = telemetry->exportToCsv("test_telemetry.csv");
    EXPECT_TRUE(success);
    
    // Clear telemetry data
    telemetry->clear();
}

// Test the Telemetry class
TEST_F(SimulationTest, TelemetryTest) {
    // Create telemetry system
    simulation::Telemetry telemetry(500);  // 500 samples per channel
    EXPECT_TRUE(telemetry.initialize());
    
    // Test adding channels
    EXPECT_TRUE(telemetry.addChannel("test_float", "m/s", simulation::TelemetryType::Float));
    EXPECT_TRUE(telemetry.addChannel("test_int", "", simulation::TelemetryType::Int));
    EXPECT_TRUE(telemetry.addChannel("test_bool", "", simulation::TelemetryType::Bool));
    EXPECT_TRUE(telemetry.addChannel("test_string", "", simulation::TelemetryType::String));
    EXPECT_TRUE(telemetry.addChannel("test_vector2", "m", simulation::TelemetryType::Vector2));
    EXPECT_TRUE(telemetry.addChannel("test_vector3", "m", simulation::TelemetryType::Vector3));
    
    // Test adding samples
    EXPECT_TRUE(telemetry.addSample("test_float", 0.0f, 10.5f));
    EXPECT_TRUE(telemetry.addSample("test_int", 0.0f, 42));
    EXPECT_TRUE(telemetry.addSample("test_bool", 0.0f, true));
    EXPECT_TRUE(telemetry.addSample("test_string", 0.0f, "Hello World"));
    EXPECT_TRUE(telemetry.addSample("test_vector2", 0.0f, 1.0f, 2.0f));
    EXPECT_TRUE(telemetry.addSample("test_vector3", 0.0f, 1.0f, 2.0f, 3.0f));
    
    // Test adding samples to non-existent channel (should create the channel)
    EXPECT_TRUE(telemetry.addSample("new_channel", 0.0f, 123.45f));
    
    // Test type mismatch (should fail)
    EXPECT_FALSE(telemetry.addSample("test_float", 0.0f, "Invalid"));
    
    // Test getting channel names
    auto channelNames = telemetry.getChannelNames();
    EXPECT_EQ(channelNames.size(), 7);  // 6 initial channels + 1 new channel
    
    // Test getting channel data
    auto floatData = telemetry.getChannelData("test_float", 0.0f, 1.0f);
    EXPECT_EQ(floatData.size(), 1);
    
    // Test saving and loading telemetry data
    EXPECT_TRUE(telemetry.saveToFile("test_telemetry.dat"));
    
    // Create a new telemetry system to load the data
    simulation::Telemetry loadedTelemetry;
    EXPECT_TRUE(loadedTelemetry.initialize());
    EXPECT_TRUE(loadedTelemetry.loadFromFile("test_telemetry.dat"));
    
    // Check that data was loaded
    auto loadedChannelNames = loadedTelemetry.getChannelNames();
    EXPECT_EQ(loadedChannelNames.size(), channelNames.size());
    
    // Test listener
    class TestListener : public simulation::TelemetryListener {
    public:
        int dataCallCount = 0;
        int lapCallCount = 0;
        
        void onTelemetryData(const std::string& channelName, const simulation::TelemetrySample& sample) override {
            dataCallCount++;
        }
        
        void onLapCompleted(const std::string& driverName, float lapTime, int lapNumber) override {
            lapCallCount++;
        }
    };
    
    TestListener listener;
    telemetry.registerListener(&listener);
    
    // Add a sample, should trigger listener
    telemetry.addSample("test_float", 1.0f, 20.0f);
    EXPECT_EQ(listener.dataCallCount, 1);
    
    // Unregister listener
    telemetry.unregisterListener(&listener);
    
    // Add another sample, should not trigger listener
    telemetry.addSample("test_float", 2.0f, 30.0f);
    EXPECT_EQ(listener.dataCallCount, 1);  // Still 1
    
    // Shutdown telemetry
    telemetry.shutdown();
}

// Test track and vehicle interaction
TEST_F(SimulationTest, TrackVehicleInteractionTest) {
    // Create track
    auto track = simulation::TrackFactory::createRandom(4, 1000.0f, 0.5f);
    
    // Create physics engine
    auto physicsEngine = std::make_shared<physics::Engine>();
    physicsEngine->initialize();
    
    // Create vehicle
    physics::VehicleSpec spec;
    spec.name = "Test Car";
    spec.mass = 720.0f;
    spec.power = 500.0f;
    auto vehicle = std::make_shared<physics::Vehicle>(spec, physicsEngine);
    
    // Set vehicle at track start
    // Assume first waypoint is at track start
    const auto& waypoints = track->getWaypoints();
    if (!waypoints.empty()) {
        vehicle->setPosition(waypoints[0].x, waypoints[0].y, waypoints[0].heading);
    } else {
        vehicle->setPosition(0.0f, 0.0f, 0.0f);
    }
    
    // Apply controls to drive along track
    vehicle->setControls(0.5f, 0.0f, 0.0f);  // Half throttle
    
    // Update vehicle several times
    for (int i = 0; i < 10; i++) {
        vehicle->update(0.1f);
        
        // Get vehicle state
        auto state = vehicle->getState();
        
        // Check if vehicle is on track
        bool onTrack = track->isOnTrack(state.positionX, state.positionY, 2.0f);  // 2m margin
        
        // Not a strict test since we're not steering to follow the track
        // Just make sure the function works without crashing
        if (!onTrack && !waypoints.empty()) {
            // Find nearest waypoint
            const auto& nearest = track->findNearestWaypoint(state.positionX, state.positionY);
            
            // Log distance to nearest waypoint (for debugging)
            float dx = state.positionX - nearest.x;
            float dy = state.positionY - nearest.y;
            float distance = std::sqrt(dx * dx + dy * dy);
            
            std::cout << "Vehicle off track, distance to nearest waypoint: " << distance << std::endl;
        }
    }
}

// Test continuous integration of race, vehicles, and telemetry
TEST_F(SimulationTest, IntegrationTest) {
    // Create track
    auto track = simulation::TrackFactory::createFromPreset("Monaco");
    
    // Create physics engine
    auto physicsEngine = std::make_shared<physics::Engine>();
    physicsEngine->initialize();
    
    // Create race
    auto race = std::make_shared<simulation::Race>(track);
    
    // Configure race
    simulation::RaceConfig config;
    config.laps = 1;
    config.timeScale = 2.0f;
    race->setConfig(config);
    
    // Create multiple vehicles with different specifications
    std::vector<std::shared_ptr<physics::Vehicle>> vehicles;
    std::vector<std::shared_ptr<ai::Driver>> drivers;
    
    for (int i = 0; i < 3; i++) {
        physics::VehicleSpec spec;
        spec.name = "Car " + std::to_string(i + 1);
        spec.mass = 700.0f + i * 20.0f;
        spec.power = 480.0f + i * 10.0f;
        
        auto vehicle = std::make_shared<physics::Vehicle>(spec, physicsEngine);
        vehicles.push_back(vehicle);
        
        // Create driver with different skill levels
        float skill = 0.6f + i * 0.1f;
        auto driver = ai::DriverFactory::createWithSkillLevel(skill, vehicle, nullptr, "Driver " + std::to_string(i + 1));
        drivers.push_back(driver);
        
        // Add driver to race
        race->addDriver(driver);
    }
    
    // Create telemetry system
    auto telemetry = std::make_shared<simulation::Telemetry>();
    telemetry->initialize();
    
    // Record telemetry for all vehicles and drivers
    for (size_t i = 0; i < vehicles.size(); i++) {
        telemetry->recordVehicle(vehicles[i], "vehicle" + std::to_string(i + 1) + ".");
        telemetry->recordDriver(drivers[i], "driver" + std::to_string(i + 1) + ".");
    }
    
    // Start race
    EXPECT_TRUE(race->initialize());
    EXPECT_TRUE(race->start());
    
    // Run simulation for a short time
    for (int i = 0; i < 20; i++) {
        race->update(0.1f);
        telemetry->update(race);
    }
    
    // Check driver positions
    auto positions = race->getDriverPositions();
    EXPECT_EQ(positions.size(), drivers.size());
    
    // Stop race
    race->stop();
    
    // Export telemetry
    telemetry->exportToCsv("integration_test.csv");
    
    // Shutdown systems
    race->shutdown();
    physicsEngine->shutdown();
    telemetry->shutdown();
}

} // namespace testing
} // namespace neural_racer

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}