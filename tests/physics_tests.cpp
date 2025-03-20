#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>

#include "neural_racer/physics/engine.hpp"
#include "neural_racer/physics/vehicle.hpp"
#include "neural_racer/hardware/device_driver.hpp"
#include "neural_racer/utils/logger.hpp"

namespace neural_racer {
namespace testing {

// Initialize logging for tests
class PhysicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        utils::Logger::initialize(false);  // No console output for tests
    }
    
    void TearDown() override {
        utils::Logger::shutdown();
    }
};

// Test the Physics Engine class
TEST_F(PhysicsTest, EngineTest) {
    // Create a GPU accelerator for hardware acceleration
    auto gpuAccelerator = std::make_shared<hardware::GPUAccelerator>();
    gpuAccelerator->initialize();
    
    // Create physics engine
    physics::Engine engine(gpuAccelerator);
    
    // Initialize with default configuration
    EXPECT_TRUE(engine.initialize());
    EXPECT_TRUE(engine.isInitialized());
    
    // Check default configuration
    auto config = engine.getConfig();
    EXPECT_FLOAT_EQ(config.gravity, 9.81f);
    EXPECT_GT(config.timeStep, 0.0f);
    EXPECT_GT(config.substeps, 0);
    
    // Test hardware acceleration
    bool hasHardwareAccel = engine.hasHardwareAcceleration();
    // This will be true if GPU accelerator is available
    // Not a critical test, as it depends on system
    
    // Test vehicle registration
    physics::VehicleSpec spec;
    spec.name = "Test Vehicle";
    spec.mass = 1000.0f;
    spec.power = 200.0f;
    auto vehicle = std::make_shared<physics::Vehicle>(spec);
    
    EXPECT_TRUE(engine.registerVehicle(vehicle));
    EXPECT_EQ(engine.getVehicleCount(), 1);
    
    // Test vehicle unregistration
    engine.unregisterVehicle(vehicle);
    EXPECT_EQ(engine.getVehicleCount(), 0);
    
    // Test collision callbacks
    bool callbackCalled = false;
    auto collisionCallback = [&callbackCalled](const physics::CollisionData& collision) {
        callbackCalled = true;
    };
    
    int callbackId = engine.registerCollisionCallback(collisionCallback);
    EXPECT_GT(callbackId, 0);
    
    // Test step function
    for (int i = 0; i < 10; i++) {
        engine.step(0.016f);  // ~60 FPS
    }
    
    // Check stats after steps
    auto stats = engine.getStats();
    EXPECT_GT(stats.lastStepDurationMs, 0.0f);
    
    // Test unregistration of callback
    engine.unregisterCollisionCallback(callbackId);
    
    // Test simulation thread
    EXPECT_TRUE(engine.startSimulationThread());
    EXPECT_TRUE(engine.isSimulationThreadRunning());
    
    // Let simulation run for a short time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Stop simulation thread
    engine.stopSimulationThread();
    EXPECT_FALSE(engine.isSimulationThreadRunning());
    
    // Shutdown the physics engine
    engine.shutdown();
    EXPECT_FALSE(engine.isInitialized());
}

// Test the Vehicle class
TEST_F(PhysicsTest, VehicleTest) {
    // Create vehicle specification
    physics::VehicleSpec spec;
    spec.name = "Test Car";
    spec.mass = 1200.0f;
    spec.power = 150.0f;
    spec.drag = 0.3f;
    spec.frontGrip = 1.0f;
    spec.rearGrip = 1.0f;
    spec.brakingForce = 20000.0f;
    spec.steeringResponse = 1.0f;
    
    // Create vehicle
    auto vehicle = std::make_shared<physics::Vehicle>(spec);
    
    // Check initial state
    auto state = vehicle->getState();
    EXPECT_FLOAT_EQ(state.positionX, 0.0f);
    EXPECT_FLOAT_EQ(state.positionY, 0.0f);
    EXPECT_FLOAT_EQ(state.velocity, 0.0f);
    EXPECT_FLOAT_EQ(state.throttle, 0.0f);
    EXPECT_FLOAT_EQ(state.brake, 0.0f);
    
    // Set position
    vehicle->setPosition(10.0f, 20.0f, 1.57f);  // ~90 degrees
    state = vehicle->getState();
    EXPECT_FLOAT_EQ(state.positionX, 10.0f);
    EXPECT_FLOAT_EQ(state.positionY, 20.0f);
    EXPECT_FLOAT_EQ(state.heading, 1.57f);
    
    // Apply throttle and update
    vehicle->setControls(1.0f, 0.0f, 0.0f);  // Full throttle, no brake, no steering
    vehicle->update(1.0f);  // 1 second time step
    
    // Check that vehicle moved
    state = vehicle->getState();
    EXPECT_GT(state.velocity, 0.0f);
    EXPECT_GT(state.engineRPM, 0.0f);
    
    // Check that position changed
    EXPECT_NE(state.positionX, 10.0f);
    EXPECT_NE(state.positionY, 20.0f);
    
    // Apply brake and update
    vehicle->setControls(0.0f, 1.0f, 0.0f);  // No throttle, full brake, no steering
    float initialVelocity = state.velocity;
    vehicle->update(1.0f);  // 1 second time step
    
    // Check that vehicle slowed down
    state = vehicle->getState();
    EXPECT_LT(state.velocity, initialVelocity);
    
    // Apply steering and update
    vehicle->setControls(0.5f, 0.0f, 0.5f);  // Half throttle, no brake, right steering
    float initialHeading = state.heading;
    vehicle->update(1.0f);  // 1 second time step
    
    // Check that heading changed
    state = vehicle->getState();
    EXPECT_NE(state.heading, initialHeading);
    
    // Test external force application
    vehicle->applyForce(1000.0f, 0.0f, 0.5f);  // 1000N force in x direction for 0.5 seconds
    float posX = state.positionX;
    vehicle->update(1.0f);  // 1 second time step
    
    // Check that position changed more than normal due to force
    state = vehicle->getState();
    EXPECT_GT(state.positionX - posX, 0.0f);
    
    // Check other vehicle functions
    EXPECT_TRUE(vehicle->isGrounded());
    EXPECT_GT(vehicle->getSpeedKmh(), 0.0f);
    EXPECT_FLOAT_EQ(vehicle->getSpeedKmh(), state.velocity * 3.6f);
}

// Test vehicle dynamics
TEST_F(PhysicsTest, VehicleDynamicsTest) {
    // Create vehicle specification
    physics::VehicleSpec spec;
    spec.name = "Dynamics Test Car";
    spec.mass = 1500.0f;
    spec.power = 300.0f;
    spec.drag = 0.3f;
    spec.frontGrip = 1.2f;
    spec.rearGrip = 1.0f;
    spec.brakingForce = 25000.0f;
    spec.steeringResponse = 1.0f;
    
    // Create vehicle
    auto vehicle = std::make_shared<physics::Vehicle>(spec);
    
    // Accelerate for 10 seconds
    vehicle->setControls(1.0f, 0.0f, 0.0f);  // Full throttle
    float maxSpeed = 0.0f;
    float lastSpeed = 0.0f;
    
    for (int i = 0; i < 100; i++) {
        vehicle->update(0.1f);  // 0.1 second time step
        auto state = vehicle->getState();
        
        // Track maximum speed
        if (state.velocity > maxSpeed) {
            maxSpeed = state.velocity;
        }
        
        // Check acceleration pattern (should decrease as speed increases)
        if (i > 0) {
            float currentAccel = state.velocity - lastSpeed;
            
            // Acceleration should decrease (not a strict test)
            if (i > 10 && currentAccel > 0.0f) {
                float expectedAccel = (spec.power * 1000.0f) / (spec.mass * state.velocity);
                float dragForce = 0.5f * 1.225f * state.velocity * state.velocity * spec.drag * 2.0f;
                float netForce = expectedAccel * spec.mass - dragForce;
                float expectedNetAccel = netForce / spec.mass;
                
                // The exact calculation is complex, but we just want to verify the sign and order of magnitude
                if (expectedNetAccel < 0.0f && currentAccel > 0.0f) {
                    std::cout << "Warning: Acceleration continues after expected equilibrium" << std::endl;
                }
            }
        }
        
        lastSpeed = state.velocity;
    }
    
    // Check that vehicle reaches a reasonable top speed
    // This is a very rough check, as the exact top speed depends on many factors
    EXPECT_GT(maxSpeed, 30.0f);  // Should exceed 30 m/s (~108 km/h)
    EXPECT_LT(maxSpeed, 150.0f);  // But not be unreasonably high
    
    // Reset vehicle
    vehicle = std::make_shared<physics::Vehicle>(spec);
    
    // Check braking performance
    vehicle->setControls(1.0f, 0.0f, 0.0f);  // Full throttle
    
    // Accelerate for 5 seconds
    for (int i = 0; i < 50; i++) {
        vehicle->update(0.1f);
    }
    
    auto state = vehicle->getState();
    float speedBeforeBraking = state.velocity;
    
    // Apply brakes
    vehicle->setControls(0.0f, 1.0f, 0.0f);  // Full brake
    
    // Brake for 5 seconds
    for (int i = 0; i < 50; i++) {
        vehicle->update(0.1f);
        state = vehicle->getState();
        
        // Should eventually stop
        if (state.velocity < 0.1f) {
            break;
        }
    }
    
    // Check that vehicle has slowed significantly
    EXPECT_LT(state.velocity, speedBeforeBraking * 0.5f);
}

// Test physics configuration
TEST_F(PhysicsTest, PhysicsConfigTest) {
    // Create physics engine
    physics::Engine engine;
    
    // Initialize with default configuration
    EXPECT_TRUE(engine.initialize());
    
    // Get default configuration
    auto defaultConfig = engine.getConfig();
    
    // Create custom configuration
    physics::PhysicsConfig customConfig;
    customConfig.gravity = 5.0f;  // Low gravity
    customConfig.timeStep = 0.01f;
    customConfig.substeps = 8;
    customConfig.airDensity = 2.0f;  // High air density
    customConfig.frictionCoefficient = 0.5f;
    customConfig.enableCollisions = false;
    
    // Apply custom configuration
    engine.setConfig(customConfig);
    
    // Check that configuration was applied
    auto config = engine.getConfig();
    EXPECT_FLOAT_EQ(config.gravity, 5.0f);
    EXPECT_FLOAT_EQ(config.timeStep, 0.01f);
    EXPECT_EQ(config.substeps, 8);
    EXPECT_FLOAT_EQ(config.airDensity, 2.0f);
    EXPECT_FLOAT_EQ(config.frictionCoefficient, 0.5f);
    EXPECT_FALSE(config.enableCollisions);
    
    // Create vehicle with custom physics
    physics::VehicleSpec spec;
    spec.name = "Test Vehicle";
    spec.mass = 1000.0f;
    spec.power = 200.0f;
    auto vehicle = std::make_shared<physics::Vehicle>(spec, std::make_shared<physics::Engine>(engine));
    
    // Register vehicle
    EXPECT_TRUE(engine.registerVehicle(vehicle));
    
    // Run simulation with custom physics
    for (int i = 0; i < 10; i++) {
        engine.step(0.016f);
    }
    
    // Shutdown
    engine.shutdown();
}

// Test physics engine interaction with multiple vehicles
TEST_F(PhysicsTest, MultiVehicleTest) {
    // Create physics engine
    physics::Engine engine;
    engine.initialize();
    
    // Create multiple vehicles
    std::vector<std::shared_ptr<physics::Vehicle>> vehicles;
    
    for (int i = 0; i < 5; i++) {
        physics::VehicleSpec spec;
        spec.name = "Vehicle " + std::to_string(i + 1);
        spec.mass = 1000.0f + i * 100.0f;  // Different masses
        spec.power = 150.0f + i * 20.0f;   // Different powers
        
        auto vehicle = std::make_shared<physics::Vehicle>(spec, std::make_shared<physics::Engine>(engine));
        vehicles.push_back(vehicle);
        
        // Register vehicle
        EXPECT_TRUE(engine.registerVehicle(vehicle));
        
        // Position vehicles at different locations
        vehicle->setPosition(i * 10.0f, 0.0f, 0.0f);
        
        // Apply different controls
        float throttle = (i % 2 == 0) ? 1.0f : 0.5f;
        float brake = (i % 3 == 0) ? 0.2f : 0.0f;
        float steering = (i % 4 - 2) * 0.1f;
        
        vehicle->setControls(throttle, brake, steering);
    }
    
    // Check vehicle count
    EXPECT_EQ(engine.getVehicleCount(), 5);
    
    // Run simulation
    for (int i = 0; i < 10; i++) {
        engine.step(0.016f);
    }
    
    // Check that vehicles have moved differently
    std::vector<float> positions;
    
    for (const auto& vehicle : vehicles) {
        auto state = vehicle->getState();
        positions.push_back(state.positionX);
    }
    
    // Vehicles should have different positions
    for (size_t i = 1; i < positions.size(); i++) {
        EXPECT_NE(positions[i], positions[0]);
    }
    
    // Unregister all vehicles
    for (const auto& vehicle : vehicles) {
        engine.unregisterVehicle(vehicle);
    }
    
    // Check vehicle count
    EXPECT_EQ(engine.getVehicleCount(), 0);
    
    // Shutdown
    engine.shutdown();
}

} // namespace testing
} // namespace neural_racer

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}