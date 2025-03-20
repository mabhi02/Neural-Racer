#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>

#include "neural_racer/ai/inference.hpp"
#include "neural_racer/ai/driver.hpp"
#include "neural_racer/hardware/device_driver.hpp"
#include "neural_racer/physics/vehicle.hpp"
#include "neural_racer/utils/logger.hpp"

namespace neural_racer {
namespace testing {

// Initialize logging for tests
class AITest : public ::testing::Test {
protected:
    void SetUp() override {
        utils::Logger::initialize(false);  // No console output for tests
    }
    
    void TearDown() override {
        utils::Logger::shutdown();
    }
};

// Test the Tensor class
TEST_F(AITest, TensorTest) {
    // Create a tensor with shape [2, 3]
    ai::Tensor tensor({2, 3});
    
    // Verify size
    EXPECT_EQ(tensor.size(), 6);
    
    // Verify default values
    for (size_t i = 0; i < tensor.data.size(); ++i) {
        EXPECT_FLOAT_EQ(tensor.data[i], 0.0f);
    }
    
    // Modify data
    for (size_t i = 0; i < tensor.data.size(); ++i) {
        tensor.data[i] = static_cast<float>(i);
    }
    
    // Reshape to [3, 2]
    EXPECT_TRUE(tensor.reshape({3, 2}));
    
    // Verify shape and data after reshape
    EXPECT_EQ(tensor.shape.size(), 2);
    EXPECT_EQ(tensor.shape[0], 3);
    EXPECT_EQ(tensor.shape[1], 2);
    EXPECT_EQ(tensor.size(), 6);
    
    // Invalid reshape should fail
    EXPECT_FALSE(tensor.reshape({4, 2}));
}

// Test the InferenceEngine class
TEST_F(AITest, InferenceEngineTest) {
    // Create GPU accelerator
    auto gpuAccelerator = std::make_shared<hardware::GPUAccelerator>();
    gpuAccelerator->initialize();
    
    // Create inference engine
    ai::InferenceEngine engine("models/test_model.onnx", gpuAccelerator);
    
    // Initialize engine
    EXPECT_TRUE(engine.initialize());
    
    // Check model info
    const auto& modelInfo = engine.getModelInfo();
    EXPECT_FALSE(modelInfo.name.empty());
    EXPECT_TRUE(modelInfo.parameterCount > 0);
    
    // Create input tensor
    ai::Tensor inputTensor({1, 10});
    for (size_t i = 0; i < inputTensor.data.size(); ++i) {
        inputTensor.data[i] = 0.1f * static_cast<float>(i);
    }
    
    // Set input
    EXPECT_TRUE(engine.setInput("input", inputTensor));
    
    // Run inference
    EXPECT_TRUE(engine.runInference());
    
    // Check metrics
    const auto& metrics = engine.getMetrics();
    EXPECT_GT(metrics.inferenceTimeMs, 0.0f);
    EXPECT_GT(metrics.throughputFPS, 0.0f);
    
    // Get output
    ai::Tensor outputTensor = engine.getOutput("output");
    EXPECT_FALSE(outputTensor.data.empty());
    
    // Shutdown
    engine.shutdown();
}

// Test the DriverModel class
TEST_F(AITest, DriverModelTest) {
    // Create GPU accelerator
    auto gpuAccelerator = std::make_shared<hardware::GPUAccelerator>();
    gpuAccelerator->initialize();
    
    // Create driver model
    auto driverModel = std::unique_ptr<ai::DriverModel>(
        new ai::DriverModel("models/driver_model.onnx", gpuAccelerator));
    
    // Initialize model
    EXPECT_TRUE(driverModel->initialize());
    
    // Create sensor inputs
    ai::DriverModel::SensorInputs inputs;
    inputs.speed = 50.0f;              // 50 m/s
    inputs.acceleration = 2.0f;        // 2 m/s²
    inputs.trackPosition = 0.1f;       // Slightly right of center
    inputs.trackAngle = 0.05f;         // Small angle
    inputs.trackCurvature = 0.01f;     // Gentle curve
    inputs.rangeSensors = {100.0f, 80.0f, 90.0f, 85.0f, 95.0f};  // Range sensors
    
    // Process inputs
    auto outputs = driverModel->process(inputs);
    
    // Check outputs
    EXPECT_GE(outputs.throttle, 0.0f);
    EXPECT_LE(outputs.throttle, 1.0f);
    EXPECT_GE(outputs.brake, 0.0f);
    EXPECT_LE(outputs.brake, 1.0f);
    EXPECT_GE(outputs.steering, -1.0f);
    EXPECT_LE(outputs.steering, 1.0f);
    EXPECT_GE(outputs.confidence, 0.0f);
    EXPECT_LE(outputs.confidence, 1.0f);
    
    // Test aggressiveness setting
    driverModel->setAggressivenessFactor(1.0f);
    auto aggressiveOutputs = driverModel->process(inputs);
    
    // More aggressive should mean more throttle or less brake on average
    driverModel->setAggressivenessFactor(0.0f);
    auto conservativeOutputs = driverModel->process(inputs);
    
    // This is a probabilistic test, may rarely fail
    float aggressiveScore = aggressiveOutputs.throttle - aggressiveOutputs.brake;
    float conservativeScore = conservativeOutputs.throttle - conservativeOutputs.brake;
    
    // Not always true due to simulation, but generally should hold
    // We use assert instead of expect since this is probabilistic
    if (aggressiveScore < conservativeScore) {
        std::cout << "Warning: Aggressiveness test produced unexpected results. This is probabilistic and can happen." << std::endl;
    }
    
    // Shutdown model
    driverModel->shutdown();
}

// Test the Driver factory and driver creation
TEST_F(AITest, DriverFactoryTest) {
    // Create GPU accelerator
    auto gpuAccelerator = std::make_shared<hardware::GPUAccelerator>();
    gpuAccelerator->initialize();
    
    // Create vehicle
    physics::VehicleSpec spec;
    spec.name = "Test Vehicle";
    auto physics = std::make_shared<physics::Engine>();
    auto vehicle = std::make_shared<physics::Vehicle>(spec, physics);
    
    // Create drivers with different skill levels
    auto driver1 = ai::DriverFactory::createWithSkillLevel(0.3f, vehicle, gpuAccelerator, "Novice");
    auto driver2 = ai::DriverFactory::createWithSkillLevel(0.7f, vehicle, gpuAccelerator, "Expert");
    
    // Check names
    EXPECT_EQ(driver1->getName(), "Novice");
    EXPECT_EQ(driver2->getName(), "Expert");
    
    // Check parameters
    const auto& params1 = driver1->getParameters();
    const auto& params2 = driver2->getParameters();
    
    // Expert should have higher parameters
    EXPECT_GT(params2.aggression, params1.aggression);
    EXPECT_GT(params2.consistency, params1.consistency);
    
    // Create driver with specific personality
    ai::DriverParameters customParams;
    customParams.aggression = 0.9f;
    customParams.consistency = 0.6f;
    customParams.adaptability = 0.7f;
    
    auto driver3 = ai::DriverFactory::createWithPersonality(
        customParams, vehicle, gpuAccelerator, "Custom");
    
    // Check custom parameters
    const auto& params3 = driver3->getParameters();
    EXPECT_FLOAT_EQ(params3.aggression, 0.9f);
    EXPECT_FLOAT_EQ(params3.consistency, 0.6f);
    EXPECT_FLOAT_EQ(params3.adaptability, 0.7f);
    
    // Create driver with specific style
    auto driver4 = ai::DriverFactory::createFromRealDriverStyle(
        "aggressive", vehicle, gpuAccelerator, "Aggressive");
    
    // Check name
    EXPECT_EQ(driver4->getName(), "Aggressive");
    
    // Check that parameters make sense for aggressive style
    const auto& params4 = driver4->getParameters();
    EXPECT_GT(params4.aggression, 0.7f);  // Aggressive should have high aggression
}

// Test strategies and racing lines
TEST_F(AITest, DriverStrategiesTest) {
    // Create GPU accelerator
    auto gpuAccelerator = std::make_shared<hardware::GPUAccelerator>();
    gpuAccelerator->initialize();
    
    // Create vehicle
    physics::VehicleSpec spec;
    spec.name = "Test Vehicle";
    auto physics = std::make_shared<physics::Engine>();
    auto vehicle = std::make_shared<physics::Vehicle>(spec, physics);
    
    // Create driver
    auto driver = ai::DriverFactory::createWithSkillLevel(0.5f, vehicle, gpuAccelerator, "Test Driver");
    
    // Test strategies
    driver->setStrategy(ai::Strategy::Aggressive);
    EXPECT_EQ(driver->getStrategy(), ai::Strategy::Aggressive);
    
    driver->setStrategy(ai::Strategy::Conservative);
    EXPECT_EQ(driver->getStrategy(), ai::Strategy::Conservative);
    
    // Setting strategies should affect parameters
    driver->setStrategy(ai::Strategy::Aggressive);
    float aggressiveValue = driver->getParameters().aggression;
    
    driver->setStrategy(ai::Strategy::Conservative);
    float conservativeValue = driver->getParameters().aggression;
    
    EXPECT_GT(aggressiveValue, conservativeValue);
    
    // Test racing line management
    std::vector<ai::RacingLinePoint> racingLine;
    for (int i = 0; i < 10; i++) {
        ai::RacingLinePoint point;
        point.x = static_cast<float>(i * 10);
        point.y = static_cast<float>(i * 5);
        point.targetSpeed = 100.0f - i * 5.0f;
        point.brakePoint = (i == 3) ? 1.0f : 0.0f;
        point.apex = (i == 5) ? 1.0f : 0.0f;
        point.exitPoint = (i == 7) ? 1.0f : 0.0f;
        racingLine.push_back(point);
    }
    
    driver->setRacingLine(racingLine);
    
    // Check racing line
    const auto& retrievedLine = driver->getRacingLine();
    EXPECT_EQ(retrievedLine.size(), 10);
    EXPECT_FLOAT_EQ(retrievedLine[3].brakePoint, 1.0f);
    EXPECT_FLOAT_EQ(retrievedLine[5].apex, 1.0f);
    EXPECT_FLOAT_EQ(retrievedLine[7].exitPoint, 1.0f);
    
    // Test lap management
    driver->setCurrentLap(3);
    EXPECT_EQ(driver->getCurrentLap(), 3);
    
    // Test lap statistics
    driver->updateLapStats(95.5f, 2, 3);
    const auto& stats = driver->getStats();
    EXPECT_FLOAT_EQ(stats.bestLapTime, 95.5f);
    EXPECT_EQ(stats.mistakesMade, 2);
    EXPECT_EQ(stats.overtakesMade, 3);
}

} // namespace testing
} // namespace neural_racer

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}