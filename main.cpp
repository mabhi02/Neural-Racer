#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <iomanip>

// Include core headers
#include "neural_racer/hardware/device_driver.hpp"
#include "neural_racer/hardware/system_info.hpp"
#include "neural_racer/ai/inference.hpp"
#include "neural_racer/ai/driver.hpp"
#include "neural_racer/physics/vehicle.hpp"
#include "neural_racer/physics/engine.hpp"
#include "neural_racer/simulation/track.hpp"
#include "neural_racer/simulation/race.hpp"
#include "neural_racer/simulation/telemetry.hpp"
#include "neural_racer/utils/logger.hpp"
#include "neural_racer/utils/profiler.hpp"

using namespace neural_racer;

// Helper function to print hardware metrics
void printHardwareMetrics(const hardware::HardwareMetrics& metrics) {
    std::cout << "Hardware Metrics:" << std::endl;
    std::cout << "  Utilization: " << std::fixed << std::setprecision(1) 
              << metrics.utilizationPercent << "%" << std::endl;
    std::cout << "  Temperature: " << metrics.temperatureCelsius << "°C" << std::endl;
    std::cout << "  Clock Speed: " << metrics.clockSpeedMHz << " MHz" << std::endl;
    std::cout << "  Memory Used: " << (metrics.memoryUsedBytes / (1024.0 * 1024.0)) 
              << " MB / " << (metrics.memoryTotalBytes / (1024.0 * 1024.0)) << " MB" << std::endl;
    std::cout << "  Power: " << metrics.powerConsumptionWatts << " W" << std::endl;
    std::cout << "  Efficiency: " << metrics.performanceEfficiency << " ops/W" << std::endl;
}

// Helper function to print inference metrics
void printInferenceMetrics(const ai::InferenceMetrics& metrics) {
    std::cout << "Inference Metrics:" << std::endl;
    std::cout << "  Preprocess Time: " << std::fixed << std::setprecision(3) 
              << metrics.preprocessTimeMs << " ms" << std::endl;
    std::cout << "  Inference Time: " << metrics.inferenceTimeMs << " ms" << std::endl;
    std::cout << "  Postprocess Time: " << metrics.postprocessTimeMs << " ms" << std::endl;
    std::cout << "  Total Time: " << metrics.totalTimeMs << " ms" << std::endl;
    std::cout << "  Throughput: " << metrics.throughputFPS << " FPS" << std::endl;
    std::cout << "  Batch Size: " << metrics.batchSize << std::endl;
    std::cout << "  Hardware Accelerated: " << (metrics.hardwareAccelerated ? "Yes" : "No") << std::endl;
    if (metrics.hardwareAccelerated) {
        std::cout << "  Accelerator: " << metrics.acceleratorName << std::endl;
    }
}

// Display system information
void printSystemInfo() {
    std::cout << "=== System Information ===" << std::endl;
    std::cout << "OS: " << hardware::SystemInfo::getOSName() << " " 
              << hardware::SystemInfo::getOSVersion() << std::endl;
    std::cout << "CPU Cores: " << hardware::SystemInfo::getProcessorCoreCount() << std::endl;
    std::cout << "Memory: " << (hardware::SystemInfo::getTotalSystemMemory() / (1024.0 * 1024.0 * 1024.0))
              << " GB total, " 
              << (hardware::SystemInfo::getAvailableSystemMemory() / (1024.0 * 1024.0 * 1024.0))
              << " GB available" << std::endl;
    
    std::cout << "GPUs:" << std::endl;
    auto gpuInfo = hardware::SystemInfo::getGPUInfo();
    if (gpuInfo.empty()) {
        std::cout << "  No GPUs detected" << std::endl;
    } else {
        for (const auto& gpu : gpuInfo) {
            std::cout << "  " << gpu << std::endl;
        }
    }
    
    std::cout << "CUDA Support: " << (hardware::SystemInfo::hasCUDASupport() ? "Yes" : "No") << std::endl;
    std::cout << "OpenCL Support: " << (hardware::SystemInfo::hasOpenCLSupport() ? "Yes" : "No") << std::endl;
    std::cout << std::endl;
}

// Example 1: Basic hardware access and monitoring
void runHardwareExample() {
    std::cout << "=== Hardware Interface Example ===" << std::endl;
    
    // Create GPU accelerator
    auto gpuAccelerator = std::make_shared<hardware::GPUAccelerator>();
    
    // Initialize GPU accelerator
    if (!gpuAccelerator->initialize()) {
        std::cout << "Failed to initialize GPU accelerator, using simulation mode" << std::endl;
    } else {
        std::cout << "GPU accelerator initialized successfully" << std::endl;
        
        // Set performance mode
        if (gpuAccelerator->setPerformanceMode(2)) {
            std::cout << "Set GPU to performance mode" << std::endl;
        }
    }
    
    // Monitor GPU metrics for a few seconds
    std::cout << "Monitoring GPU metrics..." << std::endl;
    for (int i = 0; i < 5; i++) {
        auto metrics = gpuAccelerator->getMetrics();
        printHardwareMetrics(metrics);
        
        std::cout << "Waiting 1 second..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // Shutdown GPU accelerator
    gpuAccelerator->shutdown();
    std::cout << "GPU accelerator shut down" << std::endl;
    std::cout << std::endl;
}

// Example 2: Neural network inference
void runInferenceExample() {
    std::cout << "=== Neural Network Inference Example ===" << std::endl;
    
    // Create GPU accelerator
    auto gpuAccelerator = std::make_shared<hardware::GPUAccelerator>();
    gpuAccelerator->initialize();
    
    // Create inference engine with a simulated model path
    // In a real implementation, this would be a path to an ONNX or other model file
    ai::InferenceEngine inferenceEngine("models/driver_model.onnx", gpuAccelerator);
    
    // Initialize inference engine
    if (!inferenceEngine.initialize()) {
        std::cout << "Failed to initialize inference engine" << std::endl;
        return;
    }
    
    std::cout << "Inference engine initialized successfully" << std::endl;
    
    // Get model information
    const auto& modelInfo = inferenceEngine.getModelInfo();
    std::cout << "Model Information:" << std::endl;
    std::cout << "  Name: " << modelInfo.name << std::endl;
    std::cout << "  Format: ";
    switch (modelInfo.format) {
        case ai::ModelFormat::ONNX: std::cout << "ONNX"; break;
        case ai::ModelFormat::TensorFlow: std::cout << "TensorFlow"; break;
        case ai::ModelFormat::PyTorch: std::cout << "PyTorch"; break;
        case ai::ModelFormat::Custom: std::cout << "Custom"; break;
    }
    std::cout << std::endl;
    
    std::cout << "  Precision: ";
    switch (modelInfo.precision) {
        case ai::PrecisionType::FP32: std::cout << "FP32"; break;
        case ai::PrecisionType::FP16: std::cout << "FP16"; break;
        case ai::PrecisionType::INT8: std::cout << "INT8"; break;
        case ai::PrecisionType::Mixed: std::cout << "Mixed"; break;
    }
    std::cout << std::endl;
    
    std::cout << "  Parameters: " << (modelInfo.parameterCount / 1000000.0) << " million" << std::endl;
    std::cout << "  Memory Footprint: " << (modelInfo.memoryFootprintBytes / (1024.0 * 1024.0)) 
              << " MB" << std::endl;
    
    // Create an input tensor
    // In a real implementation, this would be filled with sensor data
    ai::Tensor inputTensor({1, 10}, std::vector<float>(10, 0.5f));
    
    // Set the input tensor
    inferenceEngine.setInput("input", inputTensor);
    
    // Run inference multiple times
    std::cout << "Running inference..." << std::endl;
    for (int i = 0; i < 5; i++) {
        // Run inference
        if (!inferenceEngine.runInference()) {
            std::cout << "Inference failed" << std::endl;
            continue;
        }
        
        // Get output tensor
        ai::Tensor outputTensor = inferenceEngine.getOutput("output");
        
        // Print output values (first few)
        std::cout << "Output values: ";
        for (size_t j = 0; j < std::min(size_t(5), outputTensor.data.size()); j++) {
            std::cout << outputTensor.data[j] << " ";
        }
        std::cout << std::endl;
        
        // Print inference metrics
        printInferenceMetrics(inferenceEngine.getMetrics());
        
        // Wait a bit
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // Shutdown inference engine
    inferenceEngine.shutdown();
    std::cout << "Inference engine shut down" << std::endl;
    std::cout << std::endl;
}

// Example 3: AI driver and vehicle simulation
void runSimulationExample() {
    std::cout << "=== Simulation Example ===" << std::endl;
    
    // Create GPU accelerator
    auto gpuAccelerator = std::make_shared<hardware::GPUAccelerator>();
    gpuAccelerator->initialize();
    
    // Create physics engine
    auto physicsEngine = std::make_shared<physics::Engine>(gpuAccelerator);
    physicsEngine->initialize();
    
    // Create track
    auto track = std::make_shared<simulation::Track>("Monza");
    // In a real implementation, we would load track data here
    
    // Create vehicle
    physics::VehicleSpec vehicleSpec;
    vehicleSpec.name = "Formula Racing Car";
    vehicleSpec.mass = 720.0f; // kg
    vehicleSpec.power = 735.0f; // kW (~ 1000 HP)
    vehicleSpec.drag = 0.35f;
    vehicleSpec.frontGrip = 1.5f;
    vehicleSpec.rearGrip = 1.4f;
    vehicleSpec.brakingForce = 60000.0f; // N
    
    auto vehicle = std::make_shared<physics::Vehicle>(vehicleSpec, physicsEngine);
    
    // Create AI driver
    auto driverModel = ai::DriverModelFactory::createWithSkillLevel(0.8f, gpuAccelerator);
    auto driver = std::make_shared<ai::Driver>(driverModel, vehicle);
    
    // Create race simulation
    auto race = std::make_shared<simulation::Race>(track);
    race->addDriver(driver);
    
    // Create telemetry system
    auto telemetry = std::make_shared<simulation::Telemetry>();
    
    // Run simulation for a few steps
    std::cout << "Running simulation..." << std::endl;
    const float timeStep = 0.016f; // 60 FPS
    for (int i = 0; i < 60; i++) { // 1 second of simulation
        // Update race simulation
        race->update(timeStep);
        
        // Update telemetry
        telemetry->update(race);
        
        // Every 10 frames, print some stats
        if (i % 10 == 0) {
            // Get vehicle state
            auto vehicleState = vehicle->getState();
            
            std::cout << "Vehicle State:" << std::endl;
            std::cout << "  Position: (" << vehicleState.positionX << ", " 
                      << vehicleState.positionY << ")" << std::endl;
            std::cout << "  Speed: " << vehicleState.velocity * 3.6 << " km/h" << std::endl;
            std::cout << "  Acceleration: " << vehicleState.acceleration << " m/s²" << std::endl;
            std::cout << "  Engine RPM: " << vehicleState.engineRPM << std::endl;
            std::cout << "  Gear: " << vehicleState.gear << std::endl;
            std::cout << "  Throttle: " << vehicleState.throttle * 100.0f << "%" << std::endl;
            std::cout << "  Brake: " << vehicleState.brake * 100.0f << "%" << std::endl;
            std::cout << "  Steering: " << vehicleState.steeringAngle << " rad" << std::endl;
        }
        
        // Simulate time passing
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }
    
    // Print lap statistics
    auto lapStats = telemetry->getLapStatistics();
    std::cout << "Lap Statistics:" << std::endl;
    std::cout << "  Current Lap: " << lapStats.currentLap << std::endl;
    std::cout << "  Best Lap Time: " << lapStats.bestLapTimeSeconds << " seconds" << std::endl;
    std::cout << "  Last Lap Time: " << lapStats.lastLapTimeSeconds << " seconds" << std::endl;
    std::cout << "  Average Speed: " << lapStats.averageSpeedKmh << " km/h" << std::endl;
    std::cout << "  Top Speed: " << lapStats.topSpeedKmh << " km/h" << std::endl;
    
    // Shutdown systems
    race->shutdown();
    physicsEngine->shutdown();
    gpuAccelerator->shutdown();
    std::cout << "Simulation shut down" << std::endl;
    std::cout << std::endl;
}

int main() {
    try {
        // Setup logging
        utils::Logger::initialize();
        utils::Logger::setLogLevel(utils::LogLevel::Info);
        
        // Print system information
        printSystemInfo();
        
        // Run examples
        runHardwareExample();
        runInferenceExample();
        runSimulationExample();
        
        // Clean up
        utils::Logger::shutdown();
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}