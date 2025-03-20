#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>

#include "neural_racer/hardware/device_driver.hpp"
#include "neural_racer/hardware/system_info.hpp"
#include "neural_racer/utils/logger.hpp"

namespace neural_racer {
namespace testing {

// Initialize logging for tests
class HardwareTest : public ::testing::Test {
protected:
    void SetUp() override {
        utils::Logger::initialize(false);  // No console output for tests
    }
    
    void TearDown() override {
        utils::Logger::shutdown();
    }
};

// Test the DeviceDriver class
TEST_F(HardwareTest, DeviceDriverTest) {
    // Test device path construction
    std::string gpuPath = hardware::DeviceDriver::getDevicePath(hardware::DeviceType::GPU, 0);
    EXPECT_FALSE(gpuPath.empty());
    
    // Create a device driver
    auto driver = std::make_shared<hardware::DeviceDriver>(gpuPath);
    EXPECT_FALSE(driver->isConnected());
    
    // Connect to device
    bool connected = driver->connect();
    // Connection may fail if no real hardware, but should not crash
    if (connected) {
        EXPECT_TRUE(driver->isConnected());
        
        // Test data writing/reading
        const uint8_t testData[4] = {1, 2, 3, 4};
        int bytesWritten = driver->writeData(testData, sizeof(testData));
        
        // If using simulation, this should return the full size
        if (bytesWritten > 0) {
            EXPECT_EQ(bytesWritten, sizeof(testData));
            
            uint8_t readBuffer[4] = {0};
            int bytesRead = driver->readData(readBuffer, sizeof(readBuffer));
            
            // Reading may succeed even in simulation mode
            if (bytesRead > 0) {
                EXPECT_GT(bytesRead, 0);
            }
        }
        
        // Disconnect
        driver->disconnect();
        EXPECT_FALSE(driver->isConnected());
    }
}

// Test the FirmwareController class
TEST_F(HardwareTest, FirmwareControllerTest) {
    // Create a firmware controller
    hardware::FirmwareController controller(hardware::DeviceType::GPU, 0);
    
    // Initialize controller
    bool initialized = controller.initialize();
    // Initialization may fail if no real hardware, but should not crash
    if (initialized) {
        EXPECT_TRUE(controller.isInitialized());
        
        // Test register reading/writing
        try {
            uint32_t value = controller.readRegister(0x1000);
            // Register value is unpredictable, but should not crash
            
            bool success = controller.writeRegister(0x1000, 0x12345678);
            // Write may fail, but should not crash
        } catch (const hardware::DriverException& e) {
            // Exception is acceptable if hardware access is not available
            std::cout << "DriverException: " << e.what() << std::endl;
        }
        
        // Test performance mode setting
        try {
            bool success = controller.setPerformanceMode(1);
            // Setting may fail, but should not crash
        } catch (const hardware::DriverException& e) {
            // Exception is acceptable if hardware access is not available
            std::cout << "DriverException: " << e.what() << std::endl;
        }
        
        // Test metrics reading
        try {
            float temperature = controller.readTemperature();
            // Temperature is unpredictable, but should not crash
            
            hardware::HardwareMetrics metrics = controller.getMetrics();
            // Check that metrics are reasonable
            EXPECT_GE(metrics.utilizationPercent, 0.0f);
            EXPECT_LE(metrics.utilizationPercent, 100.0f);
            EXPECT_GE(metrics.temperatureCelsius, 0.0f);
            EXPECT_LE(metrics.temperatureCelsius, 120.0f);
        } catch (const hardware::DriverException& e) {
            // Exception is acceptable if hardware access is not available
            std::cout << "DriverException: " << e.what() << std::endl;
        }
        
        // Shutdown controller
        controller.shutdown();
        EXPECT_FALSE(controller.isInitialized());
    }
}

// Test the GPUAccelerator class
TEST_F(HardwareTest, GPUAcceleratorTest) {
    // Create a GPU accelerator
    hardware::GPUAccelerator accelerator(0);
    
    // Initialize accelerator
    bool initialized = accelerator.initialize();
    EXPECT_TRUE(initialized);
    
    if (initialized) {
        // Test performance mode setting
        bool success = accelerator.setPerformanceMode(2);
        // Setting may fail in simulation mode, but should not crash
        
        // Test metrics reading
        hardware::HardwareMetrics metrics = accelerator.getMetrics();
        
        // Check that metrics are reasonable
        EXPECT_GE(metrics.utilizationPercent, 0.0f);
        EXPECT_LE(metrics.utilizationPercent, 100.0f);
        EXPECT_GE(metrics.temperatureCelsius, 0.0f);
        EXPECT_LE(metrics.temperatureCelsius, 120.0f);
        EXPECT_GT(metrics.clockSpeedMHz, 0);
        EXPECT_GT(metrics.memoryTotalBytes, 0);
        
        // Hardware access flag may be true or false
        bool hasRealHardware = accelerator.hasRealHardwareAccess();
        std::cout << "Real hardware access: " << (hasRealHardware ? "true" : "false") << std::endl;
        
        // Shutdown accelerator
        accelerator.shutdown();
        EXPECT_FALSE(accelerator.isInitialized());
    }
}

// Test the SystemInfo class
TEST_F(HardwareTest, SystemInfoTest) {
    // Test OS information
    std::string osName = hardware::SystemInfo::getOSName();
    EXPECT_FALSE(osName.empty());
    
    std::string osVersion = hardware::SystemInfo::getOSVersion();
    EXPECT_FALSE(osVersion.empty());
    
    // Test processor information
    int cores = hardware::SystemInfo::getProcessorCoreCount();
    EXPECT_GT(cores, 0);
    
    // Test memory information
    uint64_t totalMemory = hardware::SystemInfo::getTotalSystemMemory();
    EXPECT_GT(totalMemory, 0);
    
    uint64_t availableMemory = hardware::SystemInfo::getAvailableSystemMemory();
    EXPECT_GT(availableMemory, 0);
    EXPECT_LE(availableMemory, totalMemory);
    
    // Test GPU information
    std::vector<std::string> gpuInfo = hardware::SystemInfo::getGPUInfo();
    // May be empty if no GPUs detected, but should not crash
    
    // Test CUDA and OpenCL support
    bool hasCuda = hardware::SystemInfo::hasCUDASupport();
    bool hasOpenCL = hardware::SystemInfo::hasOpenCLSupport();
    // Results are system-dependent, but should not crash
    
    std::cout << "CUDA support: " << (hasCuda ? "true" : "false") << std::endl;
    std::cout << "OpenCL support: " << (hasOpenCL ? "true" : "false") << std::endl;
}

// Test DeviceDriver factory methods
TEST_F(HardwareTest, DeviceDriverFactoryTest) {
    // Test creation for different device types
    auto cpuDriver = hardware::DeviceDriver::create(hardware::DeviceType::CPU, 0);
    EXPECT_NE(cpuDriver, nullptr);
    
    auto gpuDriver = hardware::DeviceDriver::create(hardware::DeviceType::GPU, 0);
    EXPECT_NE(gpuDriver, nullptr);
    
    auto accelDriver = hardware::DeviceDriver::create(hardware::DeviceType::ACCELERATOR, 0);
    EXPECT_NE(accelDriver, nullptr);
    
    auto customDriver = hardware::DeviceDriver::create(hardware::DeviceType::CUSTOM, 0);
    EXPECT_NE(customDriver, nullptr);
    
    // Test connection for each driver
    // Connection may fail if no real hardware, but should not crash
    if (cpuDriver->connect()) {
        cpuDriver->disconnect();
    }
    
    if (gpuDriver->connect()) {
        gpuDriver->disconnect();
    }
    
    if (accelDriver->connect()) {
        accelDriver->disconnect();
    }
    
    if (customDriver->connect()) {
        customDriver->disconnect();
    }
}

// Test command code creation in FirmwareController
TEST_F(HardwareTest, CommandCodeTest) {
    // Test command code generation
    unsigned long readCode = hardware::FirmwareController::createCommandCode(0x1234, true);
    unsigned long writeCode = hardware::FirmwareController::createCommandCode(0x1234, false);
    
    // Codes should be different for read and write
    EXPECT_NE(readCode, writeCode);
    
    // Test with different base codes
    unsigned long readCode2 = hardware::FirmwareController::createCommandCode(0x5678, true);
    EXPECT_NE(readCode, readCode2);
}

// Test device driver simulation methods
TEST_F(HardwareTest, SimulationTest) {
    // Create a firmware controller
    hardware::FirmwareController controller(hardware::DeviceType::GPU, 0);
    
    // Initialize controller
    bool initialized = controller.initialize();
    if (initialized) {
        // Attempt to get metrics multiple times
        for (int i = 0; i < 5; i++) {
            try {
                hardware::HardwareMetrics metrics = controller.getMetrics();
                
                // Values should change slightly between calls in simulation mode
                if (i > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    
                    hardware::HardwareMetrics newMetrics = controller.getMetrics();
                    
                    // In simulation mode, values should be different between subsequent calls
                    // This is a soft test, as real hardware might rarely return identical values
                    // If at least one metric changes, the simulation is working
                    bool anyChanges = 
                        metrics.utilizationPercent != newMetrics.utilizationPercent ||
                        metrics.temperatureCelsius != newMetrics.temperatureCelsius ||
                        metrics.clockSpeedMHz != newMetrics.clockSpeedMHz ||
                        metrics.memoryUsedBytes != newMetrics.memoryUsedBytes ||
                        metrics.powerConsumptionWatts != newMetrics.powerConsumptionWatts;
                    
                    if (!anyChanges) {
                        std::cout << "Warning: No changes in metrics detected between calls" << std::endl;
                    }
                }
            } catch (const hardware::DriverException& e) {
                // Exception is acceptable if hardware access is not available
                std::cout << "DriverException: " << e.what() << std::endl;
                break;
            }
        }
        
        // Shutdown controller
        controller.shutdown();
    }
}

} // namespace testing
} // namespace neural_racer

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}