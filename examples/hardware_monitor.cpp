#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <atomic>

#include "neural_racer/hardware/device_driver.hpp"
#include "neural_racer/hardware/system_info.hpp"
#include "neural_racer/utils/logger.hpp"

using namespace neural_racer;

// ANSI color codes for color output
const std::string COLOR_RESET = "\033[0m";
const std::string COLOR_GREEN = "\033[32m";
const std::string COLOR_YELLOW = "\033[33m";
const std::string COLOR_RED = "\033[31m";
const std::string COLOR_BLUE = "\033[34m";
const std::string COLOR_CYAN = "\033[36m";
const std::string COLOR_BOLD = "\033[1m";

// Function to print a progress bar
void printProgressBar(float percentage, int width = 50) {
    int pos = static_cast<int>(width * percentage / 100.0f);
    
    // Choose color based on value
    std::string color;
    if (percentage < 30.0f) {
        color = COLOR_GREEN;
    } else if (percentage < 70.0f) {
        color = COLOR_YELLOW;
    } else {
        color = COLOR_RED;
    }
    
    std::cout << "[";
    for (int i = 0; i < width; ++i) {
        if (i < pos) std::cout << color << "=" << COLOR_RESET;
        else if (i == pos) std::cout << color << ">" << COLOR_RESET;
        else std::cout << " ";
    }
    
    std::cout << "] " << color << std::fixed << std::setprecision(1) << percentage << "%" << COLOR_RESET;
}

// Class for monitoring hardware devices
class HardwareMonitor {
public:
    HardwareMonitor() : running(false) {}
    
    ~HardwareMonitor() {
        if (running) {
            stop();
        }
    }
    
    // Initialize monitor with given GPU accelerator
    bool initialize(std::shared_ptr<hardware::GPUAccelerator> accel) {
        accelerator = accel;
        
        if (!accelerator || !accelerator->isInitialized()) {
            utils::Logger::error("HardwareMonitor", "Cannot initialize: GPU accelerator not initialized");
            return false;
        }
        
        utils::Logger::info("HardwareMonitor", "Hardware monitor initialized");
        return true;
    }
    
    // Start monitoring thread
    void start(int intervalMs = 1000) {
        if (running) {
            return;
        }
        
        running = true;
        monitorThread = std::thread(&HardwareMonitor::monitorFunction, this, intervalMs);
        utils::Logger::info("HardwareMonitor", "Monitoring started");
    }
    
    // Stop monitoring thread
    void stop() {
        if (!running) {
            return;
        }
        
        running = false;
        
        if (monitorThread.joinable()) {
            monitorThread.join();
        }
        
        utils::Logger::info("HardwareMonitor", "Monitoring stopped");
    }
    
private:
    std::shared_ptr<hardware::GPUAccelerator> accelerator;
    std::thread monitorThread;
    std::atomic<bool> running;
    
    // Monitoring thread function
    void monitorFunction(int intervalMs) {
        // Clear screen
        std::cout << "\033[2J\033[1;1H";
        
        while (running) {
            // Get metrics
            hardware::HardwareMetrics metrics = accelerator->getMetrics();
            
            // Get system info
            uint64_t totalMemory = hardware::SystemInfo::getTotalSystemMemory();
            uint64_t availableMemory = hardware::SystemInfo::getAvailableSystemMemory();
            int cpuCores = hardware::SystemInfo::getProcessorCoreCount();
            
            // Calculate memory usage percentage
            float memoryUsagePercent = 100.0f * (1.0f - static_cast<float>(availableMemory) / static_cast<float>(totalMemory));
            
            // Move cursor to top
            std::cout << "\033[1;1H";
            
            // Print system info
            std::cout << COLOR_BOLD << "=== System Information ===" << COLOR_RESET << std::endl;
            std::cout << "OS: " << hardware::SystemInfo::getOSName() << " " 
                      << hardware::SystemInfo::getOSVersion() << std::endl;
            std::cout << "CPU Cores: " << cpuCores << std::endl;
            std::cout << "Memory: " << std::fixed << std::setprecision(2)
                      << (totalMemory / (1024.0 * 1024.0 * 1024.0)) << " GB total, "
                      << (availableMemory / (1024.0 * 1024.0 * 1024.0)) << " GB available" << std::endl;
            std::cout << "System Memory Usage: ";
            printProgressBar(memoryUsagePercent);
            std::cout << std::endl;
            
            // Print GPUs
            std::cout << std::endl << COLOR_BOLD << "=== GPU Information ===" << COLOR_RESET << std::endl;
            auto gpuInfo = hardware::SystemInfo::getGPUInfo();
            for (const auto& gpu : gpuInfo) {
                std::cout << gpu << std::endl;
            }
            
            // Print real-time metrics
            std::cout << std::endl << COLOR_BOLD << "=== GPU Metrics ===" << COLOR_RESET << std::endl;
            
            // GPU utilization
            std::cout << "Utilization:     ";
            printProgressBar(metrics.utilizationPercent);
            std::cout << std::endl;
            
            // GPU memory
            float memoryUsedGB = metrics.memoryUsedBytes / (1024.0f * 1024.0f * 1024.0f);
            float memoryTotalGB = metrics.memoryTotalBytes / (1024.0f * 1024.0f * 1024.0f);
            float memoryPercent = 100.0f * memoryUsedGB / memoryTotalGB;
            
            std::cout << "Memory:          ";
            printProgressBar(memoryPercent);
            std::cout << " (" << std::fixed << std::setprecision(2) << memoryUsedGB 
                      << " GB / " << memoryTotalGB << " GB)" << std::endl;
            
            // GPU temperature
            std::cout << "Temperature:     ";
            std::string tempColor = metrics.temperatureCelsius < 60.0f ? COLOR_GREEN :
                                   (metrics.temperatureCelsius < 80.0f ? COLOR_YELLOW : COLOR_RED);
            printProgressBar(metrics.temperatureCelsius / 100.0f * 100.0f);
            std::cout << " (" << tempColor << metrics.temperatureCelsius << "°C" << COLOR_RESET << ")" << std::endl;
            
            // GPU clock speed
            std::cout << "Clock Speed:     " << metrics.clockSpeedMHz << " MHz" << std::endl;
            
            // GPU power consumption
            std::cout << "Power:           " << std::fixed << std::setprecision(2)
                      << metrics.powerConsumptionWatts << " W" << std::endl;
            
            // GPU efficiency
            std::cout << "Efficiency:      " << std::fixed << std::setprecision(2)
                      << metrics.performanceEfficiency << " ops/W" << std::endl;
            
            // Hardware acceleration status
            std::cout << "Hardware Acceleration: " << (accelerator->hasRealHardwareAccess() ? 
                      COLOR_GREEN + std::string("ENABLED") : 
                      COLOR_YELLOW + std::string("SIMULATED")) << COLOR_RESET << std::endl;
            
            // Add exit instructions
            std::cout << std::endl << "Press Ctrl+C to exit" << std::endl;
            
            // Sleep for interval
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        }
    }
};

int main(int argc, char* argv[]) {
    try {
        // Initialize logging
        utils::Logger::initialize();
        utils::Logger::setLogLevel(utils::LogLevel::Info);
        
        std::cout << "Neural Racer - Hardware Monitor Example" << std::endl;
        std::cout << "=====================================" << std::endl;
        
        // Display initial system information
        std::cout << "System Information:" << std::endl;
        std::cout << "  OS: " << hardware::SystemInfo::getOSName() << " " 
                  << hardware::SystemInfo::getOSVersion() << std::endl;
        std::cout << "  CPU Cores: " << hardware::SystemInfo::getProcessorCoreCount() << std::endl;
        std::cout << "  Memory: " << (hardware::SystemInfo::getTotalSystemMemory() / (1024 * 1024 * 1024))
                  << " GB total, " 
                  << (hardware::SystemInfo::getAvailableSystemMemory() / (1024 * 1024 * 1024))
                  << " GB available" << std::endl;
        
        // Display GPU information
        std::cout << "GPUs:" << std::endl;
        auto gpuInfo = hardware::SystemInfo::getGPUInfo();
        if (gpuInfo.empty()) {
            std::cout << "  No GPUs detected" << std::endl;
        } else {
            for (const auto& gpu : gpuInfo) {
                std::cout << "  " << gpu << std::endl;
            }
        }
        
        // Check for CUDA and OpenCL support
        std::cout << "CUDA Support: " << (hardware::SystemInfo::hasCUDASupport() ? "Yes" : "No") << std::endl;
        std::cout << "OpenCL Support: " << (hardware::SystemInfo::hasOpenCLSupport() ? "Yes" : "No") << std::endl;
        
        // Create GPU accelerator
        std::cout << "Initializing GPU accelerator..." << std::endl;
        auto gpuAccelerator = std::make_shared<hardware::GPUAccelerator>();
        
        // Initialize GPU accelerator
        if (!gpuAccelerator->initialize()) {
            std::cout << "Failed to initialize GPU accelerator, using simulation mode" << std::endl;
        } else {
            std::cout << "GPU accelerator initialized successfully" << std::endl;
            
            // Set to performance mode
            gpuAccelerator->setPerformanceMode(2);
        }
        
        // Create hardware monitor
        HardwareMonitor monitor;
        if (!monitor.initialize(gpuAccelerator)) {
            std::cout << "Failed to initialize hardware monitor" << std::endl;
            return 1;
        }
        
        // Start monitoring (500ms update interval)
        monitor.start(500);
        
        // Keep program running until user interrupts
        std::cout << "Press Enter to exit..." << std::endl;
        std::cin.get();
        
        // Stop monitoring
        monitor.stop();
        
        // Shutdown GPU accelerator
        gpuAccelerator->shutdown();
        std::cout << "GPU accelerator shut down" << std::endl;
        
        // Shutdown logging
        utils::Logger::shutdown();
        
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}