#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace neural_racer {
namespace hardware {

/**
 * @brief System information utility
 * 
 * This class provides information about the system hardware and
 * operating system.
 */
class SystemInfo {
public:
    /**
     * @brief Get operating system name
     * 
     * @return std::string Operating system name
     */
    static std::string getOSName();
    
    /**
     * @brief Get operating system version
     * 
     * @return std::string Operating system version
     */
    static std::string getOSVersion();
    
    /**
     * @brief Get processor core count
     * 
     * @return int Number of processor cores
     */
    static int getProcessorCoreCount();
    
    /**
     * @brief Get total system memory
     * 
     * @return uint64_t Total system memory in bytes
     */
    static uint64_t getTotalSystemMemory();
    
    /**
     * @brief Get available system memory
     * 
     * @return uint64_t Available system memory in bytes
     */
    static uint64_t getAvailableSystemMemory();
    
    /**
     * @brief Get information about available GPUs
     * 
     * @return std::vector<std::string> Vector of GPU information strings
     */
    static std::vector<std::string> getGPUInfo();
    
    /**
     * @brief Check if the system has CUDA support
     * 
     * @return true if CUDA is supported, false otherwise
     */
    static bool hasCUDASupport();
    
    /**
     * @brief Check if the system has OpenCL support
     * 
     * @return true if OpenCL is supported, false otherwise
     */
    static bool hasOpenCLSupport();
};

} // namespace hardware
} // namespace neural_racer