#pragma once

#include <string>
#include <memory>
#include <vector>
#include <cstdint>

// Platform-specific includes
#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <sys/ioctl.h>
#endif

namespace neural_racer {
namespace hardware {

/**
 * @brief Hardware device type enumeration
 */
enum class DeviceType {
    CPU,         ///< Central processing unit
    GPU,         ///< Graphics processing unit
    ACCELERATOR, ///< Neural network accelerator
    CUSTOM       ///< Custom hardware device
};

/**
 * @brief Hardware performance metrics structure
 */
struct HardwareMetrics {
    float utilizationPercent = 0.0f; ///< Device utilization percentage
    float temperatureCelsius = 0.0f; ///< Device temperature in Celsius
    uint32_t clockSpeedMHz = 0;      ///< Clock speed in MHz
    uint64_t memoryUsedBytes = 0;    ///< Memory usage in bytes
    uint64_t memoryTotalBytes = 0;   ///< Total memory in bytes
    float powerConsumptionWatts = 0.0f; ///< Power consumption in watts
    float performanceEfficiency = 0.0f;  ///< Operations per watt
};

/**
 * @brief Exception thrown for driver-related errors
 */
class DriverException : public std::runtime_error {
public:
    explicit DriverException(const std::string& message) 
        : std::runtime_error(message) {}
};

/**
 * @brief Low-level interface to hardware device drivers
 * 
 * This class provides a cross-platform abstraction for interacting with
 * hardware device drivers at the operating system level.
 */
class DeviceDriver {
public:
    /**
     * @brief Construct a device driver interface
     * 
     * @param devicePath Path to the device file or identifier
     */
    explicit DeviceDriver(const std::string& devicePath);
    
    /**
     * @brief Destroy the device driver interface and release resources
     */
    virtual ~DeviceDriver();
    
    /**
     * @brief Connect to the device driver
     * 
     * @return true if connection was successful, false otherwise
     * @throws DriverException if the connection fails with a specific error
     */
    bool connect();
    
    /**
     * @brief Disconnect from the device driver
     */
    void disconnect();
    
    /**
     * @brief Check if the driver is connected
     * 
     * @return true if connected, false otherwise
     */
    bool isConnected() const;
    
    /**
     * @brief Send a command to the device driver
     * 
     * @param command Command code to send
     * @param data Pointer to data buffer for the command
     * @param size Size of the data buffer in bytes
     * @return true if the command was successful, false otherwise
     * @throws DriverException if the command fails with a specific error
     */
    bool sendCommand(unsigned long command, void* data, size_t size);
    
    /**
     * @brief Read data from the device
     * 
     * @param buffer Buffer to store the read data
     * @param size Size of the buffer in bytes
     * @return Number of bytes read, or -1 on error
     * @throws DriverException if the read fails with a specific error
     */
    int readData(void* buffer, size_t size);
    
    /**
     * @brief Write data to the device
     * 
     * @param buffer Buffer containing data to write
     * @param size Size of the buffer in bytes
     * @return Number of bytes written, or -1 on error
     * @throws DriverException if the write fails with a specific error
     */
    int writeData(const void* buffer, size_t size);
    
    /**
     * @brief Get the device path
     * 
     * @return The device path string
     */
    const std::string& getDevicePath() const;
    
    /**
     * @brief Create a device driver for a specific hardware type
     * 
     * @param type Hardware device type
     * @param deviceIndex Device index (for multiple devices of same type)
     * @return std::shared_ptr<DeviceDriver> Shared pointer to device driver
     */
    static std::shared_ptr<DeviceDriver> create(DeviceType type, int deviceIndex = 0);

private:
    std::string devicePath;
    bool connected = false;
    
#ifdef _WIN32
    HANDLE deviceHandle = INVALID_HANDLE_VALUE;
#else
    int deviceHandle = -1;
#endif

    // Get the last error message from the operating system
    std::string getLastErrorMessage() const;
    
    // Get the appropriate device path for the given device type and index
    static std::string getDevicePath(DeviceType type, int deviceIndex);
};

/**
 * @brief Firmware access controller for hardware devices
 * 
 * This class provides firmware-level access to hardware devices,
 * allowing for low-level control and monitoring of device functionality.
 */
class FirmwareController {
public:
    /**
     * @brief Construct a firmware controller for a device
     * 
     * @param type Hardware device type
     * @param deviceIndex Device index (for multiple devices of same type)
     */
    FirmwareController(DeviceType type, int deviceIndex = 0);
    
    /**
     * @brief Initialize firmware controller and connect to device
     * 
     * @return true if initialization was successful, false otherwise
     * @throws DriverException if initialization fails with a specific error
     */
    bool initialize();
    
    /**
     * @brief Shutdown firmware controller and disconnect from device
     */
    void shutdown();
    
    /**
     * @brief Check if firmware controller is initialized
     * 
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;
    
    /**
     * @brief Read a register value from the device firmware
     * 
     * @param address Register address
     * @return uint32_t Register value
     * @throws DriverException if the read fails with a specific error
     */
    uint32_t readRegister(uint32_t address);
    
    /**
     * @brief Write a value to a device register
     * 
     * @param address Register address
     * @param value Value to write
     * @return true if the write was successful, false otherwise
     * @throws DriverException if the write fails with a specific error
     */
    bool writeRegister(uint32_t address, uint32_t value);
    
    /**
     * @brief Set device performance mode
     * 
     * @param mode Performance mode (0=Power saving, 1=Balanced, 2=Performance)
     * @return true if the mode was set successfully, false otherwise
     * @throws DriverException if setting the mode fails with a specific error
     */
    bool setPerformanceMode(uint32_t mode);
    
    /**
     * @brief Read current device temperature
     * 
     * @return float Temperature in degrees Celsius
     * @throws DriverException if reading the temperature fails with a specific error
     */
    float readTemperature();
    
    /**
     * @brief Read current device performance metrics
     * 
     * @return HardwareMetrics Current hardware metrics
     * @throws DriverException if reading the metrics fails with a specific error
     */
    HardwareMetrics getMetrics();
    
    /**
     * @brief Update device firmware
     * 
     * @param firmwareData Vector containing firmware binary data
     * @return true if update was successful, false otherwise
     * @throws DriverException if the update fails with a specific error
     */
    bool updateFirmware(const std::vector<uint8_t>& firmwareData);
    
    /**
     * @brief Create a device-specific command code
     * 
     * This method creates a command code that can be used with the device driver,
     * taking into account platform-specific command code requirements.
     * 
     * @param base Base command code
     * @param read True for read command, false for write command
     * @return unsigned long Platform-specific command code
     */
    static unsigned long createCommandCode(uint32_t base, bool read);

private:
    std::shared_ptr<DeviceDriver> driver;
    DeviceType deviceType;
    int deviceIndex;
    bool initialized = false;
    
    // Command codes for firmware operations
    static constexpr uint32_t CMD_READ_REGISTER = 0x0001;
    static constexpr uint32_t CMD_WRITE_REGISTER = 0x0002;
    static constexpr uint32_t CMD_SET_PERFORMANCE = 0x0003;
    static constexpr uint32_t CMD_GET_TEMPERATURE = 0x0004;
    static constexpr uint32_t CMD_GET_METRICS = 0x0005;
    static constexpr uint32_t CMD_UPDATE_FIRMWARE = 0x0006;
};

/**
 * @brief GPU hardware accelerator
 * 
 * This class provides access to GPU hardware for acceleration of
 * physics simulations and neural network inference.
 */
class GPUAccelerator {
public:
    /**
     * @brief Construct a GPU accelerator
     * 
     * @param deviceIndex GPU device index
     */
    explicit GPUAccelerator(int deviceIndex = 0);
    
    /**
     * @brief Initialize the GPU accelerator
     * 
     * @return true if initialization was successful, false otherwise
     * @throws DriverException if initialization fails with a specific error
     */
    bool initialize();
    
    /**
     * @brief Shutdown the GPU accelerator
     */
    void shutdown();
    
    /**
     * @brief Check if GPU accelerator is initialized
     * 
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;
    
    /**
     * @brief Set GPU performance mode
     * 
     * @param mode Performance mode (0=Power saving, 1=Balanced, 2=Performance)
     * @return true if the mode was set successfully, false otherwise
     */
    bool setPerformanceMode(uint32_t mode);
    
    /**
     * @brief Get current GPU metrics
     * 
     * @return HardwareMetrics Current GPU metrics
     */
    HardwareMetrics getMetrics();
    
    /**
     * @brief Check if GPU is available with real hardware access
     * 
     * @return true if real GPU access is available, false if using simulation
     */
    bool hasRealHardwareAccess() const;

private:
    std::unique_ptr<FirmwareController> firmwareController;
    int deviceIndex;
    bool initialized = false;
    bool realHardwareAccess = false;
    
    // Use simulated hardware if real hardware access is not available
    void simulateHardwareMetrics(HardwareMetrics& metrics);
};

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