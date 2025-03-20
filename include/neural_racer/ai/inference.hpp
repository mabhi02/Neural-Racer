#pragma once

#define _USE_MATH_DEFINES  // For M_PI and other math constants
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <unordered_map>
#include <thread>  // For std::this_thread
#include <cmath>   // For math functions
#include "../hardware/device_driver.hpp"

namespace neural_racer {
namespace ai {

/**
 * @brief Neural network model format enumeration
 */
enum class ModelFormat {
    ONNX,       ///< Open Neural Network Exchange format
    TensorFlow, ///< TensorFlow SavedModel or frozen graph
    PyTorch,    ///< PyTorch TorchScript format
    Custom      ///< Custom model format
};

/**
 * @brief Inference precision type enumeration
 */
enum class PrecisionType {
    FP32,  ///< 32-bit floating point
    FP16,  ///< 16-bit floating point
    INT8,  ///< 8-bit integer quantization
    Mixed  ///< Mixed precision
};

/**
 * @brief Tensor shape and data
 */
struct Tensor {
    std::vector<int64_t> shape;  ///< Tensor dimensions
    std::vector<float> data;     ///< Tensor data (flattened)
    
    /**
     * @brief Create an empty tensor
     */
    Tensor() = default;
    
    /**
     * @brief Create a tensor with the given shape and initialize with zeros
     * 
     * @param shape Tensor dimensions
     */
    explicit Tensor(const std::vector<int64_t>& shape);
    
    /**
     * @brief Create a tensor with the given shape and data
     * 
     * @param shape Tensor dimensions
     * @param data Tensor data (flattened)
     */
    Tensor(const std::vector<int64_t>& shape, const std::vector<float>& data);
    
    /**
     * @brief Get the total number of elements in the tensor
     * 
     * @return size_t Number of elements
     */
    size_t size() const;
    
    /**
     * @brief Reshape the tensor to the given dimensions
     * 
     * The total number of elements must remain the same.
     * 
     * @param newShape New tensor dimensions
     * @return true if reshape was successful, false otherwise
     */
    bool reshape(const std::vector<int64_t>& newShape);
};

/**
 * @brief Neural network model information
 */
struct ModelInfo {
    std::string name;                           ///< Model name
    ModelFormat format;                          ///< Model format
    PrecisionType precision;                     ///< Model precision
    std::unordered_map<std::string, Tensor> inputs;  ///< Input tensors (name -> tensor)
    std::unordered_map<std::string, Tensor> outputs; ///< Output tensors (name -> tensor)
    uint64_t parameterCount;                     ///< Number of model parameters
    uint64_t memoryFootprintBytes;               ///< Approximate memory footprint in bytes
};

/**
 * @brief Inference performance metrics
 */
struct InferenceMetrics {
    float preprocessTimeMs = 0.0f;      ///< Preprocessing time in milliseconds
    float inferenceTimeMs = 0.0f;       ///< Inference time in milliseconds
    float postprocessTimeMs = 0.0f;     ///< Postprocessing time in milliseconds
    float totalTimeMs = 0.0f;           ///< Total processing time in milliseconds
    int batchSize = 1;                  ///< Inference batch size
    int inferenceCount = 0;             ///< Total number of inferences run
    float throughputFPS = 0.0f;         ///< Throughput in frames per second
    bool hardwareAccelerated = false;   ///< Whether hardware acceleration was used
    std::string acceleratorName;        ///< Name of the accelerator hardware
};

/**
 * @brief Check if a string ends with a specific suffix (C++17 compatible replacement for std::string::ends_with)
 * 
 * @param str The string to check
 * @param suffix The suffix to look for
 * @return true if the string ends with the suffix, false otherwise
 */
inline bool endsWith(const std::string& str, const std::string& suffix) {
    if (str.length() < suffix.length()) {
        return false;
    }
    return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

/**
 * @brief Neural network inference engine
 * 
 * This class provides hardware-accelerated neural network inference
 * with cross-platform support and fallback paths for different hardware.
 */
class InferenceEngine {
public:
    /**
     * @brief Construct an inference engine
     * 
     * @param modelPath Path to the model file
     * @param gpuAccelerator Optional GPU accelerator for hardware acceleration
     */
    InferenceEngine(const std::string& modelPath, 
                   std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator = nullptr);
    
    /**
     * @brief Destroy the inference engine and free resources
     */
    ~InferenceEngine();
    
    /**
     * @brief Initialize the inference engine
     * 
     * @return true if initialization was successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Shutdown the inference engine and release resources
     */
    void shutdown();
    
    /**
     * @brief Check if inference engine is initialized
     * 
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;
    
    /**
     * @brief Set the input tensor for the model
     * 
     * @param name Input tensor name
     * @param tensor Input tensor data
     * @return true if input was set successfully, false otherwise
     */
    bool setInput(const std::string& name, const Tensor& tensor);
    
    /**
     * @brief Get the output tensor from the model
     * 
     * @param name Output tensor name
     * @return Tensor Output tensor data
     */
    Tensor getOutput(const std::string& name) const;
    
    /**
     * @brief Run inference on the loaded model
     * 
     * @return true if inference was successful, false otherwise
     */
    bool runInference();
    
    /**
     * @brief Get model information
     * 
     * @return const ModelInfo& Model information
     */
    const ModelInfo& getModelInfo() const;
    
    /**
     * @brief Get inference metrics
     * 
     * @return const InferenceMetrics& Inference metrics
     */
    const InferenceMetrics& getMetrics() const;
    
    /**
     * @brief Set the desired precision for inference
     * 
     * @param precision Desired precision type
     * @return true if precision was set successfully, false otherwise
     */
    bool setPrecision(PrecisionType precision);
    
    /**
     * @brief Set the batch size for inference
     * 
     * @param batchSize Desired batch size
     * @return true if batch size was set successfully, false otherwise
     */
    bool setBatchSize(int batchSize);
    
    /**
     * @brief Enable or disable hardware acceleration
     * 
     * @param enable True to enable hardware acceleration, false to disable
     * @return true if setting was applied successfully, false otherwise
     */
    bool enableHardwareAcceleration(bool enable);

private:
    std::string modelPath;
    std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator;
    bool initialized = false;
    bool useHardwareAcceleration = true;
    
    ModelInfo modelInfo;
    InferenceMetrics metrics;
    
    // Model input and output data
    std::unordered_map<std::string, Tensor> inputTensors;
    std::unordered_map<std::string, Tensor> outputTensors;
    
    // Timing information
    std::chrono::time_point<std::chrono::high_resolution_clock> lastInferenceStart;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastInferenceEnd;
    
    // Implementation-specific methods
    bool loadModel();
    bool validateInputs();
    void updateMetrics();
    
    // Simulation methods for testing without actual ML frameworks
    bool simulateInference();
    void simulateOutputs();
};

/**
 * @brief Neural network driver model for controlling vehicles
 * 
 * This class implements a neural network-based driver model for
 * controlling vehicles in the simulation.
 */
class DriverModel {
public:
    /**
     * @brief Control outputs from the driver model
     */
    struct ControlOutputs {
        float throttle = 0.0f;      ///< Throttle control (0.0-1.0)
        float brake = 0.0f;         ///< Brake control (0.0-1.0)
        float steering = 0.0f;      ///< Steering control (-1.0 to 1.0)
        float confidence = 0.0f;    ///< Confidence in the decision (0.0-1.0)
    };
    
    /**
     * @brief Sensor inputs to the driver model
     */
    struct SensorInputs {
        float speed = 0.0f;          ///< Current speed (m/s)
        float acceleration = 0.0f;   ///< Current acceleration (m/s²)
        float trackPosition = 0.0f;  ///< Position on track (-1.0 to 1.0, 0 = center)
        float trackAngle = 0.0f;     ///< Angle relative to track (radians)
        float trackCurvature = 0.0f; ///< Upcoming track curvature (1/m)
        std::vector<float> rangeSensors; ///< Distance sensors (meters)
    };
    
    /**
     * @brief Construct a driver model
     * 
     * @param modelPath Path to the neural network model
     * @param gpuAccelerator Optional GPU accelerator for hardware acceleration
     */
    DriverModel(const std::string& modelPath,
               std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator = nullptr);
    
    /**
     * @brief Initialize the driver model
     * 
     * @return true if initialization was successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Shutdown the driver model and release resources
     */
    void shutdown();
    
    /**
     * @brief Process sensor inputs and generate control outputs
     * 
     * @param inputs Sensor inputs from the vehicle and environment
     * @return ControlOutputs Control outputs for the vehicle
     */
    ControlOutputs process(const SensorInputs& inputs);
    
    /**
     * @brief Get the underlying inference engine
     * 
     * @return const InferenceEngine& Reference to the inference engine
     */
    const InferenceEngine& getInferenceEngine() const;
    
    /**
     * @brief Enable or disable adaptive driving behavior
     * 
     * When enabled, the driver model will adapt its behavior based on
     * track conditions and performance metrics.
     * 
     * @param enable True to enable adaptive behavior, false to disable
     */
    void enableAdaptiveBehavior(bool enable);
    
    /**
     * @brief Set the aggressiveness factor for the driver
     * 
     * Higher values result in more aggressive driving behavior.
     * 
     * @param factor Aggressiveness factor (0.0-1.0)
     */
    void setAggressivenessFactor(float factor);
    
    /**
     * @brief Get the physics vehicle associated with this driver model
     * 
     * @return std::shared_ptr<physics::Vehicle> The vehicle
     */
    std::shared_ptr<physics::Vehicle> getPhysicsVehicle() const {
        return nullptr; // Implement this if needed
    }

private:
    std::unique_ptr<InferenceEngine> inferenceEngine;
    bool initialized = false;
    bool adaptiveBehaviorEnabled = true;
    float aggressivenessFactor = 0.5f;
    
    // Previous control outputs for smoothing
    ControlOutputs previousOutputs;
    
    // Preprocessing methods
    Tensor preprocessInputs(const SensorInputs& inputs);
    
    // Postprocessing methods
    ControlOutputs postprocessOutputs(const Tensor& outputTensor);
    
    // Apply smoothing to control outputs
    ControlOutputs smoothControls(const ControlOutputs& rawOutputs);
    
    // Adaptive behavior adjustments
    void applyAdaptiveBehavior(ControlOutputs& outputs, const SensorInputs& inputs);
};

/**
 * @brief Factory for creating driver models
 */
class DriverModelFactory {
public:
    /**
     * @brief Create a driver model for a specific racing scenario
     * 
     * @param scenarioType Type of racing scenario (e.g., "circuit", "rally", "drift")
     * @param gpuAccelerator Optional GPU accelerator for hardware acceleration
     * @return std::unique_ptr<DriverModel> Unique pointer to the created driver model
     */
    static std::unique_ptr<DriverModel> createForScenario(
        const std::string& scenarioType,
        std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator = nullptr);
    
    /**
     * @brief Create a driver model with a specified skill level
     * 
     * @param skillLevel Skill level (0.0-1.0, higher is more skilled)
     * @param gpuAccelerator Optional GPU accelerator for hardware acceleration
     * @return std::unique_ptr<DriverModel> Unique pointer to the created driver model
     */
    static std::unique_ptr<DriverModel> createWithSkillLevel(
        float skillLevel,
        std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator = nullptr);
    
    /**
     * @brief Create a driver model from a custom model file
     * 
     * @param modelPath Path to the custom model file
     * @param gpuAccelerator Optional GPU accelerator for hardware acceleration
     * @return std::unique_ptr<DriverModel> Unique pointer to the created driver model
     */
    static std::unique_ptr<DriverModel> createFromCustomModel(
        const std::string& modelPath,
        std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator = nullptr);
};

} // namespace ai
} // namespace neural_racer