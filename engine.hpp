#pragma once

#include <memory>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>
#include "../hardware/device_driver.hpp"

namespace neural_racer {
namespace physics {

// Forward declarations
class Vehicle;

/**
 * @brief Physics configuration
 */
struct PhysicsConfig {
    float gravity;               ///< Gravity acceleration in m/s²
    float timeStep;              ///< Simulation time step in seconds
    int substeps;                ///< Number of substeps per step
    float airDensity;            ///< Air density in kg/m³
    float frictionCoefficient;   ///< Global friction coefficient
    bool enableCollisions;       ///< Enable collision detection
    bool enableSuspension;       ///< Enable suspension simulation
    bool enableAerodynamics;     ///< Enable aerodynamic forces
    
    PhysicsConfig() :
        gravity(9.81f),
        timeStep(0.016f),        // 60Hz
        substeps(4),
        airDensity(1.225f),
        frictionCoefficient(0.7f),
        enableCollisions(true),
        enableSuspension(true),
        enableAerodynamics(true) {}
};

/**
 * @brief Physics engine statistics
 */
struct PhysicsStats {
    float simulationTimeMs;      ///< Time spent in simulation per step
    float collisionTimeMs;       ///< Time spent in collision detection per step
    float forcesTimeMs;          ///< Time spent calculating forces per step
    float updateTimeMs;          ///< Time spent updating objects per step
    int objectCount;             ///< Number of simulated objects
    int constraintCount;         ///< Number of constraints
    int collisionPairCount;      ///< Number of collision pairs
    float lastStepDurationMs;    ///< Duration of last step in ms
    
    PhysicsStats() :
        simulationTimeMs(0.0f),
        collisionTimeMs(0.0f),
        forcesTimeMs(0.0f),
        updateTimeMs(0.0f),
        objectCount(0),
        constraintCount(0),
        collisionPairCount(0),
        lastStepDurationMs(0.0f) {}
};

/**
 * @brief Collision data structure
 */
struct CollisionData {
    void* objectA;               ///< First colliding object
    void* objectB;               ///< Second colliding object
    float impactVelocity;        ///< Impact velocity in m/s
    float penetrationDepth;      ///< Penetration depth in m
    float normalX;               ///< X component of collision normal
    float normalY;               ///< Y component of collision normal
    float impactX;               ///< X component of impact point
    float impactY;               ///< Y component of impact point
    
    CollisionData() :
        objectA(nullptr),
        objectB(nullptr),
        impactVelocity(0.0f),
        penetrationDepth(0.0f),
        normalX(0.0f),
        normalY(0.0f),
        impactX(0.0f),
        impactY(0.0f) {}
};

/**
 * @brief Physics engine for vehicle dynamics simulation
 * 
 * This class provides a physics engine for simulating vehicle dynamics,
 * including engine, transmission, suspension, and tire physics.
 */
class Engine {
public:
    /**
     * @brief Collision callback function type
     */
    using CollisionCallback = std::function<void(const CollisionData&)>;
    
    /**
     * @brief Construct a physics engine
     * 
     * @param gpuAccelerator Optional GPU accelerator for hardware acceleration
     */
    explicit Engine(std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator = nullptr);
    
    /**
     * @brief Destroy the physics engine
     */
    ~Engine();
    
    /**
     * @brief Initialize the physics engine
     * 
     * @param config Physics configuration
     * @return true if initialization was successful, false otherwise
     */
    bool initialize(const PhysicsConfig& config = PhysicsConfig());
    
    /**
     * @brief Shutdown the physics engine
     */
    void shutdown();
    
    /**
     * @brief Step the simulation forward
     * 
     * @param deltaTime Time step in seconds, or 0 to use configured time step
     */
    void step(float deltaTime = 0.0f);
    
    /**
     * @brief Register a vehicle with the physics engine
     * 
     * @param vehicle Vehicle to register
     * @return true if vehicle was registered successfully, false otherwise
     */
    bool registerVehicle(std::shared_ptr<Vehicle> vehicle);
    
    /**
     * @brief Unregister a vehicle from the physics engine
     * 
     * @param vehicle Vehicle to unregister
     */
    void unregisterVehicle(std::shared_ptr<Vehicle> vehicle);
    
    /**
     * @brief Register a collision callback
     * 
     * @param callback Collision callback function
     * @return int Callback ID for unregistering
     */
    int registerCollisionCallback(CollisionCallback callback);
    
    /**
     * @brief Unregister a collision callback
     * 
     * @param callbackId Callback ID returned from registerCollisionCallback
     */
    void unregisterCollisionCallback(int callbackId);
    
    /**
     * @brief Set physics configuration
     * 
     * @param config Physics configuration
     */
    void setConfig(const PhysicsConfig& config);
    
    /**
     * @brief Get physics configuration
     * 
     * @return const PhysicsConfig& Physics configuration
     */
    const PhysicsConfig& getConfig() const;
    
    /**
     * @brief Get physics statistics
     * 
     * @return const PhysicsStats& Physics statistics
     */
    const PhysicsStats& getStats() const;
    
    /**
     * @brief Check if physics engine is initialized
     * 
     * @return true if initialized, false otherwise
     */
    bool isInitialized() const;
    
    /**
     * @brief Check if hardware acceleration is available
     * 
     * @return true if hardware acceleration is available, false otherwise
     */
    bool hasHardwareAcceleration() const;
    
    /**
     * @brief Enable or disable hardware acceleration
     * 
     * @param enable True to enable hardware acceleration, false to disable
     * @return true if setting was applied successfully, false otherwise
     */
    bool enableHardwareAcceleration(bool enable);
    
    /**
     * @brief Get the number of registered vehicles
     * 
     * @return int Number of registered vehicles
     */
    int getVehicleCount() const;
    
    /**
     * @brief Start the physics simulation thread
     * 
     * @param fixedTimeStep True to use fixed time step, false for variable
     * @return true if started successfully, false otherwise
     */
    bool startSimulationThread(bool fixedTimeStep = true);
    
    /**
     * @brief Stop the physics simulation thread
     */
    void stopSimulationThread();
    
    /**
     * @brief Check if simulation thread is running
     * 
     * @return true if running, false otherwise
     */
    bool isSimulationThreadRunning() const;

private:
    PhysicsConfig config;
    PhysicsStats stats;
    std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator;
    
    // Simulation state
    bool initialized;
    bool useHardwareAcceleration;
    std::vector<std::shared_ptr<Vehicle>> vehicles;
    mutable std::mutex engineMutex;
    
    // Simulation thread
    std::thread simulationThread;
    std::atomic<bool> threadRunning;
    std::condition_variable threadCondition;
    std::atomic<float> accumulatedTime;
    
    // Collision callbacks
    struct CallbackInfo {
        int id;
        CollisionCallback callback;
    };
    std::vector<CallbackInfo> collisionCallbacks;
    int nextCallbackId;
    
    // Physics simulation methods
    void simulationThreadFunction(bool fixedTimeStep);
    void updatePhysics(float deltaTime);
    void detectCollisions();
    void resolveCollisions();
    void updateVehicles(float deltaTime);
    void applyForces(float deltaTime);
    
    // Helper methods
    void fireCollisionEvent(const CollisionData& collision);
    void updateStats(float deltaTime);
};

} // namespace physics
} // namespace neural_racer