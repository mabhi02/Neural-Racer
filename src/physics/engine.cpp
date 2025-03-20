#include "neural_racer/physics/engine.hpp"
#include "neural_racer/physics/vehicle.hpp"
#include "neural_racer/utils/logger.hpp"
#include "neural_racer/utils/profiler.hpp"
#include <iostream>
#include <algorithm>
#include <vector>

namespace neural_racer {
namespace physics {

Engine::Engine(std::shared_ptr<hardware::GPUAccelerator> gpuAccelerator)
    : gpuAccelerator(gpuAccelerator), 
      initialized(false), 
      useHardwareAcceleration(false), 
      threadRunning(false), 
      nextCallbackId(1) {
    
    // Initialize with default configuration
    config = PhysicsConfig();
    stats = PhysicsStats();
    
    utils::Logger::info("Physics", "Physics engine created");
}

Engine::~Engine() {
    shutdown();
}

bool Engine::initialize(const PhysicsConfig& config) {
    // Lock mutex to prevent concurrent initialization
    std::lock_guard<std::mutex> lock(engineMutex);
    
    if (initialized) {
        utils::Logger::warning("Physics", "Physics engine already initialized");
        return true;  // Already initialized
    }
    
    utils::Logger::info("Physics", "Initializing physics engine");
    
    // Store configuration
    this->config = config;
    
    // Initialize hardware acceleration if available
    if (gpuAccelerator && gpuAccelerator->isInitialized()) {
        useHardwareAcceleration = true;
        utils::Logger::info("Physics", "Hardware acceleration enabled");
    } else {
        useHardwareAcceleration = false;
        utils::Logger::info("Physics", "Hardware acceleration not available, using CPU");
    }
    
    // Reset stats
    stats = PhysicsStats();
    
    initialized = true;
    utils::Logger::info("Physics", "Physics engine initialized successfully");
    return true;
}

void Engine::shutdown() {
    // Lock mutex to prevent concurrent shutdown
    std::lock_guard<std::mutex> lock(engineMutex);
    
    if (!initialized) {
        return;  // Nothing to do
    }
    
    utils::Logger::info("Physics", "Shutting down physics engine");
    
    // Stop simulation thread if running
    if (isSimulationThreadRunning()) {
        stopSimulationThread();
    }
    
    // Clear all registered vehicles
    vehicles.clear();
    
    // Clear all callbacks
    collisionCallbacks.clear();
    
    initialized = false;
    
    utils::Logger::info("Physics", "Physics engine shut down");
}

void Engine::step(float deltaTime) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(engineMutex);
    
    PROFILE_SCOPE("Physics::Engine::step");
    
    if (!initialized) {
        utils::Logger::warning("Physics", "Cannot step physics engine: not initialized");
        return;
    }
    
    // Use configured time step if none specified
    if (deltaTime <= 0.0f) {
        deltaTime = config.timeStep;
    }
    
    // Record start time for stats
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Perform substeps for better stability
    float substepTime = deltaTime / static_cast<float>(config.substeps);
    
    for (int i = 0; i < config.substeps; i++) {
        // Update physics simulation
        updatePhysics(substepTime);
    }
    
    // Record end time and update stats
    auto endTime = std::chrono::high_resolution_clock::now();
    stats.lastStepDurationMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();
    stats.simulationTimeMs = stats.lastStepDurationMs;
    
    utils::Logger::trace("Physics", "Physics step completed in " + 
                       std::to_string(stats.lastStepDurationMs) + " ms");
}

bool Engine::registerVehicle(std::shared_ptr<Vehicle> vehicle) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(engineMutex);
    
    if (!initialized) {
        utils::Logger::warning("Physics", "Cannot register vehicle: physics engine not initialized");
        return false;
    }
    
    // Check if vehicle is already registered
    auto it = std::find(vehicles.begin(), vehicles.end(), vehicle);
    if (it != vehicles.end()) {
        utils::Logger::warning("Physics", "Vehicle already registered");
        return false;
    }
    
    // Add vehicle to list
    vehicles.push_back(vehicle);
    
    // Update stats
    stats.objectCount = static_cast<int>(vehicles.size());
    
    utils::Logger::info("Physics", "Vehicle registered: " + vehicle->getSpec().name);
    return true;
}

void Engine::unregisterVehicle(std::shared_ptr<Vehicle> vehicle) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(engineMutex);
    
    if (!initialized) {
        return;
    }
    
    // Find and remove vehicle
    auto it = std::find(vehicles.begin(), vehicles.end(), vehicle);
    if (it != vehicles.end()) {
        vehicles.erase(it);
        
        // Update stats
        stats.objectCount = static_cast<int>(vehicles.size());
        
        utils::Logger::info("Physics", "Vehicle unregistered: " + vehicle->getSpec().name);
    }
}

int Engine::registerCollisionCallback(CollisionCallback callback) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(engineMutex);
    
    if (!initialized) {
        utils::Logger::warning("Physics", "Cannot register collision callback: physics engine not initialized");
        return 0;
    }
    
    // Create callback info
    CallbackInfo info{nextCallbackId, callback};
    
    // Add to list
    collisionCallbacks.push_back(info);
    
    // Increment callback ID for next registration
    nextCallbackId++;
    
    utils::Logger::debug("Physics", "Collision callback registered with ID " + std::to_string(info.id));
    return info.id;
}

void Engine::unregisterCollisionCallback(int callbackId) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(engineMutex);
    
    if (!initialized) {
        return;
    }
    
    // Find and remove callback
    auto it = std::find_if(collisionCallbacks.begin(), collisionCallbacks.end(),
                          [callbackId](const CallbackInfo& info) { return info.id == callbackId; });
    
    if (it != collisionCallbacks.end()) {
        collisionCallbacks.erase(it);
        utils::Logger::debug("Physics", "Collision callback unregistered: " + std::to_string(callbackId));
    }
}

void Engine::setConfig(const PhysicsConfig& config) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(engineMutex);
    
    this->config = config;
    utils::Logger::info("Physics", "Physics configuration updated");
}

const PhysicsConfig& Engine::getConfig() const {
    return config;
}

const PhysicsStats& Engine::getStats() const {
    return stats;
}

bool Engine::isInitialized() const {
    return initialized;
}

bool Engine::hasHardwareAcceleration() const {
    return useHardwareAcceleration && gpuAccelerator && gpuAccelerator->isInitialized();
}

bool Engine::enableHardwareAcceleration(bool enable) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(engineMutex);
    
    if (!initialized) {
        utils::Logger::warning("Physics", "Cannot change hardware acceleration: physics engine not initialized");
        return false;
    }
    
    if (enable && (!gpuAccelerator || !gpuAccelerator->isInitialized())) {
        utils::Logger::warning("Physics", "Cannot enable hardware acceleration: GPU accelerator not available");
        return false;
    }
    
    useHardwareAcceleration = enable;
    
    utils::Logger::info("Physics", "Hardware acceleration " + std::string(enable ? "enabled" : "disabled"));
    return true;
}

int Engine::getVehicleCount() const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(engineMutex);
    
    return static_cast<int>(vehicles.size());
}

bool Engine::startSimulationThread(bool fixedTimeStep) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(engineMutex);
    
    if (!initialized) {
        utils::Logger::warning("Physics", "Cannot start simulation thread: physics engine not initialized");
        return false;
    }
    
    if (threadRunning) {
        utils::Logger::warning("Physics", "Simulation thread already running");
        return true;  // Already running
    }
    
    // Reset accumulated time
    accumulatedTime = 0.0f;
    
    // Start thread
    threadRunning = true;
    simulationThread = std::thread(&Engine::simulationThreadFunction, this, fixedTimeStep);
    
    utils::Logger::info("Physics", "Simulation thread started");
    return true;
}

void Engine::stopSimulationThread() {
    {
        // Lock mutex to prevent concurrent modification
        std::lock_guard<std::mutex> lock(engineMutex);
        
        if (!threadRunning) {
            return;  // Already stopped
        }
        
        // Signal thread to stop
        threadRunning = false;
    }
    
    // Notify thread
    threadCondition.notify_one();
    
    // Wait for thread to exit
    if (simulationThread.joinable()) {
        simulationThread.join();
    }
    
    utils::Logger::info("Physics", "Simulation thread stopped");
}

bool Engine::isSimulationThreadRunning() const {
    return threadRunning;
}

void Engine::simulationThreadFunction(bool fixedTimeStep) {
    utils::Logger::info("Physics", "Simulation thread started");
    
    // Thread timing variables
    auto lastTime = std::chrono::high_resolution_clock::now();
    
    while (threadRunning) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        float deltaTime = std::chrono::duration<float>(currentTime - lastTime).count();
        lastTime = currentTime;
        
        // Limit maximum delta time to prevent spiral of death
        deltaTime = std::min(deltaTime, 0.1f);
        
        if (fixedTimeStep) {
            // Fixed time step mode
            accumulatedTime += deltaTime;
            
            // Step physics engine as many times as needed
            while (accumulatedTime >= config.timeStep) {
                step(config.timeStep);
                accumulatedTime -= config.timeStep;
            }
        } else {
            // Variable time step mode
            step(deltaTime);
        }
        
        // Sleep to avoid using 100% CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    utils::Logger::info("Physics", "Simulation thread exited");
}

void Engine::updatePhysics(float deltaTime) {
    PROFILE_SCOPE("Physics::updatePhysics");
    
    // Apply forces
    {
        PROFILE_SCOPE("Physics::applyForces");
        applyForces(deltaTime);
    }
    
    // Detect collisions
    if (config.enableCollisions) {
        PROFILE_SCOPE("Physics::detectCollisions");
        detectCollisions();
    }
    
    // Resolve collisions
    if (config.enableCollisions) {
        PROFILE_SCOPE("Physics::resolveCollisions");
        resolveCollisions();
    }
    
    // Update vehicles
    {
        PROFILE_SCOPE("Physics::updateVehicles");
        updateVehicles(deltaTime);
    }
    
    // Update stats
    updateStats(deltaTime);
}

void Engine::detectCollisions() {
    PROFILE_SCOPE("Physics::detectCollisions");
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Reset collision pair count
    stats.collisionPairCount = 0;
    
    // Simple collision detection between vehicles
    // In a real implementation, this would use a spatial partitioning data structure
    // to avoid O(n²) complexity
    
    for (size_t i = 0; i < vehicles.size(); i++) {
        for (size_t j = i + 1; j < vehicles.size(); j++) {
            // Check if vehicles collide
            auto& vehicle1 = vehicles[i];
            auto& vehicle2 = vehicles[j];
            
            auto state1 = vehicle1->getState();
            auto state2 = vehicle2->getState();
            
            // Simple distance check (spherical collision)
            // In a real implementation, this would use more sophisticated collision detection
            float dx = state1.positionX - state2.positionX;
            float dy = state1.positionY - state2.positionY;
            float distSq = dx * dx + dy * dy;
            
            // Assume vehicle radius is 1.5m (diameter 3m)
            const float collisionDistSq = 3.0f * 3.0f;
            
            if (distSq < collisionDistSq) {
                // Collision detected
                stats.collisionPairCount++;
                
                // Determine collision normal
                float dist = std::sqrt(distSq);
                float nx = dx / dist;
                float ny = dy / dist;
                
                // Calculate relative velocity
                float relVelX = state2.velocity * std::cos(state2.heading) - 
                                state1.velocity * std::cos(state1.heading);
                float relVelY = state2.velocity * std::sin(state2.heading) - 
                                state1.velocity * std::sin(state1.heading);
                
                // Project relative velocity onto normal
                float relVelNormal = relVelX * nx + relVelY * ny;
                
                // Only colliding if moving toward each other
                if (relVelNormal < 0) {
                    // Fire collision event
                    CollisionData collision;
                    collision.objectA = vehicle1.get();
                    collision.objectB = vehicle2.get();
                    collision.impactVelocity = -relVelNormal; // Relative velocity at impact
                    collision.penetrationDepth = 3.0f - dist;
                    collision.normalX = nx;
                    collision.normalY = ny;
                    collision.impactX = state1.positionX + nx * 1.5f;
                    collision.impactY = state1.positionY + ny * 1.5f;
                    
                    fireCollisionEvent(collision);
                }
            }
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    stats.collisionTimeMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();
}

void Engine::resolveCollisions() {
    PROFILE_SCOPE("Physics::resolveCollisions");
    
    // In a real implementation, this would apply impulses to resolve collisions
    // For this simplified version, we assume collisions are handled by vehicles
    // when they receive collision events
}

void Engine::updateVehicles(float deltaTime) {
    PROFILE_SCOPE("Physics::updateVehicles");
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Update all vehicles
    for (auto& vehicle : vehicles) {
        vehicle->update(deltaTime);
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    stats.updateTimeMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();
}

void Engine::applyForces(float deltaTime) {
    PROFILE_SCOPE("Physics::applyForces");
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Apply global forces like gravity to all vehicles
    // In a real implementation, this would handle more forces
    
    auto endTime = std::chrono::high_resolution_clock::now();
    stats.forcesTimeMs = std::chrono::duration<float, std::milli>(endTime - startTime).count();
}

void Engine::fireCollisionEvent(const CollisionData& collision) {
    // Notify all registered callbacks
    for (const auto& callbackInfo : collisionCallbacks) {
        callbackInfo.callback(collision);
    }
}

void Engine::updateStats(float deltaTime) {
    // Update constraint count
    stats.constraintCount = 0; // No constraints in this simplified version
    
    // Object count is updated when vehicles are registered/unregistered
    
    // Other stats are updated in their respective methods
}

} // namespace physics
} // namespace neural_racer