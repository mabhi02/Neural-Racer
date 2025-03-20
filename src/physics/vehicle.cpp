#include "neural_racer/physics/vehicle.hpp"
#include "neural_racer/physics/engine.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

namespace neural_racer {
namespace physics {

Vehicle::Vehicle(const VehicleSpec& spec, std::shared_ptr<Engine> physicsEngine)
    : spec(spec), physicsEngine(physicsEngine), grounded(true) {
    
    // Initialize vehicle state
    state = VehicleState();
    
    // Set wheel base and track width based on a typical car
    wheelBaseMeters = 2.6f;  // Distance between front and rear axles
    trackWidthMeters = 1.8f;  // Distance between left and right wheels
    
    std::cout << "Created vehicle: " << spec.name << std::endl;
    std::cout << "  Mass: " << spec.mass << " kg" << std::endl;
    std::cout << "  Power: " << spec.power << " kW" << std::endl;
    
    // Register with physics engine if available
    if (physicsEngine) {
        physicsEngine->registerVehicle(shared_from_this());
    }
}

void Vehicle::update(float deltaTime) {
    // Lock mutex to prevent concurrent updates
    std::lock_guard<std::mutex> lock(vehicleMutex);
    
    // Update internal physics
    updatePhysics(deltaTime);
    
    // Update engine
    updateEngine(deltaTime);
    
    // Update transmission
    updateTransmission(deltaTime);
    
    // Update suspension
    updateSuspension(deltaTime);
    
    // Update wheels
    updateWheels(deltaTime);
    
    // Update external forces
    updateExternalForces(deltaTime);
}

void Vehicle::setControls(float throttle, float brake, float steeringAngle) {
    // Lock mutex to prevent concurrent updates
    std::lock_guard<std::mutex> lock(vehicleMutex);
    
    // Clamp inputs to valid ranges
    state.throttle = std::max(0.0f, std::min(1.0f, throttle));
    state.brake = std::max(0.0f, std::min(1.0f, brake));
    state.steeringAngle = std::max(-1.0f, std::min(1.0f, steeringAngle));
}

const VehicleSpec& Vehicle::getSpec() const {
    return spec;
}

VehicleState Vehicle::getState() const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(vehicleMutex);
    
    return state;
}

void Vehicle::setPosition(float x, float y, float heading) {
    // Lock mutex to prevent concurrent updates
    std::lock_guard<std::mutex> lock(vehicleMutex);
    
    state.positionX = x;
    state.positionY = y;
    state.heading = heading;
}

void Vehicle::applyForce(float forceX, float forceY, float duration) {
    // Lock mutex to prevent concurrent updates
    std::lock_guard<std::mutex> lock(vehicleMutex);
    
    // Add force to the list of external forces
    ExternalForce force;
    force.forceX = forceX;
    force.forceY = forceY;
    force.remainingTime = duration;
    
    externalForces.push_back(force);
}

bool Vehicle::isGrounded() const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(vehicleMutex);
    
    return grounded;
}

float Vehicle::getSpeedKmh() const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(vehicleMutex);
    
    // Convert m/s to km/h
    return state.velocity * 3.6f;
}

void Vehicle::updatePhysics(float deltaTime) {
    // Skip if deltaTime is too small
    if (deltaTime < 0.001f) {
        return;
    }
    
    // Calculate forces
    float tractionForce = calculateTractionForce(state.throttle);
    float dragForce = calculateDragForce(state.velocity);
    float brakeForce = calculateBrakingForce(state.brake);
    
    // Calculate net force (traction - drag - braking)
    float netForce = tractionForce - dragForce - brakeForce;
    
    // Calculate acceleration (F = ma)
    state.acceleration = netForce / spec.mass;
    
    // Update velocity
    state.velocity += state.acceleration * deltaTime;
    
    // Ensure velocity doesn't go negative
    state.velocity = std::max(0.0f, state.velocity);
    
    // Calculate movement based on heading
    float dx = state.velocity * std::cos(state.heading) * deltaTime;
    float dy = state.velocity * std::sin(state.heading) * deltaTime;
    
    // Update position
    state.positionX += dx;
    state.positionY += dy;
    
    // Handle steering
    if (std::abs(state.steeringAngle) > 0.001f && state.velocity > 0.1f) {
        // Calculate turn radius (higher steering angle = sharper turn)
        float steeringFactor = state.steeringAngle * spec.steeringResponse;
        float turnRadius = wheelBaseMeters / std::sin(std::abs(steeringFactor));
        
        // Calculate angular velocity
        float angularVelocity = state.velocity / turnRadius;
        
        // Apply sign based on steering direction
        if (state.steeringAngle < 0) {
            angularVelocity = -angularVelocity;
        }
        
        // Update heading
        state.heading += angularVelocity * deltaTime;
        
        // Normalize heading to 0-2π range
        while (state.heading < 0) {
            state.heading += 2.0f * M_PI;
        }
        while (state.heading >= 2.0f * M_PI) {
            state.heading -= 2.0f * M_PI;
        }
    }
    
    // Update lateral velocity (simplified model)
    state.lateralVelocity = state.velocity * std::sin(state.steeringAngle * spec.steeringResponse);
}

void Vehicle::updateEngine(float deltaTime) {
    // Engine RPM calculation
    // Simplified model: RPM depends on velocity and gear
    float wheelRadiusMeters = 0.3f;  // Typical wheel radius
    float gearRatio = 0.0f;
    
    // Simple gear ratios
    switch (state.gear) {
        case 1: gearRatio = 3.5f; break;
        case 2: gearRatio = 2.5f; break;
        case 3: gearRatio = 1.8f; break;
        case 4: gearRatio = 1.3f; break;
        case 5: gearRatio = 1.0f; break;
        case 6: gearRatio = 0.8f; break;
        default: gearRatio = 3.5f; break;
    }
    
    float differentialRatio = 3.7f;  // Typical differential ratio
    float transmissionRatio = gearRatio * differentialRatio;
    
    // Calculate wheel RPM from velocity
    float wheelRpm = (state.velocity * 60.0f) / (2.0f * M_PI * wheelRadiusMeters);
    
    // Calculate engine RPM from wheel RPM
    state.engineRPM = wheelRpm * transmissionRatio;
    
    // Idle RPM when stationary
    if (state.velocity < 0.1f) {
        state.engineRPM = 800.0f;
    }
    
    // Limit max RPM
    state.engineRPM = std::min(state.engineRPM, 8000.0f);
    
    // Check for automatic gear shifting
    if (state.throttle > 0.1f) {  // Only shift when accelerating
        shiftGears();
    }
}

void Vehicle::updateTransmission(float deltaTime) {
    // Transmission logic is handled in shiftGears() and updateEngine()
}

void Vehicle::updateSuspension(float deltaTime) {
    // Simplified suspension model
    // In a real implementation, this would model the suspension forces
    // and their effect on vehicle dynamics
    
    // For now, just ensure the vehicle stays grounded
    grounded = true;
}

void Vehicle::updateWheels(float deltaTime) {
    // Simplified wheel model
    // In a real implementation, this would model tire forces,
    // slip angles, and grip levels for each wheel
    
    // For now, just use simplified physics
}

void Vehicle::updateExternalForces(float deltaTime) {
    // Apply external forces and update remaining durations
    auto it = externalForces.begin();
    
    while (it != externalForces.end()) {
        // Calculate force components in global coordinates
        float globalForceX = it->forceX;
        float globalForceY = it->forceY;
        
        // Apply force (F = ma, so a = F/m)
        float accelerationX = globalForceX / spec.mass;
        float accelerationY = globalForceY / spec.mass;
        
        // Update velocity components
        float velocityX = state.velocity * std::cos(state.heading);
        float velocityY = state.velocity * std::sin(state.heading);
        
        velocityX += accelerationX * deltaTime;
        velocityY += accelerationY * deltaTime;
        
        // Recalculate speed and heading
        state.velocity = std::sqrt(velocityX * velocityX + velocityY * velocityY);
        
        if (state.velocity > 0.001f) {
            state.heading = std::atan2(velocityY, velocityX);
        }
        
        // Update remaining duration
        it->remainingTime -= deltaTime;
        
        // Remove forces that have expired
        if (it->remainingTime <= 0.0f) {
            it = externalForces.erase(it);
        } else {
            ++it;
        }
    }
}

float Vehicle::calculateDragForce(float velocity) const {
    // Drag force formula: Fd = 0.5 * rho * v^2 * Cd * A
    // where:
    // - rho is air density
    // - v is velocity
    // - Cd is drag coefficient
    // - A is frontal area
    
    const float airDensity = 1.225f;  // kg/m^3
    const float frontalArea = 2.0f;   // m^2 (typical car)
    
    return 0.5f * airDensity * velocity * velocity * spec.drag * frontalArea;
}

float Vehicle::calculateTractionForce(float throttle) const {
    // Traction force depends on engine power and current gear
    
    // Maximum force available (power / velocity)
    float maxPower = spec.power * 1000.0f;  // Convert kW to W
    
    // At very low speeds, use a high force to get moving
    if (state.velocity < 0.1f) {
        return throttle * maxPower / 0.1f;
    }
    
    // Otherwise, F = P/v
    float tractionForce = throttle * maxPower / state.velocity;
    
    // Apply tire grip limit
    float maxGripForce = spec.mass * 9.81f * spec.rearGrip;  // F = μmg
    
    return std::min(tractionForce, maxGripForce);
}

float Vehicle::calculateBrakingForce(float brake) const {
    // Braking force depends on braking input and maximum braking force
    float maxBrakingForce = spec.brakingForce;
    
    // Apply tire grip limit
    float maxGripForce = spec.mass * 9.81f * spec.frontGrip;  // F = μmg
    
    return brake * std::min(maxBrakingForce, maxGripForce);
}

void Vehicle::shiftGears() {
    // Simple automatic transmission logic
    
    // Upshift RPM thresholds
    const float upshiftRpm = 6500.0f;
    
    // Downshift RPM thresholds
    const float downshiftRpm = 2000.0f;
    
    // Check for upshift
    if (state.engineRPM > upshiftRpm && state.gear < 6) {
        state.gear++;
        std::cout << "Upshift to gear " << state.gear << std::endl;
    }
    // Check for downshift
    else if (state.engineRPM < downshiftRpm && state.gear > 1) {
        state.gear--;
        std::cout << "Downshift to gear " << state.gear << std::endl;
    }
}