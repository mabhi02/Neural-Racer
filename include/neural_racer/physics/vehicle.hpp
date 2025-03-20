#pragma once

#define _USE_MATH_DEFINES  // For M_PI and other math constants
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <cmath>  // For math functions
// Include Engine definition instead of forward declaration
#include "neural_racer/physics/engine.hpp"

namespace neural_racer {
namespace physics {

/**
 * @brief Vehicle specifications structure
 */
struct VehicleSpec {
    std::string name;          ///< Vehicle name
    float mass;                ///< Vehicle mass in kg
    float power;               ///< Engine power in kW
    float drag;                ///< Aerodynamic drag coefficient
    float frontGrip;           ///< Front tire grip coefficient
    float rearGrip;            ///< Rear tire grip coefficient
    float brakingForce;        ///< Maximum braking force in N
    float steeringResponse;    ///< Steering response factor
    
    VehicleSpec() : 
        name("Default Vehicle"),
        mass(1200.0f),
        power(150.0f),
        drag(0.3f),
        frontGrip(1.0f),
        rearGrip(1.0f),
        brakingForce(20000.0f),
        steeringResponse(1.0f) {}
};

/**
 * @brief Vehicle state structure
 */
struct VehicleState {
    float positionX;           ///< X position in world coordinates (m)
    float positionY;           ///< Y position in world coordinates (m)
    float heading;             ///< Heading angle in radians
    float velocity;            ///< Forward velocity in m/s
    float lateralVelocity;     ///< Lateral velocity in m/s
    float acceleration;        ///< Forward acceleration in m/s²
    float engineRPM;           ///< Engine RPM
    int gear;                  ///< Current gear
    float steeringAngle;       ///< Steering angle in radians
    float throttle;            ///< Throttle input (0.0-1.0)
    float brake;               ///< Brake input (0.0-1.0)
    
    VehicleState() :
        positionX(0.0f),
        positionY(0.0f),
        heading(0.0f),
        velocity(0.0f),
        lateralVelocity(0.0f),
        acceleration(0.0f),
        engineRPM(0.0f),
        gear(1),
        steeringAngle(0.0f),
        throttle(0.0f),
        brake(0.0f) {}
};

/**
 * @brief Vehicle dynamics simulation
 * 
 * This class simulates the physics of a vehicle, including
 * engine, transmission, suspension, and tire dynamics.
 */
class Vehicle : public std::enable_shared_from_this<Vehicle> {
public:
    /**
     * @brief Construct a vehicle with the given specifications
     * 
     * @param spec Vehicle specifications
     * @param physicsEngine Optional physics engine to use
     */
    Vehicle(const VehicleSpec& spec, std::shared_ptr<Engine> physicsEngine = nullptr);
    
    /**
     * @brief Update vehicle state based on inputs and time step
     * 
     * @param deltaTime Time step in seconds
     */
    void update(float deltaTime);
    
    /**
     * @brief Set control inputs for the vehicle
     * 
     * @param throttle Throttle input (0.0-1.0)
     * @param brake Brake input (0.0-1.0)
     * @param steeringAngle Steering angle input (-1.0 to 1.0)
     */
    void setControls(float throttle, float brake, float steeringAngle);
    
    /**
     * @brief Get vehicle specifications
     * 
     * @return const VehicleSpec& Vehicle specifications
     */
    const VehicleSpec& getSpec() const;
    
    /**
     * @brief Get current vehicle state
     * 
     * @return VehicleState Current vehicle state
     */
    VehicleState getState() const;
    
    /**
     * @brief Set vehicle position
     * 
     * @param x X position in world coordinates
     * @param y Y position in world coordinates
     * @param heading Heading angle in radians
     */
    void setPosition(float x, float y, float heading);
    
    /**
     * @brief Apply external force to the vehicle
     * 
     * @param forceX X component of force in N
     * @param forceY Y component of force in N
     * @param duration Duration of force application in seconds
     */
    void applyForce(float forceX, float forceY, float duration = 0.0f);
    
    /**
     * @brief Check if vehicle is on the ground
     * 
     * @return true if the vehicle is on the ground, false otherwise
     */
    bool isGrounded() const;
    
    /**
     * @brief Get vehicle speed in km/h
     * 
     * @return float Vehicle speed in km/h
     */
    float getSpeedKmh() const;

private:
    VehicleSpec spec;
    VehicleState state;
    std::shared_ptr<Engine> physicsEngine;
    mutable std::mutex vehicleMutex;
    
    // Persistent force application
    struct ExternalForce {
        float forceX;
        float forceY;
        float remainingTime;
    };
    std::vector<ExternalForce> externalForces;
    
    // Internal state
    float wheelBaseMeters;     // Distance between front and rear axles
    float trackWidthMeters;    // Distance between left and right wheels
    bool grounded;             // Whether the vehicle is on the ground
    
    // Internal physics calculations
    void updatePhysics(float deltaTime);
    void updateEngine(float deltaTime);
    void updateTransmission(float deltaTime);
    void updateSuspension(float deltaTime);
    void updateWheels(float deltaTime);
    void updateExternalForces(float deltaTime);
    
    // Helper functions
    float calculateDragForce(float velocity) const;
    float calculateTractionForce(float throttle) const;
    float calculateBrakingForce(float brake) const;
    
    // Automatic transmission logic
    void shiftGears();
};

} // namespace physics
} // namespace neural_racer