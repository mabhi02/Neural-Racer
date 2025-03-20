#include "neural_racer/simulation/telemetry.hpp"
#include "neural_racer/simulation/race.hpp"
#include "neural_racer/physics/vehicle.hpp"
#include "neural_racer/ai/driver.hpp"
#include "neural_racer/utils/logger.hpp"
#include "neural_racer/utils/profiler.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <set>

namespace neural_racer {
namespace simulation {

Telemetry::Telemetry(size_t maxSamplesPerChannel)
    : maxSamplesPerChannel(maxSamplesPerChannel) {
    
    // Initialize lap statistics
    lapStats = LapStatistics();
    
    utils::Logger::info("Telemetry", "Telemetry system created with " + 
                       std::to_string(maxSamplesPerChannel) + " max samples per channel");
}

Telemetry::~Telemetry() {
    // Clean up registered listeners
    listeners.clear();
    
    utils::Logger::info("Telemetry", "Telemetry system destroyed");
}

bool Telemetry::initialize() {
    // Lock mutex to prevent concurrent initialization
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    utils::Logger::info("Telemetry", "Initializing telemetry system");
    
    // Create some default channels
    addChannel("time", "s", TelemetryType::Float);
    addChannel("lap", "", TelemetryType::Int);
    addChannel("position", "", TelemetryType::Int);
    
    return true;
}

void Telemetry::shutdown() {
    // Lock mutex to prevent concurrent shutdown
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    utils::Logger::info("Telemetry", "Shutting down telemetry system");
    
    // Clear all data
    clear();
    
    // Clear recorded vehicles and drivers
    recordedVehicles.clear();
    recordedDrivers.clear();
    channelPrefixes.clear();
    vehicleStates.clear();
}

bool Telemetry::addChannel(const std::string& name, const std::string& units, TelemetryType type) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if channel already exists
    if (channels.find(name) != channels.end()) {
        utils::Logger::warning("Telemetry", "Channel already exists: " + name);
        return false;
    }
    
    // Create channel
    TelemetryChannel channel(name, units, type, maxSamplesPerChannel);
    channels[name] = channel;
    
    utils::Logger::debug("Telemetry", "Added channel: " + name + " (" + units + ")");
    return true;
}

bool Telemetry::addSample(const std::string& channelName, float timestamp, float value) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if channel exists
    auto it = channels.find(channelName);
    if (it == channels.end()) {
        // Create channel if it doesn't exist
        addChannel(channelName, "", TelemetryType::Float);
        it = channels.find(channelName);
    }
    
    // Check channel type
    if (it->second.type != TelemetryType::Float) {
        utils::Logger::warning("Telemetry", "Type mismatch for channel: " + channelName);
        return false;
    }
    
    // Create sample
    TelemetrySample sample;
    sample.timestamp = timestamp;
    sample.value.floatValue = value;
    
    // Add sample to channel
    it->second.data.push_back(sample);
    
    // Limit channel size
    if (it->second.data.size() > it->second.maxSamples) {
        it->second.data.pop_front();
    }
    
    // Notify listeners
    notifyListeners(channelName, sample);
    
    return true;
}

bool Telemetry::addSample(const std::string& channelName, float timestamp, int value) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if channel exists
    auto it = channels.find(channelName);
    if (it == channels.end()) {
        // Create channel if it doesn't exist
        addChannel(channelName, "", TelemetryType::Int);
        it = channels.find(channelName);
    }
    
    // Check channel type
    if (it->second.type != TelemetryType::Int) {
        utils::Logger::warning("Telemetry", "Type mismatch for channel: " + channelName);
        return false;
    }
    
    // Create sample
    TelemetrySample sample;
    sample.timestamp = timestamp;
    sample.value.intValue = value;
    
    // Add sample to channel
    it->second.data.push_back(sample);
    
    // Limit channel size
    if (it->second.data.size() > it->second.maxSamples) {
        it->second.data.pop_front();
    }
    
    // Notify listeners
    notifyListeners(channelName, sample);
    
    return true;
}

bool Telemetry::addSample(const std::string& channelName, float timestamp, bool value) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if channel exists
    auto it = channels.find(channelName);
    if (it == channels.end()) {
        // Create channel if it doesn't exist
        addChannel(channelName, "", TelemetryType::Bool);
        it = channels.find(channelName);
    }
    
    // Check channel type
    if (it->second.type != TelemetryType::Bool) {
        utils::Logger::warning("Telemetry", "Type mismatch for channel: " + channelName);
        return false;
    }
    
    // Create sample
    TelemetrySample sample;
    sample.timestamp = timestamp;
    sample.value.boolValue = value;
    
    // Add sample to channel
    it->second.data.push_back(sample);
    
    // Limit channel size
    if (it->second.data.size() > it->second.maxSamples) {
        it->second.data.pop_front();
    }
    
    // Notify listeners
    notifyListeners(channelName, sample);
    
    return true;
}

bool Telemetry::addSample(const std::string& channelName, float timestamp, const std::string& value) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if channel exists
    auto it = channels.find(channelName);
    if (it == channels.end()) {
        // Create channel if it doesn't exist
        addChannel(channelName, "", TelemetryType::String);
        it = channels.find(channelName);
    }
    
    // Check channel type
    if (it->second.type != TelemetryType::String) {
        utils::Logger::warning("Telemetry", "Type mismatch for channel: " + channelName);
        return false;
    }
    
    // Create sample
    TelemetrySample sample;
    sample.timestamp = timestamp;
    sample.stringValue = value;
    
    // Add sample to channel
    it->second.data.push_back(sample);
    
    // Limit channel size
    if (it->second.data.size() > it->second.maxSamples) {
        it->second.data.pop_front();
    }
    
    // Notify listeners
    notifyListeners(channelName, sample);
    
    return true;
}

bool Telemetry::addSample(const std::string& channelName, float timestamp, float x, float y) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if channel exists
    auto it = channels.find(channelName);
    if (it == channels.end()) {
        // Create channel if it doesn't exist
        addChannel(channelName, "", TelemetryType::Vector2);
        it = channels.find(channelName);
    }
    
    // Check channel type
    if (it->second.type != TelemetryType::Vector2) {
        utils::Logger::warning("Telemetry", "Type mismatch for channel: " + channelName);
        return false;
    }
    
    // Create sample
    TelemetrySample sample;
    sample.timestamp = timestamp;
    sample.value.vec2Value[0] = x;
    sample.value.vec2Value[1] = y;
    
    // Add sample to channel
    it->second.data.push_back(sample);
    
    // Limit channel size
    if (it->second.data.size() > it->second.maxSamples) {
        it->second.data.pop_front();
    }
    
    // Notify listeners
    notifyListeners(channelName, sample);
    
    return true;
}

bool Telemetry::addSample(const std::string& channelName, float timestamp, float x, float y, float z) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if channel exists
    auto it = channels.find(channelName);
    if (it == channels.end()) {
        // Create channel if it doesn't exist
        addChannel(channelName, "", TelemetryType::Vector3);
        it = channels.find(channelName);
    }
    
    // Check channel type
    if (it->second.type != TelemetryType::Vector3) {
        utils::Logger::warning("Telemetry", "Type mismatch for channel: " + channelName);
        return false;
    }
    
    // Create sample
    TelemetrySample sample;
    sample.timestamp = timestamp;
    sample.value.vec3Value[0] = x;
    sample.value.vec3Value[1] = y;
    sample.value.vec3Value[2] = z;
    
    // Add sample to channel
    it->second.data.push_back(sample);
    
    // Limit channel size
    if (it->second.data.size() > it->second.maxSamples) {
        it->second.data.pop_front();
    }
    
    // Notify listeners
    notifyListeners(channelName, sample);
    
    return true;
}

bool Telemetry::addSample(const std::string& channelName, float timestamp, float w, float x, float y, float z) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if channel exists
    auto it = channels.find(channelName);
    if (it == channels.end()) {
        // Create channel if it doesn't exist
        addChannel(channelName, "", TelemetryType::Quaternion);
        it = channels.find(channelName);
    }
    
    // Check channel type
    if (it->second.type != TelemetryType::Quaternion) {
        utils::Logger::warning("Telemetry", "Type mismatch for channel: " + channelName);
        return false;
    }
    
    // Create sample
    TelemetrySample sample;
    sample.timestamp = timestamp;
    sample.value.quatValue[0] = w;
    sample.value.quatValue[1] = x;
    sample.value.quatValue[2] = y;
    sample.value.quatValue[3] = z;
    
    // Add sample to channel
    it->second.data.push_back(sample);
    
    // Limit channel size
    if (it->second.data.size() > it->second.maxSamples) {
        it->second.data.pop_front();
    }
    
    // Notify listeners
    notifyListeners(channelName, sample);
    
    return true;
}

void Telemetry::registerListener(TelemetryListener* listener) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if listener is already registered
    auto it = std::find(listeners.begin(), listeners.end(), listener);
    if (it != listeners.end()) {
        utils::Logger::warning("Telemetry", "Listener already registered");
        return;
    }
    
    // Add listener
    listeners.push_back(listener);
    
    utils::Logger::debug("Telemetry", "Telemetry listener registered");
}

void Telemetry::unregisterListener(TelemetryListener* listener) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Find and remove listener
    auto it = std::find(listeners.begin(), listeners.end(), listener);
    if (it != listeners.end()) {
        listeners.erase(it);
        utils::Logger::debug("Telemetry", "Telemetry listener unregistered");
    }
}

void Telemetry::recordVehicle(std::shared_ptr<physics::Vehicle> vehicle, const std::string& prefix) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if vehicle is already recorded
    for (auto& weakVehicle : recordedVehicles) {
        if (auto storedVehicle = weakVehicle.lock()) {
            if (storedVehicle == vehicle) {
                utils::Logger::warning("Telemetry", "Vehicle already recorded");
                return;
            }
        }
    }
    
    // Add vehicle to recorded list
    recordedVehicles.push_back(vehicle);
    
    // Store prefix
    channelPrefixes[vehicle.get()] = prefix;
    
    // Initialize vehicle state
    VehicleState state;
    state.lastRecordTime = 0.0f;
    state.lastSpeed = 0.0f;
    state.topSpeed = 0.0f;
    state.totalDistance = 0.0f;
    vehicleStates[vehicle] = state;
    
    utils::Logger::info("Telemetry", "Vehicle recorded: " + vehicle->getSpec().name);
    
    // Create channels for this vehicle
    addChannel(prefix + "position.x", "m", TelemetryType::Float);
    addChannel(prefix + "position.y", "m", TelemetryType::Float);
    addChannel(prefix + "heading", "rad", TelemetryType::Float);
    addChannel(prefix + "velocity", "m/s", TelemetryType::Float);
    addChannel(prefix + "acceleration", "m/s²", TelemetryType::Float);
    addChannel(prefix + "engine_rpm", "rpm", TelemetryType::Float);
    addChannel(prefix + "gear", "", TelemetryType::Int);
    addChannel(prefix + "throttle", "%", TelemetryType::Float);
    addChannel(prefix + "brake", "%", TelemetryType::Float);
    addChannel(prefix + "steering", "rad", TelemetryType::Float);
}

void Telemetry::recordDriver(std::shared_ptr<ai::Driver> driver, const std::string& prefix) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Check if driver is already recorded
    for (auto& weakDriver : recordedDrivers) {
        if (auto storedDriver = weakDriver.lock()) {
            if (storedDriver == driver) {
                utils::Logger::warning("Telemetry", "Driver already recorded");
                return;
            }
        }
    }
    
    // Add driver to recorded list
    recordedDrivers.push_back(driver);
    
    // Store prefix
    channelPrefixes[driver.get()] = prefix;
    
    utils::Logger::info("Telemetry", "Driver recorded: " + driver->getName());
    
    // Create channels for this driver
    addChannel(prefix + "lap", "", TelemetryType::Int);
    addChannel(prefix + "position", "", TelemetryType::Int);
    addChannel(prefix + "lap_time", "s", TelemetryType::Float);
    addChannel(prefix + "best_lap_time", "s", TelemetryType::Float);
    addChannel(prefix + "aggression", "", TelemetryType::Float);
    addChannel(prefix + "consistency", "", TelemetryType::Float);
    addChannel(prefix + "adaptability", "", TelemetryType::Float);
}

void Telemetry::update(std::shared_ptr<Race> race) {
    PROFILE_SCOPE("Telemetry::update");
    
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    if (!race) {
        return;
    }
    
    // Get current timestamp
    float timestamp = race->getElapsedTime();
    
    // Record race state
    addSample("time", timestamp, timestamp);
    addSample("race_state", timestamp, static_cast<int>(race->getState()));
    
    // Record driver positions
    auto positions = race->getDriverPositions();
    for (const auto& pos : positions) {
        // Find driver prefix
        std::string prefix = "";
        for (auto& weakDriver : recordedDrivers) {
            if (auto driver = weakDriver.lock()) {
                if (driver->getName() == pos.driverName) {
                    prefix = channelPrefixes[driver.get()];
                    break;
                }
            }
        }
        
        if (!prefix.empty()) {
            // Record driver position
            addSample(prefix + "position", timestamp, pos.position);
            addSample(prefix + "lap", timestamp, pos.lap);
            addSample(prefix + "lap_time", timestamp, pos.lapTime);
            addSample(prefix + "best_lap_time", timestamp, pos.bestLapTime);
        }
    }
    
    // Record vehicle states
    for (auto& weakVehicle : recordedVehicles) {
        if (auto vehicle = weakVehicle.lock()) {
            recordVehicleState(vehicle, timestamp);
        }
    }
    
    // Record driver states
    for (auto& weakDriver : recordedDrivers) {
        if (auto driver = weakDriver.lock()) {
            recordDriverState(driver, timestamp);
        }
    }
    
    // Update lap statistics
    updateLapStatistics(*race);
}

const LapStatistics& Telemetry::getLapStatistics() const {
    return lapStats;
}

std::vector<std::string> Telemetry::getChannelNames() const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    std::vector<std::string> names;
    names.reserve(channels.size());
    
    for (const auto& pair : channels) {
        names.push_back(pair.first);
    }
    
    return names;
}

const TelemetryChannel& Telemetry::getChannelInfo(const std::string& channelName) const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    auto it = channels.find(channelName);
    if (it == channels.end()) {
        static TelemetryChannel emptyChannel;
        utils::Logger::warning("Telemetry", "Channel not found: " + channelName);
        return emptyChannel;
    }
    
    return it->second;
}

std::vector<TelemetrySample> Telemetry::getChannelData(const std::string& channelName, 
                                               float startTime, float endTime) const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    std::vector<TelemetrySample> result;
    
    auto it = channels.find(channelName);
    if (it == channels.end()) {
        utils::Logger::warning("Telemetry", "Channel not found: " + channelName);
        return result;
    }
    
    // Copy samples in the time range
    for (const auto& sample : it->second.data) {
        if (sample.timestamp >= startTime && sample.timestamp <= endTime) {
            result.push_back(sample);
        }
    }
    
    return result;
}

bool Telemetry::saveToFile(const std::string& filename) const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    try {
        // Open file
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            utils::Logger::error("Telemetry", "Failed to open file for writing: " + filename);
            return false;
        }
        
        // Write file header
        file << "# Neural Racer Telemetry Data\n";
        file << "# Channels: " << channels.size() << "\n";
        
        // Write channel information
        for (const auto& pair : channels) {
            const auto& channel = pair.second;
            file << "CHANNEL:" << channel.name << ":" << static_cast<int>(channel.type) 
                 << ":" << channel.units << ":" << channel.data.size() << "\n";
            
            // Write samples
            for (const auto& sample : channel.data) {
                file << sample.timestamp << ",";
                
                // Write value based on type
                switch (channel.type) {
                    case TelemetryType::Float:
                        file << sample.value.floatValue;
                        break;
                        
                    case TelemetryType::Int:
                        file << sample.value.intValue;
                        break;
                        
                    case TelemetryType::Bool:
                        file << (sample.value.boolValue ? "1" : "0");
                        break;
                        
                    case TelemetryType::String:
                        file << sample.stringValue;
                        break;
                        
                    case TelemetryType::Vector2:
                        file << sample.value.vec2Value[0] << "," << sample.value.vec2Value[1];
                        break;
                        
                    case TelemetryType::Vector3:
                        file << sample.value.vec3Value[0] << "," << sample.value.vec3Value[1] 
                             << "," << sample.value.vec3Value[2];
                        break;
                        
                    case TelemetryType::Quaternion:
                        file << sample.value.quatValue[0] << "," << sample.value.quatValue[1] 
                             << "," << sample.value.quatValue[2] << "," << sample.value.quatValue[3];
                        break;
                }
                
                file << "\n";
            }
        }
        
        utils::Logger::info("Telemetry", "Telemetry data saved to file: " + filename);
        return true;
    } catch (const std::exception& e) {
        utils::Logger::error("Telemetry", "Failed to save telemetry data: " + std::string(e.what()));
        return false;
    }
}

bool Telemetry::loadFromFile(const std::string& filename) {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    try {
        // Open file
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            utils::Logger::error("Telemetry", "Failed to open file for reading: " + filename);
            return false;
        }
        
        // Clear existing data
        clear();
        
        // Read file line by line
        std::string line;
        std::string currentChannel;
        TelemetryType currentType = TelemetryType::Float;
        
        while (std::getline(file, line)) {
            // Skip comments and empty lines
            if (line.empty() || line[0] == '#') {
                continue;
            }
            
            // Check for channel definition
            if (line.substr(0, 8) == "CHANNEL:") {
                // Parse channel information
                size_t pos1 = line.find(':', 8);
                size_t pos2 = line.find(':', pos1 + 1);
                size_t pos3 = line.find(':', pos2 + 1);
                
                if (pos1 != std::string::npos && pos2 != std::string::npos && pos3 != std::string::npos) {
                    std::string name = line.substr(8, pos1 - 8);
                    int type = std::stoi(line.substr(pos1 + 1, pos2 - pos1 - 1));
                    std::string units = line.substr(pos2 + 1, pos3 - pos2 - 1);
                    
                    // Create channel
                    currentChannel = name;
                    currentType = static_cast<TelemetryType>(type);
                    addChannel(name, units, currentType);
                }
            } else {
                // Parse sample data
                std::stringstream ss(line);
                std::string token;
                std::vector<std::string> tokens;
                
                while (std::getline(ss, token, ',')) {
                    tokens.push_back(token);
                }
                
                if (!tokens.empty() && !currentChannel.empty()) {
                    float timestamp = std::stof(tokens[0]);
                    
                    // Parse value based on type
                    switch (currentType) {
                        case TelemetryType::Float:
                            if (tokens.size() > 1) {
                                addSample(currentChannel, timestamp, std::stof(tokens[1]));
                            }
                            break;
                            
                        case TelemetryType::Int:
                            if (tokens.size() > 1) {
                                addSample(currentChannel, timestamp, std::stoi(tokens[1]));
                            }
                            break;
                            
                        case TelemetryType::Bool:
                            if (tokens.size() > 1) {
                                addSample(currentChannel, timestamp, tokens[1] == "1");
                            }
                            break;
                            
                        case TelemetryType::String:
                            if (tokens.size() > 1) {
                                addSample(currentChannel, timestamp, tokens[1]);
                            }
                            break;
                            
                        case TelemetryType::Vector2:
                            if (tokens.size() > 2) {
                                addSample(currentChannel, timestamp, 
                                         std::stof(tokens[1]), std::stof(tokens[2]));
                            }
                            break;
                            
                        case TelemetryType::Vector3:
                            if (tokens.size() > 3) {
                                addSample(currentChannel, timestamp, 
                                         std::stof(tokens[1]), std::stof(tokens[2]), std::stof(tokens[3]));
                            }
                            break;
                            
                        case TelemetryType::Quaternion:
                            if (tokens.size() > 4) {
                                addSample(currentChannel, timestamp, 
                                         std::stof(tokens[1]), std::stof(tokens[2]), 
                                         std::stof(tokens[3]), std::stof(tokens[4]));
                            }
                            break;
                    }
                }
            }
        }
        
        utils::Logger::info("Telemetry", "Telemetry data loaded from file: " + filename);
        return true;
    } catch (const std::exception& e) {
        utils::Logger::error("Telemetry", "Failed to load telemetry data: " + std::string(e.what()));
        return false;
    }
}

bool Telemetry::exportToCsv(const std::string& filename) const {
    // Lock mutex to prevent concurrent access
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    try {
        // Open file
        std::ofstream file(filename);
        if (!file.is_open()) {
            utils::Logger::error("Telemetry", "Failed to open CSV file for writing: " + filename);
            return false;
        }
        
        // Get all channel names
        std::vector<std::string> channelNames;
        for (const auto& pair : channels) {
            channelNames.push_back(pair.first);
        }
        
        // Write CSV header
        file << "Timestamp";
        for (const auto& name : channelNames) {
            file << "," << name;
        }
        file << "\n";
        
        // Find time channel if it exists
        auto timeIt = channels.find("time");
        if (timeIt == channels.end()) {
            utils::Logger::warning("Telemetry", "Time channel not found for CSV export");
            return false;
        }
        
        // Get all unique timestamps
        std::set<float> timestamps;
        for (const auto& sample : timeIt->second.data) {
            timestamps.insert(sample.timestamp);
        }
        
        // Write data rows
        for (float timestamp : timestamps) {
            file << timestamp;
            
            // Find values for each channel at this timestamp
            for (const auto& name : channelNames) {
                if (name == "time") {
                    // Skip time column (already used as timestamp)
                    file << ",";
                    continue;
                }
                
                auto it = channels.find(name);
                if (it != channels.end()) {
                    // Find sample closest to timestamp
                    const TelemetrySample* closestSample = nullptr;
                    float closestDiff = std::numeric_limits<float>::max();
                    
                    for (const auto& sample : it->second.data) {
                        float diff = std::abs(sample.timestamp - timestamp);
                        if (diff < closestDiff) {
                            closestDiff = diff;
                            closestSample = &sample;
                        }
                    }
                    
                    // Write value if found and within 0.1s
                    if (closestSample && closestDiff < 0.1f) {
                        file << ",";
                        
                        // Write value based on type
                        switch (it->second.type) {
                            case TelemetryType::Float:
                                file << closestSample->value.floatValue;
                                break;
                                
                            case TelemetryType::Int:
                                file << closestSample->value.intValue;
                                break;
                                
                            case TelemetryType::Bool:
                                file << (closestSample->value.boolValue ? "1" : "0");
                                break;
                                
                            case TelemetryType::String:
                                file << closestSample->stringValue;
                                break;
                                
                            case TelemetryType::Vector2:
                                file << closestSample->value.vec2Value[0] << ";" 
                                     << closestSample->value.vec2Value[1];
                                break;
                                
                            case TelemetryType::Vector3:
                                file << closestSample->value.vec3Value[0] << ";" 
                                     << closestSample->value.vec3Value[1] << ";" 
                                     << closestSample->value.vec3Value[2];
                                break;
                                
                            case TelemetryType::Quaternion:
                                file << closestSample->value.quatValue[0] << ";" 
                                     << closestSample->value.quatValue[1] << ";" 
                                     << closestSample->value.quatValue[2] << ";" 
                                     << closestSample->value.quatValue[3];
                                break;
                        }
                    } else {
                        // No data found for this timestamp
                        file << ",";
                    }
                } else {
                    // Channel not found
                    file << ",";
                }
            }
            
            file << "\n";
        }
        
        utils::Logger::info("Telemetry", "Telemetry data exported to CSV: " + filename);
        return true;
    } catch (const std::exception& e) {
        utils::Logger::error("Telemetry", "Failed to export telemetry data to CSV: " + std::string(e.what()));
        return false;
    }
}

void Telemetry::clear() {
    // Lock mutex to prevent concurrent modification
    std::lock_guard<std::mutex> lock(telemetryMutex);
    
    // Clear all channels
    channels.clear();
    
    // Reset lap statistics
    lapStats = LapStatistics();
    
    utils::Logger::info("Telemetry", "Telemetry data cleared");
}

void Telemetry::notifyListeners(const std::string& channelName, const TelemetrySample& sample) {
    // Notify all registered listeners
    for (auto listener : listeners) {
        listener->onTelemetryData(channelName, sample);
    }
}

void Telemetry::recordVehicleState(std::shared_ptr<physics::Vehicle> vehicle, float timestamp) {
    // Get vehicle state
    auto state = vehicle->getState();
    
    // Get vehicle prefix
    std::string prefix = channelPrefixes[vehicle.get()];
    
    // Record vehicle state
    addSample(prefix + "position.x", timestamp, state.positionX);
    addSample(prefix + "position.y", timestamp, state.positionY);
    addSample(prefix + "heading", timestamp, state.heading);
    addSample(prefix + "velocity", timestamp, state.velocity);
    addSample(prefix + "acceleration", timestamp, state.acceleration);
    addSample(prefix + "engine_rpm", timestamp, state.engineRPM);
    addSample(prefix + "gear", timestamp, state.gear);
    addSample(prefix + "throttle", timestamp, state.throttle * 100.0f);
    addSample(prefix + "brake", timestamp, state.brake * 100.0f);
    addSample(prefix + "steering", timestamp, state.steeringAngle);
    
    // Update vehicle state and calculate derived metrics
    auto& vState = vehicleStates[vehicle];
    
    // Calculate distance traveled since last record
    if (vState.lastRecordTime > 0.0f) {
        float deltaTime = timestamp - vState.lastRecordTime;
        float avgSpeed = (vState.lastSpeed + state.velocity) * 0.5f;
        float distance = avgSpeed * deltaTime;
        
        vState.totalDistance += distance;
        
        // Record distance traveled
        addSample(prefix + "distance", timestamp, vState.totalDistance);
    }
    
    // Update last speed and record time
    vState.lastSpeed = state.velocity;
    vState.lastRecordTime = timestamp;
    
    // Update top speed
    if (state.velocity > vState.topSpeed) {
        vState.topSpeed = state.velocity;
        addSample(prefix + "top_speed", timestamp, vState.topSpeed);
    }
    
    // Calculate speed in km/h
    float speedKmh = state.velocity * 3.6f;
    addSample(prefix + "speed_kmh", timestamp, speedKmh);
}

void Telemetry::recordDriverState(std::shared_ptr<ai::Driver> driver, float timestamp) {
    // Get driver parameters
    const auto& params = driver->getParameters();
    
    // Get driver prefix
    std::string prefix = channelPrefixes[driver.get()];
    
    // Record driver parameters
    addSample(prefix + "aggression", timestamp, params.aggression);
    addSample(prefix + "consistency", timestamp, params.consistency);
    addSample(prefix + "adaptability", timestamp, params.adaptability);
    addSample(prefix + "defensiveness", timestamp, params.defensiveness);
    addSample(prefix + "tire_management", timestamp, params.tireManagement);
    addSample(prefix + "fuel_management", timestamp, params.fuelManagement);
    addSample(prefix + "wet_skill", timestamp, params.wetSkill);
    addSample(prefix + "racecraft", timestamp, params.racecraft);
    
    // Record current strategy
    addSample(prefix + "strategy", timestamp, static_cast<int>(driver->getStrategy()));
    
    // Record current lap
    addSample(prefix + "lap", timestamp, driver->getCurrentLap());
    
    // Record driver statistics
    const auto& stats = driver->getStats();
    addSample(prefix + "best_lap_time", timestamp, stats.bestLapTime);
    addSample(prefix + "avg_lap_time", timestamp, stats.avgLapTime);
    addSample(prefix + "races_completed", timestamp, stats.racesCompleted);
    addSample(prefix + "races_won", timestamp, stats.racesWon);
    addSample(prefix + "podium_finishes", timestamp, stats.podiumFinishes);
}

void Telemetry::updateLapStatistics(const Race& race) {
    // Get driver positions
    auto positions = race.getDriverPositions();
    
    // Check if we have any positions
    if (positions.empty()) {
        return;
    }
    
    // Find leader
    const DriverPosition* leader = nullptr;
    for (const auto& pos : positions) {
        if (pos.position == 1) {
            leader = &pos;
            break;
        }
    }
    
    if (!leader) {
        return;
    }
    
    // Update lap statistics
    lapStats.currentLap = leader->lap;
    
    // Find best lap time
    for (const auto& pos : positions) {
        if (pos.bestLapTime > 0.0f && 
            (lapStats.bestLapTimeSeconds == 0.0f || pos.bestLapTime < lapStats.bestLapTimeSeconds)) {
            lapStats.bestLapTimeSeconds = pos.bestLapTime;
        }
    }
    
    // Update average and top speed
    // This would require more data from vehicles
    
    // Notify listeners about lap completion
    for (const auto& pos : positions) {
        if (pos.lap > 0 && pos.lapTime < 0.1f) {
            // Just completed a lap
            for (auto listener : listeners) {
                listener->onLapCompleted(pos.driverName, pos.bestLapTime, pos.lap);
            }
        }
    }
}

} // namespace simulation
} // namespace neural_racer