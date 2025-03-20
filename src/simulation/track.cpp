#include "neural_racer/simulation/track.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <random>
#include <numeric>

namespace neural_racer {
namespace simulation {

Track::Track(const std::string& name) : name(name) {
    info.name = name;
}

bool Track::loadFromFile(const std::string& filename) {
    std::cout << "Loading track from file: " << filename << std::endl;
    
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open track file: " << filename << std::endl;
            return false;
        }
        
        // Clear existing track data
        segments.clear();
        waypoints.clear();
        checkpoints.clear();
        sectors.clear();
        
        // Parse the file
        std::string line;
        std::string section;
        
        while (std::getline(file, line)) {
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#') {
                continue;
            }
            
            // Check for section markers
            if (line[0] == '[' && line.back() == ']') {
                section = line.substr(1, line.size() - 2);
                continue;
            }
            
            // Process data based on section
            if (section == "info") {
                processInfoLine(line);
            } else if (section == "environment") {
                processEnvironmentLine(line);
            } else if (section == "segment") {
                processSegmentLine(line);
            } else if (section == "checkpoint") {
                processCheckpointLine(line);
            } else if (section == "sector") {
                processSectorLine(line);
            }
        }
        
        // Generate waypoints from segments
        buildWaypoints();
        
        // Calculate optimal speeds
        calculateOptimalSpeeds();
        
        // Update track info
        updateTrackInfo();
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading track: " << e.what() << std::endl;
        return false;
    }
}

bool Track::saveToFile(const std::string& filename) const {
    std::cout << "Saving track to file: " << filename << std::endl;
    
    try {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open track file for writing: " << filename << std::endl;
            return false;
        }
        
        // Write header
        file << "# Neural Racer Track Definition\n";
        file << "# Track: " << name << "\n\n";
        
        // Write track info
        file << "[info]\n";
        file << "name = " << info.name << "\n";
        file << "location = " << info.location << "\n";
        file << "record_time = " << info.recordLapTime << "\n";
        file << "record_holder = " << info.recordHolder << "\n\n";
        
        // Write environment
        file << "[environment]\n";
        file << "time_of_day = " << static_cast<int>(environment.timeOfDay) << "\n";
        file << "temperature = " << environment.baseTemperature << "\n";
        file << "humidity = " << environment.humidity << "\n";
        file << "wind_speed = " << environment.windBaseSpeed << "\n";
        file << "wind_variability = " << environment.windVariability << "\n";
        file << "rain_probability = " << environment.rainProbability << "\n\n";
        
        // Write segments
        file << "[segment]\n";
        for (const auto& segment : segments) {
            file << static_cast<int>(segment.type) << ","
                 << segment.length << ","
                 << segment.width << ","
                 << segment.curvature << ","
                 << segment.bankAngle << ","
                 << segment.elevation << ","
                 << static_cast<int>(segment.surface) << ","
                 << segment.gripFactor << "\n";
        }
        file << "\n";
        
        // Write checkpoints
        file << "[checkpoint]\n";
        for (const auto& checkpoint : checkpoints) {
            file << checkpoint.x << ","
                 << checkpoint.y << ","
                 << checkpoint.width << ","
                 << checkpoint.heading << ","
                 << checkpoint.distanceFromStart << ","
                 << checkpoint.name << "\n";
        }
        file << "\n";
        
        // Write sectors
        file << "[sector]\n";
        for (const auto& sector : sectors) {
            file << sector.startDistance << ","
                 << sector.endDistance << ","
                 << sector.name << "\n";
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error saving track: " << e.what() << std::endl;
        return false;
    }
}

void Track::addSegment(const TrackSegment& segment) {
    segments.push_back(segment);
    updateTrackInfo();
}

void Track::addWaypoint(const Waypoint& waypoint) {
    waypoints.push_back(waypoint);
}

void Track::addCheckpoint(const Checkpoint& checkpoint) {
    checkpoints.push_back(checkpoint);
    updateTrackInfo();
}

void Track::addSector(const Sector& sector) {
    sectors.push_back(sector);
    updateTrackInfo();
}

const std::string& Track::getName() const {
    return name;
}

void Track::setInfo(const TrackInfo& newInfo) {
    info = newInfo;
}

const TrackInfo& Track::getInfo() const {
    return info;
}

void Track::setEnvironment(const TrackEnvironment& newEnvironment) {
    environment = newEnvironment;
}

const TrackEnvironment& Track::getEnvironment() const {
    return environment;
}

const std::vector<TrackSegment>& Track::getSegments() const {
    return segments;
}

const std::vector<Waypoint>& Track::getWaypoints() const {
    return waypoints;
}

const std::vector<Checkpoint>& Track::getCheckpoints() const {
    return checkpoints;
}

const std::vector<Sector>& Track::getSectors() const {
    return sectors;
}

const Waypoint& Track::findNearestWaypoint(float x, float y) const {
    if (waypoints.empty()) {
        throw std::runtime_error("No waypoints available");
    }
    
    float minDistance = std::numeric_limits<float>::max();
    size_t nearestIndex = 0;
    
    for (size_t i = 0; i < waypoints.size(); i++) {
        float distance = calculateDistanceToWaypoint(x, y, waypoints[i]);
        if (distance < minDistance) {
            minDistance = distance;
            nearestIndex = i;
        }
    }
    
    return waypoints[nearestIndex];
}

void Track::findWaypointsInRange(float x, float y, float distance, WaypointCallback callback) const {
    for (const auto& waypoint : waypoints) {
        float waypointDistance = calculateDistanceToWaypoint(x, y, waypoint);
        if (waypointDistance <= distance) {
            callback(waypoint);
        }
    }
}

bool Track::isOnTrack(float x, float y, float margin) const {
    if (waypoints.empty()) {
        return false;
    }
    
    // Find the nearest waypoint
    const Waypoint& nearest = findNearestWaypoint(x, y);
    
    // Calculate the distance to the nearest waypoint
    float distance = calculateDistanceToWaypoint(x, y, nearest);
    
    // Check if the distance is within the track width plus margin
    return distance <= (nearest.width / 2.0f + margin);
}

int Track::getSectorAtDistance(float distance) const {
    for (size_t i = 0; i < sectors.size(); i++) {
        if (distance >= sectors[i].startDistance && distance < sectors[i].endDistance) {
            return static_cast<int>(i + 1);  // 1-based sector index
        }
    }
    
    return 0;  // Not in any sector
}

std::vector<Waypoint> Track::calculateRacingLine() const {
    std::cout << "Calculating racing line..." << std::endl;
    
    // In a real implementation, this would use a sophisticated algorithm
    // to calculate the optimal racing line. For simplicity, we'll use a
    // basic approach that follows the track centerline with some
    // optimization for turns.
    
    if (waypoints.empty()) {
        std::cerr << "No waypoints available for racing line calculation" << std::endl;
        return {};
    }
    
    // Create a copy of waypoints to modify
    std::vector<Waypoint> racingLine = waypoints;
    
    // Optimize racing line by shifting waypoints in turns
    for (size_t i = 0; i < racingLine.size(); i++) {
        // Check if this waypoint is in a turn
        const Waypoint& wp = racingLine[i];
        size_t prevIdx = (i > 0) ? i - 1 : racingLine.size() - 1;
        size_t nextIdx = (i < racingLine.size() - 1) ? i + 1 : 0;
        
        // Calculate track direction change
        float prevHeading = racingLine[prevIdx].heading;
        float nextHeading = racingLine[nextIdx].heading;
        
        // Normalize heading difference
        float headingDiff = nextHeading - prevHeading;
        while (headingDiff > M_PI) headingDiff -= 2.0f * M_PI;
        while (headingDiff < -M_PI) headingDiff += 2.0f * M_PI;
        
        // If there's a significant turn, adjust racing line
        if (std::abs(headingDiff) > 0.1f) {
            // Outside-inside-outside line
            // For right turns, move left initially, then right, then left again
            // For left turns, move right initially, then left, then right again
            
            // Simplified: just shift the racing line based on heading difference
            float shift = 0.0f;
            
            // Determine the segment index
            int segmentIdx = wp.segmentIndex;
            if (segmentIdx >= 0 && segmentIdx < static_cast<int>(segments.size())) {
                // Get the segment
                const TrackSegment& segment = segments[segmentIdx];
                
                // Adjust shift based on segment type
                if (segment.type == SegmentType::LeftTurn) {
                    shift = -0.3f * segment.curvature * wp.width;  // Move right
                } else if (segment.type == SegmentType::RightTurn) {
                    shift = 0.3f * segment.curvature * wp.width;  // Move left
                }
            }
            
            // Apply shift perpendicular to track direction
            float perpX = -std::sin(wp.heading);
            float perpY = std::cos(wp.heading);
            
            racingLine[i].x += shift * perpX;
            racingLine[i].y += shift * perpY;
        }
    }
    
    return racingLine;
}

bool Track::generateTrack(int turnCount, float length, float difficulty) {
    std::cout << "Generating random track with " << turnCount << " turns" << std::endl;
    
    // Clear existing track data
    segments.clear();
    waypoints.clear();
    checkpoints.clear();
    sectors.clear();
    
    // Create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> lengthDist(50.0f, 300.0f);
    std::uniform_real_distribution<float> widthDist(10.0f, 20.0f);
    std::uniform_real_distribution<float> curvatureDist(0.01f, 0.05f * difficulty);
    std::uniform_int_distribution<int> turnTypeDist(0, 1);  // 0 = left, 1 = right
    
    // Create initial straight segment
    TrackSegment startSegment;
    startSegment.type = SegmentType::Straight;
    startSegment.length = 200.0f;
    startSegment.width = 15.0f;
    segments.push_back(startSegment);
    
    // Add turns and straights
    float remainingLength = length - startSegment.length;
    float straightLength = remainingLength / (turnCount * 2);
    
    for (int i = 0; i < turnCount; i++) {
        // Add a turn
        TrackSegment turnSegment;
        turnSegment.type = turnTypeDist(gen) == 0 ? SegmentType::LeftTurn : SegmentType::RightTurn;
        turnSegment.length = lengthDist(gen);
        turnSegment.width = widthDist(gen);
        turnSegment.curvature = curvatureDist(gen);
        segments.push_back(turnSegment);
        
        // Add a straight after the turn (except for the last turn)
        if (i < turnCount - 1) {
            TrackSegment straightSegment;
            straightSegment.type = SegmentType::Straight;
            straightSegment.length = straightLength;
            straightSegment.width = turnSegment.width;
            segments.push_back(straightSegment);
        }
    }
    
    // Generate waypoints from segments
    buildWaypoints();
    
    // Create track layout (close the loop)
    // This is a simplified approach - in a real implementation,
    // we would use a more sophisticated algorithm to create a
    // properly closed and smooth track
    
    // Add a final segment to close the loop
    // In a real implementation, this would be calculated to
    // connect back to the start
    
    // Calculate optimal speeds
    calculateOptimalSpeeds();
    
    // Add checkpoints
    // Start/finish line
    Checkpoint startFinish;
    startFinish.x = waypoints[0].x;
    startFinish.y = waypoints[0].y;
    startFinish.width = waypoints[0].width;
    startFinish.heading = waypoints[0].heading + M_PI / 2.0f;  // Perpendicular to track
    startFinish.distanceFromStart = 0.0f;
    startFinish.name = "Start/Finish";
    checkpoints.push_back(startFinish);
    
    // Add sectors
    // Divide the track into 3 equal sectors
    float totalLength = std::accumulate(segments.begin(), segments.end(), 0.0f,
                                       [](float sum, const TrackSegment& seg) { return sum + seg.length; });
    
    for (int i = 0; i < 3; i++) {
        Sector sector;
        sector.startDistance = i * totalLength / 3.0f;
        sector.endDistance = (i + 1) * totalLength / 3.0f;
        sector.name = "Sector " + std::to_string(i + 1);
        sectors.push_back(sector);
    }
    
    // Update track info
    updateTrackInfo();
    
    return true;
}

void Track::buildWaypoints(float waypointSpacing) {
    std::cout << "Building waypoints with spacing: " << waypointSpacing << "m" << std::endl;
    
    // Clear existing waypoints
    waypoints.clear();
    
    // Start at origin
    float x = 0.0f;
    float y = 0.0f;
    float heading = 0.0f;  // Initial heading (along x-axis)
    
    // Track the total distance from start
    float distanceFromStart = 0.0f;
    
    // Generate waypoints for each segment
    for (size_t i = 0; i < segments.size(); i++) {
        generateWaypointsForSegment(segments[i], static_cast<int>(i), x, y, heading, waypointSpacing);
        
        // Update position for next segment
        const TrackSegment& segment = segments[i];
        
        // Calculate end position based on segment type
        if (segment.type == SegmentType::Straight) {
            // Move forward
            x += segment.length * std::cos(heading);
            y += segment.length * std::sin(heading);
        } else if (segment.type == SegmentType::LeftTurn || segment.type == SegmentType::RightTurn) {
            // Calculate turn parameters
            float radius = 1.0f / segment.curvature;
            float arcAngle = segment.length / radius;
            
            // Adjust angle sign for turn direction
            if (segment.type == SegmentType::RightTurn) {
                arcAngle = -arcAngle;
            }
            
            // Calculate center of arc
            float centerX, centerY;
            if (segment.type == SegmentType::LeftTurn) {
                centerX = x - radius * std::sin(heading);
                centerY = y + radius * std::cos(heading);
            } else {  // RightTurn
                centerX = x + radius * std::sin(heading);
                centerY = y - radius * std::cos(heading);
            }
            
            // Update heading
            heading += arcAngle;
            
            // Normalize heading
            while (heading > 2.0f * M_PI) heading -= 2.0f * M_PI;
            while (heading < 0.0f) heading += 2.0f * M_PI;
            
            // Calculate end position
            x = centerX + radius * std::sin(heading + M_PI/2.0f * (segment.type == SegmentType::LeftTurn ? 1.0f : -1.0f));
            y = centerY - radius * std::cos(heading + M_PI/2.0f * (segment.type == SegmentType::LeftTurn ? 1.0f : -1.0f));
        }
        
        // Update total distance
        distanceFromStart += segment.length;
    }
}

void Track::calculateOptimalSpeeds(float maxSpeed, float maxLateralG) {
    std::cout << "Calculating optimal speeds..." << std::endl;
    
    if (waypoints.empty()) {
        std::cerr << "No waypoints available for speed calculation" << std::endl;
        return;
    }
    
    // Forward pass: limit speed based on upcoming corners
    for (size_t i = 0; i < waypoints.size(); i++) {
        Waypoint& wp = waypoints[i];
        
        // Start with maximum speed
        wp.optimalSpeed = maxSpeed;
        
        // Check if this waypoint is in a turn
        int segmentIdx = wp.segmentIndex;
        if (segmentIdx >= 0 && segmentIdx < static_cast<int>(segments.size())) {
            const TrackSegment& segment = segments[segmentIdx];
            
            if (segment.type == SegmentType::LeftTurn || segment.type == SegmentType::RightTurn) {
                // Calculate maximum speed for this corner based on lateral G
                // v^2 = a * r, where a is lateral acceleration and r is radius
                float radius = 1.0f / std::max(0.001f, segment.curvature);
                float maxCornerSpeed = std::sqrt(maxLateralG * 9.81f * radius);
                
                // Limit speed
                wp.optimalSpeed = std::min(wp.optimalSpeed, maxCornerSpeed);
            }
        }
    }
    
    // Backward pass: ensure proper braking zones
    for (int i = static_cast<int>(waypoints.size()) - 2; i >= 0; i--) {
        Waypoint& current = waypoints[i];
        const Waypoint& next = waypoints[i + 1];
        
        // Skip if current speed is already lower
        if (current.optimalSpeed <= next.optimalSpeed) {
            continue;
        }
        
        // Calculate distance to next waypoint
        float dx = next.x - current.x;
        float dy = next.y - current.y;
        float distance = std::sqrt(dx * dx + dy * dy);
        
        // Apply deceleration limit (3.0 m/s^2 is a reasonable braking rate)
        float deceleration = 3.0f;
        float maxNextSpeed = std::sqrt(current.optimalSpeed * current.optimalSpeed - 2.0f * deceleration * distance);
        
        if (maxNextSpeed < next.optimalSpeed) {
            // Can't slow down enough, must start braking earlier
            current.optimalSpeed = std::sqrt(next.optimalSpeed * next.optimalSpeed + 2.0f * deceleration * distance);
        }
    }
}

void Track::updateTrackInfo() {
    // Update track information based on segments
    info.segmentCount = static_cast<int>(segments.size());
    info.sectorCount = static_cast<int>(sectors.size());
    info.checkpointCount = static_cast<int>(checkpoints.size());
    
    // Calculate total length
    info.length = std::accumulate(segments.begin(), segments.end(), 0.0f,
                                 [](float sum, const TrackSegment& seg) { return sum + seg.length; });
}

float Track::calculateDistanceToWaypoint(float x, float y, const Waypoint& waypoint) const {
    // Calculate Euclidean distance
    float dx = x - waypoint.x;
    float dy = y - waypoint.y;
    return std::sqrt(dx * dx + dy * dy);
}

void Track::generateWaypointsForSegment(const TrackSegment& segment, int segmentIndex, 
                                       float startX, float startY, float startHeading,
                                       float waypointSpacing) {
    // Calculate the number of waypoints for this segment
    int numWaypoints = std::max(1, static_cast<int>(segment.length / waypointSpacing));
    
    // Track total distance from start
    float distanceFromStart = 0.0f;
    if (!waypoints.empty()) {
        distanceFromStart = waypoints.back().distanceFromStart + waypointSpacing;
    }
    
    if (segment.type == SegmentType::Straight) {
        // Generate waypoints along the straight
        for (int i = 0; i < numWaypoints; i++) {
            float t = static_cast<float>(i) / numWaypoints;
            float distance = t * segment.length;
            
            Waypoint wp;
            wp.x = startX + distance * std::cos(startHeading);
            wp.y = startY + distance * std::sin(startHeading);
            wp.heading = startHeading;
            wp.width = segment.width;
            wp.distanceFromStart = distanceFromStart + distance;
            wp.segmentIndex = segmentIndex;
            wp.optimalSpeed = 100.0f;  // Default speed, will be recalculated later
            
            waypoints.push_back(wp);
        }
    } else if (segment.type == SegmentType::LeftTurn || segment.type == SegmentType::RightTurn) {
        // Calculate turn parameters
        float radius = 1.0f / segment.curvature;
        float arcAngle = segment.length / radius;
        
        // Adjust angle sign for turn direction
        if (segment.type == SegmentType::RightTurn) {
            arcAngle = -arcAngle;
        }
        
        // Calculate center of arc
        float centerX, centerY;
        if (segment.type == SegmentType::LeftTurn) {
            centerX = startX - radius * std::sin(startHeading);
            centerY = startY + radius * std::cos(startHeading);
        } else {  // RightTurn
            centerX = startX + radius * std::sin(startHeading);
            centerY = startY - radius * std::cos(startHeading);
        }
        
        // Generate waypoints along the arc
        for (int i = 0; i < numWaypoints; i++) {
            float t = static_cast<float>(i) / numWaypoints;
            float angle = startHeading + t * arcAngle;
            
            // Normalize angle
            while (angle > 2.0f * M_PI) angle -= 2.0f * M_PI;
            while (angle < 0.0f) angle += 2.0f * M_PI;
            
            Waypoint wp;
            wp.heading = angle;
            
            // Calculate position
            if (segment.type == SegmentType::LeftTurn) {
                wp.x = centerX + radius * std::sin(angle + M_PI/2.0f);
                wp.y = centerY - radius * std::cos(angle + M_PI/2.0f);
            } else {  // RightTurn
                wp.x = centerX + radius * std::sin(angle - M_PI/2.0f);
                wp.y = centerY - radius * std::cos(angle - M_PI/2.0f);
            }
            
            wp.width = segment.width;
            wp.distanceFromStart = distanceFromStart + t * segment.length;
            wp.segmentIndex = segmentIndex;
            wp.optimalSpeed = 100.0f;  // Default speed, will be recalculated later
            
            waypoints.push_back(wp);
        }
    } else if (segment.type == SegmentType::Chicane) {
        // Implementation for chicane (S-curve)
        // This would be more complex in a real implementation
        // For now, treat it as a straight
        for (int i = 0; i < numWaypoints; i++) {
            float t = static_cast<float>(i) / numWaypoints;
            float distance = t * segment.length;
            
            Waypoint wp;
            wp.x = startX + distance * std::cos(startHeading);
            wp.y = startY + distance * std::sin(startHeading);
            wp.heading = startHeading;
            wp.width = segment.width;
            wp.distanceFromStart = distanceFromStart + distance;
            wp.segmentIndex = segmentIndex;
            wp.optimalSpeed = 100.0f;  // Default speed, will be recalculated later
            
            waypoints.push_back(wp);
        }
    } else if (segment.type == SegmentType::Hairpin) {
        // Implementation for hairpin (very tight turn)
        // This would be more complex in a real implementation
        // For now, treat it as a sharp turn
        
        // Calculate turn parameters
        float radius = 1.0f / segment.curvature;
        float arcAngle = segment.length / radius;
        
        // Hairpins are usually 180 degrees
        arcAngle = M_PI;
        
        // Calculate center of arc (assuming left hairpin)
        float centerX = startX - radius * std::sin(startHeading);
        float centerY = startY + radius * std::cos(startHeading);
        
        // Generate waypoints along the arc
        for (int i = 0; i < numWaypoints; i++) {
            float t = static_cast<float>(i) / numWaypoints;
            float angle = startHeading + t * arcAngle;
            
            // Normalize angle
            while (angle > 2.0f * M_PI) angle -= 2.0f * M_PI;
            while (angle < 0.0f) angle += 2.0f * M_PI;
            
            Waypoint wp;
            wp.heading = angle;
            
            // Calculate position
            wp.x = centerX + radius * std::sin(angle + M_PI/2.0f);
            wp.y = centerY - radius * std::cos(angle + M_PI/2.0f);
            
            wp.width = segment.width;
            wp.distanceFromStart = distanceFromStart + t * segment.length;
            wp.segmentIndex = segmentIndex;
            wp.optimalSpeed = 50.0f;  // Lower default speed for hairpins
            
            waypoints.push_back(wp);
        }
    }
}

// Helper methods for file parsing
void Track::processInfoLine(const std::string& line) {
    size_t equalPos = line.find('=');
    if (equalPos == std::string::npos) return;
    
    std::string key = line.substr(0, equalPos);
    std::string value = line.substr(equalPos + 1);
    
    // Trim whitespace
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_last_not_of(" \t") + 1);
    
    if (key == "name") {
        info.name = value;
    } else if (key == "location") {
        info.location = value;
    } else if (key == "record_time") {
        info.recordLapTime = std::stof(value);
    } else if (key == "record_holder") {
        info.recordHolder = value;
    }
}

void Track::processEnvironmentLine(const std::string& line) {
    size_t equalPos = line.find('=');
    if (equalPos == std::string::npos) return;
    
    std::string key = line.substr(0, equalPos);
    std::string value = line.substr(equalPos + 1);
    
    // Trim whitespace
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_last_not_of(" \t") + 1);
    
    if (key == "time_of_day") {
        environment.timeOfDay = static_cast<TrackEnvironment::TimeOfDay>(std::stoi(value));
    } else if (key == "temperature") {
        environment.baseTemperature = std::stof(value);
    } else if (key == "humidity") {
        environment.humidity = std::stof(value);
    } else if (key == "wind_speed") {
        environment.windBaseSpeed = std::stof(value);
    } else if (key == "wind_variability") {
        environment.windVariability = std::stof(value);
    } else if (key == "rain_probability") {
        environment.rainProbability = std::stof(value);
    }
}

void Track::processSegmentLine(const std::string& line) {
    std::stringstream ss(line);
    std::string item;
    std::vector<std::string> tokens;
    
    while (std::getline(ss, item, ',')) {
        tokens.push_back(item);
    }
    
    if (tokens.size() < 7) return;
    
    TrackSegment segment;
    segment.type = static_cast<SegmentType>(std::stoi(tokens[0]));
    segment.length = std::stof(tokens[1]);
    segment.width = std::stof(tokens[2]);
    segment.curvature = std::stof(tokens[3]);
    segment.bankAngle = std::stof(tokens[4]);
    segment.elevation = std::stof(tokens[5]);
    segment.surface = static_cast<SurfaceType>(std::stoi(tokens[6]));
    
    if (tokens.size() > 7) {
        segment.gripFactor = std::stof(tokens[7]);
    }
    
    segments.push_back(segment);
}

void Track::processCheckpointLine(const std::string& line) {
    std::stringstream ss(line);
    std::string item;
    std::vector<std::string> tokens;
    
    while (std::getline(ss, item, ',')) {
        tokens.push_back(item);
    }
    
    if (tokens.size() < 5) return;
    
    Checkpoint checkpoint;
    checkpoint.x = std::stof(tokens[0]);
    checkpoint.y = std::stof(tokens[1]);
    checkpoint.width = std::stof(tokens[2]);
    checkpoint.heading = std::stof(tokens[3]);
    checkpoint.distanceFromStart = std::stof(tokens[4]);
    
    if (tokens.size() > 5) {
        checkpoint.name = tokens[5];
    }
    
    checkpoints.push_back(checkpoint);
}

void Track::processSectorLine(const std::string& line) {
    std::stringstream ss(line);
    std::string item;
    std::vector<std::string> tokens;
    
    while (std::getline(ss, item, ',')) {
        tokens.push_back(item);
    }
    
    if (tokens.size() < 2) return;
    
    Sector sector;
    sector.startDistance = std::stof(tokens[0]);
    sector.endDistance = std::stof(tokens[1]);
    
    if (tokens.size() > 2) {
        sector.name = tokens[2];
    }
    
    sectors.push_back(sector);
}

// TrackFactory implementation
std::shared_ptr<Track> TrackFactory::createFromPreset(const std::string& presetName) {
    std::shared_ptr<Track> track = std::make_shared<Track>(presetName);
    
    // Predefined presets
    if (presetName == "Monaco") {
        // Monaco-inspired street circuit
        track->setInfo(TrackInfo{
            "Monaco", 
            "Monte Carlo", 
            3337.0f,  // 3.337 km
            19,       // 19 segments
            3,        // 3 sectors
            1,        // 1 checkpoint
            70.0f,    // Record time in seconds
            "AI Champion"
        });
        
        // Set environment
        track->setEnvironment(TrackEnvironment{
            TrackEnvironment::TimeOfDay::Afternoon,
            25.0f,   // 25°C
            0.5f,    // 50% humidity
            2.0f,    // 2 m/s wind
            0.5f,    // Wind variability
            0.1f     // 10% rain probability
        });
        
        // Add segments (simplified)
        TrackSegment straight1;
        straight1.type = SegmentType::Straight;
        straight1.length = 300.0f;
        straight1.width = 12.0f;
        track->addSegment(straight1);
        
        TrackSegment turn1;
        turn1.type = SegmentType::RightTurn;
        turn1.length = 50.0f;
        turn1.width = 10.0f;
        turn1.curvature = 0.03f;
        track->addSegment(turn1);
        
        TrackSegment straight2;
        straight2.type = SegmentType::Straight;
        straight2.length = 200.0f;
        straight2.width = 12.0f;
        track->addSegment(straight2);
        
        // Add more segments to complete the circuit
        
        // Build waypoints
        track->buildWaypoints();
        
        // Calculate optimal speeds
        track->calculateOptimalSpeeds();
        
        // Add checkpoints
        Checkpoint startFinish;
        startFinish.x = 0.0f;
        startFinish.y = 0.0f;
        startFinish.width = 12.0f;
        startFinish.heading = M_PI / 2.0f;
        startFinish.distanceFromStart = 0.0f;
        startFinish.name = "Start/Finish";
        track->addCheckpoint(startFinish);
        
        // Add sectors
        Sector sector1;
        sector1.startDistance = 0.0f;
        sector1.endDistance = 1112.3f;
        sector1.name = "Sector 1";
        track->addSector(sector1);
        
        Sector sector2;
        sector2.startDistance = 1112.3f;
        sector2.endDistance = 2224.6f;
        sector2.name = "Sector 2";
        track->addSector(sector2);
        
        Sector sector3;
        sector3.startDistance = 2224.6f;
        sector3.endDistance = 3337.0f;
        sector3.name = "Sector 3";
        track->addSector(sector3);
    } else if (presetName == "Monza") {
        // Monza-inspired high speed circuit
        track->setInfo(TrackInfo{
            "Monza", 
            "Italy", 
            5793.0f,  // 5.793 km
            11,       // 11 segments
            3,        // 3 sectors
            1,        // 1 checkpoint
            82.0f,    // Record time in seconds
            "AI Champion"
        });
        
        // Set environment and add segments similar to above
        
        // Build waypoints, calculate speeds, add checkpoints and sectors
    } else if (presetName == "Nurburgring") {
        // Nurburgring-inspired complex circuit
        track->setInfo(TrackInfo{
            "Nurburgring", 
            "Germany", 
            20800.0f,  // 20.8 km
            154,       // 154 segments
            3,         // 3 sectors
            1,         // 1 checkpoint
            360.0f,    // Record time in seconds
            "AI Champion"
        });
        
        // Set environment and add segments similar to above
        
        // Build waypoints, calculate speeds, add checkpoints and sectors
    } else if (presetName == "Silverstone") {
        // Silverstone-inspired fast circuit
        track->setInfo(TrackInfo{
            "Silverstone", 
            "United Kingdom", 
            5891.0f,  // 5.891 km
            18,       // 18 segments
            3,        // 3 sectors
            1,        // 1 checkpoint
            85.0f,    // Record time in seconds
            "AI Champion"
        });
        
        // Set environment and add segments similar to above
        
        // Build waypoints, calculate speeds, add checkpoints and sectors
    } else {
        // Default oval track
        track->generateTrack(4, 2000.0f, 0.5f);
    }
    
    return track;
}

std::shared_ptr<Track> TrackFactory::createRandom(int turnCount, float length, float difficulty) {
    std::shared_ptr<Track> track = std::make_shared<Track>("Random Track");
    track->generateTrack(turnCount, length, difficulty);
    return track;
}

std::shared_ptr<Track> TrackFactory::createFromFile(const std::string& filename) {
    std::shared_ptr<Track> track = std::make_shared<Track>("Loaded Track");
    if (track->loadFromFile(filename)) {
        return track;
    }
    
    // Fallback to a default track if loading fails
    return createRandom(8, 3000.0f, 0.5f);
}

} // namespace simulation
} // namespace neural_racer