#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>

namespace neural_racer {
namespace simulation {

/**
 * @brief Track segment types
 */
enum class SegmentType {
    Straight,
    LeftTurn,
    RightTurn,
    Chicane,
    Hairpin
};

/**
 * @brief Track surface types
 */
enum class SurfaceType {
    Asphalt,
    Concrete,
    Gravel,
    Dirt,
    Grass,
    Sand,
    Snow,
    Ice
};

/**
 * @brief Track segment definition
 */
struct TrackSegment {
    SegmentType type;          ///< Segment type
    float length;              ///< Segment length in meters
    float width;               ///< Segment width in meters
    float curvature;           ///< Curvature (1/radius) - higher is tighter
    float bankAngle;           ///< Banking angle in radians
    float elevation;           ///< Elevation change over segment in meters
    SurfaceType surface;       ///< Surface type
    float gripFactor;          ///< Grip multiplier (1.0 = standard)
    
    TrackSegment() :
        type(SegmentType::Straight),
        length(100.0f),
        width(15.0f),
        curvature(0.0f),
        bankAngle(0.0f),
        elevation(0.0f),
        surface(SurfaceType::Asphalt),
        gripFactor(1.0f) {}
};

/**
 * @brief Track waypoint for navigation
 */
struct Waypoint {
    float x;                   ///< X position in world coordinates
    float y;                   ///< Y position in world coordinates
    float heading;             ///< Heading in radians
    float width;               ///< Track width at this point
    float distanceFromStart;   ///< Distance from track start in meters
    int segmentIndex;          ///< Index of containing segment
    float optimalSpeed;        ///< Optimal speed at this point
    
    Waypoint() :
        x(0.0f),
        y(0.0f),
        heading(0.0f),
        width(15.0f),
        distanceFromStart(0.0f),
        segmentIndex(0),
        optimalSpeed(100.0f) {}
};

/**
 * @brief Track sector for timing
 */
struct Sector {
    float startDistance;       ///< Start distance from track start
    float endDistance;         ///< End distance from track start
    std::string name;          ///< Sector name
    
    Sector() :
        startDistance(0.0f),
        endDistance(0.0f),
        name("Sector") {}
};

/**
 * @brief Checkpoint for timing and race position
 */
struct Checkpoint {
    float x;                   ///< X position in world coordinates
    float y;                   ///< Y position in world coordinates
    float width;               ///< Width of checkpoint line
    float heading;             ///< Heading of checkpoint in radians
    float distanceFromStart;   ///< Distance from track start in meters
    std::string name;          ///< Checkpoint name
    
    Checkpoint() :
        x(0.0f),
        y(0.0f),
        width(15.0f),
        heading(0.0f),
        distanceFromStart(0.0f),
        name("Checkpoint") {}
};

/**
 * @brief Track environment settings
 */
struct TrackEnvironment {
    enum class TimeOfDay {
        Morning,
        Noon,
        Afternoon,
        Evening,
        Night
    };
    
    TimeOfDay timeOfDay;        ///< Time of day
    float baseTemperature;      ///< Base temperature in Celsius
    float humidity;             ///< Humidity (0.0-1.0)
    float windBaseSpeed;        ///< Base wind speed in m/s
    float windVariability;      ///< Wind speed variability
    float rainProbability;      ///< Probability of rain (0.0-1.0)
    
    TrackEnvironment() :
        timeOfDay(TimeOfDay::Noon),
        baseTemperature(25.0f),
        humidity(0.5f),
        windBaseSpeed(2.0f),
        windVariability(1.0f),
        rainProbability(0.1f) {}
};

/**
 * @brief Track metadata
 */
struct TrackInfo {
    std::string name;          ///< Track name
    std::string location;      ///< Track location
    float length;              ///< Total track length in meters
    int segmentCount;          ///< Number of segments
    int sectorCount;           ///< Number of timing sectors
    int checkpointCount;       ///< Number of checkpoints
    float recordLapTime;       ///< Track record lap time in seconds
    std::string recordHolder;  ///< Name of record holder
    
    TrackInfo() :
        length(0.0f),
        segmentCount(0),
        sectorCount(0),
        checkpointCount(0),
        recordLapTime(0.0f) {}
};

/**
 * @brief Race track definition
 * 
 * This class defines a race track with segments, waypoints,
 * sectors, and checkpoints for racing simulation.
 */
class Track {
public:
    /**
     * @brief Waypoint lookup callback function type
     */
    using WaypointCallback = std::function<void(const Waypoint&)>;
    
    /**
     * @brief Construct a track with the given name
     * 
     * @param name Track name
     */
    explicit Track(const std::string& name);
    
    /**
     * @brief Load track from a file
     * 
     * @param filename Track definition file
     * @return true if track was loaded successfully, false otherwise
     */
    bool loadFromFile(const std::string& filename);
    
    /**
     * @brief Save track to a file
     * 
     * @param filename Target file
     * @return true if track was saved successfully, false otherwise
     */
    bool saveToFile(const std::string& filename) const;
    
    /**
     * @brief Add a segment to the track
     * 
     * @param segment Segment to add
     */
    void addSegment(const TrackSegment& segment);
    
    /**
     * @brief Add a waypoint to the track
     * 
     * @param waypoint Waypoint to add
     */
    void addWaypoint(const Waypoint& waypoint);
    
    /**
     * @brief Add a checkpoint to the track
     * 
     * @param checkpoint Checkpoint to add
     */
    void addCheckpoint(const Checkpoint& checkpoint);
    
    /**
     * @brief Add a sector to the track
     * 
     * @param sector Sector to add
     */
    void addSector(const Sector& sector);
    
    /**
     * @brief Get track name
     * 
     * @return const std::string& Track name
     */
    const std::string& getName() const;
    
    /**
     * @brief Set track information
     * 
     * @param info Track information
     */
    void setInfo(const TrackInfo& info);
    
    /**
     * @brief Get track information
     * 
     * @return const TrackInfo& Track information
     */
    const TrackInfo& getInfo() const;
    
    /**
     * @brief Set track environment
     * 
     * @param environment Track environment
     */
    void setEnvironment(const TrackEnvironment& environment);
    
    /**
     * @brief Get track environment
     * 
     * @return const TrackEnvironment& Track environment
     */
    const TrackEnvironment& getEnvironment() const;
    
    /**
     * @brief Get all track segments
     * 
     * @return const std::vector<TrackSegment>& Vector of track segments
     */
    const std::vector<TrackSegment>& getSegments() const;
    
    /**
     * @brief Get all track waypoints
     * 
     * @return const std::vector<Waypoint>& Vector of waypoints
     */
    const std::vector<Waypoint>& getWaypoints() const;
    
    /**
     * @brief Get all checkpoints
     * 
     * @return const std::vector<Checkpoint>& Vector of checkpoints
     */
    const std::vector<Checkpoint>& getCheckpoints() const;
    
    /**
     * @brief Get all sectors
     * 
     * @return const std::vector<Sector>& Vector of sectors
     */
    const std::vector<Sector>& getSectors() const;
    
    /**
     * @brief Find the nearest waypoint to a position
     * 
     * @param x X position
     * @param y Y position
     * @return const Waypoint& Nearest waypoint
     */
    const Waypoint& findNearestWaypoint(float x, float y) const;
    
    /**
     * @brief Find waypoints within the given distance
     * 
     * @param x X position
     * @param y Y position
     * @param distance Search distance
     * @param callback Callback function for each found waypoint
     */
    void findWaypointsInRange(float x, float y, float distance, WaypointCallback callback) const;
    
    /**
     * @brief Check if a position is on the track
     * 
     * @param x X position
     * @param y Y position
     * @param margin Margin beyond track edges to consider on-track
     * @return true if position is on the track, false otherwise
     */
    bool isOnTrack(float x, float y, float margin = 0.0f) const;
    
    /**
     * @brief Get the sector at the given distance from start
     * 
     * @param distance Distance from track start in meters
     * @return int Sector index (1-based), or 0 if not found
     */
    int getSectorAtDistance(float distance) const;
    
    /**
     * @brief Calculate the racing line
     * 
     * @return std::vector<Waypoint> Optimal racing line waypoints
     */
    std::vector<Waypoint> calculateRacingLine() const;
    
    /**
     * @brief Generate a track from parameters
     * 
     * @param turnCount Number of turns
     * @param length Approximate track length in meters
     * @param difficulty Track difficulty (0.0-1.0)
     * @return true if track was generated successfully, false otherwise
     */
    bool generateTrack(int turnCount, float length, float difficulty = 0.5f);
    
    /**
     * @brief Build waypoints from segments
     * 
     * This regenerates all waypoints based on the segment definitions.
     * 
     * @param waypointSpacing Distance between waypoints in meters
     */
    void buildWaypoints(float waypointSpacing = 10.0f);
    
    /**
     * @brief Calculate optimal speeds for each waypoint
     * 
     * This calculates the optimal racing speed for each waypoint.
     * 
     * @param maxSpeed Maximum speed in m/s
     * @param maxLateralG Maximum lateral G-force
     */
    void calculateOptimalSpeeds(float maxSpeed = 100.0f, float maxLateralG = 2.0f);

private:
    std::string name;
    TrackInfo info;
    TrackEnvironment environment;
    
    std::vector<TrackSegment> segments;
    std::vector<Waypoint> waypoints;
    std::vector<Checkpoint> checkpoints;
    std::vector<Sector> sectors;
    
    // Helper methods
    void updateTrackInfo();
    float calculateDistanceToWaypoint(float x, float y, const Waypoint& waypoint) const;
    void generateWaypointsForSegment(const TrackSegment& segment, int segmentIndex, 
                                    float startX, float startY, float startHeading,
                                    float waypointSpacing);
    
    // Add these helper methods for file parsing
    void processInfoLine(const std::string& line);
    void processEnvironmentLine(const std::string& line);
    void processSegmentLine(const std::string& line);
    void processCheckpointLine(const std::string& line);
    void processSectorLine(const std::string& line);
};

/**
 * @brief Factory for creating tracks
 */
class TrackFactory {
public:
    /**
     * @brief Create a track from a preset
     * 
     * @param presetName Name of the track preset
     * @return std::shared_ptr<Track> Shared pointer to the created track
     */
    static std::shared_ptr<Track> createFromPreset(const std::string& presetName);
    
    /**
     * @brief Create a randomly generated track
     * 
     * @param turnCount Number of turns
     * @param length Approximate track length in meters
     * @param difficulty Track difficulty (0.0-1.0)
     * @return std::shared_ptr<Track> Shared pointer to the created track
     */
    static std::shared_ptr<Track> createRandom(int turnCount, float length, float difficulty = 0.5f);
    
    /**
     * @brief Create a track from a file
     * 
     * @param filename Track definition file
     * @return std::shared_ptr<Track> Shared pointer to the created track
     */
    static std::shared_ptr<Track> createFromFile(const std::string& filename);
};

} // namespace simulation
} // namespace neural_racer