# Neural Racer Project Structure

```
neural-racer/
├── CMakeLists.txt                   # Main build configuration
├── README.md                        # Project documentation
├── .gitignore                       # Git ignore file
├── docs/                            # Documentation
│   ├── architecture.md              # System architecture overview
│   ├── hardware_interface.md        # Hardware interface documentation
│   └── ml_inference.md              # ML inference documentation
├── include/                         # Public header files
│   ├── neural_racer/                # Main include directory
│   │   ├── ai/                      # AI driver headers
│   │   │   ├── driver.hpp           # AI driver interface
│   │   │   ├── inference.hpp        # ML inference engine
│   │   │   └── model_loader.hpp     # ML model loader
│   │   ├── hardware/                # Hardware interface headers
│   │   │   ├── device_driver.hpp    # Device driver interface
│   │   │   ├── firmware.hpp         # Firmware controller
│   │   │   └── system_info.hpp      # System information utilities
│   │   ├── physics/                 # Physics engine headers
│   │   │   ├── engine.hpp           # Physics engine
│   │   │   ├── collision.hpp        # Collision detection
│   │   │   └── vehicle.hpp          # Vehicle model
│   │   ├── simulation/              # Simulation headers
│   │   │   ├── race.hpp             # Race controller
│   │   │   ├── telemetry.hpp        # Telemetry system
│   │   │   └── track.hpp            # Track definition
│   │   └── utils/                   # Utility headers
│   │       ├── config.hpp           # Configuration utilities
│   │       ├── logger.hpp           # Logging utilities
│   │       └── profiler.hpp         # Performance profiling
├── src/                             # Implementation files
│   ├── ai/                          # AI implementation
│   │   ├── driver.cpp               # AI driver implementation
│   │   ├── inference.cpp            # ML inference implementation
│   │   └── model_loader.cpp         # ML model loader implementation
│   ├── hardware/                    # Hardware implementation
│   │   ├── device_driver.cpp        # Device driver implementation
│   │   ├── firmware.cpp             # Firmware controller implementation
│   │   └── system_info.cpp          # System information implementation
│   ├── physics/                     # Physics implementation
│   │   ├── engine.cpp               # Physics engine implementation
│   │   ├── collision.cpp            # Collision detection implementation
│   │   └── vehicle.cpp              # Vehicle model implementation
│   ├── simulation/                  # Simulation implementation
│   │   ├── race.cpp                 # Race controller implementation
│   │   ├── telemetry.cpp            # Telemetry system implementation
│   │   └── track.cpp                # Track definition implementation
│   ├── utils/                       # Utility implementation
│   │   ├── config.cpp               # Configuration utilities implementation
│   │   ├── logger.cpp               # Logging utilities implementation
│   │   └── profiler.cpp             # Performance profiling implementation
│   └── main.cpp                     # Main entry point
├── tests/                           # Test files
│   ├── CMakeLists.txt               # Test build configuration
│   ├── ai_tests.cpp                 # AI tests
│   ├── hardware_tests.cpp           # Hardware interface tests
│   ├── physics_tests.cpp            # Physics engine tests
│   └── simulation_tests.cpp         # Simulation tests
├── examples/                        # Example programs
│   ├── CMakeLists.txt               # Examples build configuration
│   ├── basic_race.cpp               # Basic race example
│   └── hardware_monitor.cpp         # Hardware monitoring example
└── third_party/                     # Third-party dependencies
    └── README.md                    # Third-party dependencies documentation
```