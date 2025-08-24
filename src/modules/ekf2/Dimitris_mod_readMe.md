# Research EKF System - Developer Documentation

## Overview

This document describes the implementation of a configurable multi-instance Extended Kalman Filter (EKF) system designed for research purposes. The system allows running multiple EKF instances simultaneously with different sensor assignments and parameter configurations, enabling comparative analysis of estimator performance under various conditions.

## System Architecture

### Core Concept

The research EKF system extends PX4's existing multi-instance EKF capability by adding:
- **Configurable sensor assignment** for each EKF instance
- **Custom parameter sets** for research instances
- **Research-specific parameter validation** and management
- **Backward compatibility** with standard EKF operation

### Instance Types

The system operates with three types of EKF instances:

1. **Primary Instance**: The main navigation estimator used by the flight controller
2. **Research Instance 1**: First research estimator with configurable sensors and parameters
3. **Research Instance 2**: Second research estimator with configurable sensors and parameters

## File-by-File Implementation Logic

### 1. module.yaml - Parameter Definitions

The parameter definition file establishes the configuration interface for the research system.

#### Key Parameter Groups:

**Research Mode Control:**
- `EKFR_EN`: Master switch for research mode (0-2 instances)
- Controls whether research instances are created alongside the primary instance

**Sensor Assignment Parameters:**
- `EKFR_IMU_PRIMARY/1/2`: IMU instance selection (0-3) for each EKF instance
- `EKFR_MAG_PRIMARY/1/2`: Magnetometer instance selection (0-3) for each EKF instance
- Allows mixing and matching different sensor combinations

**EKF Configuration Parameters:**
- `EKFR_HGT_REF_1/2`: Height reference source (Baro/GPS/Range/Vision) for research instances
- `EKFR_GPS_CTRL_1/2`: GPS fusion control bitmask for research instances
- Enables testing different estimation strategies simultaneously

#### Design Philosophy:
The parameter system follows a clear naming convention where `EKFR_` prefixed parameters only affect research mode operation. When `EKFR_EN = 0`, all research parameters are ignored, ensuring clean separation between research and operational modes.

### 2. common.h - Core Data Structures

The common header file extends the core EKF parameter structure to support research configurations.

#### Enhanced Parameters Structure:

```cpp
struct parameters {
    // Standard EKF parameters (unchanged)
    int32_t height_sensor_ref{static_cast<int32_t>(HeightSensor::BARO)};
    int32_t gnss_ctrl{static_cast<int32_t>(GnssCtrl::HPOS) | static_cast<int32_t>(GnssCtrl::VEL)};

    // Research-specific parameter storage
    int32_t height_sensor_ref_r1{static_cast<int32_t>(HeightSensor::BARO)};  // Research instance 1
    int32_t height_sensor_ref_r2{static_cast<int32_t>(HeightSensor::BARO)};  // Research instance 2
    int32_t gnss_ctrl_r1{static_cast<int32_t>(GnssCtrl::HPOS) | static_cast<int32_t>(GnssCtrl::VEL)};
    int32_t gnss_ctrl_r2{static_cast<int32_t>(GnssCtrl::HPOS) | static_cast<int32_t>(GnssCtrl::VEL)};
};
```

#### Key Design Decision:
The research parameters are stored as separate fields in the main parameters structure rather than as separate structures. This approach ensures that the research parameters integrate seamlessly with the existing EKF parameter management system while maintaining clear separation.

### 3. EKF2.hpp - Class Interface and Research Logic

The header file defines the research instance management interface and extends the EKF2 class with research capabilities.

#### Research Instance Management:

```cpp
class EKF2 {
private:
    // Research instance identification
    bool _force_research{false};                    // Flag indicating this is a research instance
    int _research_instance_id{-1};                  // Research instance ID (-1 = primary, 0+ = research)

    // Research instance methods
    bool isResearchInstance() const;                // Query if this instance is for research
    int getResearchInstanceId() const;              // Get the research instance ID
    void setAsResearchInstance(bool val, int research_id = 0);  // Configure as research instance

    // Static factory method for creating research instances
    static bool createResearchInstances(int num_research_instances, int default_imu_idx, int default_mag_idx);
};
```

#### Parameter Management:
The class includes parameter objects for all research-specific parameters, enabling runtime access to configuration values:

```cpp
// Research parameter objects
(ParamBool<px4::params::EKFR_EN>) _param_ekfr_en;
(ParamInt<px4::params::EKFR_IMU_PRIMARY>) _param_ekfr_imu_primary;
(ParamInt<px4::params::EKFR_MAG_PRIMARY>) _param_ekfr_mag_primary;
// ... additional research parameters
```

#### Design Pattern:
The research functionality is implemented as an extension to the existing EKF2 class rather than creating separate research-specific classes. This approach minimizes code duplication while maintaining the ability to have different configurations for different instances.

### 4. EKF2.cpp - Implementation Logic

The implementation file contains the core research system logic, including instance creation, parameter management, and runtime configuration.

#### Research Instance Creation Logic:

```cpp
bool EKF2::createResearchInstances(int num_research_instances, int default_imu_idx, int default_mag_idx)
{
    // Phase 1: Determine sensor assignments from parameters
    // Read EKFR_IMU_PRIMARY, EKFR_MAG_PRIMARY for primary instance
    // Read EKFR_IMU_1/2, EKFR_MAG_1/2 for research instances

    // Phase 2: Create primary instance with configured sensors
    EKF2 *ekf2_primary = new EKF2(true, px4::ins_instance_to_wq(primary_imu_idx), false);
    ekf2_primary->multi_init(primary_imu_idx, primary_mag_idx);

    // Phase 3: Create research instances
    for (int research_id = 0; research_id < num_research_instances; research_id++) {
        EKF2 *ekf2_research = new EKF2(true, px4::ins_instance_to_wq(imu_idx), false);

        // CRITICAL: Set research flag BEFORE multi_init()
        ekf2_research->setAsResearchInstance(true, research_id);

        ekf2_research->multi_init(imu_idx, mag_idx);
    }
}
```

#### Runtime Parameter Override Logic:

The research instances apply custom parameters during the standard parameter update cycle:

```cpp
void EKF2::Run() {
    if (_parameter_update_sub.updated()) {
        updateParams();  // Load standard parameters first

        // Apply research-specific parameter overrides
        if (isResearchInstance()) {
            int research_id = getResearchInstanceId();
            switch (research_id) {
                case 0:
                    _params->height_sensor_ref = _param_ekfr_hgt_ref_1.get();
                    _params->gnss_ctrl = _param_ekfr_gps_ctrl_1.get();
                    break;
                case 1:
                    _params->height_sensor_ref = _param_ekfr_hgt_ref_2.get();
                    _params->gnss_ctrl = _param_ekfr_gps_ctrl_2.get();
                    break;
            }
        }

        VerifyParams();  // Validate final parameter set
    }
}
```

#### Task Spawning Logic:

The system integrates with PX4's task spawning mechanism to enable research mode when requested:

```cpp
int EKF2::task_spawn(int argc, char *argv[]) {
    // Check for research mode enablement
    param_get(param_find("EKFR_EN"), &ekfr_enabled);

    if (ekfr_enabled > 0 && multi_mode) {
        // Create research instances with configurable sensors
        success = createResearchInstances(ekfr_enabled, default_imu_idx, default_mag_idx);
    } else {
        // Fall back to standard multi-instance or single-instance mode
        // Standard PX4 behavior is preserved
    }
}
```

## Key Debugging Insights and Lessons Learned

### 1. Initialization Order is Critical

**Problem Discovered:** Initially, research instances were created first and then marked as research instances afterward. This caused the parameter override logic to fail because the research flag was not set during the critical `multi_init()` phase.

**Solution:** Always call `setAsResearchInstance()` **before** calling `multi_init()`. The research instance identification must be established before the instance attempts to initialize its publications and parameter systems.

**Code Pattern:**
```cpp
// WRONG - research flag set too late
ekf2_research->multi_init(imu_idx, mag_idx);
ekf2_research->setAsResearchInstance(true, research_id);

// CORRECT - research flag set before initialization
ekf2_research->setAsResearchInstance(true, research_id);
ekf2_research->multi_init(imu_idx, mag_idx);
```

### 2. Parameter Override Timing

**Problem Discovered:** Parameter overrides must happen after `updateParams()` but before `VerifyParams()` to ensure that research-specific settings are applied correctly and validated properly.

**Solution:** The parameter override logic is inserted at the precise point in the parameter update cycle where custom configurations can be safely applied.

### 3. Sensor Instance Management Dependencies

**Critical Discovery:** The ability to assign different sensors to different EKF instances depends on PX4's sensor management configuration.

**Key Requirements:**
- `SENS_IMU_MODE = 0`: Required for multi-IMU instance selection
- `SENS_MAG_MODE = 0`: Required for multi-magnetometer instance selection

**Problem:** When `SENS_MAG_MODE = 1` (single magnetometer mode), attempting to assign different magnetometers to different EKF instances results in publication instance conflicts and initialization failures.

**Root Cause:** PX4's sensor management system publishes sensor data differently depending on these mode settings. In single-sensor modes, only one instance of each sensor type is published, preventing multi-instance consumers from accessing different sensors.

### 4. uORB Publication Instance Conflicts

**Problem Manifestation:** Error messages like "publication instance problem: 1 att: 1 lpos: 1 gpos: 1" indicated that multiple EKF instances were attempting to use the same uORB publication instance numbers.

**Root Cause Analysis:** This occurred when the sensor assignment logic tried to create instances with sensor combinations that conflicted with PX4's automatic instance numbering system.

**Solution:** Ensuring proper sensor management configuration (`SENS_MAG_MODE = 0`) resolved the publication conflicts by enabling proper multi-instance sensor data flow.

### 5. Backward Compatibility Design

**Design Success:** The research system maintains complete backward compatibility. When `EKFR_EN = 0`, the system operates exactly as standard PX4 with no performance impact or behavioral changes.

**Implementation Pattern:** All research-specific logic is conditionally executed based on research mode status, ensuring that standard operations remain unaffected.

## Usage Patterns and Examples

### Basic Research Configuration

Compare GPS-enabled vs GPS-disabled estimation:

```bash
# Enable research mode with 1 research instance
param set EKFR_EN 1

# Primary instance: GPS enabled, GPS height reference
# (uses standard EKF2_HGT_REF and EKF2_GPS_CTRL parameters)

# Research instance: GPS disabled, barometric height reference
param set EKFR_HGT_REF_1 0    # Barometric height
param set EKFR_GPS_CTRL_1 0   # Disable GPS

# Use same sensors for both instances
param set EKFR_IMU_PRIMARY 0
param set EKFR_MAG_PRIMARY 0
param set EKFR_IMU_1 0
param set EKFR_MAG_1 0
```

### Advanced Multi-Sensor Configuration

Compare different sensor combinations:

```bash
# Enable research mode with 2 research instances
param set EKFR_EN 2

# Configure different sensors for each instance
param set EKFR_IMU_PRIMARY 0    # Primary uses IMU 0
param set EKFR_MAG_PRIMARY 0    # Primary uses internal magnetometer

param set EKFR_IMU_1 1          # Research 1 uses IMU 1
param set EKFR_MAG_1 1          # Research 1 uses external magnetometer

param set EKFR_IMU_2 2          # Research 2 uses IMU 2
param set EKFR_MAG_2 2          # Research 2 uses second external magnetometer

# Ensure proper sensor management
param set EKF2_MULTI_IMU  X       # Multi-IMU mode
param set SENS_MAG_MODE 0       # Multi-magnetometer publication
param set EKF2_MULTI_MAG X      # Multi-magnetometer for EKF2
```

## System Requirements and Prerequisites

### PX4 Configuration Requirements

1. **Multi-Instance Support:** `SENS_IMU_MODE = 0`
2. **Multi-Magnetometer Support:** `SENS_MAG_MODE = 0` (when using different magnetometers)
3. **Multi-Instance EKF:** `EKF2_MULTI_IMU ≥ 1`

### Hardware Requirements

- Multiple IMU sensors (for IMU comparison studies)
- Multiple magnetometer sensors (for magnetometer comparison studies)
- Sufficient computational resources for running multiple EKF instances simultaneously

## Future Development Considerations

### Potential Enhancements

1. **Dynamic Parameter Updates:** Currently, research parameters require restart to take effect. Future versions could support runtime parameter changes.

2. **Automatic Validation:** Consider adding automatic validation of sensor management settings when research mode is enabled.

3. **Performance Monitoring:** Integration with logging and analysis tools to facilitate automated comparison of research instance performance.

4. **Extended Parameter Sets:** Additional research parameters could be added for other EKF configuration aspects (noise parameters, fusion gates, etc.).

### Architectural Lessons

1. **Initialization Order Matters:** Always establish instance identity before initialization in multi-instance systems.

2. **Parameter Management Complexity:** Multi-instance parameter management requires careful consideration of override timing and validation.

3. **System Integration Dependencies:** Multi-instance functionality often depends on broader system configuration that may not be immediately obvious.

4. **Backward Compatibility Value:** Maintaining backward compatibility significantly eases adoption and testing of new features.

This research EKF system provides a powerful platform for comparative analysis of estimation algorithms and sensor configurations while maintaining the reliability and compatibility expected in production flight systems.
