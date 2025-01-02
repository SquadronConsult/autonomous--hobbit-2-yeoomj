/**
 * @fileoverview Robot type definitions and enumerations for agricultural management system
 * Supports ROS 2 integration and fleet management operations
 * @version 1.0.0
 */

/**
 * Primary classification of robot types in the fleet
 * Used for high-level fleet management and ROS 2 integration
 */
export enum RobotType {
    AERIAL_DRONE = 'AERIAL_DRONE',
    GROUND_ROBOT = 'GROUND_ROBOT'
}

/**
 * Operational capabilities of robots in the system
 * Used for mission planning and task allocation
 */
export enum RobotCapability {
    SURVEILLANCE = 'SURVEILLANCE',   // Aerial monitoring and data collection
    TREATMENT = 'TREATMENT',         // Precision application of treatments
    MONITORING = 'MONITORING',       // Environmental and crop condition monitoring
    INTERVENTION = 'INTERVENTION'    // Active response to detected issues
}

/**
 * Current operational status of robots
 * Used for fleet management and monitoring systems
 */
export enum RobotStatus {
    ACTIVE = 'ACTIVE',           // Currently executing a mission
    IDLE = 'IDLE',              // Available for task assignment
    CHARGING = 'CHARGING',       // Recharging batteries
    MAINTENANCE = 'MAINTENANCE', // Under maintenance or repair
    ERROR = 'ERROR'             // Experiencing operational issues
}

/**
 * Specific classification of aerial drone types
 * Used for specialized mission planning and capability assessment
 */
export enum DroneType {
    QUAD_COPTER = 'QUAD_COPTER', // Four-rotor configuration
    HEXA_COPTER = 'HEXA_COPTER'  // Six-rotor configuration
}

/**
 * Specific classification of ground robot types
 * Used for terrain-based task allocation and path planning
 */
export enum GroundRobotType {
    TRACKED = 'TRACKED',  // Track-based locomotion for rough terrain
    WHEELED = 'WHEELED'   // Wheel-based locomotion for even surfaces
}