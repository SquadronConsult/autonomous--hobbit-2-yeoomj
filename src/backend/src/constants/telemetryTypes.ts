/**
 * @fileoverview Defines standardized telemetry data types and units for the agricultural robotics system
 * @version 1.0.0
 * @license MIT
 */

/**
 * Enumeration of all telemetry data types supported by the system for monitoring
 * agricultural robots and drones. Includes comprehensive coverage of positional,
 * operational, and status metrics.
 */
export enum TelemetryType {
    /**
     * Geographic position of the robot/drone
     */
    LOCATION = 'LOCATION',

    /**
     * Current battery charge level
     */
    BATTERY = 'BATTERY',

    /**
     * Current movement speed
     */
    SPEED = 'SPEED',

    /**
     * Current height above ground level
     */
    ALTITUDE = 'ALTITUDE',

    /**
     * Current directional orientation
     */
    HEADING = 'HEADING',

    /**
     * Operational status of onboard sensors
     */
    SENSOR_STATUS = 'SENSOR_STATUS',

    /**
     * Operational status of imaging systems
     */
    CAMERA_STATUS = 'CAMERA_STATUS',

    /**
     * Operational status of propulsion systems
     */
    MOTOR_STATUS = 'MOTOR_STATUS',

    /**
     * Current level of treatment substance remaining
     */
    TREATMENT_LEVEL = 'TREATMENT_LEVEL',

    /**
     * Current error or warning status
     */
    ERROR_STATUS = 'ERROR_STATUS'
}

/**
 * Enumeration of standardized measurement units used for telemetry data.
 * Ensures consistent data representation across the entire system.
 */
export enum TelemetryUnit {
    /**
     * Distance measurements in meters
     */
    METERS = 'METERS',

    /**
     * Percentage values (0-100)
     */
    PERCENT = 'PERCENT',

    /**
     * Velocity measurements in meters per second
     */
    METERS_PER_SECOND = 'METERS_PER_SECOND',

    /**
     * Angular measurements in degrees (0-360)
     */
    DEGREES = 'DEGREES',

    /**
     * Volume measurements in liters
     */
    LITERS = 'LITERS'
}