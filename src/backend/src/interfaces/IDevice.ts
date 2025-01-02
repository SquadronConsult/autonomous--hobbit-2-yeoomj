/**
 * @fileoverview Core interface definitions for agricultural robot devices
 * Provides comprehensive type definitions for fleet management, telemetry monitoring,
 * and hardware integration with ROS 2 and DeepStream
 * @version 1.0.0
 */

import {
    RobotType,
    RobotCapability,
    RobotStatus,
    DroneType,
    GroundRobotType
} from '../constants/robotTypes';

/**
 * Standardized interface for precise device location and orientation tracking
 * Supports real-time positioning and navigation for both aerial and ground units
 */
export interface IDeviceLocation {
    /** Latitude in decimal degrees */
    latitude: number;
    
    /** Longitude in decimal degrees */
    longitude: number;
    
    /** Altitude in meters above sea level */
    altitude: number;
    
    /** Heading in degrees from true north (0-359) */
    heading: number;
}

/**
 * Comprehensive interface for agricultural robot device management and monitoring
 * Supports fleet management, telemetry tracking, and ROS 2 integration
 */
export interface IDevice {
    /** Unique identifier for the device */
    id: string;
    
    /** Human-readable name for the device */
    name: string;
    
    /** Primary classification of the robot (aerial or ground) */
    type: RobotType;
    
    /** Specific robot configuration based on primary type */
    subType: DroneType | GroundRobotType;
    
    /** Array of operational capabilities supported by the device */
    capabilities: RobotCapability[];
    
    /** Current operational status of the device */
    status: RobotStatus;
    
    /** Current battery level as percentage (0-100) */
    batteryLevel: number;
    
    /** Current device location and orientation */
    location: IDeviceLocation;
    
    /** Timestamp of last activity or status update */
    lastActive: Date;
    
    /** Additional device-specific metadata for extensibility */
    metadata: Record<string, any>;
}