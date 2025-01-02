/**
 * Device type enumeration for agricultural robots
 * Provides type-safe categorization of supported autonomous units
 */
export enum DeviceType {
    DRONE = 'DRONE',
    GROUND_ROBOT = 'GROUND_ROBOT'
}

/**
 * Operational status enumeration for device states
 * Enables real-time status tracking and monitoring
 */
export enum DeviceStatus {
    ACTIVE = 'ACTIVE',
    IDLE = 'IDLE',
    CHARGING = 'CHARGING',
    MAINTENANCE = 'MAINTENANCE',
    ERROR = 'ERROR',
    OFFLINE = 'OFFLINE'
}

/**
 * Device capability enumeration defining supported operations
 * Used for mission planning and task assignment
 */
export enum DeviceCapability {
    SURVEILLANCE = 'SURVEILLANCE',
    TREATMENT = 'TREATMENT',
    SAMPLING = 'SAMPLING',
    MONITORING = 'MONITORING'
}

/**
 * Interface for precise device location tracking
 * Supports real-time positioning and movement monitoring
 */
export interface IDeviceLocation {
    /** Latitude in decimal degrees */
    latitude: number;
    
    /** Longitude in decimal degrees */
    longitude: number;
    
    /** Altitude in meters above sea level */
    altitude: number;
    
    /** Heading in degrees from true north */
    heading: number;
    
    /** Ground speed in meters per second */
    speed: number;
    
    /** Position accuracy in meters */
    accuracy: number;
}

/**
 * Interface for device metadata and maintenance information
 * Tracks device specifications and operational history
 */
export interface IDeviceMetadata {
    /** Device model identifier */
    model: string;
    
    /** Current firmware version */
    firmware: string;
    
    /** Date of last maintenance check */
    lastMaintenance: Date;
    
    /** Total operational hours */
    operationalHours: number;
}

/**
 * Comprehensive interface for agricultural robot devices
 * Provides complete type safety and monitoring support for the fleet management system
 */
export interface IDevice {
    /** Unique device identifier */
    id: string;
    
    /** Human-readable device name */
    name: string;
    
    /** Primary device classification */
    type: DeviceType;
    
    /** Specific model variant */
    subType: string;
    
    /** Array of supported operational capabilities */
    capabilities: DeviceCapability[];
    
    /** Current operational status */
    status: DeviceStatus;
    
    /** Current battery level percentage (0-100) */
    batteryLevel: number;
    
    /** Current device location and movement data */
    location: IDeviceLocation;
    
    /** Timestamp of last activity */
    lastActive: Date;
    
    /** Extended device information */
    metadata: IDeviceMetadata;
    
    /** Currently assigned mission ID if active */
    currentMission: string | null;
    
    /** Error code if device is in ERROR status */
    errorCode: string | null;
}