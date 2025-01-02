/**
 * @fileoverview Mission interface definition for autonomous agricultural operations
 * @version 1.0.0
 * 
 * Defines the core data structure for agricultural missions in the autonomous management system.
 * Supports real-time mission execution, fleet coordination, and data persistence requirements.
 */

import { Document } from 'mongoose'; // v6.9.1
import { MissionStatus } from '../constants/missionStatus';
import { RobotType } from '../constants/robotTypes';

/**
 * Core mission interface extending MongoDB Document for agricultural operations.
 * Provides comprehensive mission specification for drone and ground robot coordination.
 * 
 * @interface IMission
 * @extends {Document}
 */
export interface IMission extends Document {
    /**
     * Unique identifier for the mission
     */
    id: string;

    /**
     * Human-readable mission name
     */
    name: string;

    /**
     * Optional detailed description of mission objectives
     */
    description?: string;

    /**
     * Current mission execution status
     */
    status: MissionStatus;

    /**
     * Mission classification type (e.g., 'surveillance', 'treatment')
     */
    type: string;

    /**
     * Devices assigned to execute this mission
     */
    assignedDevices: Array<{
        deviceId: string;
        type: RobotType;
    }>;

    /**
     * GeoJSON representation of mission coverage area
     */
    coverageArea: {
        type: string;
        coordinates: number[][];
        properties: Record<string, unknown>;
    };

    /**
     * Scheduled or actual mission start time
     */
    startTime: Date;

    /**
     * Mission completion time (null if not completed)
     */
    endTime: Date | null;

    /**
     * Mission completion percentage (0-100)
     */
    progress: number;

    /**
     * Mission execution parameters
     */
    parameters: {
        /** Flight altitude in meters for aerial operations */
        altitude?: number;
        /** Movement speed in meters per second */
        speed?: number;
        /** Scan overlap percentage for coverage missions */
        overlap?: number;
        /** Sensor resolution in pixels/meter */
        resolution?: number;
        /** Type of treatment for intervention missions */
        treatmentType?: string;
        /** Treatment application rate in units/area */
        applicationRate?: number;
        /** Additional custom parameters */
        customParams?: Record<string, unknown>;
    };

    /**
     * Mission metadata for system management
     */
    metadata: {
        /** Mission creation timestamp */
        createdAt: Date;
        /** Last update timestamp */
        updatedAt: Date;
        /** User ID of mission creator */
        createdBy: string;
        /** Mission specification version */
        version: number;
        /** Mission classification tags */
        tags: string[];
        /** Mission priority level (higher number = higher priority) */
        priority: number;
        /** IDs of dependent missions that must complete first */
        dependencies: string[];
        /** Indicates if mission can execute without network connectivity */
        offline: boolean;
    };
}