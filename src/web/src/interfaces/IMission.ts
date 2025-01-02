/**
 * @fileoverview Mission interface definitions for agricultural operations
 * @version 1.0.0
 * 
 * Provides TypeScript interfaces for mission data structures used in the
 * agricultural management system's web application. These interfaces support
 * mission management, monitoring, and fleet coordination capabilities.
 * 
 * @see Technical Specification Section 1.2 - Core Features
 * @see Technical Specification Section 2.2.1 - ROS 2 Framework
 * @see Technical Specification Section 6.2.2 - Mission Control Interface
 */

import { MissionTypes } from '../constants/missionTypes';
import { MissionStatusCodes } from '../constants/statusCodes';
import type { GeoJSON } from '@types/geojson'; // @types/geojson@7.0.0

/**
 * Core interface defining the structure of an agricultural mission
 * Used for mission creation, monitoring, and management throughout the application
 */
export interface IMission {
    /** Unique identifier for the mission */
    id: string;

    /** Human-readable mission name */
    name: string;

    /** Detailed mission description */
    description: string;

    /** Type of agricultural mission (survey, treatment, monitoring) */
    type: MissionTypes;

    /** Current status of the mission */
    status: MissionStatusCodes;

    /** Array of device IDs assigned to this mission */
    assignedDevices: string[];

    /** Geographical area to be covered by the mission */
    coverageArea: GeoJSON.Polygon;

    /** Scheduled or actual mission start time */
    startTime: Date;

    /** Mission completion time (null if not completed) */
    endTime: Date | null;

    /** Mission completion percentage (0-100) */
    progress: number;

    /** Mission-specific configuration parameters */
    parameters: MissionParameters;
}

/**
 * Interface defining mission-specific configuration parameters
 * Contains settings that control mission execution behavior
 */
export interface MissionParameters {
    /** Flight altitude in meters for aerial operations */
    altitude: number;

    /** Movement speed in meters per second */
    speed: number;

    /** Scan resolution in centimeters per pixel for survey missions */
    scanResolution: number;

    /** Type of treatment to be applied (if applicable) */
    treatmentType: string;

    /** Treatment application density in liters per hectare */
    treatmentDensity: number;
}