/**
 * @fileoverview Core interface definitions for agricultural treatment operations
 * @version 1.0.0
 * 
 * Defines the data structures for precision agricultural treatments including
 * pest control applications and resource management interventions.
 */

import { Document } from 'mongoose'; // v6.9.1
import { MissionStatus } from '../constants/missionStatus';
import { RobotType } from '../constants/robotTypes';

/**
 * Represents the precise geographical location of a treatment application
 * including altitude for aerial operations
 */
export interface ITreatmentLocation {
    /** Latitude coordinate in decimal degrees */
    latitude: number;
    
    /** Longitude coordinate in decimal degrees */
    longitude: number;
    
    /** Altitude in meters above ground level */
    altitude: number;
}

/**
 * Defines the geographical coverage area of a treatment application
 * using GeoJSON-compatible format
 */
export interface ITreatmentCoverage {
    /** GeoJSON geometry type (e.g., 'Polygon', 'MultiPolygon') */
    type: string;
    
    /** Array of coordinate arrays defining the treatment area */
    coordinates: number[][];
}

/**
 * Core interface for agricultural treatment operations
 * Extends Document for MongoDB integration
 */
export interface ITreatment extends Document {
    /** Unique identifier for the treatment operation */
    id: string;

    /** Reference to the parent mission */
    missionId: string;

    /** Identifier of the device performing the treatment */
    deviceId: string;

    /** Type of treatment being applied (e.g., 'PESTICIDE', 'FERTILIZER') */
    type: string;

    /** Current status of the treatment operation */
    status: MissionStatus;

    /** Precise location of treatment application */
    location: ITreatmentLocation;

    /** Timestamp of treatment application */
    appliedAt: Date;

    /** Quantity of treatment applied in liters or grams */
    quantity: number;

    /** Geographical coverage area of treatment */
    coverage: ITreatmentCoverage;

    /** Additional treatment-specific parameters */
    parameters: Record<string, any>;

    /** Type of robot performing the treatment */
    robotType: RobotType;

    /** Chemical concentration in parts per million (ppm) */
    concentration?: number;

    /** Weather conditions during application */
    weatherConditions?: {
        windSpeed: number;
        temperature: number;
        humidity: number;
    };

    /** Treatment effectiveness metrics */
    effectiveness?: {
        coverage: number;
        uniformity: number;
        wastage: number;
    };
}