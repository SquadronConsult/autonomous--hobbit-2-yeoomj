/**
 * @fileoverview Comprehensive validation utilities for agricultural management system
 * @version 1.0.0
 * 
 * Provides robust validation for mission data, device telemetry, and system operations
 * with enhanced security measures and real-time validation capabilities.
 */

import { z } from 'zod'; // zod@3.22.0
import { IMission } from '../interfaces/IMission';
import { IDevice, DeviceType, DeviceStatus, DeviceCapability } from '../interfaces/IDevice';
import { MissionTypes } from '../constants/missionTypes';
import { HttpStatusCodes } from '../constants/statusCodes';

// Validation Constants
export const MISSION_NAME_MAX_LENGTH = 100;
export const MISSION_DESCRIPTION_MAX_LENGTH = 500;
export const MIN_ALTITUDE = 10; // meters
export const MAX_ALTITUDE = 120; // meters
export const MIN_SPEED = 0; // m/s
export const MAX_SPEED = 20; // m/s
export const MAX_BATTERY_LEVEL = 100;
export const MIN_BATTERY_LEVEL = 0;
export const TELEMETRY_TIMEOUT_MS = 5000;
export const MAX_VALIDATION_ATTEMPTS = 3;

/**
 * Enhanced validation error class with detailed context tracking
 */
export class ValidationError extends Error {
    public readonly code: number;
    public readonly details: Record<string, any>;
    public readonly context: string;
    public readonly timestamp: Date;

    constructor(
        message: string,
        code: number = HttpStatusCodes.BAD_REQUEST,
        details: Record<string, any> = {},
        context: string = 'validation'
    ) {
        super(message);
        this.name = 'ValidationError';
        this.code = code;
        this.details = details;
        this.context = context;
        this.timestamp = new Date();
    }
}

/**
 * Mission parameter schema with enhanced validation rules
 */
const missionParametersSchema = z.object({
    altitude: z.number()
        .min(MIN_ALTITUDE)
        .max(MAX_ALTITUDE)
        .optional(),
    speed: z.number()
        .min(MIN_SPEED)
        .max(MAX_SPEED),
    scanResolution: z.number()
        .positive()
        .optional(),
    treatmentType: z.string()
        .optional(),
    treatmentDensity: z.number()
        .positive()
        .optional()
});

/**
 * Device location schema with coordinate validation
 */
const deviceLocationSchema = z.object({
    latitude: z.number().min(-90).max(90),
    longitude: z.number().min(-180).max(180),
    altitude: z.number().min(0),
    heading: z.number().min(0).max(360),
    speed: z.number().min(0),
    accuracy: z.number().min(0)
});

/**
 * Validates mission data with comprehensive checks
 * @throws {ValidationError} If validation fails
 */
export function validateMission(mission: IMission): boolean {
    try {
        // Basic mission data validation
        if (!mission.id || !mission.name || !mission.type) {
            throw new ValidationError('Missing required mission fields');
        }

        // Name length validation
        if (mission.name.length > MISSION_NAME_MAX_LENGTH) {
            throw new ValidationError(
                'Mission name exceeds maximum length',
                HttpStatusCodes.BAD_REQUEST,
                { maxLength: MISSION_NAME_MAX_LENGTH }
            );
        }

        // Description length validation
        if (mission.description?.length > MISSION_DESCRIPTION_MAX_LENGTH) {
            throw new ValidationError(
                'Mission description exceeds maximum length',
                HttpStatusCodes.BAD_REQUEST,
                { maxLength: MISSION_DESCRIPTION_MAX_LENGTH }
            );
        }

        // Mission type validation
        if (!Object.values(MissionTypes).includes(mission.type)) {
            throw new ValidationError(
                'Invalid mission type',
                HttpStatusCodes.BAD_REQUEST,
                { validTypes: Object.values(MissionTypes) }
            );
        }

        // Parameters validation based on mission type
        const parametersResult = missionParametersSchema.safeParse(mission.parameters);
        if (!parametersResult.success) {
            throw new ValidationError(
                'Invalid mission parameters',
                HttpStatusCodes.BAD_REQUEST,
                parametersResult.error.format()
            );
        }

        // Coverage area validation
        if (!mission.coverageArea || !mission.coverageArea.coordinates) {
            throw new ValidationError('Invalid coverage area');
        }

        // Time constraints validation
        if (mission.endTime && mission.startTime > mission.endTime) {
            throw new ValidationError('End time cannot be before start time');
        }

        return true;
    } catch (error) {
        if (error instanceof ValidationError) {
            throw error;
        }
        throw new ValidationError(
            'Mission validation failed',
            HttpStatusCodes.BAD_REQUEST,
            { originalError: error }
        );
    }
}

/**
 * Validates device data with enhanced telemetry checks
 * @throws {ValidationError} If validation fails
 */
export function validateDevice(device: IDevice): boolean {
    try {
        // Basic device data validation
        if (!device.id || !device.name || !device.type) {
            throw new ValidationError('Missing required device fields');
        }

        // Device type validation
        if (!Object.values(DeviceType).includes(device.type)) {
            throw new ValidationError(
                'Invalid device type',
                HttpStatusCodes.BAD_REQUEST,
                { validTypes: Object.values(DeviceType) }
            );
        }

        // Battery level validation
        if (device.batteryLevel < MIN_BATTERY_LEVEL || device.batteryLevel > MAX_BATTERY_LEVEL) {
            throw new ValidationError(
                'Invalid battery level',
                HttpStatusCodes.BAD_REQUEST,
                { range: `${MIN_BATTERY_LEVEL}-${MAX_BATTERY_LEVEL}` }
            );
        }

        // Location validation
        const locationResult = deviceLocationSchema.safeParse(device.location);
        if (!locationResult.success) {
            throw new ValidationError(
                'Invalid device location',
                HttpStatusCodes.BAD_REQUEST,
                locationResult.error.format()
            );
        }

        // Capabilities validation
        if (!device.capabilities?.every(cap => Object.values(DeviceCapability).includes(cap))) {
            throw new ValidationError(
                'Invalid device capabilities',
                HttpStatusCodes.BAD_REQUEST,
                { validCapabilities: Object.values(DeviceCapability) }
            );
        }

        // Status validation
        if (!Object.values(DeviceStatus).includes(device.status)) {
            throw new ValidationError(
                'Invalid device status',
                HttpStatusCodes.BAD_REQUEST,
                { validStatuses: Object.values(DeviceStatus) }
            );
        }

        return true;
    } catch (error) {
        if (error instanceof ValidationError) {
            throw error;
        }
        throw new ValidationError(
            'Device validation failed',
            HttpStatusCodes.BAD_REQUEST,
            { originalError: error }
        );
    }
}

/**
 * Validates telemetry data with real-time constraints
 * @throws {ValidationError} If validation fails
 */
export function validateTelemetry(telemetry: any): boolean {
    try {
        // Timestamp validation
        const timestamp = new Date(telemetry.timestamp);
        const now = new Date();
        if (isNaN(timestamp.getTime()) || 
            now.getTime() - timestamp.getTime() > TELEMETRY_TIMEOUT_MS) {
            throw new ValidationError(
                'Invalid or stale telemetry timestamp',
                HttpStatusCodes.BAD_REQUEST,
                { maxAge: TELEMETRY_TIMEOUT_MS }
            );
        }

        // Sensor readings validation
        if (telemetry.sensorData) {
            for (const [sensor, value] of Object.entries(telemetry.sensorData)) {
                if (typeof value !== 'number' || isNaN(value)) {
                    throw new ValidationError(
                        'Invalid sensor reading',
                        HttpStatusCodes.BAD_REQUEST,
                        { sensor, value }
                    );
                }
            }
        }

        // Performance metrics validation
        if (telemetry.metrics) {
            const requiredMetrics = ['cpuUsage', 'memoryUsage', 'signalStrength'];
            for (const metric of requiredMetrics) {
                if (!(metric in telemetry.metrics)) {
                    throw new ValidationError(
                        'Missing required telemetry metric',
                        HttpStatusCodes.BAD_REQUEST,
                        { missingMetric: metric }
                    );
                }
            }
        }

        return true;
    } catch (error) {
        if (error instanceof ValidationError) {
            throw error;
        }
        throw new ValidationError(
            'Telemetry validation failed',
            HttpStatusCodes.BAD_REQUEST,
            { originalError: error }
        );
    }
}