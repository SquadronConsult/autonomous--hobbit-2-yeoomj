/**
 * @fileoverview Comprehensive validation utilities for agricultural management system data structures
 * Implements robust validation rules ensuring data integrity and security compliance
 * @version 1.0.0
 */

import { validate, ValidationError } from 'class-validator'; // v0.14.0
import { sanitize } from 'class-sanitizer'; // v1.0.1
import { ErrorCodes } from '../constants/errorCodes';
import { IMission } from '../interfaces/IMission';
import { IDevice } from '../interfaces/IDevice';
import { ITelemetry } from '../interfaces/ITelemetry';
import { MissionStatus } from '../constants/missionStatus';
import { RobotType, RobotStatus } from '../constants/robotTypes';
import { TelemetryType, TelemetryUnit } from '../constants/telemetryTypes';

// Validation constants
const MISSION_ID_REGEX = /^AGM-M-\d{6}$/;
const DEVICE_ID_REGEX = /^AGM-D-\d{6}$/;
const MIN_BATTERY_LEVEL = 10;
const MAX_BATTERY_LEVEL = 100;
const MAX_MISSION_DURATION_HOURS = 24;
const MAX_TELEMETRY_AGE_MINUTES = 5;
const VALID_COORDINATE_RANGE = {
    minLat: -90,
    maxLat: 90,
    minLon: -180,
    maxLon: 180
};

/**
 * Validates mission data against schema, business rules, and security requirements
 * @param mission Mission data to validate
 * @returns Array of validation errors if any
 */
export async function validateMission(mission: IMission): Promise<ValidationError[]> {
    // Sanitize input data
    sanitize(mission);
    const errors: ValidationError[] = [];

    // Validate mission ID format
    if (!MISSION_ID_REGEX.test(mission.id)) {
        errors.push({
            property: 'id',
            constraints: {
                format: 'Mission ID must match format AGM-M-XXXXXX'
            }
        });
    }

    // Validate mission status transitions
    if (!Object.values(MissionStatus).includes(mission.status)) {
        errors.push({
            property: 'status',
            constraints: {
                enum: 'Invalid mission status'
            }
        });
    }

    // Validate coverage area coordinates
    if (mission.coverageArea?.coordinates) {
        for (const coordinate of mission.coverageArea.coordinates) {
            const [lon, lat] = coordinate;
            if (lat < VALID_COORDINATE_RANGE.minLat || lat > VALID_COORDINATE_RANGE.maxLat ||
                lon < VALID_COORDINATE_RANGE.minLon || lon > VALID_COORDINATE_RANGE.maxLon) {
                errors.push({
                    property: 'coverageArea',
                    constraints: {
                        range: 'Coordinates out of valid range'
                    }
                });
                break;
            }
        }
    }

    // Validate mission duration
    if (mission.startTime && mission.endTime) {
        const durationHours = (mission.endTime.getTime() - mission.startTime.getTime()) / (1000 * 60 * 60);
        if (durationHours > MAX_MISSION_DURATION_HOURS) {
            errors.push({
                property: 'duration',
                constraints: {
                    max: `Mission duration cannot exceed ${MAX_MISSION_DURATION_HOURS} hours`
                }
            });
        }
    }

    // Validate mission parameters
    if (mission.parameters) {
        if (mission.parameters.altitude && (mission.parameters.altitude < 0 || mission.parameters.altitude > 120)) {
            errors.push({
                property: 'parameters.altitude',
                constraints: {
                    range: 'Altitude must be between 0 and 120 meters'
                }
            });
        }

        if (mission.parameters.speed && (mission.parameters.speed < 0 || mission.parameters.speed > 15)) {
            errors.push({
                property: 'parameters.speed',
                constraints: {
                    range: 'Speed must be between 0 and 15 meters per second'
                }
            });
        }
    }

    return errors;
}

/**
 * Validates device data, operational parameters, and security requirements
 * @param device Device data to validate
 * @returns Array of validation errors if any
 */
export async function validateDevice(device: IDevice): Promise<ValidationError[]> {
    sanitize(device);
    const errors: ValidationError[] = [];

    // Validate device ID format
    if (!DEVICE_ID_REGEX.test(device.id)) {
        errors.push({
            property: 'id',
            constraints: {
                format: 'Device ID must match format AGM-D-XXXXXX'
            }
        });
    }

    // Validate device type
    if (!Object.values(RobotType).includes(device.type)) {
        errors.push({
            property: 'type',
            constraints: {
                enum: 'Invalid robot type'
            }
        });
    }

    // Validate battery level
    if (device.batteryLevel < MIN_BATTERY_LEVEL || device.batteryLevel > MAX_BATTERY_LEVEL) {
        errors.push({
            property: 'batteryLevel',
            constraints: {
                range: `Battery level must be between ${MIN_BATTERY_LEVEL} and ${MAX_BATTERY_LEVEL}`
            }
        });
    }

    // Validate device location
    if (device.location) {
        const { latitude, longitude, altitude } = device.location;
        if (latitude < VALID_COORDINATE_RANGE.minLat || latitude > VALID_COORDINATE_RANGE.maxLat ||
            longitude < VALID_COORDINATE_RANGE.minLon || longitude > VALID_COORDINATE_RANGE.maxLon) {
            errors.push({
                property: 'location',
                constraints: {
                    range: 'Device location coordinates out of valid range'
                }
            });
        }

        if (altitude < 0 || altitude > 120) {
            errors.push({
                property: 'location.altitude',
                constraints: {
                    range: 'Altitude must be between 0 and 120 meters'
                }
            });
        }
    }

    return errors;
}

/**
 * Validates telemetry data points, measurements, and security requirements
 * @param telemetry Telemetry data to validate
 * @returns Array of validation errors if any
 */
export async function validateTelemetry(telemetry: ITelemetry): Promise<ValidationError[]> {
    sanitize(telemetry);
    const errors: ValidationError[] = [];

    // Validate device ID reference
    if (!DEVICE_ID_REGEX.test(telemetry.deviceId)) {
        errors.push({
            property: 'deviceId',
            constraints: {
                format: 'Device ID must match format AGM-D-XXXXXX'
            }
        });
    }

    // Validate telemetry type
    if (!Object.values(TelemetryType).includes(telemetry.type)) {
        errors.push({
            property: 'type',
            constraints: {
                enum: 'Invalid telemetry type'
            }
        });
    }

    // Validate timestamp freshness
    const ageMinutes = (new Date().getTime() - telemetry.timestamp.getTime()) / (1000 * 60);
    if (ageMinutes > MAX_TELEMETRY_AGE_MINUTES) {
        errors.push({
            property: 'timestamp',
            constraints: {
                fresh: `Telemetry data cannot be older than ${MAX_TELEMETRY_AGE_MINUTES} minutes`
            }
        });
    }

    // Validate telemetry value based on type
    switch (telemetry.type) {
        case TelemetryType.BATTERY:
            if (typeof telemetry.value !== 'number' || 
                telemetry.value < MIN_BATTERY_LEVEL || 
                telemetry.value > MAX_BATTERY_LEVEL) {
                errors.push({
                    property: 'value',
                    constraints: {
                        range: `Battery level must be between ${MIN_BATTERY_LEVEL} and ${MAX_BATTERY_LEVEL}`
                    }
                });
            }
            break;
        case TelemetryType.SPEED:
            if (typeof telemetry.value !== 'number' || telemetry.value < 0 || telemetry.value > 15) {
                errors.push({
                    property: 'value',
                    constraints: {
                        range: 'Speed must be between 0 and 15 meters per second'
                    }
                });
            }
            break;
        case TelemetryType.ALTITUDE:
            if (typeof telemetry.value !== 'number' || telemetry.value < 0 || telemetry.value > 120) {
                errors.push({
                    property: 'value',
                    constraints: {
                        range: 'Altitude must be between 0 and 120 meters'
                    }
                });
            }
            break;
    }

    return errors;
}