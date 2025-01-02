/**
 * @fileoverview Device validator implementation for agricultural robot fleet management
 * Implements comprehensive validation rules ensuring data integrity, operational safety,
 * and security compliance with real-time validation capabilities
 * @version 1.0.0
 */

import { Injectable } from '@nestjs/common';
import { IsString, IsEnum, IsNumber, IsDate, IsArray, validateSync } from 'class-validator';
import { createHash, randomBytes } from 'crypto';
import { IDevice } from '../interfaces/IDevice';
import { validateDevice } from '../utils/validation';
import { RobotType, RobotStatus, RobotCapability, DroneType, GroundRobotType } from '../constants/robotTypes';
import { ErrorCodes } from '../constants/errorCodes';

/**
 * Result interface for validation operations
 */
interface ValidationResult {
    isValid: boolean;
    errors: string[];
    errorCode?: ErrorCodes;
}

/**
 * Result interface for security validation operations
 */
interface SecurityValidationResult extends ValidationResult {
    securityLevel: 'HIGH' | 'MEDIUM' | 'LOW';
    tokenExpiry?: Date;
}

/**
 * Comprehensive device validator ensuring data integrity and security compliance
 */
@Injectable()
export class DeviceValidator {
    // Validation constants
    private readonly DEVICE_ID_REGEX = /^AGM-D-[A-Z0-9]{8}$/;
    private readonly MIN_BATTERY_LEVEL = 10;
    private readonly MAX_BATTERY_LEVEL = 100;
    private readonly MAX_ALTITUDE_METERS = 120;
    private readonly TELEMETRY_TIMEOUT_MS = 5000;
    private readonly SECURITY_TOKEN_PATTERN = /^AGM-ST-[A-Z0-9]{32}$/;

    constructor(
        private readonly configService: any,
        private readonly cacheService: any
    ) {}

    /**
     * Validates device ID format and uniqueness
     * @param deviceId Device identifier to validate
     */
    public async validateDeviceId(deviceId: string): Promise<ValidationResult> {
        const result: ValidationResult = {
            isValid: true,
            errors: []
        };

        if (!this.DEVICE_ID_REGEX.test(deviceId)) {
            result.isValid = false;
            result.errors.push('Device ID must match format AGM-D-XXXXXXXX');
            result.errorCode = ErrorCodes.VALIDATION_ERROR;
        }

        // Check device ID uniqueness in cache
        const existingDevice = await this.cacheService.get(`device:${deviceId}`);
        if (existingDevice) {
            result.isValid = false;
            result.errors.push('Device ID already exists');
            result.errorCode = ErrorCodes.VALIDATION_ERROR;
        }

        return result;
    }

    /**
     * Validates device type and capabilities
     * @param device Device data to validate
     */
    public async validateDeviceType(device: IDevice): Promise<ValidationResult> {
        const result: ValidationResult = {
            isValid: true,
            errors: []
        };

        // Validate primary type
        if (!Object.values(RobotType).includes(device.type)) {
            result.isValid = false;
            result.errors.push('Invalid robot type');
        }

        // Validate subtype based on primary type
        if (device.type === RobotType.AERIAL_DRONE) {
            if (!Object.values(DroneType).includes(device.subType as DroneType)) {
                result.isValid = false;
                result.errors.push('Invalid drone type');
            }
        } else if (device.type === RobotType.GROUND_ROBOT) {
            if (!Object.values(GroundRobotType).includes(device.subType as GroundRobotType)) {
                result.isValid = false;
                result.errors.push('Invalid ground robot type');
            }
        }

        if (!result.isValid) {
            result.errorCode = ErrorCodes.VALIDATION_ERROR;
        }

        return result;
    }

    /**
     * Validates device capabilities and operational parameters
     * @param device Device data to validate
     */
    public async validateDeviceCapabilities(device: IDevice): Promise<ValidationResult> {
        const result: ValidationResult = {
            isValid: true,
            errors: []
        };

        // Validate capabilities array
        if (!Array.isArray(device.capabilities) || device.capabilities.length === 0) {
            result.isValid = false;
            result.errors.push('Device must have at least one capability');
        } else {
            for (const capability of device.capabilities) {
                if (!Object.values(RobotCapability).includes(capability)) {
                    result.isValid = false;
                    result.errors.push(`Invalid capability: ${capability}`);
                }
            }
        }

        // Validate battery level
        if (device.batteryLevel < this.MIN_BATTERY_LEVEL || device.batteryLevel > this.MAX_BATTERY_LEVEL) {
            result.isValid = false;
            result.errors.push(`Battery level must be between ${this.MIN_BATTERY_LEVEL} and ${this.MAX_BATTERY_LEVEL}`);
        }

        if (!result.isValid) {
            result.errorCode = ErrorCodes.VALIDATION_ERROR;
        }

        return result;
    }

    /**
     * Validates device location data
     * @param device Device data to validate
     */
    public async validateDeviceLocation(device: IDevice): Promise<ValidationResult> {
        const result: ValidationResult = {
            isValid: true,
            errors: []
        };

        if (device.location) {
            const { latitude, longitude, altitude, heading } = device.location;

            // Validate latitude
            if (latitude < -90 || latitude > 90) {
                result.isValid = false;
                result.errors.push('Invalid latitude value');
            }

            // Validate longitude
            if (longitude < -180 || longitude > 180) {
                result.isValid = false;
                result.errors.push('Invalid longitude value');
            }

            // Validate altitude
            if (altitude < 0 || altitude > this.MAX_ALTITUDE_METERS) {
                result.isValid = false;
                result.errors.push(`Altitude must be between 0 and ${this.MAX_ALTITUDE_METERS} meters`);
            }

            // Validate heading
            if (heading < 0 || heading > 359) {
                result.isValid = false;
                result.errors.push('Heading must be between 0 and 359 degrees');
            }
        } else {
            result.isValid = false;
            result.errors.push('Device location is required');
        }

        if (!result.isValid) {
            result.errorCode = ErrorCodes.VALIDATION_ERROR;
        }

        return result;
    }

    /**
     * Validates device telemetry data freshness and integrity
     * @param device Device data to validate
     */
    public async validateDeviceTelemetry(device: IDevice): Promise<ValidationResult> {
        const result: ValidationResult = {
            isValid: true,
            errors: []
        };

        if (device.lastTelemetry) {
            const telemetryAge = Date.now() - device.lastTelemetry.getTime();
            
            if (telemetryAge > this.TELEMETRY_TIMEOUT_MS) {
                result.isValid = false;
                result.errors.push('Device telemetry data is stale');
                result.errorCode = ErrorCodes.TELEMETRY_COLLECTION_ERROR;
            }
        } else {
            result.isValid = false;
            result.errors.push('Device telemetry data is required');
            result.errorCode = ErrorCodes.VALIDATION_ERROR;
        }

        return result;
    }

    /**
     * Validates device security token and authentication
     * @param token Security token to validate
     * @param deviceId Associated device ID
     */
    public async validateSecurityToken(token: string, deviceId: string): Promise<SecurityValidationResult> {
        const result: SecurityValidationResult = {
            isValid: true,
            errors: [],
            securityLevel: 'HIGH'
        };

        // Validate token format
        if (!this.SECURITY_TOKEN_PATTERN.test(token)) {
            result.isValid = false;
            result.errors.push('Invalid security token format');
            result.securityLevel = 'LOW';
            result.errorCode = ErrorCodes.AUTHENTICATION_ERROR;
            return result;
        }

        // Validate token signature
        const tokenHash = createHash('sha256')
            .update(`${deviceId}:${token}`)
            .digest('hex');

        const storedHash = await this.cacheService.get(`token:${deviceId}`);
        if (!storedHash || tokenHash !== storedHash) {
            result.isValid = false;
            result.errors.push('Invalid security token');
            result.securityLevel = 'LOW';
            result.errorCode = ErrorCodes.AUTHENTICATION_ERROR;
        }

        return result;
    }
}