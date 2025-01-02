/**
 * @fileoverview Implements comprehensive validation rules for telemetry data from agricultural robots and drones
 * @version 1.0.0
 * @license MIT
 */

import {
  IsString,
  IsUUID,
  IsDate,
  IsEnum,
  IsNumber,
  IsObject,
  ValidateNested,
  IsOptional,
  Min,
  Max,
  IsLatLong,
  IsDateString,
  ArrayMinSize,
  ArrayMaxSize,
  ValidateIf,
  ValidationError
} from 'class-validator'; // version: 0.14.0
import { Type, Transform } from 'class-transformer'; // version: 0.5.1
import { ITelemetry } from '../interfaces/ITelemetry';
import { TelemetryType, TelemetryUnit } from '../constants/telemetryTypes';

// Validation constants
const MAX_BATCH_SIZE = 1000;
const MAX_ALTITUDE_METERS = 500;
const MIN_BATTERY_PERCENT = 0;
const MAX_BATTERY_PERCENT = 100;
const MAX_SPEED_MPS = 30;
const MAX_FUTURE_TIME_MS = 5000;
const MIN_TIMESTAMP_INTERVAL_MS = 100;

/**
 * Class implementing comprehensive validation rules for telemetry data points
 * with support for batch processing and custom validation rules
 */
export class TelemetryValidator {
  private errors: ValidationError[] = [];
  private customValidators: Map<string, Function> = new Map();
  private readonly config: ValidationConfig;

  constructor(config: ValidationConfig = {}) {
    this.config = {
      maxBatchSize: config.maxBatchSize || MAX_BATCH_SIZE,
      maxAltitude: config.maxAltitude || MAX_ALTITUDE_METERS,
      maxSpeed: config.maxSpeed || MAX_SPEED_MPS,
      maxFutureTime: config.maxFutureTime || MAX_FUTURE_TIME_MS,
      minTimestampInterval: config.minTimestampInterval || MIN_TIMESTAMP_INTERVAL_MS
    };
    this.initializeCustomValidators();
  }

  /**
   * Validates a single telemetry data point with comprehensive checks
   * @param telemetry The telemetry data point to validate
   * @returns Promise<ValidationError[]> Array of validation errors if any
   */
  public async validateTelemetryData(telemetry: ITelemetry): Promise<ValidationError[]> {
    this.errors = [];

    // Basic field validation
    if (!this.validateBasicFields(telemetry)) {
      return this.errors;
    }

    // Type-specific validation
    await this.validateByType(telemetry);

    // Metadata validation if present
    if (telemetry.metadata) {
      await this.validateMetadata(telemetry.metadata);
    }

    // Apply custom validators
    await this.applyCustomValidators(telemetry);

    return this.errors;
  }

  /**
   * Validates an array of telemetry data points with batch-specific checks
   * @param telemetryBatch Array of telemetry data points to validate
   * @returns Promise<ValidationError[]> Array of validation errors if any
   */
  public async validateTelemetryBatch(telemetryBatch: ITelemetry[]): Promise<ValidationError[]> {
    this.errors = [];

    // Validate batch size
    if (telemetryBatch.length > this.config.maxBatchSize) {
      this.errors.push({
        property: 'batchSize',
        value: telemetryBatch.length,
        constraints: {
          max: `Batch size exceeds maximum of ${this.config.maxBatchSize}`
        }
      });
      return this.errors;
    }

    // Track timestamps per device for sequence validation
    const deviceTimestamps: Map<string, Date[]> = new Map();

    // Validate each telemetry point and collect timestamps
    for (const telemetry of telemetryBatch) {
      const pointErrors = await this.validateTelemetryData(telemetry);
      this.errors.push(...pointErrors);

      if (!deviceTimestamps.has(telemetry.deviceId)) {
        deviceTimestamps.set(telemetry.deviceId, []);
      }
      deviceTimestamps.get(telemetry.deviceId)?.push(telemetry.timestamp);
    }

    // Validate timestamp sequences per device
    this.validateTimestampSequences(deviceTimestamps);

    return this.errors;
  }

  /**
   * Validates metadata object structure and content
   * @param metadata Metadata object to validate
   * @returns Promise<void>
   */
  private async validateMetadata(metadata: Record<string, any>): Promise<void> {
    if (!IsObject()(metadata)) {
      this.errors.push({
        property: 'metadata',
        value: metadata,
        constraints: {
          isObject: 'Metadata must be a valid object'
        }
      });
      return;
    }

    // Validate known metadata fields
    for (const [key, value] of Object.entries(metadata)) {
      switch (key) {
        case 'environmentalConditions':
          this.validateEnvironmentalConditions(value);
          break;
        case 'deviceConfig':
          this.validateDeviceConfig(value);
          break;
        case 'qualityIndicators':
          this.validateQualityIndicators(value);
          break;
      }
    }
  }

  /**
   * Validates basic telemetry fields
   * @param telemetry Telemetry data point to validate
   * @returns boolean Validation result
   */
  private validateBasicFields(telemetry: ITelemetry): boolean {
    // Validate IDs
    if (!IsUUID(4)(telemetry.id) || !IsUUID(4)(telemetry.deviceId)) {
      this.errors.push({
        property: 'id/deviceId',
        value: telemetry.id,
        constraints: {
          isUuid: 'ID fields must be valid UUID v4'
        }
      });
      return false;
    }

    // Validate timestamp
    const now = new Date();
    const timestamp = new Date(telemetry.timestamp);
    if (timestamp > new Date(now.getTime() + this.config.maxFutureTime)) {
      this.errors.push({
        property: 'timestamp',
        value: timestamp,
        constraints: {
          isFuture: 'Timestamp cannot be too far in the future'
        }
      });
      return false;
    }

    // Validate type and unit
    if (!IsEnum(TelemetryType)(telemetry.type) || !IsEnum(TelemetryUnit)(telemetry.unit)) {
      this.errors.push({
        property: 'type/unit',
        value: telemetry.type,
        constraints: {
          isEnum: 'Invalid telemetry type or unit'
        }
      });
      return false;
    }

    return true;
  }

  /**
   * Validates telemetry data based on its type
   * @param telemetry Telemetry data point to validate
   */
  private async validateByType(telemetry: ITelemetry): Promise<void> {
    switch (telemetry.type) {
      case TelemetryType.LOCATION:
        this.validateLocation(telemetry);
        break;
      case TelemetryType.BATTERY:
        this.validateBattery(telemetry);
        break;
      case TelemetryType.SPEED:
        this.validateSpeed(telemetry);
        break;
      case TelemetryType.ALTITUDE:
        this.validateAltitude(telemetry);
        break;
      case TelemetryType.HEADING:
        this.validateHeading(telemetry);
        break;
      default:
        this.validateGenericStatus(telemetry);
    }
  }

  /**
   * Validates location telemetry data
   * @param telemetry Location telemetry data
   */
  private validateLocation(telemetry: ITelemetry): void {
    if (!IsLatLong()(telemetry.value)) {
      this.errors.push({
        property: 'value',
        value: telemetry.value,
        constraints: {
          isLatLong: 'Invalid location format'
        }
      });
    }
  }

  /**
   * Validates battery telemetry data
   * @param telemetry Battery telemetry data
   */
  private validateBattery(telemetry: ITelemetry): void {
    if (!IsNumber()(telemetry.value) || 
        telemetry.value < MIN_BATTERY_PERCENT || 
        telemetry.value > MAX_BATTERY_PERCENT) {
      this.errors.push({
        property: 'value',
        value: telemetry.value,
        constraints: {
          range: `Battery percentage must be between ${MIN_BATTERY_PERCENT} and ${MAX_BATTERY_PERCENT}`
        }
      });
    }
  }

  /**
   * Validates speed telemetry data
   * @param telemetry Speed telemetry data
   */
  private validateSpeed(telemetry: ITelemetry): void {
    if (!IsNumber()(telemetry.value) || 
        telemetry.value < 0 || 
        telemetry.value > this.config.maxSpeed) {
      this.errors.push({
        property: 'value',
        value: telemetry.value,
        constraints: {
          range: `Speed must be between 0 and ${this.config.maxSpeed} m/s`
        }
      });
    }
  }

  /**
   * Validates altitude telemetry data
   * @param telemetry Altitude telemetry data
   */
  private validateAltitude(telemetry: ITelemetry): void {
    if (!IsNumber()(telemetry.value) || 
        telemetry.value < 0 || 
        telemetry.value > this.config.maxAltitude) {
      this.errors.push({
        property: 'value',
        value: telemetry.value,
        constraints: {
          range: `Altitude must be between 0 and ${this.config.maxAltitude} meters`
        }
      });
    }
  }

  /**
   * Validates heading telemetry data
   * @param telemetry Heading telemetry data
   */
  private validateHeading(telemetry: ITelemetry): void {
    if (!IsNumber()(telemetry.value) || 
        telemetry.value < 0 || 
        telemetry.value > 360) {
      this.errors.push({
        property: 'value',
        value: telemetry.value,
        constraints: {
          range: 'Heading must be between 0 and 360 degrees'
        }
      });
    }
  }

  /**
   * Validates generic status telemetry data
   * @param telemetry Status telemetry data
   */
  private validateGenericStatus(telemetry: ITelemetry): void {
    if (typeof telemetry.value !== 'string' && 
        typeof telemetry.value !== 'boolean' && 
        !IsObject()(telemetry.value)) {
      this.errors.push({
        property: 'value',
        value: telemetry.value,
        constraints: {
          type: 'Invalid status value type'
        }
      });
    }
  }

  /**
   * Validates timestamp sequences for each device
   * @param deviceTimestamps Map of device IDs to timestamp arrays
   */
  private validateTimestampSequences(deviceTimestamps: Map<string, Date[]>): void {
    for (const [deviceId, timestamps] of deviceTimestamps) {
      // Sort timestamps
      timestamps.sort((a, b) => a.getTime() - b.getTime());

      // Check for duplicates and minimum interval
      for (let i = 1; i < timestamps.length; i++) {
        const interval = timestamps[i].getTime() - timestamps[i-1].getTime();
        
        if (interval < this.config.minTimestampInterval) {
          this.errors.push({
            property: 'timestamp',
            value: timestamps[i],
            constraints: {
              interval: `Timestamp interval for device ${deviceId} is below minimum ${this.config.minTimestampInterval}ms`
            }
          });
        }
      }
    }
  }

  /**
   * Initializes custom validators for specific validation scenarios
   */
  private initializeCustomValidators(): void {
    this.customValidators.set('validateEnvironmentalConditions', this.validateEnvironmentalConditions);
    this.customValidators.set('validateDeviceConfig', this.validateDeviceConfig);
    this.customValidators.set('validateQualityIndicators', this.validateQualityIndicators);
  }

  /**
   * Applies all registered custom validators to telemetry data
   * @param telemetry Telemetry data to validate
   */
  private async applyCustomValidators(telemetry: ITelemetry): Promise<void> {
    for (const validator of this.customValidators.values()) {
      await validator.call(this, telemetry);
    }
  }
}

/**
 * Configuration interface for telemetry validation
 */
interface ValidationConfig {
  maxBatchSize?: number;
  maxAltitude?: number;
  maxSpeed?: number;
  maxFutureTime?: number;
  minTimestampInterval?: number;
}