/**
 * @fileoverview Defines the core interface for telemetry data points collected from agricultural robots and drones
 * @version 1.0.0
 * @license MIT
 */

import { TelemetryType, TelemetryUnit } from '../constants/telemetryTypes';

/**
 * Interface representing a single telemetry data point from an agricultural robot or drone.
 * Provides a standardized structure for real-time monitoring and analytics data collection.
 * 
 * @interface ITelemetry
 * @property {string} id - Unique identifier for the telemetry data point
 * @property {string} deviceId - Identifier of the device generating the telemetry
 * @property {Date} timestamp - Precise timestamp when the telemetry was recorded
 * @property {TelemetryType} type - Classification of the telemetry data type
 * @property {number | string | boolean | object} value - Actual telemetry measurement or status
 * @property {TelemetryUnit} unit - Unit of measurement for the telemetry value
 * @property {Record<string, any>} metadata - Additional contextual information about the telemetry
 */
export interface ITelemetry {
    /**
     * Unique identifier for the telemetry data point
     * Format: UUID v4
     */
    id: string;

    /**
     * Identifier of the device (robot/drone) generating the telemetry
     * Format: UUID v4
     */
    deviceId: string;

    /**
     * Precise timestamp when the telemetry was recorded
     * Resolution: Millisecond precision
     */
    timestamp: Date;

    /**
     * Classification of the telemetry data type
     * Supports comprehensive device monitoring including:
     * - Location tracking
     * - Battery monitoring
     * - Speed measurement
     * - Altitude tracking
     * - Directional heading
     * - Sensor status
     * - Camera operations
     * - Motor conditions
     * - Treatment substance levels
     * - Error reporting
     */
    type: TelemetryType;

    /**
     * Actual telemetry measurement or status value
     * Supports multiple data types for flexibility:
     * - number: For quantitative measurements
     * - string: For status messages and identifiers
     * - boolean: For binary state indicators
     * - object: For complex structured data
     */
    value: number | string | boolean | object;

    /**
     * Unit of measurement for the telemetry value
     * Standardized units including:
     * - METERS: For distance measurements
     * - PERCENT: For percentage values
     * - METERS_PER_SECOND: For velocity
     * - DEGREES: For angular measurements
     * - LITERS: For volume measurements
     */
    unit: TelemetryUnit;

    /**
     * Additional contextual information about the telemetry
     * Flexible key-value store for extended metadata:
     * - Environmental conditions
     * - Device configuration
     * - Operation context
     * - Quality indicators
     */
    metadata: Record<string, any>;
}