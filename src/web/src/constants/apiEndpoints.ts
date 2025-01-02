/**
 * @fileoverview Centralized API endpoint configuration for the Agricultural Management System
 * Defines all REST endpoints for frontend-backend communication with comprehensive type safety
 * @version 1.0.0
 */

/**
 * Base configuration for API endpoints
 */
export const BASE_URL = '/api' as const;
export const API_VERSION = 'v1' as const;

/**
 * Mission management endpoints
 */
export const MISSIONS = {
    BASE: '/missions',
    GET_ALL: '/missions',
    GET_BY_ID: '/missions/:id',
    CREATE: '/missions',
    UPDATE: '/missions/:id',
    DELETE: '/missions/:id',
    STATUS: '/missions/:id/status',
    BATCH_CREATE: '/missions/batch',
    SCHEDULE: '/missions/:id/schedule',
    ABORT: '/missions/:id/abort',
    PAUSE: '/missions/:id/pause',
    RESUME: '/missions/:id/resume',
    HISTORY: '/missions/:id/history',
    METRICS: '/missions/:id/metrics'
} as const;

/**
 * Device management endpoints
 */
export const DEVICES = {
    BASE: '/devices',
    GET_ALL: '/devices',
    GET_BY_ID: '/devices/:id',
    UPDATE: '/devices/:id',
    STATUS: '/devices/:id/status',
    TELEMETRY: '/devices/:id/telemetry',
    CALIBRATE: '/devices/:id/calibrate',
    DIAGNOSTICS: '/devices/:id/diagnostics',
    MAINTENANCE: '/devices/:id/maintenance',
    FIRMWARE: '/devices/:id/firmware',
    BATCH_STATUS: '/devices/batch/status',
    FLEET_STATUS: '/devices/fleet/status'
} as const;

/**
 * Analytics and reporting endpoints
 */
export const ANALYTICS = {
    BASE: '/analytics',
    PERFORMANCE: '/analytics/performance',
    PEST_DETECTION: '/analytics/pest-detection',
    TREATMENT_COVERAGE: '/analytics/treatment-coverage',
    SYSTEM_METRICS: '/analytics/system-metrics',
    HISTORICAL_TRENDS: '/analytics/historical-trends',
    EFFICIENCY_METRICS: '/analytics/efficiency-metrics',
    RESOURCE_USAGE: '/analytics/resource-usage',
    ANOMALY_DETECTION: '/analytics/anomaly-detection',
    PREDICTIVE_MAINTENANCE: '/analytics/predictive-maintenance'
} as const;

/**
 * Telemetry data endpoints
 */
export const TELEMETRY = {
    BASE: '/telemetry',
    STREAM: '/telemetry/stream',
    HISTORICAL: '/telemetry/historical',
    METRICS: '/telemetry/metrics',
    REAL_TIME: '/telemetry/real-time',
    AGGREGATED: '/telemetry/aggregated',
    DEVICE_STREAM: '/telemetry/:deviceId/stream',
    MISSION_STREAM: '/telemetry/:missionId/stream',
    BATCH_METRICS: '/telemetry/batch/metrics',
    EXPORT: '/telemetry/export'
} as const;

/**
 * Treatment operation endpoints
 */
export const TREATMENTS = {
    BASE: '/treatments',
    GET_ALL: '/treatments',
    GET_BY_ID: '/treatments/:id',
    CREATE: '/treatments',
    UPDATE: '/treatments/:id',
    STATUS: '/treatments/:id/status',
    SCHEDULE: '/treatments/:id/schedule',
    VERIFY: '/treatments/:id/verify',
    COVERAGE: '/treatments/:id/coverage',
    EFFICIENCY: '/treatments/:id/efficiency',
    BATCH_CREATE: '/treatments/batch',
    ZONE_SUMMARY: '/treatments/zone/:zoneId/summary'
} as const;

/**
 * Complete API endpoint configuration with type safety and immutability
 */
export const API_ENDPOINTS = {
    BASE_URL,
    API_VERSION,
    MISSIONS,
    DEVICES,
    ANALYTICS,
    TELEMETRY,
    TREATMENTS
} as const;

// Type definitions for endpoint parameters
type MissionId = string;
type DeviceId = string;
type ZoneId = string;

/**
 * Utility type for replacing URL parameters
 * Usage: ReplaceParams<typeof MISSIONS.GET_BY_ID, { id: string }>
 */
type ReplaceParams<T extends string, P extends Record<string, string>> = T extends `${infer Start}:${infer Param}/${infer Rest}`
    ? P extends { [K in Param]: string }
        ? `${Start}${P[Param]}/${ReplaceParams<Rest, P>}`
        : never
    : T extends `${infer Start}:${infer Param}`
        ? P extends { [K in Param]: string }
            ? `${Start}${P[Param]}`
            : never
        : T;

export type {
    MissionId,
    DeviceId,
    ZoneId,
    ReplaceParams
};