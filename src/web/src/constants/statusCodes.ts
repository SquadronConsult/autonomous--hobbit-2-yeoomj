/**
 * @fileoverview Status code definitions for HTTP responses, mission states, and device states
 * @version 1.0.0
 * @license MIT
 */

/**
 * Prefix for HTTP status codes in logging and tracking
 */
export const HTTP_STATUS_PREFIX = "HTTP";

/**
 * Prefix for mission status codes in logging and tracking
 */
export const MISSION_STATUS_PREFIX = "MSN";

/**
 * Prefix for device status codes in logging and tracking
 */
export const DEVICE_STATUS_PREFIX = "DEV";

/**
 * Starting range for HTTP status codes
 */
export const HTTP_STATUS_RANGE_START = 100;

/**
 * Starting range for mission status codes
 */
export const MISSION_STATUS_RANGE_START = 1000;

/**
 * Starting range for device status codes
 */
export const DEVICE_STATUS_RANGE_START = 2000;

/**
 * Standard HTTP status codes following RFC specifications
 * Used for consistent API response handling across the application
 * @enum {number}
 */
export enum HttpStatusCodes {
    OK = 200,
    CREATED = 201,
    ACCEPTED = 202,
    NO_CONTENT = 204,
    BAD_REQUEST = 400,
    UNAUTHORIZED = 401,
    FORBIDDEN = 403,
    NOT_FOUND = 404,
    CONFLICT = 409,
    INTERNAL_SERVER_ERROR = 500,
    SERVICE_UNAVAILABLE = 503
}

/**
 * Mission status codes for tracking mission lifecycle states
 * Range: 1000-1999
 * @enum {number}
 */
export enum MissionStatusCodes {
    PENDING = 1000,
    IN_PROGRESS = 1001,
    COMPLETED = 1002,
    FAILED = 1003,
    CANCELLED = 1004,
    PAUSED = 1005,
    RESUMED = 1006,
    QUEUED = 1007
}

/**
 * Device status codes for monitoring device states
 * Range: 2000-2999
 * @enum {number}
 */
export enum DeviceStatusCodes {
    ONLINE = 2000,
    OFFLINE = 2001,
    BUSY = 2002,
    ERROR = 2003,
    MAINTENANCE = 2004,
    LOW_BATTERY = 2005,
    CHARGING = 2006,
    CALIBRATING = 2007
}