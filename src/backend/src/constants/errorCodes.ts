/**
 * @fileoverview Centralized error code definitions and messages for the agricultural management system.
 * Provides standardized error handling across mission control, device management, and data processing.
 * Version: 1.0.0
 */

/**
 * Prefix for all system error codes to uniquely identify errors from this system
 */
export const ERROR_CODE_PREFIX = "AGM";

/**
 * Enumeration of all possible error codes in the system.
 * Organized by subsystem and severity with ranges:
 * - 1000-1999: Validation/Authentication/Authorization
 * - 2000-2999: Mission Management
 * - 3000-3999: Device Management
 * - 4000-4999: Telemetry
 * - 5000-5999: Treatment Management
 * - 9000-9999: System Level
 */
export enum ErrorCodes {
    // Authentication & Validation (1000-1999)
    VALIDATION_ERROR = 1000,
    AUTHENTICATION_ERROR = 1001,
    AUTHORIZATION_ERROR = 1002,
    RESOURCE_NOT_FOUND = 1004,

    // Mission Management (2000-2999)
    MISSION_PLANNING_ERROR = 2000,
    MISSION_EXECUTION_ERROR = 2001,

    // Device Management (3000-3999)
    DEVICE_CONNECTION_ERROR = 3000,
    DEVICE_CONTROL_ERROR = 3001,

    // Telemetry (4000-4999)
    TELEMETRY_COLLECTION_ERROR = 4000,
    TELEMETRY_PROCESSING_ERROR = 4001,

    // Treatment Management (5000-5999)
    TREATMENT_PLANNING_ERROR = 5000,
    TREATMENT_EXECUTION_ERROR = 5001,

    // System Level (9000-9999)
    INTERNAL_SERVER_ERROR = 9000
}

/**
 * Standard HTTP status codes used in API responses.
 * Mapped to correspond with system error codes for consistent error handling.
 */
export enum HttpStatusCodes {
    BAD_REQUEST = 400,
    UNAUTHORIZED = 401,
    FORBIDDEN = 403,
    NOT_FOUND = 404,
    INTERNAL_SERVER_ERROR = 500
}

/**
 * Detailed, user-friendly error messages corresponding to error codes.
 * Includes problem description and potential resolution steps.
 */
export const ErrorMessages = {
    [ErrorCodes.VALIDATION_ERROR]: "Invalid input parameters. Please verify the request data and try again.",
    [ErrorCodes.AUTHENTICATION_ERROR]: "Authentication failed. Please check your credentials and try again.",
    [ErrorCodes.AUTHORIZATION_ERROR]: "Insufficient permissions to perform this operation.",
    [ErrorCodes.RESOURCE_NOT_FOUND]: "The requested resource could not be found.",
    
    [ErrorCodes.MISSION_PLANNING_ERROR]: "Failed to create mission plan. Please check mission parameters and try again.",
    [ErrorCodes.MISSION_EXECUTION_ERROR]: "Error during mission execution. Mission has been safely aborted.",
    
    [ErrorCodes.DEVICE_CONNECTION_ERROR]: "Unable to establish connection with device. Please check device status and network connectivity.",
    [ErrorCodes.DEVICE_CONTROL_ERROR]: "Failed to send control commands to device. Device may be in an invalid state.",
    
    [ErrorCodes.TELEMETRY_COLLECTION_ERROR]: "Failed to collect telemetry data from device. Check device sensors and connectivity.",
    [ErrorCodes.TELEMETRY_PROCESSING_ERROR]: "Error processing telemetry data. Data may be corrupted or in unexpected format.",
    
    [ErrorCodes.TREATMENT_PLANNING_ERROR]: "Failed to generate treatment plan. Please verify treatment parameters and field conditions.",
    [ErrorCodes.TREATMENT_EXECUTION_ERROR]: "Error during treatment execution. Treatment has been safely aborted.",
    
    [ErrorCodes.INTERNAL_SERVER_ERROR]: "An unexpected internal error occurred. Please contact system administrator."
} as const;

/**
 * Maps system error codes to corresponding HTTP status codes for API responses
 */
export const ErrorCodeToHttpStatus = {
    [ErrorCodes.VALIDATION_ERROR]: HttpStatusCodes.BAD_REQUEST,
    [ErrorCodes.AUTHENTICATION_ERROR]: HttpStatusCodes.UNAUTHORIZED,
    [ErrorCodes.AUTHORIZATION_ERROR]: HttpStatusCodes.FORBIDDEN,
    [ErrorCodes.RESOURCE_NOT_FOUND]: HttpStatusCodes.NOT_FOUND,
    [ErrorCodes.MISSION_PLANNING_ERROR]: HttpStatusCodes.BAD_REQUEST,
    [ErrorCodes.MISSION_EXECUTION_ERROR]: HttpStatusCodes.INTERNAL_SERVER_ERROR,
    [ErrorCodes.DEVICE_CONNECTION_ERROR]: HttpStatusCodes.INTERNAL_SERVER_ERROR,
    [ErrorCodes.DEVICE_CONTROL_ERROR]: HttpStatusCodes.INTERNAL_SERVER_ERROR,
    [ErrorCodes.TELEMETRY_COLLECTION_ERROR]: HttpStatusCodes.INTERNAL_SERVER_ERROR,
    [ErrorCodes.TELEMETRY_PROCESSING_ERROR]: HttpStatusCodes.INTERNAL_SERVER_ERROR,
    [ErrorCodes.TREATMENT_PLANNING_ERROR]: HttpStatusCodes.BAD_REQUEST,
    [ErrorCodes.TREATMENT_EXECUTION_ERROR]: HttpStatusCodes.INTERNAL_SERVER_ERROR,
    [ErrorCodes.INTERNAL_SERVER_ERROR]: HttpStatusCodes.INTERNAL_SERVER_ERROR
} as const;