/**
 * @fileoverview Advanced validation module for agricultural analytics data
 * Implements comprehensive validation rules for real-time video analytics and pest detection
 * @version 1.0.0
 */

import { validate, ValidationError } from 'class-validator'; // v0.14.0
import { sanitize } from 'class-sanitizer'; // v1.0.1
import { IAnalytics, AnalyticsType, MIN_CONFIDENCE_THRESHOLD, IDetection } from '../interfaces/IAnalytics';
import { validateTelemetry } from '../utils/validation';
import { ErrorCodes } from '../constants/errorCodes';

// Analytics validation constants
const ANALYTICS_ID_REGEX = /^AGM-A-\d{6}$/;
const MAX_DETECTIONS_PER_FRAME = 100;
const MAX_LATENCY_MS = 100;
const MIN_BOUNDING_BOX_SIZE = 10;
const MAX_BOUNDING_BOX_SIZE = 1000;
const VALID_COORDINATE_RANGE = {
    minLat: -90,
    maxLat: 90,
    minLon: -180,
    maxLon: 180
};

/**
 * Validates analytics data against schema, performance requirements, and security rules
 * @param analytics Analytics data to validate
 * @param options Optional validation configuration
 * @returns Array of validation errors if any
 */
export async function validateAnalytics(
    analytics: IAnalytics,
    options: { checkPerformance?: boolean; validateSecurity?: boolean } = {}
): Promise<ValidationError[]> {
    // Sanitize input data for security
    sanitize(analytics);
    const errors: ValidationError[] = [];

    // Validate analytics ID format
    if (!ANALYTICS_ID_REGEX.test(analytics.id)) {
        errors.push({
            property: 'id',
            constraints: {
                format: 'Analytics ID must match format AGM-A-XXXXXX'
            }
        });
    }

    // Validate analytics type
    if (!Object.values(AnalyticsType).includes(analytics.type)) {
        errors.push({
            property: 'type',
            constraints: {
                enum: 'Invalid analytics type'
            }
        });
    }

    // Validate confidence threshold (95% requirement)
    if (analytics.confidence < MIN_CONFIDENCE_THRESHOLD) {
        errors.push({
            property: 'confidence',
            constraints: {
                min: `Confidence must be at least ${MIN_CONFIDENCE_THRESHOLD * 100}%`
            }
        });
    }

    // Validate location coordinates
    if (analytics.location) {
        const [longitude, latitude] = analytics.location.coordinates;
        if (latitude < VALID_COORDINATE_RANGE.minLat || latitude > VALID_COORDINATE_RANGE.maxLat ||
            longitude < VALID_COORDINATE_RANGE.minLon || longitude > VALID_COORDINATE_RANGE.maxLon) {
            errors.push({
                property: 'location',
                constraints: {
                    range: 'Location coordinates out of valid range'
                }
            });
        }
    }

    // Validate detections array
    if (analytics.detections) {
        if (analytics.detections.length > MAX_DETECTIONS_PER_FRAME) {
            errors.push({
                property: 'detections',
                constraints: {
                    maxSize: `Maximum of ${MAX_DETECTIONS_PER_FRAME} detections per frame allowed`
                }
            });
        }

        // Validate individual detections
        const detectionErrors = await validateDetection(analytics.detections);
        errors.push(...detectionErrors);
    }

    // Performance validation if enabled
    if (options.checkPerformance) {
        const performanceErrors = await validatePerformance(analytics);
        errors.push(...performanceErrors);
    }

    return errors;
}

/**
 * Validates individual detection results for pest identification and analysis
 * @param detections Array of detection results to validate
 * @returns Array of validation errors if any
 */
export async function validateDetection(detections: IDetection[]): Promise<ValidationError[]> {
    const errors: ValidationError[] = [];

    for (const detection of detections) {
        // Validate detection confidence
        if (detection.confidence < MIN_CONFIDENCE_THRESHOLD) {
            errors.push({
                property: 'detection.confidence',
                constraints: {
                    min: `Detection confidence must be at least ${MIN_CONFIDENCE_THRESHOLD * 100}%`
                }
            });
        }

        // Validate bounding box dimensions
        const { x, y, width, height } = detection.boundingBox;
        if (width < MIN_BOUNDING_BOX_SIZE || width > MAX_BOUNDING_BOX_SIZE ||
            height < MIN_BOUNDING_BOX_SIZE || height > MAX_BOUNDING_BOX_SIZE) {
            errors.push({
                property: 'detection.boundingBox',
                constraints: {
                    size: `Bounding box dimensions must be between ${MIN_BOUNDING_BOX_SIZE} and ${MAX_BOUNDING_BOX_SIZE} pixels`
                }
            });
        }

        // Validate bounding box coordinates
        if (x < 0 || y < 0) {
            errors.push({
                property: 'detection.boundingBox',
                constraints: {
                    coordinates: 'Bounding box coordinates must be non-negative'
                }
            });
        }
    }

    return errors;
}

/**
 * Validates real-time performance metrics for video analytics
 * @param analytics Analytics data to validate performance
 * @returns Array of validation errors if any
 */
export async function validatePerformance(analytics: IAnalytics): Promise<ValidationError[]> {
    const errors: ValidationError[] = [];

    // Calculate processing latency
    const processingTime = new Date().getTime() - analytics.timestamp.getTime();
    
    // Validate against 100ms latency requirement
    if (processingTime > MAX_LATENCY_MS) {
        errors.push({
            property: 'performance',
            constraints: {
                latency: `Processing latency (${processingTime}ms) exceeds maximum allowed (${MAX_LATENCY_MS}ms)`
            },
            code: ErrorCodes.PERFORMANCE_ERROR
        });
    }

    // Validate frame processing rate if metadata includes it
    if (analytics.metadata?.frameRate) {
        const minFrameRate = 30; // Minimum required frame rate
        if (analytics.metadata.frameRate < minFrameRate) {
            errors.push({
                property: 'performance.frameRate',
                constraints: {
                    min: `Frame rate (${analytics.metadata.frameRate} fps) below minimum required (${minFrameRate} fps)`
                },
                code: ErrorCodes.PERFORMANCE_ERROR
            });
        }
    }

    return errors;
}