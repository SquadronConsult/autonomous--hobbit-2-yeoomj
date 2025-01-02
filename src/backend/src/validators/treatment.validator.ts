/**
 * @fileoverview Treatment validation module for agricultural operations
 * Implements comprehensive validation rules for treatment data integrity and compliance
 * @version 1.0.0
 */

import { validate, ValidationError } from 'class-validator'; // v0.14.0
import { sanitize } from 'class-sanitizer'; // v1.0.1
import { ITreatment, ITreatmentLocation, ITreatmentCoverage } from '../interfaces/ITreatment';
import { validateMission } from '../utils/validation';
import { ErrorCodes } from '../constants/errorCodes';
import { MissionStatus } from '../constants/missionStatus';
import { RobotType, RobotCapability } from '../constants/robotTypes';

// Validation constants
const TREATMENT_ID_REGEX = /^AGM-T-\d{6}$/;
const MIN_TREATMENT_QUANTITY = 0.1;
const MAX_TREATMENT_QUANTITY = 100.0;
const MIN_COVERAGE_AREA = 1.0;
const MAX_COVERAGE_AREA = 1000.0;
const COORDINATE_PRECISION = 6;
const MAX_ALTITUDE = 100.0;
const TREATMENT_CACHE_TTL = 300;

// Valid coordinate ranges
const VALID_COORDINATES = {
    minLat: -90,
    maxLat: 90,
    minLon: -180,
    maxLon: 180,
    minAlt: 0,
    maxAlt: MAX_ALTITUDE
};

/**
 * Validates treatment data against schema and business rules with security checks
 * @param treatment Treatment data to validate
 * @returns Array of validation errors if any
 */
export async function validateTreatment(treatment: ITreatment): Promise<ValidationError[]> {
    // Sanitize input data for security
    sanitize(treatment);
    const errors: ValidationError[] = [];

    // Validate treatment ID format
    if (!TREATMENT_ID_REGEX.test(treatment.id)) {
        errors.push({
            property: 'id',
            constraints: {
                format: 'Treatment ID must match format AGM-T-XXXXXX'
            }
        });
    }

    // Validate mission reference and status
    const missionErrors = await validateMission({ id: treatment.missionId } as any);
    if (missionErrors.length > 0) {
        errors.push({
            property: 'missionId',
            constraints: {
                exists: 'Invalid mission reference'
            }
        });
    }

    // Validate device capabilities
    if (!treatment.deviceId.match(/^AGM-D-\d{6}$/)) {
        errors.push({
            property: 'deviceId',
            constraints: {
                format: 'Invalid device ID format'
            }
        });
    }

    // Validate treatment quantity
    if (treatment.quantity < MIN_TREATMENT_QUANTITY || treatment.quantity > MAX_TREATMENT_QUANTITY) {
        errors.push({
            property: 'quantity',
            constraints: {
                range: `Treatment quantity must be between ${MIN_TREATMENT_QUANTITY} and ${MAX_TREATMENT_QUANTITY}`
            }
        });
    }

    // Validate treatment location
    const locationErrors = await validateTreatmentLocation(treatment.location);
    errors.push(...locationErrors);

    // Validate coverage area
    const coverageErrors = await validateTreatmentCoverage(treatment.coverage);
    errors.push(...coverageErrors);

    // Validate treatment parameters
    if (treatment.parameters) {
        if (treatment.parameters.concentration && 
            (treatment.parameters.concentration < 0 || treatment.parameters.concentration > 1000)) {
            errors.push({
                property: 'parameters.concentration',
                constraints: {
                    range: 'Concentration must be between 0 and 1000 ppm'
                }
            });
        }

        if (treatment.parameters.applicationRate && 
            (treatment.parameters.applicationRate < 0 || treatment.parameters.applicationRate > 50)) {
            errors.push({
                property: 'parameters.applicationRate',
                constraints: {
                    range: 'Application rate must be between 0 and 50 units per hectare'
                }
            });
        }
    }

    // Validate weather conditions
    if (treatment.weatherConditions) {
        const { windSpeed, temperature, humidity } = treatment.weatherConditions;
        
        if (windSpeed > 15) {
            errors.push({
                property: 'weatherConditions.windSpeed',
                constraints: {
                    max: 'Wind speed too high for safe treatment application'
                }
            });
        }

        if (humidity < 30 || humidity > 90) {
            errors.push({
                property: 'weatherConditions.humidity',
                constraints: {
                    range: 'Humidity must be between 30% and 90% for effective treatment'
                }
            });
        }
    }

    return errors;
}

/**
 * Validates treatment location coordinates with precision requirements
 * @param location Treatment location data
 * @returns Array of coordinate validation errors
 */
export async function validateTreatmentLocation(location: ITreatmentLocation): Promise<ValidationError[]> {
    const errors: ValidationError[] = [];

    // Validate coordinate ranges
    if (location.latitude < VALID_COORDINATES.minLat || location.latitude > VALID_COORDINATES.maxLat) {
        errors.push({
            property: 'location.latitude',
            constraints: {
                range: `Latitude must be between ${VALID_COORDINATES.minLat} and ${VALID_COORDINATES.maxLat}`
            }
        });
    }

    if (location.longitude < VALID_COORDINATES.minLon || location.longitude > VALID_COORDINATES.maxLon) {
        errors.push({
            property: 'location.longitude',
            constraints: {
                range: `Longitude must be between ${VALID_COORDINATES.minLon} and ${VALID_COORDINATES.maxLon}`
            }
        });
    }

    if (location.altitude < VALID_COORDINATES.minAlt || location.altitude > VALID_COORDINATES.maxAlt) {
        errors.push({
            property: 'location.altitude',
            constraints: {
                range: `Altitude must be between ${VALID_COORDINATES.minAlt} and ${VALID_COORDINATES.maxAlt} meters`
            }
        });
    }

    // Validate coordinate precision
    const latPrecision = location.latitude.toString().split('.')[1]?.length || 0;
    const lonPrecision = location.longitude.toString().split('.')[1]?.length || 0;

    if (latPrecision > COORDINATE_PRECISION || lonPrecision > COORDINATE_PRECISION) {
        errors.push({
            property: 'location',
            constraints: {
                precision: `Coordinates must not exceed ${COORDINATE_PRECISION} decimal places`
            }
        });
    }

    return errors;
}

/**
 * Validates treatment coverage area with boundary and overlap checks
 * @param coverage Treatment coverage area data
 * @returns Array of coverage validation errors
 */
export async function validateTreatmentCoverage(coverage: ITreatmentCoverage): Promise<ValidationError[]> {
    const errors: ValidationError[] = [];

    // Validate coverage type
    if (coverage.type !== 'Polygon' && coverage.type !== 'MultiPolygon') {
        errors.push({
            property: 'coverage.type',
            constraints: {
                enum: 'Coverage type must be either Polygon or MultiPolygon'
            }
        });
    }

    // Validate coordinates structure
    if (!Array.isArray(coverage.coordinates) || coverage.coordinates.length === 0) {
        errors.push({
            property: 'coverage.coordinates',
            constraints: {
                array: 'Coverage coordinates must be a non-empty array'
            }
        });
        return errors;
    }

    // Validate polygon closure
    const firstPoint = coverage.coordinates[0];
    const lastPoint = coverage.coordinates[coverage.coordinates.length - 1];
    if (firstPoint[0] !== lastPoint[0] || firstPoint[1] !== lastPoint[1]) {
        errors.push({
            property: 'coverage.coordinates',
            constraints: {
                closed: 'Coverage polygon must be closed (first and last points must match)'
            }
        });
    }

    // Calculate and validate coverage area
    const area = calculatePolygonArea(coverage.coordinates);
    if (area < MIN_COVERAGE_AREA || area > MAX_COVERAGE_AREA) {
        errors.push({
            property: 'coverage',
            constraints: {
                area: `Coverage area must be between ${MIN_COVERAGE_AREA} and ${MAX_COVERAGE_AREA} hectares`
            }
        });
    }

    return errors;
}

/**
 * Calculates the area of a polygon in hectares
 * @param coordinates Array of polygon coordinates
 * @returns Area in hectares
 */
function calculatePolygonArea(coordinates: number[][]): number {
    let area = 0;
    for (let i = 0; i < coordinates.length - 1; i++) {
        area += coordinates[i][0] * coordinates[i + 1][1] - coordinates[i + 1][0] * coordinates[i][1];
    }
    return Math.abs(area) / 2 / 10000; // Convert to hectares
}