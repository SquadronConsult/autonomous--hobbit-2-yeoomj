/**
 * @fileoverview Mission data validation implementation for agricultural operations
 * Implements comprehensive validation rules ensuring data integrity, security compliance,
 * and operational safety for autonomous agricultural missions
 * @version 1.0.0
 */

import { validate, validateSync, ValidateNested, IsString, IsEnum, IsArray, 
         IsDate, IsNumber, IsObject, Min, Max, Length, ArrayMinSize, 
         ArrayMaxSize, IsLatLong, IsOptional } from 'class-validator'; // v0.14.0
import { Transform, Type } from 'class-transformer'; // v0.5.1
import { sanitize } from 'class-sanitizer'; // v2.0.0

import { IMission } from '../interfaces/IMission';
import { MissionStatus } from '../constants/missionStatus';
import { RobotType } from '../constants/robotTypes';
import { validateMission } from '../utils/validation';
import { ErrorCodes } from '../constants/errorCodes';

// Validation constants
const MISSION_ID_REGEX = /^AGM-M-\d{6}$/;
const MIN_MISSION_NAME_LENGTH = 3;
const MAX_MISSION_NAME_LENGTH = 100;
const MIN_ASSIGNED_DEVICES = 1;
const MAX_MISSION_DURATION_HOURS = 24;
const MAX_COVERAGE_AREA_HECTARES = 1000;
const MIN_OPERATION_ALTITUDE = 10;
const MAX_OPERATION_ALTITUDE = 120;
const MAX_WIND_SPEED_KPH = 25;
const MIN_VISIBILITY_KM = 5;

/**
 * Mission parameters validation schema
 */
class MissionParametersValidationSchema {
    @IsNumber()
    @Min(MIN_OPERATION_ALTITUDE)
    @Max(MAX_OPERATION_ALTITUDE)
    altitude?: number;

    @IsNumber()
    @Min(0)
    @Max(15)
    speed?: number;

    @IsNumber()
    @Min(0)
    @Max(100)
    overlap?: number;

    @IsNumber()
    @Min(0)
    resolution?: number;

    @IsString()
    @IsOptional()
    treatmentType?: string;

    @IsNumber()
    @Min(0)
    @IsOptional()
    applicationRate?: number;
}

/**
 * Weather constraints validation schema
 */
class WeatherConstraintsValidationSchema {
    @IsNumber()
    @Min(0)
    @Max(MAX_WIND_SPEED_KPH)
    maxWindSpeed: number;

    @IsNumber()
    @Min(MIN_VISIBILITY_KM)
    minVisibility: number;

    @IsNumber()
    @Min(0)
    @Max(100)
    maxPrecipitationProbability: number;
}

/**
 * Coverage area validation schema
 */
class CoverageAreaValidationSchema {
    @IsString()
    type: string;

    @IsArray()
    @ArrayMinSize(3)
    @IsLatLong({ each: true })
    coordinates: number[][];

    @IsObject()
    properties: Record<string, unknown>;
}

/**
 * Comprehensive mission validation schema with security and operational safety checks
 */
export class MissionValidationSchema implements Partial<IMission> {
    @IsString()
    @Length(MIN_MISSION_NAME_LENGTH, MAX_MISSION_NAME_LENGTH)
    @Transform(({ value }) => value.trim())
    name: string;

    @IsString()
    @Transform(({ value }) => value.trim())
    id: string;

    @IsEnum(MissionStatus)
    status: MissionStatus;

    @IsArray()
    @ArrayMinSize(MIN_ASSIGNED_DEVICES)
    @ValidateNested({ each: true })
    assignedDevices: Array<{
        deviceId: string;
        type: RobotType;
    }>;

    @IsObject()
    @ValidateNested()
    @Type(() => CoverageAreaValidationSchema)
    coverageArea: CoverageAreaValidationSchema;

    @IsObject()
    @ValidateNested()
    @Type(() => MissionParametersValidationSchema)
    parameters: MissionParametersValidationSchema;

    @IsDate()
    @Type(() => Date)
    startTime: Date;

    @IsDate()
    @IsOptional()
    @Type(() => Date)
    endTime?: Date;

    @IsObject()
    @ValidateNested()
    @Type(() => WeatherConstraintsValidationSchema)
    weatherConstraints: WeatherConstraintsValidationSchema;
}

/**
 * Comprehensive mission data validation with security and operational checks
 * @param mission Mission data to validate
 * @returns Promise resolving to array of validation errors if any
 */
export async function validateMissionData(mission: IMission): Promise<ValidationError[]> {
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

    // Create and validate schema instance
    const validationSchema = new MissionValidationSchema();
    Object.assign(validationSchema, mission);
    const schemaErrors = await validate(validationSchema);
    errors.push(...schemaErrors);

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

    // Validate coverage area size
    if (mission.coverageArea?.coordinates) {
        const areaSize = calculateAreaSize(mission.coverageArea.coordinates);
        if (areaSize > MAX_COVERAGE_AREA_HECTARES) {
            errors.push({
                property: 'coverageArea',
                constraints: {
                    max: `Coverage area cannot exceed ${MAX_COVERAGE_AREA_HECTARES} hectares`
                }
            });
        }
    }

    // Validate device assignments
    if (mission.assignedDevices) {
        const deviceErrors = validateDeviceAssignments(mission.assignedDevices);
        errors.push(...deviceErrors);
    }

    // Perform additional validation using utility function
    const utilityErrors = await validateMission(mission);
    errors.push(...utilityErrors);

    return errors;
}

/**
 * Validates device assignments for mission requirements
 * @param devices Array of assigned devices
 * @returns Array of validation errors if any
 */
function validateDeviceAssignments(devices: Array<{ deviceId: string; type: RobotType }>): ValidationError[] {
    const errors: ValidationError[] = [];
    const hasAerialDevice = devices.some(d => d.type === RobotType.AERIAL_DRONE);
    const hasGroundDevice = devices.some(d => d.type === RobotType.GROUND_ROBOT);

    if (!hasAerialDevice && !hasGroundDevice) {
        errors.push({
            property: 'assignedDevices',
            constraints: {
                requirement: 'Mission must have at least one aerial or ground device assigned'
            }
        });
    }

    return errors;
}

/**
 * Calculates area size in hectares from coordinate array
 * @param coordinates Array of [longitude, latitude] coordinates
 * @returns Area size in hectares
 */
function calculateAreaSize(coordinates: number[][]): number {
    // Implementation of area calculation algorithm
    // This is a placeholder - actual implementation would use proper geodesic calculations
    return 0;
}