/**
 * @fileoverview Express middleware for request validation in the agricultural management system
 * Provides centralized validation for all incoming API requests using Zod schemas
 * @version 1.0.0
 */

import { Request, Response, NextFunction, RequestHandler } from 'express'; // v4.18.2
import * as z from 'zod'; // v3.21.4
import {
  validateGeolocation,
  validateMissionParameters,
  validateTelemetryData,
  validateTreatmentPlan
} from '../utils/validation';
import { ErrorCodes, HttpStatusCodes } from '../constants/errorCodes';

// Cache for compiled Zod schemas to improve performance
const schemaCache = new Map<string, z.ZodSchema>();

/**
 * Enum for specifying which part of the request to validate
 */
export enum ValidationTarget {
  BODY = 'body',
  QUERY = 'query',
  PARAMS = 'params',
  HEADERS = 'headers'
}

/**
 * Interface for structured validation error responses
 */
interface ValidationError {
  code: number;
  message: string;
  context: string;
  details: z.ZodError;
  timestamp: Date;
}

/**
 * Constants for request size limits and validation
 */
const VALIDATION_CONSTANTS = {
  MAX_BODY_SIZE: 1024 * 1024, // 1MB
  MAX_ARRAY_LENGTH: 1000,
  MAX_STRING_LENGTH: 10000,
  CACHE_SIZE_LIMIT: 100
} as const;

/**
 * Creates a validation middleware using a Zod schema
 * @param schema Zod schema for validation
 * @param target Request part to validate
 * @returns Express middleware function
 */
export function validateRequest(
  schema: z.ZodSchema,
  target: ValidationTarget = ValidationTarget.BODY
): RequestHandler {
  // Cache schema for performance
  const cacheKey = `${schema.toString()}-${target}`;
  if (!schemaCache.has(cacheKey)) {
    if (schemaCache.size >= VALIDATION_CONSTANTS.CACHE_SIZE_LIMIT) {
      // Clear cache if limit reached
      schemaCache.clear();
    }
    schemaCache.set(cacheKey, schema);
  }

  return async (req: Request, res: Response, next: NextFunction): Promise<void> => {
    try {
      // Get data based on validation target
      const dataToValidate = req[target];

      // Check request size limits
      if (target === ValidationTarget.BODY && 
          JSON.stringify(dataToValidate).length > VALIDATION_CONSTANTS.MAX_BODY_SIZE) {
        throw new Error('Request body exceeds maximum size limit');
      }

      // Perform schema validation
      const validatedData = await schema.parseAsync(dataToValidate);

      // Attach validated data to request
      req[target] = validatedData;

      next();
    } catch (error) {
      const validationError: ValidationError = {
        code: ErrorCodes.VALIDATION_ERROR,
        message: 'Validation failed',
        context: target,
        details: error as z.ZodError,
        timestamp: new Date()
      };

      res.status(HttpStatusCodes.BAD_REQUEST).json(validationError);
    }
  };
}

/**
 * Specialized middleware for mission request validation
 */
export function validateMission(): RequestHandler {
  return async (req: Request, res: Response, next: NextFunction): Promise<void> => {
    try {
      const missionData = req.body;

      // Validate mission parameters
      const paramErrors = await validateMissionParameters(missionData.parameters);
      if (paramErrors.length > 0) {
        throw new Error('Invalid mission parameters');
      }

      // Validate coverage area coordinates
      if (missionData.coverageArea) {
        const geoErrors = await validateGeolocation(missionData.coverageArea.coordinates);
        if (geoErrors.length > 0) {
          throw new Error('Invalid coverage area coordinates');
        }
      }

      // Validate treatment plan if present
      if (missionData.parameters?.treatmentType) {
        const treatmentErrors = await validateTreatmentPlan({
          type: missionData.parameters.treatmentType,
          rate: missionData.parameters.applicationRate,
          area: missionData.coverageArea
        });
        if (treatmentErrors.length > 0) {
          throw new Error('Invalid treatment plan');
        }
      }

      next();
    } catch (error) {
      const validationError: ValidationError = {
        code: ErrorCodes.VALIDATION_ERROR,
        message: error.message || 'Mission validation failed',
        context: 'mission',
        details: error,
        timestamp: new Date()
      };

      res.status(HttpStatusCodes.BAD_REQUEST).json(validationError);
    }
  };
}

/**
 * Specialized middleware for telemetry data validation
 */
export function validateTelemetryRequest(): RequestHandler {
  return async (req: Request, res: Response, next: NextFunction): Promise<void> => {
    try {
      const telemetryData = req.body;

      // Validate telemetry data
      const telemetryErrors = await validateTelemetryData(telemetryData);
      if (telemetryErrors.length > 0) {
        throw new Error('Invalid telemetry data');
      }

      next();
    } catch (error) {
      const validationError: ValidationError = {
        code: ErrorCodes.VALIDATION_ERROR,
        message: error.message || 'Telemetry validation failed',
        context: 'telemetry',
        details: error,
        timestamp: new Date()
      };

      res.status(HttpStatusCodes.BAD_REQUEST).json(validationError);
    }
  };
}

/**
 * Middleware for sanitizing and validating request arrays
 */
export function validateArraySize(): RequestHandler {
  return (req: Request, res: Response, next: NextFunction): void => {
    try {
      const checkArraySize = (obj: any) => {
        for (const key in obj) {
          if (Array.isArray(obj[key]) && obj[key].length > VALIDATION_CONSTANTS.MAX_ARRAY_LENGTH) {
            throw new Error(`Array ${key} exceeds maximum length`);
          }
          if (typeof obj[key] === 'object') {
            checkArraySize(obj[key]);
          }
        }
      };

      checkArraySize(req.body);
      next();
    } catch (error) {
      const validationError: ValidationError = {
        code: ErrorCodes.VALIDATION_ERROR,
        message: error.message || 'Array size validation failed',
        context: 'array_size',
        details: error,
        timestamp: new Date()
      };

      res.status(HttpStatusCodes.BAD_REQUEST).json(validationError);
    }
  };
}