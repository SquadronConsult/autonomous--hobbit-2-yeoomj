/**
 * @fileoverview Enterprise-grade REST API controller for agricultural mission management
 * Implements secure endpoints with rate limiting, monitoring, and comprehensive error handling
 * @version 1.0.0
 */

import { Request, Response, NextFunction } from 'express';
import { StatusCodes } from 'http-status-codes';
import { validationResult, body, param } from 'express-validator';
import rateLimit from 'express-rate-limit';
import compression from 'compression';
import { MissionService } from '../services/mission.service';
import { IMission } from '../interfaces/IMission';
import { logger } from '../utils/logger';
import { ErrorCodes, ErrorMessages } from '../constants/errorCodes';
import { MissionStatus } from '../constants/missionStatus';

// Rate limiting configuration for mission endpoints
const missionRateLimit = rateLimit({
    windowMs: 15 * 60 * 1000, // 15 minutes
    max: 100, // Limit each IP to 100 requests per window
    message: 'Too many requests from this IP, please try again later'
});

// Validation middleware for mission creation
const validateMissionCreation = [
    body('name').isString().trim().isLength({ min: 3, max: 100 }),
    body('type').isString().isIn(['surveillance', 'treatment', 'monitoring', 'intervention']),
    body('assignedDevices').isArray().notEmpty(),
    body('coverageArea').isObject().notEmpty(),
    body('parameters').isObject().notEmpty(),
    body('startTime').isISO8601()
];

// Validation middleware for mission ID
const validateMissionId = [
    param('id').matches(/^AGM-M-\d{6}$/).withMessage('Invalid mission ID format')
];

/**
 * Creates a new agricultural mission with comprehensive validation
 * @param req Express request object
 * @param res Express response object
 * @param next Express next function
 */
async function createMission(req: Request, res: Response, next: NextFunction): Promise<void> {
    const correlationId = `create-mission-${Date.now()}`;
    
    try {
        // Validate request
        const errors = validationResult(req);
        if (!errors.isEmpty()) {
            logger.error('Mission validation failed', {
                correlationId,
                component: 'MissionController',
                operation: 'createMission',
                details: errors.array()
            });
            res.status(StatusCodes.BAD_REQUEST).json({
                code: ErrorCodes.VALIDATION_ERROR,
                message: ErrorMessages[ErrorCodes.VALIDATION_ERROR],
                errors: errors.array()
            });
            return;
        }

        const missionData: IMission = req.body;
        const mission = await MissionService.createMission(missionData);

        logger.info('Mission created successfully', {
            correlationId,
            component: 'MissionController',
            operation: 'createMission',
            details: { missionId: mission.id }
        });

        res.status(StatusCodes.CREATED).json(mission);
    } catch (error) {
        next(error);
    }
}

/**
 * Retrieves mission details by ID with caching
 * @param req Express request object
 * @param res Express response object
 * @param next Express next function
 */
async function getMission(req: Request, res: Response, next: NextFunction): Promise<void> {
    const correlationId = `get-mission-${req.params.id}`;
    
    try {
        const mission = await MissionService.getMissionById(req.params.id);
        
        if (!mission) {
            logger.warn('Mission not found', {
                correlationId,
                component: 'MissionController',
                operation: 'getMission',
                details: { missionId: req.params.id }
            });
            res.status(StatusCodes.NOT_FOUND).json({
                code: ErrorCodes.RESOURCE_NOT_FOUND,
                message: ErrorMessages[ErrorCodes.RESOURCE_NOT_FOUND]
            });
            return;
        }

        res.status(StatusCodes.OK).json(mission);
    } catch (error) {
        next(error);
    }
}

/**
 * Updates mission status with validation and monitoring
 * @param req Express request object
 * @param res Express response object
 * @param next Express next function
 */
async function updateMissionStatus(req: Request, res: Response, next: NextFunction): Promise<void> {
    const correlationId = `update-status-${req.params.id}`;
    
    try {
        const { status } = req.body;
        if (!Object.values(MissionStatus).includes(status)) {
            res.status(StatusCodes.BAD_REQUEST).json({
                code: ErrorCodes.VALIDATION_ERROR,
                message: 'Invalid mission status'
            });
            return;
        }

        const mission = await MissionService.updateMissionStatus(req.params.id, status);
        
        logger.info('Mission status updated', {
            correlationId,
            component: 'MissionController',
            operation: 'updateMissionStatus',
            details: { missionId: req.params.id, newStatus: status }
        });

        res.status(StatusCodes.OK).json(mission);
    } catch (error) {
        next(error);
    }
}

/**
 * Updates mission progress with validation
 * @param req Express request object
 * @param res Express response object
 * @param next Express next function
 */
async function updateMissionProgress(req: Request, res: Response, next: NextFunction): Promise<void> {
    const correlationId = `update-progress-${req.params.id}`;
    
    try {
        const { progress } = req.body;
        if (typeof progress !== 'number' || progress < 0 || progress > 100) {
            res.status(StatusCodes.BAD_REQUEST).json({
                code: ErrorCodes.VALIDATION_ERROR,
                message: 'Progress must be a number between 0 and 100'
            });
            return;
        }

        const mission = await MissionService.updateMissionProgress(req.params.id, progress);
        
        logger.info('Mission progress updated', {
            correlationId,
            component: 'MissionController',
            operation: 'updateMissionProgress',
            details: { missionId: req.params.id, progress }
        });

        res.status(StatusCodes.OK).json(mission);
    } catch (error) {
        next(error);
    }
}

/**
 * Retrieves active missions with pagination and filtering
 * @param req Express request object
 * @param res Express response object
 * @param next Express next function
 */
async function getActiveMissions(req: Request, res: Response, next: NextFunction): Promise<void> {
    const correlationId = `get-active-missions-${Date.now()}`;
    
    try {
        const { type, region, limit = 10, offset = 0 } = req.query;
        const result = await MissionService.getActiveMissions({
            type: type as string,
            region: region as string,
            limit: Number(limit),
            offset: Number(offset)
        });

        res.status(StatusCodes.OK).json(result);
    } catch (error) {
        next(error);
    }
}

/**
 * Assigns a device to an existing mission
 * @param req Express request object
 * @param res Express response object
 * @param next Express next function
 */
async function assignDevice(req: Request, res: Response, next: NextFunction): Promise<void> {
    const correlationId = `assign-device-${req.params.id}`;
    
    try {
        const { deviceId } = req.body;
        const mission = await MissionService.assignDeviceToMission(req.params.id, deviceId);
        
        logger.info('Device assigned to mission', {
            correlationId,
            component: 'MissionController',
            operation: 'assignDevice',
            details: { missionId: req.params.id, deviceId }
        });

        res.status(StatusCodes.OK).json(mission);
    } catch (error) {
        next(error);
    }
}

// Export controller methods with rate limiting and validation
export const MissionController = {
    createMission: [compression(), missionRateLimit, validateMissionCreation, createMission],
    getMission: [compression(), validateMissionId, getMission],
    updateMissionStatus: [compression(), missionRateLimit, validateMissionId, updateMissionStatus],
    updateMissionProgress: [compression(), missionRateLimit, validateMissionId, updateMissionProgress],
    getActiveMissions: [compression(), getActiveMissions],
    assignDevice: [compression(), missionRateLimit, validateMissionId, assignDevice]
};