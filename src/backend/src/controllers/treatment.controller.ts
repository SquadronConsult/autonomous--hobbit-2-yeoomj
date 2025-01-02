/**
 * @fileoverview Enhanced controller for agricultural treatment operations
 * Implements precision treatment management with chemical usage optimization
 * @version 1.0.0
 */

import { Router, Request, Response, NextFunction } from 'express'; // v4.18.2
import rateLimit from 'express-rate-limit'; // v6.7.0
import { TreatmentService } from '../services/treatment.service';
import { ITreatment } from '../interfaces/ITreatment';
import { validateTreatment } from '../utils/validation';
import { logger } from '../utils/logger';
import { ErrorCodes, ErrorMessages, ErrorCodeToHttpStatus } from '../constants/errorCodes';
import { MissionStatus } from '../constants/missionStatus';

/**
 * Enhanced controller class for handling treatment-related HTTP requests
 * Implements chemical usage optimization and real-time monitoring
 */
export class TreatmentController {
    private readonly treatmentService: TreatmentService;
    private readonly rateLimiter: any;

    constructor(treatmentService: TreatmentService) {
        this.treatmentService = treatmentService;
        
        // Configure rate limiting for API protection
        this.rateLimiter = rateLimit({
            windowMs: 15 * 60 * 1000, // 15 minutes
            max: 100, // Limit each IP to 100 requests per window
            message: 'Too many treatment requests, please try again later'
        });
    }

    /**
     * Creates a new treatment operation with chemical usage optimization
     * @route POST /api/v1/treatments
     */
    public async createTreatment(req: Request, res: Response, next: NextFunction): Promise<Response> {
        try {
            const treatmentData: ITreatment = req.body;

            // Validate treatment request data
            const validationErrors = await validateTreatment(treatmentData);
            if (validationErrors.length > 0) {
                logger.error('Treatment validation failed', {
                    correlationId: `treatment-${Date.now()}`,
                    component: 'TreatmentController',
                    operation: 'createTreatment',
                    details: validationErrors
                });
                return res.status(400).json({
                    error: ErrorMessages[ErrorCodes.VALIDATION_ERROR],
                    details: validationErrors
                });
            }

            // Create optimized treatment
            const treatment = await this.treatmentService.createTreatment(treatmentData);

            logger.info('Treatment created successfully', {
                correlationId: `treatment-${treatment.id}`,
                component: 'TreatmentController',
                operation: 'createTreatment'
            });

            return res.status(201).json(treatment);
        } catch (error) {
            logger.error('Failed to create treatment', {
                correlationId: `error-${Date.now()}`,
                component: 'TreatmentController',
                operation: 'createTreatment',
                details: error
            });
            next(error);
        }
    }

    /**
     * Retrieves treatments for a specific mission with telemetry correlation
     * @route GET /api/v1/treatments/mission/:missionId
     */
    public async getTreatmentsByMission(req: Request, res: Response, next: NextFunction): Promise<Response> {
        try {
            const { missionId } = req.params;
            const { startDate, endDate } = req.query;

            const treatments = await this.treatmentService.getTreatmentsByMission(
                missionId,
                {
                    startDate: startDate ? new Date(startDate as string) : undefined,
                    endDate: endDate ? new Date(endDate as string) : undefined
                }
            );

            return res.status(200).json(treatments);
        } catch (error) {
            logger.error('Failed to retrieve treatments', {
                correlationId: `error-${Date.now()}`,
                component: 'TreatmentController',
                operation: 'getTreatmentsByMission',
                details: error
            });
            next(error);
        }
    }

    /**
     * Retrieves treatments performed by a specific device
     * @route GET /api/v1/treatments/device/:deviceId
     */
    public async getTreatmentsByDevice(req: Request, res: Response, next: NextFunction): Promise<Response> {
        try {
            const { deviceId } = req.params;
            const treatments = await this.treatmentService.getTreatmentsByDevice(deviceId);
            return res.status(200).json(treatments);
        } catch (error) {
            logger.error('Failed to retrieve device treatments', {
                correlationId: `error-${Date.now()}`,
                component: 'TreatmentController',
                operation: 'getTreatmentsByDevice',
                details: error
            });
            next(error);
        }
    }

    /**
     * Updates treatment status with real-time monitoring
     * @route PATCH /api/v1/treatments/:treatmentId/status
     */
    public async updateTreatmentStatus(req: Request, res: Response, next: NextFunction): Promise<Response> {
        try {
            const { treatmentId } = req.params;
            const { status } = req.body;

            if (!Object.values(MissionStatus).includes(status)) {
                return res.status(400).json({
                    error: ErrorMessages[ErrorCodes.VALIDATION_ERROR],
                    details: 'Invalid treatment status'
                });
            }

            const updatedTreatment = await this.treatmentService.updateTreatmentStatus(
                treatmentId,
                status
            );

            // Monitor treatment progress
            await this.treatmentService.monitorTreatmentProgress(treatmentId);

            return res.status(200).json(updatedTreatment);
        } catch (error) {
            logger.error('Failed to update treatment status', {
                correlationId: `error-${Date.now()}`,
                component: 'TreatmentController',
                operation: 'updateTreatmentStatus',
                details: error
            });
            next(error);
        }
    }

    /**
     * Returns the configured router with all treatment endpoints
     */
    public getRouter(): Router {
        const router = Router();

        router.post('/', this.rateLimiter, this.createTreatment.bind(this));
        router.get('/mission/:missionId', this.getTreatmentsByMission.bind(this));
        router.get('/device/:deviceId', this.getTreatmentsByDevice.bind(this));
        router.patch('/:treatmentId/status', this.updateTreatmentStatus.bind(this));

        return router;
    }
}