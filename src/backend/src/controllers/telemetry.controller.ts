/**
 * @fileoverview REST API controller for managing real-time telemetry data from agricultural robots and drones
 * @version 1.0.0
 * @license MIT
 */

import { Router, Request, Response } from 'express'; // v4.18.2
import asyncHandler from 'express-async-handler'; // v1.2.0
import rateLimit from 'express-rate-limit'; // v6.7.0
import { TelemetryService } from '../services/telemetry.service';
import { TelemetryValidator } from '../validators/telemetry.validator';
import { ITelemetry } from '../interfaces/ITelemetry';
import { metrics } from '../utils/metrics';
import { logger } from '../utils/logger';

// Rate limiting configuration
const RATE_LIMIT_WINDOW = 60 * 1000; // 1 minute
const RATE_LIMIT_MAX = 1000; // requests per window
const BATCH_SIZE_LIMIT = 100;

/**
 * Controller class for handling telemetry-related HTTP requests with enhanced
 * security, monitoring, and performance optimization
 */
export class TelemetryController {
    private readonly _router: Router;
    private readonly _telemetryService: TelemetryService;
    private readonly _validator: TelemetryValidator;
    private readonly _rateLimiter: any;

    constructor() {
        this._router = Router();
        this._telemetryService = new TelemetryService();
        this._validator = new TelemetryValidator();
        
        // Configure rate limiter
        this._rateLimiter = rateLimit({
            windowMs: RATE_LIMIT_WINDOW,
            max: RATE_LIMIT_MAX,
            message: 'Too many requests from this IP',
            standardHeaders: true,
            legacyHeaders: false
        });

        this.initializeRoutes();
    }

    /**
     * Initializes controller routes with middleware and handlers
     * @private
     */
    private initializeRoutes(): void {
        // Create telemetry data point(s)
        this._router.post('/', 
            this._rateLimiter,
            asyncHandler(this.createTelemetry.bind(this))
        );

        // Get telemetry by device
        this._router.get('/device/:deviceId',
            this._rateLimiter,
            asyncHandler(this.getTelemetryByDevice.bind(this))
        );

        // Get telemetry in time range
        this._router.get('/timerange',
            this._rateLimiter,
            asyncHandler(this.getTelemetryInTimeRange.bind(this))
        );

        // Get latest telemetry
        this._router.get('/latest/:deviceId',
            this._rateLimiter,
            asyncHandler(this.getLatestTelemetry.bind(this))
        );

        // Stream telemetry data
        this._router.get('/stream',
            asyncHandler(this.streamTelemetryData.bind(this))
        );

        // Aggregate telemetry data
        this._router.get('/aggregate',
            this._rateLimiter,
            asyncHandler(this.aggregateTelemetry.bind(this))
        );
    }

    /**
     * Creates new telemetry data point(s) with validation and monitoring
     * @param req Express request
     * @param res Express response
     */
    private async createTelemetry(req: Request, res: Response): Promise<void> {
        const startTime = Date.now();
        const correlationId = req.headers['x-correlation-id'] as string;

        try {
            const telemetryData = Array.isArray(req.body) ? req.body : [req.body];

            // Validate batch size
            if (telemetryData.length > BATCH_SIZE_LIMIT) {
                throw new Error(`Batch size exceeds limit of ${BATCH_SIZE_LIMIT}`);
            }

            // Validate telemetry data
            const validationErrors = await this._validator.validateTelemetryBatch(telemetryData);
            if (validationErrors.length > 0) {
                res.status(400).json({ errors: validationErrors });
                return;
            }

            // Create telemetry records
            const createdRecords = await this._telemetryService.createTelemetry(telemetryData);

            // Record metrics
            const latency = (Date.now() - startTime) / 1000;
            metrics.recordVideoLatency('telemetry', 'creation', latency);
            metrics.updateDeviceCount('all', 'active', 'all', telemetryData.length);

            logger.info('Telemetry data created successfully', {
                correlationId,
                component: 'TelemetryController',
                operation: 'createTelemetry',
                details: { count: createdRecords.length, latency }
            });

            res.status(201).json(createdRecords);
        } catch (error) {
            logger.error(error as Error, {
                correlationId,
                component: 'TelemetryController',
                operation: 'createTelemetry'
            });
            res.status(500).json({ error: 'Failed to create telemetry data' });
        }
    }

    /**
     * Retrieves telemetry data for specific device(s)
     * @param req Express request
     * @param res Express response
     */
    private async getTelemetryByDevice(req: Request, res: Response): Promise<void> {
        const startTime = Date.now();
        const correlationId = req.headers['x-correlation-id'] as string;
        const deviceId = req.params.deviceId;

        try {
            const options = {
                limit: parseInt(req.query.limit as string) || 100,
                offset: parseInt(req.query.offset as string) || 0,
                timeInterval: req.query.interval as string,
                aggregation: req.query.aggregation as 'avg' | 'min' | 'max' | 'count'
            };

            const telemetryData = await this._telemetryService.getTelemetryByDevice([deviceId], options);

            // Record metrics
            const latency = (Date.now() - startTime) / 1000;
            metrics.recordVideoLatency('telemetry', 'retrieval', latency);

            logger.info('Telemetry data retrieved successfully', {
                correlationId,
                component: 'TelemetryController',
                operation: 'getTelemetryByDevice',
                details: { deviceId, count: telemetryData.length, latency }
            });

            res.json(telemetryData);
        } catch (error) {
            logger.error(error as Error, {
                correlationId,
                component: 'TelemetryController',
                operation: 'getTelemetryByDevice'
            });
            res.status(500).json({ error: 'Failed to retrieve telemetry data' });
        }
    }

    /**
     * Retrieves telemetry data within specified time range
     * @param req Express request
     * @param res Express response
     */
    private async getTelemetryInTimeRange(req: Request, res: Response): Promise<void> {
        const startTime = Date.now();
        const correlationId = req.headers['x-correlation-id'] as string;

        try {
            const { start, end } = req.query;
            if (!start || !end) {
                res.status(400).json({ error: 'Start and end times are required' });
                return;
            }

            const options = {
                limit: parseInt(req.query.limit as string) || 1000,
                offset: parseInt(req.query.offset as string) || 0,
                aggregation: req.query.aggregation as 'avg' | 'min' | 'max' | 'count'
            };

            const telemetryData = await this._telemetryService.getTelemetryInTimeRange(
                new Date(start as string),
                new Date(end as string),
                options
            );

            // Record metrics
            const latency = (Date.now() - startTime) / 1000;
            metrics.recordVideoLatency('telemetry', 'timerange', latency);

            logger.info('Time range telemetry data retrieved successfully', {
                correlationId,
                component: 'TelemetryController',
                operation: 'getTelemetryInTimeRange',
                details: { start, end, count: telemetryData.length, latency }
            });

            res.json(telemetryData);
        } catch (error) {
            logger.error(error as Error, {
                correlationId,
                component: 'TelemetryController',
                operation: 'getTelemetryInTimeRange'
            });
            res.status(500).json({ error: 'Failed to retrieve telemetry data' });
        }
    }

    /**
     * Gets latest telemetry data for a device
     * @param req Express request
     * @param res Express response
     */
    private async getLatestTelemetry(req: Request, res: Response): Promise<void> {
        const startTime = Date.now();
        const correlationId = req.headers['x-correlation-id'] as string;
        const deviceId = req.params.deviceId;

        try {
            const telemetryData = await this._telemetryService.getLatestTelemetry(deviceId);

            // Record metrics
            const latency = (Date.now() - startTime) / 1000;
            metrics.recordVideoLatency('telemetry', 'latest', latency);

            logger.info('Latest telemetry data retrieved successfully', {
                correlationId,
                component: 'TelemetryController',
                operation: 'getLatestTelemetry',
                details: { deviceId, latency }
            });

            res.json(telemetryData);
        } catch (error) {
            logger.error(error as Error, {
                correlationId,
                component: 'TelemetryController',
                operation: 'getLatestTelemetry'
            });
            res.status(500).json({ error: 'Failed to retrieve latest telemetry data' });
        }
    }

    /**
     * Streams real-time telemetry data with SSE
     * @param req Express request
     * @param res Express response
     */
    private async streamTelemetryData(req: Request, res: Response): Promise<void> {
        const correlationId = req.headers['x-correlation-id'] as string;

        try {
            // Set up SSE headers
            res.setHeader('Content-Type', 'text/event-stream');
            res.setHeader('Cache-Control', 'no-cache');
            res.setHeader('Connection', 'keep-alive');

            const stream = await this._telemetryService.streamTelemetryData();
            stream.on('data', (data: ITelemetry) => {
                res.write(`data: ${JSON.stringify(data)}\n\n`);
            });

            // Handle client disconnect
            req.on('close', () => {
                stream.destroy();
                logger.info('Telemetry stream closed', {
                    correlationId,
                    component: 'TelemetryController',
                    operation: 'streamTelemetryData'
                });
            });
        } catch (error) {
            logger.error(error as Error, {
                correlationId,
                component: 'TelemetryController',
                operation: 'streamTelemetryData'
            });
            res.status(500).json({ error: 'Failed to stream telemetry data' });
        }
    }

    /**
     * Aggregates telemetry data with specified parameters
     * @param req Express request
     * @param res Express response
     */
    private async aggregateTelemetry(req: Request, res: Response): Promise<void> {
        const startTime = Date.now();
        const correlationId = req.headers['x-correlation-id'] as string;

        try {
            const options = {
                timeInterval: req.query.interval as string || '1h',
                aggregation: req.query.aggregation as 'avg' | 'min' | 'max' | 'count' || 'avg',
                deviceIds: (req.query.deviceIds as string || '').split(',').filter(Boolean)
            };

            const aggregatedData = await this._telemetryService.aggregateTelemetry(options);

            // Record metrics
            const latency = (Date.now() - startTime) / 1000;
            metrics.recordVideoLatency('telemetry', 'aggregation', latency);

            logger.info('Telemetry data aggregated successfully', {
                correlationId,
                component: 'TelemetryController',
                operation: 'aggregateTelemetry',
                details: { options, latency }
            });

            res.json(aggregatedData);
        } catch (error) {
            logger.error(error as Error, {
                correlationId,
                component: 'TelemetryController',
                operation: 'aggregateTelemetry'
            });
            res.status(500).json({ error: 'Failed to aggregate telemetry data' });
        }
    }

    /**
     * Gets the configured router instance
     * @returns Express Router instance
     */
    public getRouter(): Router {
        return this._router;
    }
}