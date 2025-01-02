import { Request, Response, NextFunction } from 'express';
import * as promClient from 'prom-client';
import { validate } from 'class-validator';
import winston from 'winston';
import { AnalyticsService } from '../services/analytics.service';
import { IAnalytics } from '../interfaces/IAnalytics';
import { authenticateRequest, authorizeRole } from '../middleware/auth.middleware';
import { validateRequest } from '../middleware/validation.middleware';
import { ErrorCodes } from '../constants/errorCodes';

// Initialize Prometheus metrics
const analyticsRequestCounter = new promClient.Counter({
    name: 'analytics_requests_total',
    help: 'Total number of analytics API requests',
    labelNames: ['method', 'endpoint', 'status', 'error']
});

const requestLatencyHistogram = new promClient.Histogram({
    name: 'analytics_request_duration_seconds',
    help: 'Analytics API request duration in seconds',
    labelNames: ['method', 'endpoint', 'status'],
    buckets: [0.1, 0.5, 1, 2, 5]
});

// Initialize logger
const logger = winston.createLogger({
    level: 'info',
    format: winston.format.json(),
    transports: [
        new winston.transports.File({ filename: 'analytics-error.log', level: 'error' }),
        new winston.transports.File({ filename: 'analytics-combined.log' })
    ]
});

export class AnalyticsController {
    private analyticsService: AnalyticsService;

    constructor() {
        this.analyticsService = new AnalyticsService();
    }

    /**
     * Creates a new analytics record
     * @route POST /api/v1/analytics
     */
    @authenticateRequest
    @authorizeRole(['administrator', 'operator'])
    @validateRequest(IAnalytics)
    public async createAnalyticsRecord(req: Request, res: Response, next: NextFunction): Promise<void> {
        const startTime = process.hrtime();
        const correlationId = `analytics-${Date.now()}`;

        try {
            logger.info('Creating analytics record', {
                correlationId,
                operation: 'createAnalyticsRecord',
                data: { ...req.body, deviceId: req.body.deviceId }
            });

            const analyticsData: IAnalytics = req.body;
            const result = await this.analyticsService.createAnalytics(analyticsData);

            const [seconds, nanoseconds] = process.hrtime(startTime);
            const duration = seconds + nanoseconds / 1e9;

            requestLatencyHistogram.observe(
                { method: 'POST', endpoint: '/analytics', status: 'success' },
                duration
            );
            analyticsRequestCounter.inc({ method: 'POST', endpoint: '/analytics', status: 'success' });

            res.status(201).json(result);
        } catch (error) {
            analyticsRequestCounter.inc({ 
                method: 'POST', 
                endpoint: '/analytics', 
                status: 'error',
                error: error.message 
            });
            next(error);
        }
    }

    /**
     * Retrieves analytics for a specific mission
     * @route GET /api/v1/analytics/mission/:missionId
     */
    @authenticateRequest
    @authorizeRole(['administrator', 'operator', 'analyst'])
    public async getMissionAnalytics(req: Request, res: Response, next: NextFunction): Promise<void> {
        const startTime = process.hrtime();
        const correlationId = `analytics-mission-${req.params.missionId}`;

        try {
            logger.info('Retrieving mission analytics', {
                correlationId,
                operation: 'getMissionAnalytics',
                missionId: req.params.missionId
            });

            const analytics = await this.analyticsService.getAnalyticsByMission(req.params.missionId);

            const [seconds, nanoseconds] = process.hrtime(startTime);
            const duration = seconds + nanoseconds / 1e9;

            requestLatencyHistogram.observe(
                { method: 'GET', endpoint: '/analytics/mission', status: 'success' },
                duration
            );
            analyticsRequestCounter.inc({ method: 'GET', endpoint: '/analytics/mission', status: 'success' });

            res.status(200).json(analytics);
        } catch (error) {
            analyticsRequestCounter.inc({ 
                method: 'GET', 
                endpoint: '/analytics/mission', 
                status: 'error',
                error: error.message 
            });
            next(error);
        }
    }

    /**
     * Processes real-time video analytics stream
     * @route POST /api/v1/analytics/stream
     */
    @authenticateRequest
    @authorizeRole(['administrator', 'operator'])
    public async processStreamAnalytics(req: Request, res: Response, next: NextFunction): Promise<void> {
        const startTime = process.hrtime();
        const correlationId = `analytics-stream-${Date.now()}`;

        try {
            logger.info('Processing analytics stream', {
                correlationId,
                operation: 'processStreamAnalytics',
                deviceId: req.body.deviceId
            });

            await this.analyticsService.processRealTimeAnalytics(req.body);

            const [seconds, nanoseconds] = process.hrtime(startTime);
            const duration = seconds + nanoseconds / 1e9;

            requestLatencyHistogram.observe(
                { method: 'POST', endpoint: '/analytics/stream', status: 'success' },
                duration
            );
            analyticsRequestCounter.inc({ method: 'POST', endpoint: '/analytics/stream', status: 'success' });

            res.status(200).json({ message: 'Stream processed successfully' });
        } catch (error) {
            analyticsRequestCounter.inc({ 
                method: 'POST', 
                endpoint: '/analytics/stream', 
                status: 'error',
                error: error.message 
            });
            next(error);
        }
    }

    /**
     * Retrieves analytics within a time range
     * @route GET /api/v1/analytics/timerange
     */
    @authenticateRequest
    @authorizeRole(['administrator', 'analyst'])
    public async getTimeRangeAnalytics(req: Request, res: Response, next: NextFunction): Promise<void> {
        const startTime = process.hrtime();
        const correlationId = `analytics-timerange-${Date.now()}`;

        try {
            const { from, to } = req.query;
            if (!from || !to) {
                throw new Error(ErrorCodes.VALIDATION_ERROR.toString());
            }

            logger.info('Retrieving time range analytics', {
                correlationId,
                operation: 'getTimeRangeAnalytics',
                timeRange: { from, to }
            });

            const analytics = await this.analyticsService.getAnalyticsByTimeRange(
                new Date(from as string),
                new Date(to as string)
            );

            const [seconds, nanoseconds] = process.hrtime(startTime);
            const duration = seconds + nanoseconds / 1e9;

            requestLatencyHistogram.observe(
                { method: 'GET', endpoint: '/analytics/timerange', status: 'success' },
                duration
            );
            analyticsRequestCounter.inc({ method: 'GET', endpoint: '/analytics/timerange', status: 'success' });

            res.status(200).json(analytics);
        } catch (error) {
            analyticsRequestCounter.inc({ 
                method: 'GET', 
                endpoint: '/analytics/timerange', 
                status: 'error',
                error: error.message 
            });
            next(error);
        }
    }

    /**
     * Retrieves aggregated analytics metrics
     * @route GET /api/v1/analytics/aggregate
     */
    @authenticateRequest
    @authorizeRole(['administrator', 'analyst'])
    public async getAggregatedAnalytics(req: Request, res: Response, next: NextFunction): Promise<void> {
        const startTime = process.hrtime();
        const correlationId = `analytics-aggregate-${Date.now()}`;

        try {
            logger.info('Retrieving aggregated analytics', {
                correlationId,
                operation: 'getAggregatedAnalytics'
            });

            const metrics = await this.analyticsService.getMetrics();

            const [seconds, nanoseconds] = process.hrtime(startTime);
            const duration = seconds + nanoseconds / 1e9;

            requestLatencyHistogram.observe(
                { method: 'GET', endpoint: '/analytics/aggregate', status: 'success' },
                duration
            );
            analyticsRequestCounter.inc({ method: 'GET', endpoint: '/analytics/aggregate', status: 'success' });

            res.status(200).json(metrics);
        } catch (error) {
            analyticsRequestCounter.inc({ 
                method: 'GET', 
                endpoint: '/analytics/aggregate', 
                status: 'error',
                error: error.message 
            });
            next(error);
        }
    }
}

export default AnalyticsController;