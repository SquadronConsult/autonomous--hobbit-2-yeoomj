/**
 * @fileoverview Express router configuration for telemetry endpoints handling real-time data collection
 * from agricultural robots and drones with comprehensive security, validation, and monitoring.
 * @version 1.0.0
 * @license MIT
 */

import { Router } from 'express'; // v4.18.2
import rateLimit from 'express-rate-limit'; // v6.7.0
import helmet from 'helmet'; // v7.0.0
import { TelemetryController } from '../controllers/telemetry.controller';
import { validateRequest } from '../middleware/validation.middleware';
import { authMiddleware } from '../middleware/auth.middleware';
import { 
    telemetrySchema, 
    telemetryBatchSchema, 
    telemetryQuerySchema, 
    aggregationSchema 
} from '../validators/telemetry.validator';
import { logger } from '../utils/logger';
import { metrics } from '../utils/metrics';

// Rate limiting configurations
const RATE_LIMITS = {
    singleTelemetry: {
        windowMs: 60 * 1000, // 1 minute
        max: 1000 // Support for 8+ simultaneous drone feeds
    },
    batchTelemetry: {
        windowMs: 60 * 1000,
        max: 200 // Batch operations are more resource-intensive
    },
    telemetryQuery: {
        windowMs: 60 * 1000,
        max: 500 // Read operations
    },
    aggregation: {
        windowMs: 60 * 1000,
        max: 100 // Complex aggregation operations
    }
};

/**
 * Configures and returns Express router with telemetry endpoints
 * Implements comprehensive security, validation, and monitoring
 * @param controller TelemetryController instance
 * @returns Configured Express router
 */
export function configureTelemetryRoutes(controller: TelemetryController): Router {
    const router = Router();

    // Apply base security middleware
    router.use(helmet({
        contentSecurityPolicy: true,
        crossOriginEmbedderPolicy: true,
        crossOriginOpenerPolicy: true,
        crossOriginResourcePolicy: true,
        dnsPrefetchControl: true,
        frameguard: true,
        hidePoweredBy: true,
        hsts: true,
        ieNoOpen: true,
        noSniff: true,
        originAgentCluster: true,
        permittedCrossDomainPolicies: true,
        referrerPolicy: true,
        xssFilter: true
    }));

    // Single telemetry data point ingestion
    router.post('/',
        rateLimit(RATE_LIMITS.singleTelemetry),
        authMiddleware(['operator', 'service']),
        validateRequest(telemetrySchema),
        async (req, res) => {
            try {
                const startTime = Date.now();
                const result = await controller.postTelemetry(req.body);
                
                // Record latency metric
                metrics.recordVideoLatency(
                    'telemetry_ingestion',
                    'single',
                    (Date.now() - startTime) / 1000
                );

                res.status(201).json(result);
            } catch (error) {
                logger.error(error as Error, {
                    correlationId: req.headers['x-correlation-id'] as string,
                    component: 'TelemetryRoutes',
                    operation: 'postTelemetry'
                });
                res.status(500).json({ error: 'Failed to process telemetry data' });
            }
        }
    );

    // Batch telemetry data ingestion
    router.post('/batch',
        rateLimit(RATE_LIMITS.batchTelemetry),
        authMiddleware(['operator', 'service']),
        validateRequest(telemetryBatchSchema),
        async (req, res) => {
            try {
                const startTime = Date.now();
                const result = await controller.postTelemetryBatch(req.body);
                
                metrics.recordVideoLatency(
                    'telemetry_ingestion',
                    'batch',
                    (Date.now() - startTime) / 1000
                );

                res.status(201).json(result);
            } catch (error) {
                logger.error(error as Error, {
                    correlationId: req.headers['x-correlation-id'] as string,
                    component: 'TelemetryRoutes',
                    operation: 'postTelemetryBatch'
                });
                res.status(500).json({ error: 'Failed to process batch telemetry data' });
            }
        }
    );

    // Retrieve telemetry data for specific device
    router.get('/:deviceId',
        rateLimit(RATE_LIMITS.telemetryQuery),
        authMiddleware(['operator', 'analyst', 'service']),
        validateRequest(telemetryQuerySchema, 'query'),
        async (req, res) => {
            try {
                const startTime = Date.now();
                const result = await controller.getTelemetry(
                    req.params.deviceId,
                    req.query
                );

                metrics.recordVideoLatency(
                    'telemetry_retrieval',
                    'query',
                    (Date.now() - startTime) / 1000
                );

                res.json(result);
            } catch (error) {
                logger.error(error as Error, {
                    correlationId: req.headers['x-correlation-id'] as string,
                    component: 'TelemetryRoutes',
                    operation: 'getTelemetry'
                });
                res.status(500).json({ error: 'Failed to retrieve telemetry data' });
            }
        }
    );

    // Retrieve aggregated telemetry metrics
    router.get('/aggregate',
        rateLimit(RATE_LIMITS.aggregation),
        authMiddleware(['operator', 'analyst']),
        validateRequest(aggregationSchema, 'query'),
        async (req, res) => {
            try {
                const startTime = Date.now();
                const result = await controller.getAggregatedTelemetry(req.query);

                metrics.recordVideoLatency(
                    'telemetry_aggregation',
                    'query',
                    (Date.now() - startTime) / 1000
                );

                res.json(result);
            } catch (error) {
                logger.error(error as Error, {
                    correlationId: req.headers['x-correlation-id'] as string,
                    component: 'TelemetryRoutes',
                    operation: 'getAggregatedTelemetry'
                });
                res.status(500).json({ error: 'Failed to retrieve aggregated telemetry data' });
            }
        }
    );

    // Error handling middleware
    router.use((err: Error, req: any, res: any, next: any) => {
        logger.error(err, {
            correlationId: req.headers['x-correlation-id'] as string,
            component: 'TelemetryRoutes',
            operation: 'errorHandler'
        });
        res.status(500).json({ error: 'Internal server error' });
    });

    return router;
}

export default configureTelemetryRoutes;