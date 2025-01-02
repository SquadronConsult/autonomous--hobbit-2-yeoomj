/**
 * @fileoverview Express router configuration for analytics endpoints
 * Implements real-time video analytics, pest detection, and treatment coverage data routes
 * with comprehensive security, validation, and performance monitoring
 * @version 1.0.0
 */

import { Router } from 'express'; // v4.18.2
import helmet from 'helmet'; // v7.0.0
import { AnalyticsController } from '../controllers/analytics.controller';
import {
    authenticateRequest,
    authorizeRole
} from '../middleware/auth.middleware';
import {
    validateRequest,
    validateArraySize
} from '../middleware/validation.middleware';
import { analyticsValidator } from '../validators/analytics.validator';

// Base path for analytics routes
const BASE_PATH = '/analytics';

// Rate limiting configurations
const RATE_LIMITS = {
    CREATE: '100/minute',
    READ: '1000/minute',
    STREAM: 'unlimited'
};

// Role-based access control configurations
const ROLES = {
    CREATE: ['analyst'],
    READ: ['operator', 'analyst', 'administrator'],
    STREAM: ['operator', 'analyst'],
    AGGREGATE: ['analyst', 'administrator']
};

// Confidence threshold for pest detection (95% requirement)
const CONFIDENCE_THRESHOLD = 0.95;

// Initialize router
const router = Router();

// Apply security headers to all routes
router.use(helmet());

// POST /analytics - Create new analytics record
router.post(
    BASE_PATH,
    helmet(),
    authenticateRequest,
    authorizeRole(ROLES.CREATE),
    validateArraySize(),
    validateRequest(analyticsValidator.validateAnalyticsData),
    validateRequest(analyticsValidator.validateConfidenceThreshold(CONFIDENCE_THRESHOLD)),
    async (req, res, next) => {
        try {
            const controller = new AnalyticsController();
            const result = await controller.createAnalytics(req.body);
            res.status(201).json(result);
        } catch (error) {
            next(error);
        }
    }
);

// GET /analytics/stream - Stream real-time analytics
router.get(
    `${BASE_PATH}/stream`,
    helmet(),
    authenticateRequest,
    authorizeRole(ROLES.STREAM),
    async (req, res, next) => {
        try {
            const controller = new AnalyticsController();
            await controller.streamAnalytics(req, res);
        } catch (error) {
            next(error);
        }
    }
);

// GET /analytics/mission/:missionId - Get analytics by mission
router.get(
    `${BASE_PATH}/mission/:missionId`,
    helmet(),
    authenticateRequest,
    authorizeRole(ROLES.READ),
    async (req, res, next) => {
        try {
            const controller = new AnalyticsController();
            const result = await controller.getAnalyticsByMission(req.params.missionId);
            res.status(200).json(result);
        } catch (error) {
            next(error);
        }
    }
);

// GET /analytics/timerange - Get analytics within time range
router.get(
    `${BASE_PATH}/timerange`,
    helmet(),
    authenticateRequest,
    authorizeRole(ROLES.READ),
    async (req, res, next) => {
        try {
            const controller = new AnalyticsController();
            const result = await controller.getAnalyticsByTimeRange(
                new Date(req.query.from as string),
                new Date(req.query.to as string)
            );
            res.status(200).json(result);
        } catch (error) {
            next(error);
        }
    }
);

// GET /analytics/aggregate - Get aggregated analytics metrics
router.get(
    `${BASE_PATH}/aggregate`,
    helmet(),
    authenticateRequest,
    authorizeRole(ROLES.AGGREGATE),
    async (req, res, next) => {
        try {
            const controller = new AnalyticsController();
            const result = await controller.getAggregatedAnalytics();
            res.status(200).json(result);
        } catch (error) {
            next(error);
        }
    }
);

// Error handling middleware
router.use((error: Error, req: any, res: any, next: any) => {
    console.error(error);
    res.status(500).json({
        error: 'Internal Server Error',
        message: error.message
    });
});

export default router;