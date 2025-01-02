/**
 * @fileoverview Mission management routes implementation with comprehensive security,
 * monitoring, and error handling for the agricultural management system.
 * @version 1.0.0
 */

import { Router } from 'express';
import compression from 'compression';
import helmet from 'helmet';
import rateLimit from 'express-rate-limit';
import { MissionController } from '../controllers/mission.controller';
import { authenticateRequest, authorizeRole } from '../middleware/auth.middleware';
import { validateMissionData, validateMissionStatus } from '../validators/mission.validator';
import errorHandler from '../middleware/error.middleware';
import { logger } from '../utils/logger';

// Constants
const MISSION_BASE_PATH = '/api/v1/missions';
const RATE_LIMIT_WINDOW = 60000; // 1 minute
const RATE_LIMIT_MAX_REQUESTS = 100;
const CACHE_TTL = 300; // 5 minutes
const REQUEST_TIMEOUT = 5000; // 5 seconds

// Configure rate limiting
const missionRateLimiter = rateLimit({
    windowMs: RATE_LIMIT_WINDOW,
    max: RATE_LIMIT_MAX_REQUESTS,
    message: 'Too many requests from this IP, please try again later',
    standardHeaders: true,
    legacyHeaders: false
});

// Create router instance
const missionRouter = Router();

// Apply global middleware
missionRouter.use(helmet({
    contentSecurityPolicy: {
        directives: {
            defaultSrc: ["'self'"],
            scriptSrc: ["'self'"],
            styleSrc: ["'self'"],
            imgSrc: ["'self'", 'data:', 'blob:'],
            connectSrc: ["'self'"]
        }
    },
    referrerPolicy: { policy: 'strict-origin-when-cross-origin' },
    noSniff: true,
    xssFilter: true,
    hsts: {
        maxAge: 31536000,
        includeSubDomains: true,
        preload: true
    }
}));
missionRouter.use(compression());
missionRouter.use(authenticateRequest);
missionRouter.use(missionRateLimiter);

// Health check endpoint
missionRouter.get('/health', (req, res) => {
    res.status(200).json({ status: 'healthy', timestamp: new Date().toISOString() });
});

// Create new mission
missionRouter.post(MISSION_BASE_PATH,
    authorizeRole(['operator', 'administrator'], ['mission:create']),
    validateMissionData,
    async (req, res, next) => {
        try {
            const correlationId = `create-mission-${Date.now()}`;
            logger.info('Creating new mission', {
                correlationId,
                component: 'MissionRoutes',
                operation: 'createMission'
            });
            await MissionController.createMission(req, res, next);
        } catch (error) {
            next(error);
        }
    }
);

// Get mission by ID
missionRouter.get(`${MISSION_BASE_PATH}/:id`,
    authorizeRole(['analyst', 'operator', 'administrator'], ['mission:read']),
    async (req, res, next) => {
        try {
            const correlationId = `get-mission-${req.params.id}`;
            logger.info('Retrieving mission', {
                correlationId,
                component: 'MissionRoutes',
                operation: 'getMission'
            });
            res.set('Cache-Control', `private, max-age=${CACHE_TTL}`);
            await MissionController.getMission(req, res, next);
        } catch (error) {
            next(error);
        }
    }
);

// Get active missions with pagination
missionRouter.get(`${MISSION_BASE_PATH}/active`,
    authorizeRole(['analyst', 'operator', 'administrator'], ['mission:read']),
    async (req, res, next) => {
        try {
            const correlationId = `get-active-missions-${Date.now()}`;
            logger.info('Retrieving active missions', {
                correlationId,
                component: 'MissionRoutes',
                operation: 'getActiveMissions'
            });
            await MissionController.getActiveMissions(req, res, next);
        } catch (error) {
            next(error);
        }
    }
);

// Update mission status
missionRouter.patch(`${MISSION_BASE_PATH}/:id/status`,
    authorizeRole(['operator', 'administrator'], ['mission:update']),
    validateMissionStatus,
    async (req, res, next) => {
        try {
            const correlationId = `update-status-${req.params.id}`;
            logger.info('Updating mission status', {
                correlationId,
                component: 'MissionRoutes',
                operation: 'updateMissionStatus'
            });
            await MissionController.updateMissionStatus(req, res, next);
        } catch (error) {
            next(error);
        }
    }
);

// Assign device to mission
missionRouter.patch(`${MISSION_BASE_PATH}/:id/devices`,
    authorizeRole(['operator', 'administrator'], ['mission:update', 'device:assign']),
    async (req, res, next) => {
        try {
            const correlationId = `assign-device-${req.params.id}`;
            logger.info('Assigning device to mission', {
                correlationId,
                component: 'MissionRoutes',
                operation: 'assignDevice'
            });
            await MissionController.assignDevice(req, res, next);
        } catch (error) {
            next(error);
        }
    }
);

// Apply error handling middleware
missionRouter.use(errorHandler);

export { missionRouter };