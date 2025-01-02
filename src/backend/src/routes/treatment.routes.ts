/**
 * @fileoverview Express router configuration for agricultural treatment operations
 * Implements secure, validated routes for precision treatment application with monitoring
 * @version 1.0.0
 */

import { Router } from 'express'; // v4.18.2
import { container } from 'tsyringe'; // v4.7.0
import cors from 'cors'; // v2.8.5
import helmet from 'helmet'; // v7.0.0
import rateLimit from 'express-rate-limit'; // v6.7.0
import { TreatmentController } from '../controllers/treatment.controller';
import { authenticateRequest, authorizeRole } from '../middleware/auth.middleware';
import { validateRequest, ValidationTarget } from '../middleware/validation.middleware';
import loggingMiddleware from '../middleware/logging.middleware';
import { z } from 'zod';

// Treatment request validation schema
const treatmentSchema = z.object({
    id: z.string().uuid(),
    missionId: z.string().uuid(),
    deviceId: z.string().uuid(),
    type: z.string(),
    status: z.enum(['CREATED', 'QUEUED', 'IN_PROGRESS', 'PAUSED', 'COMPLETED', 'FAILED', 'CANCELLED']),
    location: z.object({
        latitude: z.number().min(-90).max(90),
        longitude: z.number().min(-180).max(180),
        altitude: z.number().min(0).max(120)
    }),
    appliedAt: z.date(),
    quantity: z.number().positive(),
    coverage: z.object({
        type: z.string(),
        coordinates: z.array(z.array(z.number())),
        properties: z.record(z.unknown())
    }),
    parameters: z.record(z.unknown()),
    robotType: z.enum(['AERIAL_DRONE', 'GROUND_ROBOT']),
    concentration: z.number().optional(),
    weatherConditions: z.object({
        windSpeed: z.number(),
        temperature: z.number(),
        humidity: z.number()
    }).optional(),
    effectiveness: z.object({
        coverage: z.number(),
        uniformity: z.number(),
        wastage: z.number()
    }).optional()
});

// Rate limiting configuration
const rateLimiter = rateLimit({
    windowMs: 15 * 60 * 1000, // 15 minutes
    max: 100, // Limit each IP to 100 requests per window
    message: 'Too many requests from this IP, please try again later'
});

/**
 * Initializes and configures treatment operation routes
 * @returns Configured Express router for treatment endpoints
 */
const initializeTreatmentRoutes = (): Router => {
    const router = Router();
    const treatmentController = container.resolve(TreatmentController);

    // Apply security middleware
    router.use(helmet());
    router.use(cors({
        origin: process.env.ALLOWED_ORIGINS?.split(',') || '*',
        methods: ['GET', 'POST', 'PUT', 'DELETE'],
        allowedHeaders: ['Content-Type', 'Authorization'],
        maxAge: 86400 // 24 hours
    }));

    // Apply common middleware
    router.use(loggingMiddleware);
    router.use(rateLimiter);

    // Treatment creation endpoint
    router.post('/',
        authenticateRequest,
        authorizeRole(['operator', 'administrator']),
        validateRequest(treatmentSchema, ValidationTarget.BODY),
        treatmentController.createTreatment.bind(treatmentController)
    );

    // Treatment retrieval endpoint
    router.get('/:id',
        authenticateRequest,
        authorizeRole(['operator', 'administrator', 'analyst']),
        treatmentController.getTreatment.bind(treatmentController)
    );

    // Treatment update endpoint
    router.put('/:id',
        authenticateRequest,
        authorizeRole(['operator', 'administrator']),
        validateRequest(treatmentSchema, ValidationTarget.BODY),
        treatmentController.updateTreatment.bind(treatmentController)
    );

    // Treatment deletion endpoint
    router.delete('/:id',
        authenticateRequest,
        authorizeRole(['administrator']),
        treatmentController.deleteTreatment.bind(treatmentController)
    );

    // Mission treatments retrieval endpoint
    router.get('/missions/:missionId/treatments',
        authenticateRequest,
        authorizeRole(['operator', 'administrator', 'analyst']),
        treatmentController.getMissionTreatments.bind(treatmentController)
    );

    return router;
};

// Export configured router
export const treatmentRouter = initializeTreatmentRoutes();