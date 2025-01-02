/**
 * @fileoverview Main Express application configuration for agricultural management system
 * Implements comprehensive security, monitoring, and performance optimizations
 * @version 1.0.0
 */

import express, { Application, Request, Response, NextFunction } from 'express'; // v4.18.2
import cors from 'cors'; // v2.8.5
import helmet from 'helmet'; // v7.0.0
import compression from 'compression'; // v1.7.4
import swaggerUi from 'swagger-ui-express'; // v4.6.3
import rateLimit from 'express-rate-limit'; // v6.7.0
import { analyticsRouter } from './routes/analytics.routes';
import { missionRouter } from './routes/mission.routes';
import errorHandler from './middleware/error.middleware';
import loggingMiddleware from './middleware/logging.middleware';
import { metrics } from './utils/metrics';
import { logger } from './utils/logger';
import { securityConfig } from './config/security';

// API version prefix
const API_VERSION = '/api/v1';

// CORS configuration with strict origin validation
const CORS_OPTIONS = {
    origin: process.env.ALLOWED_ORIGINS?.split(',') || [],
    methods: ['GET', 'POST', 'PUT', 'PATCH', 'DELETE'],
    allowedHeaders: ['Content-Type', 'Authorization', 'X-Request-ID'],
    credentials: true,
    maxAge: 86400, // 24 hours
};

// Rate limiting configuration
const RATE_LIMIT_OPTIONS = {
    windowMs: 15 * 60 * 1000, // 15 minutes
    max: 100, // Limit each IP to 100 requests per window
    standardHeaders: true,
    legacyHeaders: false
};

// Helmet security configuration with strict CSP
const HELMET_OPTIONS = {
    contentSecurityPolicy: {
        directives: {
            'default-src': ["'self'"],
            'script-src': ["'self'"],
            'style-src': ["'self'"],
            'img-src': ["'self'", 'data:', 'https:'],
            'connect-src': ["'self'"]
        }
    },
    crossOriginEmbedderPolicy: true,
    crossOriginOpenerPolicy: true,
    crossOriginResourcePolicy: true,
    dnsPrefetchControl: true,
    frameguard: true,
    hidePoweredBy: true,
    hsts: true,
    ieNoOpen: true,
    noSniff: true,
    referrerPolicy: true,
    xssFilter: true
};

// Initialize Express application
const app: Application = express();

/**
 * Configures Express application middleware stack with security and monitoring
 * @param app Express application instance
 */
function configureMiddleware(app: Application): void {
    // Enable trust proxy for secure headers behind reverse proxy
    app.enable('trust proxy');

    // Apply security middleware
    app.use(helmet(HELMET_OPTIONS));
    app.use(cors(CORS_OPTIONS));

    // Request parsing and compression
    app.use(express.json({ limit: '10mb' }));
    app.use(express.urlencoded({ extended: true, limit: '10mb' }));
    app.use(compression());

    // Add correlation ID to all requests
    app.use((req: Request, res: Response, next: NextFunction) => {
        req.headers['x-correlation-id'] = req.headers['x-correlation-id'] || 
            `${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
        next();
    });

    // Apply rate limiting
    app.use(rateLimit(RATE_LIMIT_OPTIONS));

    // Request logging and monitoring
    app.use(loggingMiddleware);

    // Initialize metrics collection
    metrics.initializeMetrics(app);
}

/**
 * Configures API routes with versioning and monitoring
 * @param app Express application instance
 */
function configureRoutes(app: Application): void {
    // Health check endpoint
    app.get('/health', (req: Request, res: Response) => {
        res.status(200).json({
            status: 'healthy',
            timestamp: new Date().toISOString()
        });
    });

    // API documentation
    app.use('/api-docs', swaggerUi.serve, swaggerUi.setup(undefined, {
        swaggerOptions: {
            url: '/swagger.json'
        }
    }));

    // API routes with version prefix
    app.use(`${API_VERSION}/analytics`, analyticsRouter);
    app.use(`${API_VERSION}/missions`, missionRouter);

    // Error handling
    app.use(errorHandler);
}

/**
 * Handles graceful application shutdown
 * @param app Express application instance
 * @param server HTTP server instance
 */
async function gracefulShutdown(app: Application, server: any): Promise<void> {
    logger.info('Initiating graceful shutdown', {
        component: 'Application',
        operation: 'shutdown'
    });

    // Stop accepting new requests
    server.close(() => {
        logger.info('HTTP server closed', {
            component: 'Application',
            operation: 'shutdown'
        });
    });

    try {
        // Close database connections and cleanup
        // Add cleanup code here

        logger.info('Graceful shutdown completed', {
            component: 'Application',
            operation: 'shutdown'
        });
        process.exit(0);
    } catch (error) {
        logger.error(error as Error, {
            component: 'Application',
            operation: 'shutdown'
        });
        process.exit(1);
    }
}

// Configure application
configureMiddleware(app);
configureRoutes(app);

// Handle shutdown signals
process.on('SIGTERM', () => gracefulShutdown(app, null));
process.on('SIGINT', () => gracefulShutdown(app, null));

// Export configured application
export default app;