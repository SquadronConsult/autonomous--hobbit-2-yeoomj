/**
 * @fileoverview Express router configuration for agricultural robot device management endpoints
 * Implements secure, scalable, and real-time device fleet management with comprehensive
 * validation, monitoring, and error handling capabilities
 * @version 1.0.0
 */

import { Router } from 'express'; // v4.18.2
import rateLimit from 'express-rate-limit'; // v6.7.0
import compression from 'compression'; // v1.7.4
import { DeviceController } from '../controllers/device.controller';
import { authMiddleware } from '../middleware/auth.middleware';
import { DeviceValidator } from '../validators/device.validator';
import { logger } from '../utils/logger';

// Initialize router
const deviceRouter = Router();

// Configure rate limiters
const registrationLimiter = rateLimit({
    windowMs: 60 * 1000, // 1 minute
    max: 10, // 10 requests per minute for device registration
    message: 'Too many device registration attempts, please try again later'
});

const updateLimiter = rateLimit({
    windowMs: 60 * 1000,
    max: 100, // 100 requests per minute for updates
    message: 'Too many update requests, please try again later'
});

// Device registration endpoint
deviceRouter.post('/',
    registrationLimiter,
    compression(),
    authMiddleware.validateToken,
    authMiddleware.checkRole(['administrator', 'operator']),
    DeviceValidator.validateDeviceType,
    DeviceValidator.validateDeviceCapabilities,
    async (req, res, next) => {
        try {
            const result = await DeviceController.registerDevice(req.body);
            logger.info('Device registered successfully', {
                correlationId: `register-device-${result.id}`,
                component: 'device-routes',
                operation: 'register'
            });
            res.status(201).json(result);
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `register-device-error-${Date.now()}`,
                component: 'device-routes',
                operation: 'register'
            });
            next(error);
        }
    }
);

// Device status update endpoint
deviceRouter.patch('/:deviceId/status',
    updateLimiter,
    authMiddleware.validateToken,
    authMiddleware.checkRole(['administrator', 'operator']),
    DeviceValidator.validateDeviceId,
    async (req, res, next) => {
        try {
            const result = await DeviceController.updateDeviceStatus(
                req.params.deviceId,
                req.body.status
            );
            res.status(200).json(result);
        } catch (error) {
            next(error);
        }
    }
);

// Device location update endpoint
deviceRouter.patch('/:deviceId/location',
    updateLimiter,
    authMiddleware.validateToken,
    authMiddleware.checkRole(['administrator', 'operator']),
    DeviceValidator.validateDeviceId,
    DeviceValidator.validateDeviceLocation,
    async (req, res, next) => {
        try {
            const result = await DeviceController.updateDeviceLocation(
                req.params.deviceId,
                req.body.location
            );
            res.status(200).json(result);
        } catch (error) {
            next(error);
        }
    }
);

// Get device by ID endpoint
deviceRouter.get('/:deviceId',
    authMiddleware.validateToken,
    DeviceValidator.validateDeviceId,
    async (req, res, next) => {
        try {
            const result = await DeviceController.getDeviceById(req.params.deviceId);
            if (!result) {
                res.status(404).json({ message: 'Device not found' });
                return;
            }
            res.status(200).json(result);
        } catch (error) {
            next(error);
        }
    }
);

// Get active devices endpoint
deviceRouter.get('/',
    authMiddleware.validateToken,
    async (req, res, next) => {
        try {
            const result = await DeviceController.getActiveDevices();
            res.status(200).json(result);
        } catch (error) {
            next(error);
        }
    }
);

// Deactivate device endpoint
deviceRouter.delete('/:deviceId',
    authMiddleware.validateToken,
    authMiddleware.checkRole(['administrator']),
    DeviceValidator.validateDeviceId,
    async (req, res, next) => {
        try {
            await DeviceController.deactivateDevice(req.params.deviceId);
            res.status(204).send();
        } catch (error) {
            next(error);
        }
    }
);

// Batch device update endpoint
deviceRouter.post('/batch',
    compression(),
    authMiddleware.validateToken,
    authMiddleware.checkRole(['administrator']),
    DeviceValidator.validateBatchOperation,
    async (req, res, next) => {
        try {
            const result = await DeviceController.updateDeviceBatch(req.body);
            res.status(200).json(result);
        } catch (error) {
            next(error);
        }
    }
);

// Real-time device telemetry endpoint (SSE)
deviceRouter.get('/:deviceId/telemetry',
    authMiddleware.validateToken,
    DeviceValidator.validateDeviceId,
    DeviceValidator.validateTelemetryRequest,
    async (req, res, next) => {
        try {
            // Set SSE headers
            res.setHeader('Content-Type', 'text/event-stream');
            res.setHeader('Cache-Control', 'no-cache');
            res.setHeader('Connection', 'keep-alive');

            // Start telemetry stream
            const unsubscribe = await DeviceController.getDeviceTelemetry(
                req.params.deviceId,
                (data) => {
                    res.write(`data: ${JSON.stringify(data)}\n\n`);
                }
            );

            // Handle client disconnect
            req.on('close', () => {
                unsubscribe();
                logger.info('Telemetry connection closed', {
                    correlationId: `telemetry-close-${req.params.deviceId}`,
                    component: 'device-routes',
                    operation: 'telemetry'
                });
            });
        } catch (error) {
            next(error);
        }
    }
);

// API key validation for external integrations
deviceRouter.use('/external',
    authMiddleware.validateApiKey,
    deviceRouter
);

// Export router
export { deviceRouter };