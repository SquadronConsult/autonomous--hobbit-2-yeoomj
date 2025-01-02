/**
 * @fileoverview Express controller for agricultural robot device management
 * Implements comprehensive CRUD operations, real-time monitoring, and security validation
 * @version 1.0.0
 */

import { Request, Response, NextFunction } from 'express';
import { RateLimit } from 'rate-limiter-flexible';
import * as promClient from 'prom-client';
import winston from 'winston';
import { DeviceService } from '../services/device.service';
import { DeviceValidator } from '../validators/device.validator';
import { RobotType, RobotStatus } from '../constants/robotTypes';
import { ErrorCodes, ErrorMessages, ErrorCodeToHttpStatus } from '../constants/errorCodes';
import { IDevice } from '../interfaces/IDevice';

// Initialize metrics
const deviceMetrics = new promClient.Gauge({
    name: 'device_requests_total',
    help: 'Total number of device API requests',
    labelNames: ['method', 'endpoint', 'status', 'error']
});

// Initialize rate limiter
const deviceRateLimiter = new RateLimit({
    points: 100,
    duration: 60,
    blockDuration: 60
});

// Initialize logger
const deviceLogger = winston.createLogger({
    level: 'info',
    format: winston.format.json(),
    defaultMeta: { service: 'device-controller' }
});

/**
 * Device controller implementing comprehensive fleet management capabilities
 */
export class DeviceController {
    private validator: DeviceValidator;

    constructor() {
        this.validator = new DeviceValidator();
    }

    /**
     * Creates a new agricultural robot device with enhanced validation
     */
    public async createDevice(req: Request, res: Response, next: NextFunction): Promise<Response> {
        const startTime = Date.now();
        try {
            // Rate limiting check
            await deviceRateLimiter.consume(req.ip);

            // Security token validation
            const securityValidation = await this.validator.validateSecurityToken(
                req.headers.authorization as string,
                req.body.id
            );
            if (!securityValidation.isValid) {
                deviceMetrics.inc({
                    method: 'POST',
                    endpoint: '/devices',
                    status: 'error',
                    error: 'AUTH_ERROR'
                });
                return res.status(401).json({
                    error: ErrorMessages[ErrorCodes.AUTHENTICATION_ERROR],
                    details: securityValidation.errors
                });
            }

            // Device data validation
            const deviceValidation = await this.validator.validateDeviceType(req.body);
            if (!deviceValidation.isValid) {
                deviceMetrics.inc({
                    method: 'POST',
                    endpoint: '/devices',
                    status: 'error',
                    error: 'VALIDATION_ERROR'
                });
                return res.status(400).json({
                    error: ErrorMessages[ErrorCodes.VALIDATION_ERROR],
                    details: deviceValidation.errors
                });
            }

            // Create device
            const device = await DeviceService.createDevice(req.body);

            deviceMetrics.inc({
                method: 'POST',
                endpoint: '/devices',
                status: 'success',
                error: ''
            });

            deviceLogger.info('Device created successfully', {
                correlationId: `create-device-${device.id}`,
                duration: Date.now() - startTime,
                deviceType: device.type
            });

            return res.status(201).json(device);
        } catch (error) {
            deviceMetrics.inc({
                method: 'POST',
                endpoint: '/devices',
                status: 'error',
                error: error.name
            });

            deviceLogger.error('Error creating device', {
                correlationId: `create-device-error-${Date.now()}`,
                error: error.message,
                stack: error.stack
            });

            return res.status(500).json({
                error: ErrorMessages[ErrorCodes.INTERNAL_SERVER_ERROR]
            });
        }
    }

    /**
     * Updates device status with validation and monitoring
     */
    public async updateDeviceStatus(req: Request, res: Response, next: NextFunction): Promise<Response> {
        const startTime = Date.now();
        try {
            const { id } = req.params;
            const { status } = req.body;

            // Validate device ID
            const idValidation = await this.validator.validateDeviceId(id);
            if (!idValidation.isValid) {
                return res.status(400).json({
                    error: ErrorMessages[ErrorCodes.VALIDATION_ERROR],
                    details: idValidation.errors
                });
            }

            // Update device status
            const device = await DeviceService.updateDeviceStatus(id, status);

            deviceMetrics.inc({
                method: 'PATCH',
                endpoint: '/devices/:id/status',
                status: 'success',
                error: ''
            });

            deviceLogger.info('Device status updated', {
                correlationId: `update-status-${id}`,
                duration: Date.now() - startTime,
                newStatus: status
            });

            return res.status(200).json(device);
        } catch (error) {
            deviceMetrics.inc({
                method: 'PATCH',
                endpoint: '/devices/:id/status',
                status: 'error',
                error: error.name
            });

            deviceLogger.error('Error updating device status', {
                correlationId: `update-status-error-${Date.now()}`,
                error: error.message,
                stack: error.stack
            });

            return res.status(500).json({
                error: ErrorMessages[ErrorCodes.INTERNAL_SERVER_ERROR]
            });
        }
    }

    /**
     * Establishes SSE connection for real-time device updates
     */
    public async subscribeToDeviceUpdates(req: Request, res: Response, next: NextFunction): Promise<void> {
        try {
            // Validate security token
            const securityValidation = await this.validator.validateSecurityToken(
                req.headers.authorization as string,
                req.query.deviceId as string
            );
            if (!securityValidation.isValid) {
                res.status(401).json({
                    error: ErrorMessages[ErrorCodes.AUTHENTICATION_ERROR],
                    details: securityValidation.errors
                });
                return;
            }

            // Set SSE headers
            res.writeHead(200, {
                'Content-Type': 'text/event-stream',
                'Cache-Control': 'no-cache',
                'Connection': 'keep-alive'
            });

            // Subscribe to device updates
            const unsubscribe = await DeviceService.subscribeToDeviceUpdates(
                req.query.deviceId as string,
                (update: any) => {
                    res.write(`data: ${JSON.stringify(update)}\n\n`);
                }
            );

            // Handle client disconnect
            req.on('close', () => {
                unsubscribe();
                deviceLogger.info('SSE connection closed', {
                    correlationId: `sse-close-${req.query.deviceId}`,
                    deviceId: req.query.deviceId
                });
            });
        } catch (error) {
            deviceLogger.error('Error in SSE connection', {
                correlationId: `sse-error-${Date.now()}`,
                error: error.message,
                stack: error.stack
            });
            res.status(500).json({
                error: ErrorMessages[ErrorCodes.INTERNAL_SERVER_ERROR]
            });
        }
    }

    /**
     * Retrieves devices by type with pagination
     */
    public async getDevicesByType(req: Request, res: Response, next: NextFunction): Promise<Response> {
        try {
            const { type, limit, offset } = req.query;

            // Validate device type
            const typeValidation = await this.validator.validateDeviceType({ type } as IDevice);
            if (!typeValidation.isValid) {
                return res.status(400).json({
                    error: ErrorMessages[ErrorCodes.VALIDATION_ERROR],
                    details: typeValidation.errors
                });
            }

            const devices = await DeviceService.getDevicesByType(
                type as RobotType,
                Number(limit) || 10,
                Number(offset) || 0
            );

            deviceMetrics.inc({
                method: 'GET',
                endpoint: '/devices',
                status: 'success',
                error: ''
            });

            return res.status(200).json(devices);
        } catch (error) {
            deviceMetrics.inc({
                method: 'GET',
                endpoint: '/devices',
                status: 'error',
                error: error.name
            });

            return res.status(500).json({
                error: ErrorMessages[ErrorCodes.INTERNAL_SERVER_ERROR]
            });
        }
    }
}

export const deviceController = new DeviceController();