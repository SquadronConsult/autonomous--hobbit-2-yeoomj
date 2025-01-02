/**
 * @fileoverview Comprehensive unit test suite for device controller
 * Tests CRUD operations, real-time status updates, location tracking,
 * fleet management, and security validation for agricultural robots
 * @version 1.0.0
 */

import { Request, Response, NextFunction } from 'express';
import { DeviceController } from '../../src/controllers/device.controller';
import { DeviceService } from '../../src/services/device.service';
import { DeviceValidation } from '../../src/utils/validation';
import { RobotType } from '../../src/constants/robotTypes';
import { ErrorCodes } from '../../src/constants/errorCodes';

// Mock implementations
jest.mock('../../src/services/device.service');
jest.mock('../../src/utils/validation');

describe('DeviceController', () => {
    let deviceController: DeviceController;
    let mockRequest: Partial<Request>;
    let mockResponse: Partial<Response>;
    let mockNext: NextFunction;

    beforeEach(() => {
        // Reset all mocks
        jest.clearAllMocks();

        // Initialize controller
        deviceController = new DeviceController();

        // Setup request mock
        mockRequest = {
            body: {},
            params: {},
            headers: {},
            ip: '127.0.0.1'
        };

        // Setup response mock
        mockResponse = {
            status: jest.fn().mockReturnThis(),
            json: jest.fn().mockReturnThis()
        };

        // Setup next function mock
        mockNext = jest.fn();
    });

    describe('createDevice', () => {
        const validDeviceData = {
            id: 'AGM-D-12345678',
            name: 'Test Drone',
            type: RobotType.AERIAL_DRONE,
            capabilities: ['SURVEILLANCE', 'MONITORING'],
            batteryLevel: 100,
            location: {
                latitude: 45.0,
                longitude: -75.0,
                altitude: 50,
                heading: 180
            }
        };

        it('should create a device successfully with valid data', async () => {
            // Arrange
            mockRequest.body = validDeviceData;
            mockRequest.headers.authorization = 'Bearer valid-token';
            
            (DeviceService.createDevice as jest.Mock).mockResolvedValue(validDeviceData);
            (DeviceValidation.validateDeviceData as jest.Mock).mockResolvedValue({ isValid: true, errors: [] });

            // Act
            await deviceController.createDevice(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(201);
            expect(mockResponse.json).toHaveBeenCalledWith(validDeviceData);
            expect(DeviceService.createDevice).toHaveBeenCalledWith(validDeviceData);
        });

        it('should reject device creation with invalid security token', async () => {
            // Arrange
            mockRequest.body = validDeviceData;
            mockRequest.headers.authorization = 'Bearer invalid-token';
            
            (DeviceValidation.validateDeviceData as jest.Mock).mockResolvedValue({
                isValid: false,
                errors: ['Invalid security token'],
                errorCode: ErrorCodes.AUTHENTICATION_ERROR
            });

            // Act
            await deviceController.createDevice(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(401);
            expect(mockResponse.json).toHaveBeenCalledWith({
                error: 'Authentication failed. Please check your credentials and try again.',
                details: ['Invalid security token']
            });
        });

        it('should reject device creation with invalid device data', async () => {
            // Arrange
            mockRequest.body = { ...validDeviceData, batteryLevel: 150 };
            mockRequest.headers.authorization = 'Bearer valid-token';
            
            (DeviceValidation.validateDeviceData as jest.Mock).mockResolvedValue({
                isValid: false,
                errors: ['Battery level must be between 0 and 100'],
                errorCode: ErrorCodes.VALIDATION_ERROR
            });

            // Act
            await deviceController.createDevice(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(400);
            expect(mockResponse.json).toHaveBeenCalledWith({
                error: 'Invalid input parameters. Please verify the request data and try again.',
                details: ['Battery level must be between 0 and 100']
            });
        });
    });

    describe('updateDeviceStatus', () => {
        const validStatusUpdate = {
            id: 'AGM-D-12345678',
            status: 'ACTIVE'
        };

        it('should update device status successfully', async () => {
            // Arrange
            mockRequest.params = { id: validStatusUpdate.id };
            mockRequest.body = { status: validStatusUpdate.status };
            
            (DeviceService.updateDeviceStatus as jest.Mock).mockResolvedValue(validStatusUpdate);
            (DeviceValidation.validateDeviceData as jest.Mock).mockResolvedValue({ isValid: true, errors: [] });

            // Act
            await deviceController.updateDeviceStatus(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(200);
            expect(mockResponse.json).toHaveBeenCalledWith(validStatusUpdate);
            expect(DeviceService.updateDeviceStatus).toHaveBeenCalledWith(
                validStatusUpdate.id,
                validStatusUpdate.status
            );
        });

        it('should reject status update for non-existent device', async () => {
            // Arrange
            mockRequest.params = { id: 'non-existent-id' };
            mockRequest.body = { status: 'ACTIVE' };
            
            (DeviceService.updateDeviceStatus as jest.Mock).mockRejectedValue(new Error('Device not found'));

            // Act
            await deviceController.updateDeviceStatus(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(404);
            expect(mockResponse.json).toHaveBeenCalledWith({
                error: 'The requested resource could not be found.'
            });
        });
    });

    describe('updateDeviceLocation', () => {
        const validLocationUpdate = {
            latitude: 45.0,
            longitude: -75.0,
            altitude: 50,
            heading: 180
        };

        it('should update device location successfully', async () => {
            // Arrange
            mockRequest.params = { id: 'AGM-D-12345678' };
            mockRequest.body = validLocationUpdate;
            
            (DeviceService.updateDeviceLocation as jest.Mock).mockResolvedValue({
                id: 'AGM-D-12345678',
                location: validLocationUpdate
            });
            (DeviceValidation.validateLocationData as jest.Mock).mockResolvedValue({ isValid: true, errors: [] });

            // Act
            await deviceController.updateDeviceLocation(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(200);
            expect(DeviceService.updateDeviceLocation).toHaveBeenCalledWith(
                'AGM-D-12345678',
                validLocationUpdate
            );
        });

        it('should reject invalid location data', async () => {
            // Arrange
            mockRequest.params = { id: 'AGM-D-12345678' };
            mockRequest.body = { ...validLocationUpdate, latitude: 100 };
            
            (DeviceValidation.validateLocationData as jest.Mock).mockResolvedValue({
                isValid: false,
                errors: ['Invalid latitude value'],
                errorCode: ErrorCodes.VALIDATION_ERROR
            });

            // Act
            await deviceController.updateDeviceLocation(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(400);
            expect(mockResponse.json).toHaveBeenCalledWith({
                error: 'Invalid input parameters. Please verify the request data and try again.',
                details: ['Invalid latitude value']
            });
        });
    });

    describe('getDevicesByType', () => {
        const devicesList = [
            {
                id: 'AGM-D-12345678',
                type: RobotType.AERIAL_DRONE,
                status: 'ACTIVE'
            },
            {
                id: 'AGM-D-87654321',
                type: RobotType.AERIAL_DRONE,
                status: 'IDLE'
            }
        ];

        it('should retrieve devices by type successfully', async () => {
            // Arrange
            mockRequest.query = { type: RobotType.AERIAL_DRONE, limit: '10', offset: '0' };
            
            (DeviceService.getDevicesByType as jest.Mock).mockResolvedValue(devicesList);

            // Act
            await deviceController.getDevicesByType(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(200);
            expect(mockResponse.json).toHaveBeenCalledWith(devicesList);
            expect(DeviceService.getDevicesByType).toHaveBeenCalledWith(
                RobotType.AERIAL_DRONE,
                10,
                0
            );
        });

        it('should handle invalid type parameter', async () => {
            // Arrange
            mockRequest.query = { type: 'INVALID_TYPE' };
            
            (DeviceService.getDevicesByType as jest.Mock).mockRejectedValue(new Error('Invalid device type'));

            // Act
            await deviceController.getDevicesByType(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(400);
            expect(mockResponse.json).toHaveBeenCalledWith({
                error: 'Invalid input parameters. Please verify the request data and try again.'
            });
        });
    });

    describe('validateDeviceOperation', () => {
        it('should validate device operation successfully', async () => {
            // Arrange
            mockRequest.params = { id: 'AGM-D-12345678' };
            mockRequest.body = { operation: 'START_MISSION' };
            
            (DeviceService.validateDeviceAccess as jest.Mock).mockResolvedValue({ isValid: true });

            // Act
            await deviceController.validateDeviceOperation(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(200);
            expect(mockResponse.json).toHaveBeenCalledWith({ isValid: true });
        });

        it('should reject unauthorized device operation', async () => {
            // Arrange
            mockRequest.params = { id: 'AGM-D-12345678' };
            mockRequest.body = { operation: 'START_MISSION' };
            
            (DeviceService.validateDeviceAccess as jest.Mock).mockResolvedValue({
                isValid: false,
                errorCode: ErrorCodes.AUTHORIZATION_ERROR
            });

            // Act
            await deviceController.validateDeviceOperation(mockRequest as Request, mockResponse as Response, mockNext);

            // Assert
            expect(mockResponse.status).toHaveBeenCalledWith(403);
            expect(mockResponse.json).toHaveBeenCalledWith({
                error: 'Insufficient permissions to perform this operation.'
            });
        });
    });
});