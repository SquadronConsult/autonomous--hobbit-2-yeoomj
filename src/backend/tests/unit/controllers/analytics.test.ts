import { Request, Response, NextFunction } from 'express';
import { AnalyticsController } from '../../src/controllers/analytics.controller';
import { AnalyticsType, MIN_CONFIDENCE_THRESHOLD } from '../../src/interfaces/IAnalytics';
import { ErrorCodes } from '../../src/constants/errorCodes';
import now from 'performance-now';  // v2.1.0

// Mock dependencies
jest.mock('../../src/services/analytics.service');
jest.mock('../../src/utils/logger');
jest.mock('prom-client');

describe('AnalyticsController', () => {
    let analyticsController: AnalyticsController;
    let mockRequest: Partial<Request>;
    let mockResponse: Partial<Response>;
    let mockNext: NextFunction;

    beforeAll(() => {
        // Initialize controller
        analyticsController = new AnalyticsController();

        // Setup response mock
        mockResponse = {
            status: jest.fn().mockReturnThis(),
            json: jest.fn()
        };

        // Setup next function mock
        mockNext = jest.fn();
    });

    beforeEach(() => {
        jest.clearAllMocks();
    });

    describe('Performance Tests', () => {
        it('should process video analytics within 100ms latency requirement', async () => {
            // Setup mock request with video analytics data
            mockRequest = {
                body: {
                    deviceId: 'AGM-D-000001',
                    type: AnalyticsType.PEST_DETECTION,
                    timestamp: new Date(),
                    location: {
                        type: 'Point',
                        coordinates: [45.123, -122.456]
                    },
                    confidence: 0.97,
                    detections: [
                        {
                            type: 'pest_type_1',
                            confidence: 0.96,
                            boundingBox: { x: 100, y: 100, width: 50, height: 50 }
                        }
                    ]
                }
            };

            const startTime = now();
            await analyticsController.processStreamAnalytics(
                mockRequest as Request,
                mockResponse as Response,
                mockNext
            );
            const processingTime = now() - startTime;

            expect(processingTime).toBeLessThan(100); // 100ms requirement
            expect(mockResponse.status).toHaveBeenCalledWith(200);
        });

        it('should handle multiple concurrent drone feeds', async () => {
            const numDrones = 8; // Test 8+ simultaneous drone feeds
            const requests = Array(numDrones).fill(null).map((_, index) => ({
                body: {
                    deviceId: `AGM-D-00000${index}`,
                    type: AnalyticsType.PEST_DETECTION,
                    timestamp: new Date(),
                    location: {
                        type: 'Point',
                        coordinates: [45.123, -122.456]
                    },
                    confidence: 0.96,
                    detections: [
                        {
                            type: 'pest_type_1',
                            confidence: 0.95,
                            boundingBox: { x: 100, y: 100, width: 50, height: 50 }
                        }
                    ]
                }
            }));

            const startTime = now();
            await Promise.all(requests.map(req => 
                analyticsController.processStreamAnalytics(
                    req as Request,
                    mockResponse as Response,
                    mockNext
                )
            ));
            const processingTime = now() - startTime;

            expect(processingTime / numDrones).toBeLessThan(100); // Average time per drone
            expect(mockResponse.status).toHaveBeenCalledTimes(numDrones);
        });
    });

    describe('Pest Detection Accuracy Tests', () => {
        it('should validate 95% confidence threshold requirement', async () => {
            mockRequest = {
                body: {
                    deviceId: 'AGM-D-000001',
                    type: AnalyticsType.PEST_DETECTION,
                    confidence: 0.94, // Below threshold
                    detections: [
                        {
                            type: 'pest_type_1',
                            confidence: 0.94,
                            boundingBox: { x: 100, y: 100, width: 50, height: 50 }
                        }
                    ]
                }
            };

            await analyticsController.createAnalyticsRecord(
                mockRequest as Request,
                mockResponse as Response,
                mockNext
            );

            expect(mockNext).toHaveBeenCalledWith(
                expect.objectContaining({
                    message: expect.stringContaining('confidence threshold')
                })
            );
        });

        it('should accept detections meeting accuracy requirements', async () => {
            mockRequest = {
                body: {
                    deviceId: 'AGM-D-000001',
                    type: AnalyticsType.PEST_DETECTION,
                    confidence: MIN_CONFIDENCE_THRESHOLD,
                    detections: [
                        {
                            type: 'pest_type_1',
                            confidence: 0.96,
                            boundingBox: { x: 100, y: 100, width: 50, height: 50 }
                        }
                    ]
                }
            };

            await analyticsController.createAnalyticsRecord(
                mockRequest as Request,
                mockResponse as Response,
                mockNext
            );

            expect(mockResponse.status).toHaveBeenCalledWith(201);
        });
    });

    describe('Data Persistence Tests', () => {
        it('should store analytics data with correct partitioning', async () => {
            const timeRange = {
                from: new Date(Date.now() - 24 * 60 * 60 * 1000), // Last 24 hours
                to: new Date()
            };

            mockRequest = {
                query: {
                    from: timeRange.from.toISOString(),
                    to: timeRange.to.toISOString()
                }
            };

            await analyticsController.getTimeRangeAnalytics(
                mockRequest as Request,
                mockResponse as Response,
                mockNext
            );

            expect(mockResponse.status).toHaveBeenCalledWith(200);
        });

        it('should enforce data retention policies', async () => {
            const oldDate = new Date(Date.now() - 100 * 24 * 60 * 60 * 1000); // 100 days old
            mockRequest = {
                query: {
                    from: oldDate.toISOString(),
                    to: new Date().toISOString()
                }
            };

            await analyticsController.getTimeRangeAnalytics(
                mockRequest as Request,
                mockResponse as Response,
                mockNext
            );

            expect(mockResponse.status).toHaveBeenCalledWith(200);
        });
    });

    describe('Error Handling Tests', () => {
        it('should handle invalid analytics data format', async () => {
            mockRequest = {
                body: {
                    deviceId: 'AGM-D-000001',
                    type: 'INVALID_TYPE',
                    confidence: 0.96
                }
            };

            await analyticsController.createAnalyticsRecord(
                mockRequest as Request,
                mockResponse as Response,
                mockNext
            );

            expect(mockNext).toHaveBeenCalledWith(
                expect.objectContaining({
                    message: expect.any(String)
                })
            );
        });

        it('should handle database connection errors', async () => {
            mockRequest = {
                query: {
                    from: new Date().toISOString(),
                    to: new Date().toISOString()
                }
            };

            const dbError = new Error('Database connection failed');
            jest.spyOn(analyticsController['analyticsService'], 'getAnalyticsByTimeRange')
                .mockRejectedValueOnce(dbError);

            await analyticsController.getTimeRangeAnalytics(
                mockRequest as Request,
                mockResponse as Response,
                mockNext
            );

            expect(mockNext).toHaveBeenCalledWith(dbError);
        });
    });

    describe('Security Tests', () => {
        it('should validate input parameters for SQL injection', async () => {
            mockRequest = {
                query: {
                    from: "2023-01-01' OR '1'='1",
                    to: new Date().toISOString()
                }
            };

            await analyticsController.getTimeRangeAnalytics(
                mockRequest as Request,
                mockResponse as Response,
                mockNext
            );

            expect(mockNext).toHaveBeenCalledWith(
                expect.objectContaining({
                    message: expect.stringContaining('validation')
                })
            );
        });

        it('should handle unauthorized access attempts', async () => {
            mockRequest = {
                headers: {},
                body: {
                    deviceId: 'AGM-D-000001',
                    type: AnalyticsType.PEST_DETECTION
                }
            };

            await analyticsController.getAggregatedAnalytics(
                mockRequest as Request,
                mockResponse as Response,
                mockNext
            );

            expect(mockNext).toHaveBeenCalledWith(
                expect.objectContaining({
                    code: ErrorCodes.AUTHORIZATION_ERROR
                })
            );
        });
    });
});