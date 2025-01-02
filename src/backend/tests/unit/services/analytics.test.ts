import { describe, test, expect, jest, beforeEach, afterEach } from '@jest/globals'; // v29.0.0
import { Counter, Gauge, Histogram } from 'prom-client'; // v14.0.0
import { Point } from '@turf/helpers'; // v6.5.0

import { AnalyticsService } from '../../../src/services/analytics.service';
import { Analytics } from '../../../src/models/Analytics';
import { AnalyticsType } from '../../../src/interfaces/IAnalytics';
import { ErrorCodes } from '../../../src/constants/errorCodes';
import { validateGeolocation } from '../../../src/utils/validation';

// Mock dependencies
jest.mock('../../../src/models/Analytics');
jest.mock('../../../src/utils/validation');
jest.mock('redis');

describe('AnalyticsService', () => {
    let analyticsService: AnalyticsService;
    let mockMetrics: {
        videoProcessingLatency: Histogram;
        pestDetectionAccuracy: Gauge;
        analyticsCounter: Counter;
    };

    // Test data fixtures
    const mockLocation: Point = {
        type: 'Point',
        coordinates: [-122.4194, 37.7749]
    };

    const mockAnalyticsData = {
        id: 'test-analytics-1',
        missionId: 'test-mission-1',
        deviceId: 'test-device-1',
        timestamp: new Date(),
        location: mockLocation,
        type: AnalyticsType.PEST_DETECTION,
        confidence: 0.96,
        detections: [
            {
                type: 'aphid',
                confidence: 0.97,
                boundingBox: { x: 100, y: 100, width: 50, height: 50 }
            }
        ],
        metadata: {}
    };

    beforeEach(() => {
        // Reset all mocks
        jest.clearAllMocks();

        // Initialize metrics
        mockMetrics = {
            videoProcessingLatency: new Histogram({
                name: 'video_processing_latency',
                help: 'Video processing latency in milliseconds'
            }),
            pestDetectionAccuracy: new Gauge({
                name: 'pest_detection_accuracy',
                help: 'Pest detection accuracy percentage'
            }),
            analyticsCounter: new Counter({
                name: 'analytics_processed_total',
                help: 'Total number of analytics records processed'
            })
        };

        // Initialize service
        analyticsService = new AnalyticsService();
    });

    afterEach(() => {
        jest.resetAllMocks();
    });

    describe('Real-time Video Processing', () => {
        test('should process video analytics within 100ms latency requirement', async () => {
            const startTime = process.hrtime();

            await analyticsService.processRealTimeAnalytics(mockAnalyticsData);

            const [seconds, nanoseconds] = process.hrtime(startTime);
            const duration = seconds * 1000 + nanoseconds / 1e6;

            expect(duration).toBeLessThan(100);
        });

        test('should handle concurrent video streams efficiently', async () => {
            const streams = Array(8).fill(mockAnalyticsData);
            const processPromises = streams.map(stream => 
                analyticsService.processRealTimeAnalytics(stream)
            );

            await expect(Promise.all(processPromises)).resolves.not.toThrow();
        });

        test('should collect accurate processing metrics', async () => {
            const mockCollectMetrics = jest.spyOn(mockMetrics.videoProcessingLatency, 'observe');

            await analyticsService.processRealTimeAnalytics(mockAnalyticsData);

            expect(mockCollectMetrics).toHaveBeenCalled();
            const latencyValue = mockCollectMetrics.mock.calls[0][0];
            expect(latencyValue).toBeLessThan(0.1); // 100ms in seconds
        });
    });

    describe('Pest Detection Analysis', () => {
        test('should maintain 95% detection accuracy', async () => {
            const mockValidateAnalytics = jest.spyOn(Analytics.prototype, 'save');
            mockValidateAnalytics.mockResolvedValue(undefined);

            await analyticsService.createAnalytics({
                ...mockAnalyticsData,
                confidence: 0.96
            });

            expect(mockValidateAnalytics).toHaveBeenCalled();
        });

        test('should reject detections below confidence threshold', async () => {
            await expect(analyticsService.createAnalytics({
                ...mockAnalyticsData,
                confidence: 0.94
            })).rejects.toThrow();
        });

        test('should validate detection bounding boxes', async () => {
            const invalidDetection = {
                ...mockAnalyticsData,
                detections: [{
                    type: 'aphid',
                    confidence: 0.97,
                    boundingBox: { x: -1, y: -1, width: 0, height: 0 }
                }]
            };

            await expect(analyticsService.createAnalytics(invalidDetection))
                .rejects.toThrow();
        });
    });

    describe('Data Management', () => {
        test('should persist analytics data successfully', async () => {
            const mockSave = jest.spyOn(Analytics.prototype, 'save');
            mockSave.mockResolvedValue(undefined);

            await analyticsService.createAnalytics(mockAnalyticsData);

            expect(mockSave).toHaveBeenCalledWith();
        });

        test('should retrieve analytics by mission ID', async () => {
            const mockFind = jest.spyOn(Analytics, 'findByMissionId');
            mockFind.mockResolvedValue([mockAnalyticsData]);

            const result = await analyticsService.getAnalyticsByMission('test-mission-1');

            expect(result).toHaveLength(1);
            expect(result[0]).toMatchObject(mockAnalyticsData);
        });

        test('should handle data retrieval errors gracefully', async () => {
            const mockFind = jest.spyOn(Analytics, 'findByMissionId');
            mockFind.mockRejectedValue(new Error(ErrorCodes.TELEMETRY_PROCESSING_ERROR.toString()));

            await expect(analyticsService.getAnalyticsByMission('invalid-id'))
                .rejects.toThrow(ErrorCodes.TELEMETRY_PROCESSING_ERROR.toString());
        });

        test('should validate location data', async () => {
            const mockValidateLocation = jest.spyOn(validateGeolocation, 'validateGeolocation');
            mockValidateLocation.mockReturnValue(true);

            await analyticsService.createAnalytics(mockAnalyticsData);

            expect(mockValidateLocation).toHaveBeenCalledWith(mockLocation);
        });
    });

    describe('Performance Metrics', () => {
        test('should track video processing latency', async () => {
            const metrics = await analyticsService.getMetrics();
            
            expect(metrics).toHaveProperty('processedTotal');
            expect(metrics).toHaveProperty('latencyHistogram');
            expect(metrics).toHaveProperty('cacheHitRatio');
        });

        test('should maintain cache hit ratio metrics', async () => {
            // First call - cache miss
            await analyticsService.getAnalyticsByMission('test-mission-1');
            
            // Second call - cache hit
            await analyticsService.getAnalyticsByMission('test-mission-1');

            const metrics = await analyticsService.getMetrics();
            expect(metrics.cacheHitRatio).toBeGreaterThan(0);
        });
    });
});