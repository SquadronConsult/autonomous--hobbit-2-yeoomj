/**
 * @fileoverview End-to-end tests for telemetry data collection and processing
 * @version 1.0.0
 * @license MIT
 */

import { describe, test, expect, beforeAll, afterAll } from '@jest/globals';
import supertest from 'supertest';
import { v4 as uuidv4 } from 'uuid';
import { ITelemetry } from '../../src/interfaces/ITelemetry';
import { TelemetryType, TelemetryUnit } from '../../src/constants/telemetryTypes';
import { TelemetryService } from '../../src/services/telemetry.service';
import { metrics } from '../../src/utils/metrics';
import { logger } from '../../src/utils/logger';
import { pool } from '../../src/config/database';

// Test configuration
const TEST_TIMEOUT = 30000;
const PERFORMANCE_THRESHOLD = 100; // 100ms latency requirement
const BATCH_SIZE = 8; // Support for 8+ simultaneous feeds
const TEST_DEVICE_COUNT = 10;

// Test data generation utilities
const generateTestTelemetry = (deviceId: string, type: TelemetryType): Partial<ITelemetry> => ({
    deviceId,
    timestamp: new Date(),
    type,
    value: type === TelemetryType.BATTERY ? Math.random() * 100 :
           type === TelemetryType.ALTITUDE ? Math.random() * 1000 : 
           { lat: 45 + Math.random(), lon: -122 + Math.random() },
    unit: type === TelemetryType.BATTERY ? TelemetryUnit.PERCENT :
          type === TelemetryType.ALTITUDE ? TelemetryUnit.METERS :
          TelemetryUnit.METERS,
    metadata: {
        firmware: '1.0.0',
        sensor: 'primary',
        quality: 'high'
    }
});

describe('Telemetry E2E Tests', () => {
    let telemetryService: TelemetryService;
    const testDevices: string[] = [];

    beforeAll(async () => {
        jest.setTimeout(TEST_TIMEOUT);

        // Initialize test environment
        telemetryService = new TelemetryService();

        // Create test devices
        for (let i = 0; i < TEST_DEVICE_COUNT; i++) {
            testDevices.push(uuidv4());
        }

        // Clear existing test data
        await pool.query('DELETE FROM telemetry WHERE device_id = ANY($1)', [testDevices]);

        logger.info('Test environment initialized', {
            correlationId: 'test-init',
            component: 'TelemetryTest',
            operation: 'beforeAll'
        });
    });

    afterAll(async () => {
        // Cleanup test data
        await pool.query('DELETE FROM telemetry WHERE device_id = ANY($1)', [testDevices]);
        await pool.end();

        logger.info('Test environment cleaned up', {
            correlationId: 'test-cleanup',
            component: 'TelemetryTest',
            operation: 'afterAll'
        });
    });

    test('should create telemetry records with performance validation', async () => {
        const testData: Partial<ITelemetry>[] = [];
        
        // Generate batch test data
        for (const deviceId of testDevices.slice(0, BATCH_SIZE)) {
            testData.push(
                generateTestTelemetry(deviceId, TelemetryType.LOCATION),
                generateTestTelemetry(deviceId, TelemetryType.BATTERY),
                generateTestTelemetry(deviceId, TelemetryType.ALTITUDE)
            );
        }

        const startTime = Date.now();

        // Test concurrent creation
        const results = await Promise.all(
            testData.map(data => telemetryService.createTelemetry([data]))
        );

        const processingTime = Date.now() - startTime;

        // Validate performance
        expect(processingTime).toBeLessThan(PERFORMANCE_THRESHOLD);
        metrics.recordVideoLatency('telemetry-test', 'creation', processingTime / 1000);

        // Validate results
        results.flat().forEach(result => {
            expect(result).toHaveProperty('id');
            expect(result).toHaveProperty('timestamp');
            expect(testDevices).toContain(result.deviceId);
        });

        // Verify database persistence
        const dbResults = await telemetryService.getTelemetryByDevice(
            testDevices.slice(0, BATCH_SIZE),
            { limit: BATCH_SIZE * 3 }
        );

        expect(dbResults.length).toBe(testData.length);
    });

    test('should retrieve telemetry data with caching and pagination', async () => {
        const deviceId = testDevices[0];
        const testData = [
            generateTestTelemetry(deviceId, TelemetryType.LOCATION),
            generateTestTelemetry(deviceId, TelemetryType.BATTERY)
        ];

        // Create test data
        await telemetryService.createTelemetry(testData);

        // Test retrieval with different page sizes
        const pageSizes = [1, 2];
        for (const limit of pageSizes) {
            const startTime = Date.now();
            
            const results = await telemetryService.getTelemetryByDevice(
                [deviceId],
                { limit, offset: 0 }
            );

            const retrievalTime = Date.now() - startTime;

            // Validate performance
            expect(retrievalTime).toBeLessThan(PERFORMANCE_THRESHOLD);
            metrics.recordVideoLatency('telemetry-test', 'retrieval', retrievalTime / 1000);

            // Validate pagination
            expect(results.length).toBeLessThanOrEqual(limit);
            results.forEach(result => {
                expect(result.deviceId).toBe(deviceId);
            });
        }
    });

    test('should handle time range queries with aggregation', async () => {
        const deviceId = testDevices[1];
        const startTime = new Date(Date.now() - 3600000); // 1 hour ago
        const endTime = new Date();

        // Create test data with different timestamps
        const testData = Array(5).fill(null).map(() => ({
            ...generateTestTelemetry(deviceId, TelemetryType.BATTERY),
            timestamp: new Date(startTime.getTime() + Math.random() * 3600000)
        }));

        await telemetryService.createTelemetry(testData);

        // Test time range query with aggregation
        const results = await telemetryService.getTelemetryInTimeRange(
            startTime,
            endTime,
            { 
                aggregation: 'avg',
                timeInterval: '15 minutes'
            }
        );

        expect(results.length).toBeGreaterThan(0);
        results.forEach(result => {
            expect(result.timestamp).toBeInstanceOf(Date);
            expect(result.timestamp).toBeGreaterThanOrEqual(startTime);
            expect(result.timestamp).toBeLessThanOrEqual(endTime);
        });
    });

    test('should handle error scenarios gracefully', async () => {
        // Test invalid device ID
        await expect(
            telemetryService.getTelemetryByDevice(['invalid-id'])
        ).rejects.toThrow();

        // Test invalid time range
        const futureDate = new Date(Date.now() + 86400000);
        const results = await telemetryService.getTelemetryInTimeRange(
            futureDate,
            new Date()
        );
        expect(results.length).toBe(0);

        // Test invalid data creation
        const invalidData = {
            deviceId: testDevices[0],
            type: 'INVALID_TYPE' as TelemetryType,
            value: 'invalid'
        };
        await expect(
            telemetryService.createTelemetry([invalidData as Partial<ITelemetry>])
        ).rejects.toThrow();
    });
});