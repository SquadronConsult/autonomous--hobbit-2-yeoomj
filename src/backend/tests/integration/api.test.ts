/**
 * @fileoverview Integration tests for agricultural management system REST API
 * Implements comprehensive testing of API endpoints, security, performance, and monitoring
 * @version 1.0.0
 */

import supertest from 'supertest'; // v6.3.3
import { describe, test, expect, beforeAll, afterAll } from '@jest/globals'; // v29.5.0
import { GenericContainer, StartedTestContainer } from 'testcontainers'; // v9.8.0
import { check, sleep } from 'k6'; // v0.45.0
import pactum from 'pactum'; // v3.5.0
import app from '../../src/app';
import { MissionStatus } from '../../src/constants/missionStatus';
import { RobotType } from '../../src/constants/robotTypes';
import { ErrorCodes } from '../../src/constants/errorCodes';
import { metrics } from '../../src/utils/metrics';

// Test environment configuration
interface TestEnvironment {
    dbContainer: StartedTestContainer;
    authToken: string;
    testMissionId: string;
}

let testEnv: TestEnvironment;
const request = supertest(app);

/**
 * Sets up test environment with containerized dependencies
 */
async function setupTestEnvironment(): Promise<TestEnvironment> {
    // Start PostgreSQL container
    const dbContainer = await new GenericContainer('timescale/timescaledb:latest-pg15')
        .withExposedPorts(5432)
        .withEnvironment({
            POSTGRES_DB: 'test_db',
            POSTGRES_USER: 'test_user',
            POSTGRES_PASSWORD: 'test_password'
        })
        .start();

    // Generate test auth token
    const authToken = await generateTestToken('test-admin', ['administrator']);

    // Create test mission
    const testMissionId = await createTestMission();

    return { dbContainer, authToken, testMissionId };
}

/**
 * Cleans up test environment and resources
 */
async function cleanupTestEnvironment(): Promise<void> {
    if (testEnv?.dbContainer) {
        await testEnv.dbContainer.stop();
    }
}

// Setup and teardown hooks
beforeAll(async () => {
    testEnv = await setupTestEnvironment();
});

afterAll(async () => {
    await cleanupTestEnvironment();
});

describe('API Security Tests', () => {
    test('should require authentication for protected endpoints', async () => {
        const response = await request.get('/api/v1/missions');
        expect(response.status).toBe(401);
        expect(response.body.code).toBe(ErrorCodes.AUTHENTICATION_ERROR);
    });

    test('should validate JWT tokens', async () => {
        const response = await request
            .get('/api/v1/missions')
            .set('Authorization', 'Bearer invalid-token');
        expect(response.status).toBe(401);
    });

    test('should enforce role-based access control', async () => {
        const analyticToken = await generateTestToken('test-analyst', ['analyst']);
        const response = await request
            .post('/api/v1/missions')
            .set('Authorization', `Bearer ${analyticToken}`);
        expect(response.status).toBe(403);
    });

    test('should validate request rate limiting', async () => {
        const promises = Array(150).fill(0).map(() => 
            request.get('/api/v1/missions')
                .set('Authorization', `Bearer ${testEnv.authToken}`)
        );
        const responses = await Promise.all(promises);
        const tooManyRequests = responses.filter(r => r.status === 429);
        expect(tooManyRequests.length).toBeGreaterThan(0);
    });
});

describe('Mission Management API Tests', () => {
    test('should create new mission with valid data', async () => {
        const missionData = {
            name: 'Test Mission',
            type: 'surveillance',
            assignedDevices: [{
                deviceId: 'AGM-D-000001',
                type: RobotType.AERIAL_DRONE
            }],
            coverageArea: {
                type: 'Polygon',
                coordinates: [[[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]]
            },
            parameters: {
                altitude: 50,
                speed: 5
            },
            startTime: new Date(Date.now() + 3600000)
        };

        const response = await request
            .post('/api/v1/missions')
            .set('Authorization', `Bearer ${testEnv.authToken}`)
            .send(missionData);

        expect(response.status).toBe(201);
        expect(response.body).toHaveProperty('id');
        expect(response.body.status).toBe(MissionStatus.CREATED);
    });

    test('should retrieve mission by ID', async () => {
        const response = await request
            .get(`/api/v1/missions/${testEnv.testMissionId}`)
            .set('Authorization', `Bearer ${testEnv.authToken}`);

        expect(response.status).toBe(200);
        expect(response.body.id).toBe(testEnv.testMissionId);
    });

    test('should update mission status', async () => {
        const response = await request
            .patch(`/api/v1/missions/${testEnv.testMissionId}/status`)
            .set('Authorization', `Bearer ${testEnv.authToken}`)
            .send({ status: MissionStatus.QUEUED });

        expect(response.status).toBe(200);
        expect(response.body.status).toBe(MissionStatus.QUEUED);
    });
});

describe('Analytics API Tests', () => {
    test('should process real-time analytics stream', async () => {
        const analyticsData = {
            deviceId: 'AGM-D-000001',
            timestamp: new Date(),
            location: {
                type: 'Point',
                coordinates: [0, 0]
            },
            detections: [{
                type: 'pest',
                confidence: 0.96,
                boundingBox: { x: 100, y: 100, width: 50, height: 50 }
            }]
        };

        const response = await request
            .post('/api/v1/analytics/stream')
            .set('Authorization', `Bearer ${testEnv.authToken}`)
            .send(analyticsData);

        expect(response.status).toBe(200);
    });

    test('should enforce confidence threshold', async () => {
        const analyticsData = {
            deviceId: 'AGM-D-000001',
            timestamp: new Date(),
            location: {
                type: 'Point',
                coordinates: [0, 0]
            },
            detections: [{
                type: 'pest',
                confidence: 0.94, // Below 95% threshold
                boundingBox: { x: 100, y: 100, width: 50, height: 50 }
            }]
        };

        const response = await request
            .post('/api/v1/analytics/stream')
            .set('Authorization', `Bearer ${testEnv.authToken}`)
            .send(analyticsData);

        expect(response.status).toBe(400);
    });
});

describe('Performance Tests', () => {
    test('should meet latency requirements', async () => {
        const startTime = process.hrtime();
        
        await request
            .get('/api/v1/analytics/stream')
            .set('Authorization', `Bearer ${testEnv.authToken}`);

        const [seconds, nanoseconds] = process.hrtime(startTime);
        const latencyMs = (seconds * 1000) + (nanoseconds / 1000000);
        
        expect(latencyMs).toBeLessThan(100); // 100ms requirement
    });

    test('should handle concurrent requests', async () => {
        const concurrentRequests = 8; // 8+ simultaneous drone feeds
        const promises = Array(concurrentRequests).fill(0).map(() => 
            request
                .get('/api/v1/analytics/stream')
                .set('Authorization', `Bearer ${testEnv.authToken}`)
        );

        const responses = await Promise.all(promises);
        const successfulResponses = responses.filter(r => r.status === 200);
        
        expect(successfulResponses.length).toBe(concurrentRequests);
    });
});

describe('Monitoring Integration Tests', () => {
    test('should track metrics for API requests', async () => {
        const initialMetrics = await metrics.getMetrics();
        
        await request
            .get('/api/v1/missions')
            .set('Authorization', `Bearer ${testEnv.authToken}`);

        const updatedMetrics = await metrics.getMetrics();
        expect(updatedMetrics.requestCount).toBeGreaterThan(initialMetrics.requestCount);
    });

    test('should log security events', async () => {
        const response = await request
            .get('/api/v1/missions')
            .set('Authorization', 'Bearer invalid-token');

        const securityLogs = await getSecurityLogs();
        expect(securityLogs).toContain('Authentication failed');
    });
});

// Helper functions
async function generateTestToken(userId: string, roles: string[]): Promise<string> {
    // Implementation omitted for brevity
    return 'test-token';
}

async function createTestMission(): Promise<string> {
    // Implementation omitted for brevity
    return 'AGM-M-000001';
}

async function getSecurityLogs(): Promise<string[]> {
    // Implementation omitted for brevity
    return [];
}