/**
 * @fileoverview End-to-end test suite for mission management API endpoints
 * @version 1.0.0
 * 
 * Comprehensive test coverage for mission lifecycle management including:
 * - Mission creation and validation
 * - Status updates and transitions
 * - Device assignment and coordination
 * - Progress tracking and completion verification
 */

import { describe, test, expect, beforeAll, afterAll } from '@jest/globals'; // v29.5.0
import supertest from 'supertest'; // v6.3.3
import { StatusCodes } from 'http-status-codes'; // v2.2.0
import { IMission } from '../../src/interfaces/IMission';
import { MissionStatus } from '../../src/constants/missionStatus';
import { RobotType } from '../../src/constants/robotTypes';
import { app } from '../../src/app';

const request = supertest(app);

// Test data constants
const TEST_COVERAGE_AREA = {
  type: 'Polygon',
  coordinates: [[[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]],
  properties: { zoneId: 'A1', terrain: 'flat' }
};

const TEST_MISSION_DATA = {
  name: 'Test Survey Mission',
  description: 'Automated crop survey mission',
  type: 'SURVEY',
  coverageArea: TEST_COVERAGE_AREA,
  parameters: {
    altitude: 50,
    speed: 5,
    overlap: 30,
    resolution: 'HIGH'
  },
  metadata: {
    tags: ['survey', 'automated'],
    priority: 1,
    offline: true
  }
};

// Test context to maintain state between tests
interface TestContext {
  authToken: string;
  testMissionId: string;
  testDevices: Array<{ id: string; type: RobotType }>;
}

const ctx: TestContext = {
  authToken: '',
  testMissionId: '',
  testDevices: []
};

describe('Mission Management API E2E Tests', () => {
  beforeAll(async () => {
    // Initialize test environment
    try {
      // Setup authentication
      const authResponse = await request
        .post('/api/v1/auth/login')
        .send({ username: process.env.TEST_USER, password: process.env.TEST_PASSWORD });
      ctx.authToken = authResponse.body.token;

      // Initialize test devices
      const deviceResponses = await Promise.all([
        request
          .post('/api/v1/devices')
          .set('Authorization', `Bearer ${ctx.authToken}`)
          .send({ type: RobotType.AERIAL_DRONE, status: 'IDLE' }),
        request
          .post('/api/v1/devices')
          .set('Authorization', `Bearer ${ctx.authToken}`)
          .send({ type: RobotType.AERIAL_DRONE, status: 'IDLE' }),
        request
          .post('/api/v1/devices')
          .set('Authorization', `Bearer ${ctx.authToken}`)
          .send({ type: RobotType.GROUND_ROBOT, status: 'IDLE' })
      ]);

      ctx.testDevices = deviceResponses.map(response => ({
        id: response.body.id,
        type: response.body.type
      }));
    } catch (error) {
      console.error('Test setup failed:', error);
      throw error;
    }
  });

  afterAll(async () => {
    // Cleanup test environment
    try {
      // Remove test devices
      await Promise.all(
        ctx.testDevices.map(device =>
          request
            .delete(`/api/v1/devices/${device.id}`)
            .set('Authorization', `Bearer ${ctx.authToken}`)
        )
      );

      // Remove test mission if exists
      if (ctx.testMissionId) {
        await request
          .delete(`/api/v1/missions/${ctx.testMissionId}`)
          .set('Authorization', `Bearer ${ctx.authToken}`);
      }
    } catch (error) {
      console.error('Test cleanup failed:', error);
      throw error;
    }
  });

  test('Should create new mission with valid data', async () => {
    const response = await request
      .post('/api/v1/missions')
      .set('Authorization', `Bearer ${ctx.authToken}`)
      .send(TEST_MISSION_DATA);

    expect(response.status).toBe(StatusCodes.CREATED);
    expect(response.body).toMatchObject({
      name: TEST_MISSION_DATA.name,
      status: MissionStatus.CREATED,
      coverageArea: TEST_COVERAGE_AREA
    });

    // Store mission ID for subsequent tests
    ctx.testMissionId = response.body.id;
  });

  test('Should reject mission creation with invalid coverage area', async () => {
    const invalidMission = {
      ...TEST_MISSION_DATA,
      coverageArea: {
        type: 'Polygon',
        coordinates: [[0, 0], [1, 0]] // Invalid polygon
      }
    };

    const response = await request
      .post('/api/v1/missions')
      .set('Authorization', `Bearer ${ctx.authToken}`)
      .send(invalidMission);

    expect(response.status).toBe(StatusCodes.BAD_REQUEST);
    expect(response.body.error).toContain('Invalid coverage area');
  });

  test('Should assign devices to mission', async () => {
    const response = await request
      .post(`/api/v1/missions/${ctx.testMissionId}/devices`)
      .set('Authorization', `Bearer ${ctx.authToken}`)
      .send({ deviceIds: ctx.testDevices.map(d => d.id) });

    expect(response.status).toBe(StatusCodes.OK);
    expect(response.body.assignedDevices).toHaveLength(ctx.testDevices.length);
    expect(response.body.status).toBe(MissionStatus.QUEUED);
  });

  test('Should update mission status with valid transitions', async () => {
    // Test status transition to IN_PROGRESS
    const startResponse = await request
      .patch(`/api/v1/missions/${ctx.testMissionId}/status`)
      .set('Authorization', `Bearer ${ctx.authToken}`)
      .send({ status: MissionStatus.IN_PROGRESS });

    expect(startResponse.status).toBe(StatusCodes.OK);
    expect(startResponse.body.status).toBe(MissionStatus.IN_PROGRESS);

    // Test progress updates
    const progressResponse = await request
      .patch(`/api/v1/missions/${ctx.testMissionId}/progress`)
      .set('Authorization', `Bearer ${ctx.authToken}`)
      .send({ progress: 50 });

    expect(progressResponse.status).toBe(StatusCodes.OK);
    expect(progressResponse.body.progress).toBe(50);
  });

  test('Should reject invalid status transitions', async () => {
    // Attempt to transition directly from IN_PROGRESS to CREATED
    const response = await request
      .patch(`/api/v1/missions/${ctx.testMissionId}/status`)
      .set('Authorization', `Bearer ${ctx.authToken}`)
      .send({ status: MissionStatus.CREATED });

    expect(response.status).toBe(StatusCodes.BAD_REQUEST);
    expect(response.body.error).toContain('Invalid status transition');
  });

  test('Should complete mission successfully', async () => {
    // Update progress to 100%
    await request
      .patch(`/api/v1/missions/${ctx.testMissionId}/progress`)
      .set('Authorization', `Bearer ${ctx.authToken}`)
      .send({ progress: 100 });

    // Complete the mission
    const response = await request
      .patch(`/api/v1/missions/${ctx.testMissionId}/status`)
      .set('Authorization', `Bearer ${ctx.authToken}`)
      .send({ status: MissionStatus.COMPLETED });

    expect(response.status).toBe(StatusCodes.OK);
    expect(response.body.status).toBe(MissionStatus.COMPLETED);
    expect(response.body.endTime).toBeTruthy();
  });

  test('Should handle concurrent mission updates correctly', async () => {
    // Create a new test mission
    const missionResponse = await request
      .post('/api/v1/missions')
      .set('Authorization', `Bearer ${ctx.authToken}`)
      .send(TEST_MISSION_DATA);

    const missionId = missionResponse.body.id;

    // Attempt concurrent status updates
    const updates = Array(5).fill(null).map(() =>
      request
        .patch(`/api/v1/missions/${missionId}/status`)
        .set('Authorization', `Bearer ${ctx.authToken}`)
        .send({ status: MissionStatus.IN_PROGRESS })
    );

    const results = await Promise.all(updates);
    
    // Verify only one update succeeded
    const successfulUpdates = results.filter(r => r.status === StatusCodes.OK);
    expect(successfulUpdates).toHaveLength(1);
  });
});