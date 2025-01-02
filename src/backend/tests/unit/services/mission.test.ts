import { jest, describe, beforeAll, afterAll, beforeEach, it, expect } from '@jest/globals'; // v29.5.0
import { MongoMemoryServer } from 'mongodb-memory-server'; // v8.12.0
import mongoose from 'mongoose'; // v6.9.1
import { GenericContainer, StartedTestContainer } from 'testcontainers'; // v9.1.1
import { Registry, collectDefaultMetrics } from 'prom-client'; // v14.2.0

import { MissionService } from '../../../src/services/mission.service';
import { IMission } from '../../../src/interfaces/IMission';
import { MissionStatus } from '../../../src/constants/missionStatus';
import { RobotType } from '../../../src/constants/robotTypes';
import { ErrorCodes } from '../../../src/constants/errorCodes';
import { db } from '../../../src/utils/database';
import { logger } from '../../../src/utils/logger';

// Test metrics registry
const registry = new Registry();
collectDefaultMetrics({ register: registry });

describe('MissionService Unit Tests', () => {
    let mongoServer: MongoMemoryServer;
    let timescaleContainer: StartedTestContainer;
    let missionService: MissionService;

    // Sample valid mission data
    const validMissionData: IMission = {
        id: 'AGM-M-000001',
        name: 'Test Surveillance Mission',
        type: 'surveillance',
        status: MissionStatus.CREATED,
        assignedDevices: [{
            deviceId: 'AGM-D-000001',
            type: RobotType.AERIAL_DRONE
        }],
        coverageArea: {
            type: 'Polygon',
            coordinates: [[
                [10.0, 20.0],
                [10.1, 20.0],
                [10.1, 20.1],
                [10.0, 20.1],
                [10.0, 20.0]
            ]],
            properties: {}
        },
        startTime: new Date(Date.now() + 3600000), // 1 hour from now
        endTime: new Date(Date.now() + 7200000), // 2 hours from now
        progress: 0,
        parameters: {
            altitude: 50,
            speed: 5,
            overlap: 30
        },
        metadata: {
            createdAt: new Date(),
            updatedAt: new Date(),
            createdBy: 'test-user',
            version: 1,
            priority: 1,
            tags: ['test'],
            dependencies: [],
            offline: false
        }
    } as IMission;

    beforeAll(async () => {
        // Start MongoDB memory server
        mongoServer = await MongoMemoryServer.create();
        await mongoose.connect(mongoServer.getUri());

        // Start TimescaleDB container
        timescaleContainer = await new GenericContainer('timescale/timescaledb:latest')
            .withExposedPorts(5432)
            .withEnvironment({
                POSTGRES_DB: 'test_db',
                POSTGRES_USER: 'test_user',
                POSTGRES_PASSWORD: 'test_password'
            })
            .start();

        // Initialize database connection
        await db.initializeDatabase();

        // Initialize mission service
        missionService = new MissionService();
    });

    afterAll(async () => {
        // Cleanup resources
        await mongoose.disconnect();
        await mongoServer.stop();
        await timescaleContainer.stop();
        await db.closeDatabase();
        registry.clear();
    });

    beforeEach(async () => {
        // Clear collections before each test
        await mongoose.connection.db.dropDatabase();
        await db.executeQuery('TRUNCATE missions CASCADE;', []);
    });

    describe('Mission Creation', () => {
        it('should create a valid mission successfully', async () => {
            const mission = await missionService.createMission(validMissionData);
            
            expect(mission).toBeDefined();
            expect(mission.id).toBe(validMissionData.id);
            expect(mission.status).toBe(MissionStatus.CREATED);
            
            // Verify metrics
            const metrics = await registry.getMetricAsJSON('active_missions_total');
            expect(metrics).toBeDefined();
        });

        it('should reject mission with invalid parameters', async () => {
            const invalidMission = {
                ...validMissionData,
                parameters: {
                    ...validMissionData.parameters,
                    altitude: 150 // Exceeds maximum allowed altitude
                }
            };

            await expect(missionService.createMission(invalidMission))
                .rejects.toThrow('Mission validation failed');
        });

        it('should handle concurrent mission creation', async () => {
            const missions = await Promise.all([
                missionService.createMission({ ...validMissionData, id: 'AGM-M-000002' }),
                missionService.createMission({ ...validMissionData, id: 'AGM-M-000003' })
            ]);

            expect(missions).toHaveLength(2);
            expect(missions[0].id).not.toBe(missions[1].id);
        });
    });

    describe('Mission Status Management', () => {
        it('should update mission status with valid transition', async () => {
            const mission = await missionService.createMission(validMissionData);
            const updatedMission = await missionService.updateMissionStatus(
                mission.id,
                MissionStatus.QUEUED
            );

            expect(updatedMission.status).toBe(MissionStatus.QUEUED);
        });

        it('should reject invalid status transitions', async () => {
            const mission = await missionService.createMission(validMissionData);
            
            await expect(missionService.updateMissionStatus(
                mission.id,
                MissionStatus.COMPLETED
            )).rejects.toThrow('Invalid status transition');
        });

        it('should handle status updates with retries', async () => {
            jest.spyOn(db, 'executeQuery').mockImplementationOnce(() => {
                throw new Error('Temporary database error');
            });

            const mission = await missionService.createMission(validMissionData);
            const updatedMission = await missionService.updateMissionStatus(
                mission.id,
                MissionStatus.QUEUED
            );

            expect(updatedMission.status).toBe(MissionStatus.QUEUED);
        });
    });

    describe('Mission Progress Tracking', () => {
        it('should update mission progress correctly', async () => {
            const mission = await missionService.createMission(validMissionData);
            const updatedMission = await missionService.updateMissionProgress(
                mission.id,
                50
            );

            expect(updatedMission.progress).toBe(50);
        });

        it('should complete mission at 100% progress', async () => {
            const mission = await missionService.createMission(validMissionData);
            await missionService.updateMissionStatus(mission.id, MissionStatus.IN_PROGRESS);
            
            const completedMission = await missionService.updateMissionProgress(
                mission.id,
                100
            );

            expect(completedMission.status).toBe(MissionStatus.COMPLETED);
            expect(completedMission.progress).toBe(100);
        });

        it('should reject invalid progress values', async () => {
            const mission = await missionService.createMission(validMissionData);
            
            await expect(missionService.updateMissionProgress(
                mission.id,
                150
            )).rejects.toThrow('Invalid progress value');
        });
    });

    describe('Active Mission Management', () => {
        it('should retrieve active missions with pagination', async () => {
            // Create multiple missions
            await Promise.all([
                missionService.createMission({ ...validMissionData, id: 'AGM-M-000004' }),
                missionService.createMission({ ...validMissionData, id: 'AGM-M-000005' }),
                missionService.createMission({ ...validMissionData, id: 'AGM-M-000006' })
            ]);

            const { missions, total } = await missionService.getActiveMissions({
                limit: 2,
                offset: 0
            });

            expect(missions).toHaveLength(2);
            expect(total).toBe(3);
        });

        it('should filter active missions by type', async () => {
            await missionService.createMission({
                ...validMissionData,
                id: 'AGM-M-000007',
                type: 'treatment'
            });

            const { missions } = await missionService.getActiveMissions({
                type: 'treatment'
            });

            expect(missions).toHaveLength(1);
            expect(missions[0].type).toBe('treatment');
        });
    });

    describe('Error Handling', () => {
        it('should handle database connection errors', async () => {
            jest.spyOn(db, 'executeQuery').mockImplementationOnce(() => {
                throw new Error('Connection error');
            });

            await expect(missionService.createMission(validMissionData))
                .rejects.toThrow('Connection error');
        });

        it('should handle mission not found errors', async () => {
            await expect(missionService.updateMissionStatus(
                'non-existent-id',
                MissionStatus.QUEUED
            )).rejects.toThrow('Mission not found');
        });

        it('should log errors with correct context', async () => {
            const logSpy = jest.spyOn(logger, 'error');
            
            try {
                await missionService.updateMissionStatus(
                    'non-existent-id',
                    MissionStatus.QUEUED
                );
            } catch (error) {
                expect(logSpy).toHaveBeenCalledWith(
                    expect.any(Error),
                    expect.objectContaining({
                        component: 'MissionService',
                        operation: 'updateMissionStatus'
                    })
                );
            }
        });
    });

    describe('Performance Monitoring', () => {
        it('should track mission operation metrics', async () => {
            await missionService.createMission(validMissionData);
            
            const metrics = await registry.getMetricAsJSON('mission_operations_total');
            expect(metrics).toBeDefined();
            expect(metrics[0].values[0].value).toBeGreaterThan(0);
        });

        it('should measure operation latency', async () => {
            const startTime = Date.now();
            await missionService.createMission(validMissionData);
            const duration = Date.now() - startTime;

            const metrics = await registry.getMetricAsJSON('mission_operation_duration_seconds');
            expect(metrics).toBeDefined();
            expect(duration).toBeLessThan(1000); // Should complete within 1 second
        });
    });
});