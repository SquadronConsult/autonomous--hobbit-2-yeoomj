import { describe, it, beforeAll, afterAll, expect } from '@jest/globals';
import request from 'supertest'; // v6.3.3
import { v4 as uuidv4 } from 'uuid'; // v9.0.0
import { ITreatment } from '../../src/interfaces/ITreatment';
import { TreatmentService } from '../../src/services/treatment.service';
import { MissionStatus } from '../../src/constants/missionStatus';
import { RobotType } from '../../src/constants/robotTypes';
import { ErrorCodes } from '../../src/constants/errorCodes';
import { pool } from '../../src/config/database';

// Test constants
const API_BASE_URL = '/api/v1/treatments';
const TEST_MISSION_ID = uuidv4();
const TEST_DEVICE_ID = uuidv4();
const CHEMICAL_USAGE_BASELINE = 10; // liters per hectare
const OPTIMIZATION_THRESHOLD = 30; // 30% reduction target
const COVERAGE_AREA_THRESHOLD = 95; // 95% minimum coverage requirement

// Test data setup
const mockTreatment: ITreatment = {
    id: uuidv4(),
    missionId: TEST_MISSION_ID,
    deviceId: TEST_DEVICE_ID,
    type: 'PESTICIDE',
    status: MissionStatus.CREATED,
    location: {
        latitude: 45.5231,
        longitude: -122.6765,
        altitude: 20
    },
    appliedAt: new Date(),
    quantity: CHEMICAL_USAGE_BASELINE,
    coverage: {
        type: 'Polygon',
        coordinates: [[
            [-122.6765, 45.5231],
            [-122.6785, 45.5231],
            [-122.6785, 45.5251],
            [-122.6765, 45.5251],
            [-122.6765, 45.5231]
        ]]
    },
    parameters: {
        concentration: 100,
        speed: 5,
        altitude: 20
    },
    robotType: RobotType.AERIAL_DRONE,
    weatherConditions: {
        windSpeed: 5,
        temperature: 20,
        humidity: 60
    },
    effectiveness: {
        coverage: 0,
        uniformity: 0,
        wastage: 0
    }
};

describe('Treatment Operations E2E Tests', () => {
    let treatmentService: TreatmentService;

    beforeAll(async () => {
        // Initialize test database
        await setupTestDatabase();
        treatmentService = new TreatmentService();
    });

    afterAll(async () => {
        // Cleanup test data
        await cleanupTestDatabase();
        await pool.end();
    });

    describe('Treatment Creation with Optimization', () => {
        it('should create treatment with optimized chemical usage', async () => {
            const response = await request(API_BASE_URL)
                .post('/')
                .send(mockTreatment)
                .expect(201);

            const createdTreatment = response.body as ITreatment;
            
            // Verify chemical usage optimization
            const reductionPercentage = ((CHEMICAL_USAGE_BASELINE - createdTreatment.quantity) / CHEMICAL_USAGE_BASELINE) * 100;
            expect(reductionPercentage).toBeGreaterThanOrEqual(OPTIMIZATION_THRESHOLD);
            
            // Verify treatment effectiveness metrics
            expect(createdTreatment.effectiveness.coverage).toBeGreaterThanOrEqual(COVERAGE_AREA_THRESHOLD);
            expect(createdTreatment.effectiveness.uniformity).toBeGreaterThanOrEqual(90);
            expect(createdTreatment.effectiveness.wastage).toBeLessThanOrEqual(10);
        });

        it('should validate environmental conditions before treatment', async () => {
            const invalidTreatment = {
                ...mockTreatment,
                weatherConditions: {
                    windSpeed: 20, // Exceeds safe limit
                    temperature: 20,
                    humidity: 60
                }
            };

            await request(API_BASE_URL)
                .post('/')
                .send(invalidTreatment)
                .expect(400)
                .expect(res => {
                    expect(res.body.error.code).toBe(ErrorCodes.TREATMENT_PLANNING_ERROR);
                    expect(res.body.error.message).toContain('Unsafe environmental conditions');
                });
        });
    });

    describe('Treatment Coverage Validation', () => {
        it('should validate coverage area geometry', async () => {
            const invalidCoverage = {
                ...mockTreatment,
                coverage: {
                    type: 'Polygon',
                    coordinates: [[
                        [-122.6765, 45.5231],
                        [-122.6785, 45.5231],
                        [-122.6765, 45.5231] // Invalid polygon (not closed)
                    ]]
                }
            };

            await request(API_BASE_URL)
                .post('/')
                .send(invalidCoverage)
                .expect(400)
                .expect(res => {
                    expect(res.body.error.code).toBe(ErrorCodes.VALIDATION_ERROR);
                    expect(res.body.error.message).toContain('Invalid coverage area geometry');
                });
        });

        it('should optimize coverage pattern for efficiency', async () => {
            const response = await request(API_BASE_URL)
                .post('/')
                .send(mockTreatment)
                .expect(201);

            const createdTreatment = response.body as ITreatment;
            expect(createdTreatment.effectiveness.coverage).toBeGreaterThanOrEqual(COVERAGE_AREA_THRESHOLD);
        });
    });

    describe('Automated Intervention Flow', () => {
        it('should execute complete intervention workflow', async () => {
            // Step 1: Create initial treatment plan
            const createResponse = await request(API_BASE_URL)
                .post('/')
                .send(mockTreatment)
                .expect(201);

            const treatmentId = createResponse.body.id;

            // Step 2: Simulate pest detection event
            const detectionData = {
                treatmentId,
                location: mockTreatment.location,
                severity: 'HIGH',
                pestType: 'APHIDS'
            };

            await request(API_BASE_URL)
                .post(`/${treatmentId}/detect`)
                .send(detectionData)
                .expect(200);

            // Step 3: Verify intervention plan generation
            const planResponse = await request(API_BASE_URL)
                .get(`/${treatmentId}/plan`)
                .expect(200);

            expect(planResponse.body.optimizationMetrics).toBeDefined();
            expect(planResponse.body.interventionStrategy).toBeDefined();

            // Step 4: Execute intervention
            const executionResponse = await request(API_BASE_URL)
                .post(`/${treatmentId}/execute`)
                .expect(200);

            const finalTreatment = executionResponse.body as ITreatment;
            
            // Verify optimization results
            expect(finalTreatment.status).toBe(MissionStatus.COMPLETED);
            expect(finalTreatment.effectiveness.coverage).toBeGreaterThanOrEqual(COVERAGE_AREA_THRESHOLD);
            expect(finalTreatment.quantity).toBeLessThan(mockTreatment.quantity);
        });
    });
});

async function setupTestDatabase(): Promise<void> {
    const client = await pool.connect();
    try {
        await client.query('BEGIN');

        // Create test mission
        await client.query(`
            INSERT INTO missions (id, status, type)
            VALUES ($1, $2, $3)
        `, [TEST_MISSION_ID, MissionStatus.CREATED, 'TREATMENT']);

        // Create test device
        await client.query(`
            INSERT INTO devices (id, type, status)
            VALUES ($1, $2, $3)
        `, [TEST_DEVICE_ID, RobotType.AERIAL_DRONE, 'ACTIVE']);

        await client.query('COMMIT');
    } catch (error) {
        await client.query('ROLLBACK');
        throw error;
    } finally {
        client.release();
    }
}

async function cleanupTestDatabase(): Promise<void> {
    const client = await pool.connect();
    try {
        await client.query('BEGIN');

        // Clean up test data
        await client.query('DELETE FROM treatments WHERE mission_id = $1', [TEST_MISSION_ID]);
        await client.query('DELETE FROM missions WHERE id = $1', [TEST_MISSION_ID]);
        await client.query('DELETE FROM devices WHERE id = $1', [TEST_DEVICE_ID]);

        await client.query('COMMIT');
    } catch (error) {
        await client.query('ROLLBACK');
        throw error;
    } finally {
        client.release();
    }
}