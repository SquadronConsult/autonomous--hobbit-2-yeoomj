/**
 * @fileoverview Enterprise-grade service layer for agricultural mission management
 * Provides comprehensive mission lifecycle management with high reliability and performance
 * @version 1.0.0
 */

import { IMission } from '../interfaces/IMission';
import { Mission } from '../models/Mission';
import { validateMissionData } from '../validators/mission.validator';
import { DeviceService } from './device.service';
import { Counter, Gauge } from 'prom-client'; // v14.2.0
import { createLogger } from 'winston'; // v3.8.2
import retry from 'retry'; // v0.13.1
import CircuitBreaker from 'circuit-breaker-js'; // v0.0.1
import { MissionStatus } from '../constants/missionStatus';
import { ErrorCodes } from '../constants/errorCodes';
import { db } from '../utils/database';

// Performance monitoring metrics
const missionMetrics = new Gauge({
    name: 'active_missions_total',
    help: 'Total number of active missions',
    labelNames: ['status', 'type', 'region']
});

const deviceAssignmentMetrics = new Gauge({
    name: 'assigned_devices_total',
    help: 'Total number of assigned devices',
    labelNames: ['type', 'status']
});

const missionOperationsCounter = new Counter({
    name: 'mission_operations_total',
    help: 'Total number of mission operations',
    labelNames: ['operation', 'status']
});

// Structured logging
const logger = createLogger({
    defaultMeta: { service: 'mission-service' }
});

// Circuit breaker for device service
const deviceServiceBreaker = new CircuitBreaker(DeviceService, {
    failureThreshold: 5,
    resetTimeout: 30000
});

/**
 * Enterprise-grade mission management service implementation
 * Provides comprehensive mission lifecycle management with high reliability
 */
export class MissionService {
    /**
     * Creates a new agricultural mission with comprehensive validation
     * @param missionData Mission configuration data
     * @returns Promise resolving to created mission
     * @throws Error if validation fails or creation unsuccessful
     */
    public async createMission(missionData: IMission): Promise<IMission> {
        const correlationId = `create-mission-${Date.now()}`;
        logger.info('Creating new mission', { correlationId, missionData });

        try {
            // Validate mission data
            const validationErrors = await validateMissionData(missionData);
            if (validationErrors.length > 0) {
                throw new Error(`Mission validation failed: ${JSON.stringify(validationErrors)}`);
            }

            // Verify device availability using circuit breaker
            for (const device of missionData.assignedDevices) {
                const deviceStatus = await deviceServiceBreaker.execute(
                    async () => DeviceService.getDeviceById(device.deviceId)
                );
                if (!deviceStatus) {
                    throw new Error(`Device not found: ${device.deviceId}`);
                }
            }

            // Begin transaction
            const result = await db.executeTransaction([
                {
                    query: 'INSERT INTO missions (data) VALUES ($1) RETURNING *',
                    params: [missionData]
                },
                ...missionData.assignedDevices.map(device => ({
                    query: 'UPDATE devices SET mission_id = $1 WHERE id = $2',
                    params: [missionData.id, device.deviceId]
                }))
            ]);

            const createdMission = new Mission(result[0].rows[0]);

            // Update metrics
            missionMetrics.inc({
                status: createdMission.status,
                type: createdMission.type,
                region: createdMission.metadata.region
            });

            deviceAssignmentMetrics.inc({
                type: 'total',
                status: 'assigned'
            }, missionData.assignedDevices.length);

            missionOperationsCounter.inc({
                operation: 'create',
                status: 'success'
            });

            logger.info('Mission created successfully', {
                correlationId,
                missionId: createdMission.id
            });

            return createdMission;
        } catch (error) {
            missionOperationsCounter.inc({
                operation: 'create',
                status: 'error'
            });

            logger.error('Mission creation failed', {
                correlationId,
                error: error.message,
                stack: error.stack
            });

            throw error;
        }
    }

    /**
     * Updates mission status with validation and monitoring
     * @param missionId Mission identifier
     * @param status New mission status
     * @returns Promise resolving to updated mission
     */
    public async updateMissionStatus(
        missionId: string,
        status: MissionStatus
    ): Promise<IMission> {
        const correlationId = `update-status-${missionId}`;
        logger.info('Updating mission status', { correlationId, missionId, status });

        const operation = retry.operation({
            retries: 3,
            factor: 2,
            minTimeout: 1000,
            maxTimeout: 5000
        });

        return new Promise((resolve, reject) => {
            operation.attempt(async (currentAttempt) => {
                try {
                    const mission = await Mission.findById(missionId);
                    if (!mission) {
                        throw new Error(`Mission not found: ${missionId}`);
                    }

                    const previousStatus = mission.status;
                    mission.status = status;
                    await mission.save();

                    // Update metrics
                    missionMetrics.inc({ status, type: mission.type });
                    missionMetrics.dec({ status: previousStatus, type: mission.type });

                    missionOperationsCounter.inc({
                        operation: 'update_status',
                        status: 'success'
                    });

                    logger.info('Mission status updated', {
                        correlationId,
                        missionId,
                        previousStatus,
                        newStatus: status
                    });

                    resolve(mission);
                } catch (error) {
                    if (operation.retry(error)) {
                        return;
                    }

                    missionOperationsCounter.inc({
                        operation: 'update_status',
                        status: 'error'
                    });

                    logger.error('Mission status update failed', {
                        correlationId,
                        error: error.message,
                        attempt: currentAttempt
                    });

                    reject(operation.mainError());
                }
            });
        });
    }

    /**
     * Retrieves active missions with comprehensive filtering
     * @param options Query options for filtering and pagination
     * @returns Promise resolving to array of active missions
     */
    public async getActiveMissions(options: {
        type?: string;
        region?: string;
        limit?: number;
        offset?: number;
    }): Promise<{ missions: IMission[]; total: number }> {
        const correlationId = `get-active-missions-${Date.now()}`;
        logger.info('Retrieving active missions', { correlationId, options });

        try {
            const { missions, total } = await Mission.findByStatus(
                MissionStatus.IN_PROGRESS,
                {
                    page: Math.floor(options.offset / options.limit) + 1,
                    limit: options.limit,
                    sort: '-metadata.priority'
                }
            );

            missionOperationsCounter.inc({
                operation: 'get_active',
                status: 'success'
            });

            return { missions, total };
        } catch (error) {
            missionOperationsCounter.inc({
                operation: 'get_active',
                status: 'error'
            });

            logger.error('Failed to retrieve active missions', {
                correlationId,
                error: error.message
            });

            throw error;
        }
    }

    /**
     * Updates mission progress with validation and monitoring
     * @param missionId Mission identifier
     * @param progress Mission completion percentage
     * @returns Promise resolving to updated mission
     */
    public async updateMissionProgress(
        missionId: string,
        progress: number
    ): Promise<IMission> {
        const correlationId = `update-progress-${missionId}`;
        logger.info('Updating mission progress', { correlationId, missionId, progress });

        try {
            const mission = await Mission.updateProgress(missionId, progress);

            missionOperationsCounter.inc({
                operation: 'update_progress',
                status: 'success'
            });

            return mission;
        } catch (error) {
            missionOperationsCounter.inc({
                operation: 'update_progress',
                status: 'error'
            });

            logger.error('Failed to update mission progress', {
                correlationId,
                error: error.message
            });

            throw error;
        }
    }
}

// Export singleton instance
export const missionService = new MissionService();