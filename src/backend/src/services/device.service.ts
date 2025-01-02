/**
 * @fileoverview Service layer implementation for agricultural robot device management
 * Provides comprehensive fleet management, telemetry handling, and monitoring capabilities
 * @version 1.0.0
 */

import { Device, findDeviceById } from '../models/Device';
import { IDevice } from '../interfaces/IDevice';
import { executeQuery } from '../utils/database';
import { logger } from '../utils/logger';
import { RobotType, RobotStatus, RobotCapability } from '../constants/robotTypes';
import * as promClient from 'prom-client';

// Performance monitoring metrics
const deviceMetrics = new promClient.Gauge({
    name: 'active_devices_total',
    help: 'Total number of active devices',
    labelNames: ['type', 'status', 'connection_state']
});

const deviceOperationsMetrics = new promClient.Counter({
    name: 'device_operations_total',
    help: 'Total number of device operations',
    labelNames: ['operation_type', 'status']
});

const deviceBatteryMetrics = new promClient.Gauge({
    name: 'device_battery_level',
    help: 'Device battery level percentage',
    labelNames: ['device_id', 'type']
});

const telemetryLatencyHistogram = new promClient.Histogram({
    name: 'telemetry_processing_latency',
    help: 'Telemetry processing latency in seconds',
    labelNames: ['device_type'],
    buckets: [0.1, 0.5, 1, 2, 5]
});

/**
 * Creates a new device with comprehensive validation and monitoring
 * @param deviceData Initial device configuration
 * @returns Promise resolving to created Device instance
 */
async function createDevice(deviceData: IDevice): Promise<Device> {
    try {
        logger.info('Creating new device', {
            correlationId: `create-device-${Date.now()}`,
            component: 'DeviceService',
            operation: 'createDevice',
            details: { deviceType: deviceData.type }
        });

        const device = new Device(deviceData);
        await device.save();

        deviceMetrics.inc({
            type: device.type,
            status: device.status,
            connection_state: 'CONNECTED'
        });

        deviceOperationsMetrics.inc({
            operation_type: 'create',
            status: 'success'
        });

        return device;
    } catch (error) {
        deviceOperationsMetrics.inc({
            operation_type: 'create',
            status: 'error'
        });
        logger.error(error as Error, {
            correlationId: `create-device-error-${Date.now()}`,
            component: 'DeviceService',
            operation: 'createDevice'
        });
        throw error;
    }
}

/**
 * Updates device status with validation and monitoring
 * @param id Device identifier
 * @param status New device status
 * @returns Promise resolving to updated Device instance
 */
async function updateDeviceStatus(id: string, status: RobotStatus): Promise<Device> {
    try {
        const device = await findDeviceById(id);
        if (!device) {
            throw new Error(`Device not found: ${id}`);
        }

        const previousStatus = device.status;
        await device.updateStatus(status);

        deviceMetrics.inc({
            type: device.type,
            status: status,
            connection_state: 'CONNECTED'
        });
        deviceMetrics.dec({
            type: device.type,
            status: previousStatus,
            connection_state: 'CONNECTED'
        });

        deviceOperationsMetrics.inc({
            operation_type: 'update_status',
            status: 'success'
        });

        return device;
    } catch (error) {
        deviceOperationsMetrics.inc({
            operation_type: 'update_status',
            status: 'error'
        });
        logger.error(error as Error, {
            correlationId: `update-status-error-${Date.now()}`,
            component: 'DeviceService',
            operation: 'updateDeviceStatus'
        });
        throw error;
    }
}

/**
 * Retrieves devices by type with pagination support
 * @param type Device type filter
 * @param limit Maximum number of devices to return
 * @param offset Pagination offset
 * @returns Promise resolving to array of Device instances
 */
async function getDevicesByType(
    type: RobotType,
    limit: number = 10,
    offset: number = 0
): Promise<Device[]> {
    try {
        const query = `
            SELECT * FROM devices
            WHERE type = $1
            ORDER BY last_active DESC
            LIMIT $2 OFFSET $3;
        `;

        const result = await executeQuery(query, [type, limit, offset]);
        return result.rows.map(row => new Device(row));
    } catch (error) {
        logger.error(error as Error, {
            correlationId: `get-devices-error-${Date.now()}`,
            component: 'DeviceService',
            operation: 'getDevicesByType'
        });
        throw error;
    }
}

/**
 * Updates device telemetry data with performance monitoring
 * @param id Device identifier
 * @param telemetryData Device telemetry data
 * @returns Promise resolving to updated Device instance
 */
async function updateDeviceTelemetry(
    id: string,
    telemetryData: {
        batteryLevel: number;
        location: { latitude: number; longitude: number; altitude: number; heading: number };
        metadata: Record<string, any>;
    }
): Promise<Device> {
    const startTime = process.hrtime();

    try {
        const device = await findDeviceById(id);
        if (!device) {
            throw new Error(`Device not found: ${id}`);
        }

        // Update device data
        device.batteryLevel = telemetryData.batteryLevel;
        device.location = telemetryData.location;
        device.metadata = { ...device.metadata, ...telemetryData.metadata };
        device.lastActive = new Date();

        await device.save();

        // Update metrics
        deviceBatteryMetrics.set(
            { device_id: id, type: device.type },
            telemetryData.batteryLevel
        );

        const [seconds, nanoseconds] = process.hrtime(startTime);
        const duration = seconds + nanoseconds / 1e9;
        telemetryLatencyHistogram.observe({ device_type: device.type }, duration);

        return device;
    } catch (error) {
        deviceOperationsMetrics.inc({
            operation_type: 'update_telemetry',
            status: 'error'
        });
        logger.error(error as Error, {
            correlationId: `update-telemetry-error-${Date.now()}`,
            component: 'DeviceService',
            operation: 'updateDeviceTelemetry'
        });
        throw error;
    }
}

/**
 * Monitors device health and updates status accordingly
 * @param id Device identifier
 * @returns Promise resolving to device health status
 */
async function monitorDeviceHealth(id: string): Promise<{
    isHealthy: boolean;
    status: RobotStatus;
    batteryLevel: number;
    lastActive: Date;
}> {
    try {
        const device = await findDeviceById(id);
        if (!device) {
            throw new Error(`Device not found: ${id}`);
        }

        const healthStatus = {
            isHealthy: true,
            status: device.status,
            batteryLevel: device.batteryLevel,
            lastActive: device.lastActive
        };

        // Check battery level
        if (device.batteryLevel < 20) {
            healthStatus.isHealthy = false;
            if (device.status !== RobotStatus.CHARGING) {
                await updateDeviceStatus(id, RobotStatus.CHARGING);
                healthStatus.status = RobotStatus.CHARGING;
            }
        }

        // Check last activity
        const inactiveThreshold = 5 * 60 * 1000; // 5 minutes
        if (Date.now() - device.lastActive.getTime() > inactiveThreshold) {
            healthStatus.isHealthy = false;
        }

        return healthStatus;
    } catch (error) {
        logger.error(error as Error, {
            correlationId: `health-check-error-${Date.now()}`,
            component: 'DeviceService',
            operation: 'monitorDeviceHealth'
        });
        throw error;
    }
}

// Export service interface
export const DeviceService = {
    createDevice,
    updateDeviceStatus,
    getDevicesByType,
    updateDeviceTelemetry,
    monitorDeviceHealth
};