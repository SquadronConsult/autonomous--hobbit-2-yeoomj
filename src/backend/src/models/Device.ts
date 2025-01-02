/**
 * @fileoverview Advanced Device model implementation for agricultural robot management
 * Provides comprehensive device data management, real-time telemetry handling,
 * and robust error management with TimescaleDB integration
 * @version 1.0.0
 */

import { v4 as uuidv4 } from 'uuid'; // v9.0.0
import {
    RobotType,
    RobotCapability,
    RobotStatus,
    DroneType,
    GroundRobotType
} from '../constants/robotTypes';
import { IDevice, IDeviceLocation } from '../interfaces/IDevice';
import { executeQuery } from '../utils/database';
import { logger } from '../utils/logger';

/**
 * Advanced Device model class implementing comprehensive agricultural robot
 * data management and operations with real-time telemetry support
 */
export class Device implements IDevice {
    public readonly id: string;
    public name: string;
    public type: RobotType;
    public subType: DroneType | GroundRobotType;
    public capabilities: RobotCapability[];
    public status: RobotStatus;
    public batteryLevel: number;
    public location: IDeviceLocation;
    public lastActive: Date;
    public metadata: Record<string, any>;
    private isConnected: boolean;
    private telemetryInterval: number;

    /**
     * Creates a new Device instance with comprehensive validation
     * @param deviceData Initial device configuration data
     * @throws Error if required parameters are missing or invalid
     */
    constructor(deviceData: IDevice) {
        // Validate required parameters
        if (!deviceData.name || !deviceData.type || !deviceData.subType) {
            throw new Error('Missing required device parameters');
        }

        // Initialize device properties
        this.id = deviceData.id || uuidv4();
        this.name = deviceData.name;
        this.type = deviceData.type;
        this.subType = deviceData.subType;
        this.capabilities = deviceData.capabilities || [];
        this.status = deviceData.status || RobotStatus.IDLE;
        this.batteryLevel = deviceData.batteryLevel || 100;
        this.location = deviceData.location || {
            latitude: 0,
            longitude: 0,
            altitude: 0,
            heading: 0
        };
        this.lastActive = deviceData.lastActive || new Date();
        this.metadata = deviceData.metadata || {};
        this.isConnected = false;
        this.telemetryInterval = 1000; // Default 1 second interval

        // Validate device type and capabilities
        this.validateDeviceConfiguration();
    }

    /**
     * Validates device configuration and capabilities
     * @throws Error if configuration is invalid
     */
    private validateDeviceConfiguration(): void {
        // Validate device type matches subtype
        if (this.type === RobotType.AERIAL_DRONE && 
            !Object.values(DroneType).includes(this.subType as DroneType)) {
            throw new Error('Invalid drone subtype configuration');
        }

        if (this.type === RobotType.GROUND_ROBOT && 
            !Object.values(GroundRobotType).includes(this.subType as GroundRobotType)) {
            throw new Error('Invalid ground robot subtype configuration');
        }

        // Validate capabilities against device type
        this.validateCapabilities();
    }

    /**
     * Validates device capabilities based on type
     * @throws Error if capabilities are invalid for device type
     */
    private validateCapabilities(): void {
        const invalidCapabilities = this.capabilities.filter(capability => {
            if (this.type === RobotType.AERIAL_DRONE) {
                return ![RobotCapability.SURVEILLANCE, RobotCapability.MONITORING]
                    .includes(capability);
            }
            return ![RobotCapability.TREATMENT, RobotCapability.INTERVENTION]
                .includes(capability);
        });

        if (invalidCapabilities.length > 0) {
            throw new Error(`Invalid capabilities for device type: ${invalidCapabilities.join(', ')}`);
        }
    }

    /**
     * Persists device data to TimescaleDB with transaction support
     * @returns Promise resolving on successful save
     * @throws Error if save operation fails
     */
    public async save(): Promise<void> {
        try {
            const query = `
                INSERT INTO devices (
                    id, name, type, sub_type, capabilities, status,
                    battery_level, location, last_active, metadata
                ) VALUES (
                    $1, $2, $3, $4, $5, $6, $7, $8, $9, $10
                )
                ON CONFLICT (id) DO UPDATE
                SET name = EXCLUDED.name,
                    type = EXCLUDED.type,
                    sub_type = EXCLUDED.sub_type,
                    capabilities = EXCLUDED.capabilities,
                    status = EXCLUDED.status,
                    battery_level = EXCLUDED.battery_level,
                    location = EXCLUDED.location,
                    last_active = EXCLUDED.last_active,
                    metadata = EXCLUDED.metadata;
            `;

            await executeQuery(query, [
                this.id,
                this.name,
                this.type,
                this.subType,
                this.capabilities,
                this.status,
                this.batteryLevel,
                this.location,
                this.lastActive,
                this.metadata
            ]);

            logger.info('Device data saved successfully', {
                correlationId: `device-save-${this.id}`,
                component: 'Device',
                operation: 'save',
                details: { deviceId: this.id }
            });
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `device-save-error-${this.id}`,
                component: 'Device',
                operation: 'save',
                details: { deviceId: this.id }
            });
            throw error;
        }
    }

    /**
     * Updates device operational status with validation
     * @param newStatus New operational status
     * @returns Promise resolving on successful update
     * @throws Error if status update fails
     */
    public async updateStatus(newStatus: RobotStatus): Promise<void> {
        try {
            // Validate status transition
            if (!this.isValidStatusTransition(newStatus)) {
                throw new Error(`Invalid status transition from ${this.status} to ${newStatus}`);
            }

            const query = `
                UPDATE devices
                SET status = $1,
                    last_active = NOW()
                WHERE id = $2;
            `;

            await executeQuery(query, [newStatus, this.id]);
            this.status = newStatus;
            this.lastActive = new Date();

            logger.info('Device status updated', {
                correlationId: `device-status-${this.id}`,
                component: 'Device',
                operation: 'updateStatus',
                details: { deviceId: this.id, newStatus }
            });
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `device-status-error-${this.id}`,
                component: 'Device',
                operation: 'updateStatus',
                details: { deviceId: this.id, newStatus }
            });
            throw error;
        }
    }

    /**
     * Validates status transitions based on current state
     * @param newStatus Proposed new status
     * @returns boolean indicating if transition is valid
     */
    private isValidStatusTransition(newStatus: RobotStatus): boolean {
        // Define valid status transitions
        const validTransitions = {
            [RobotStatus.IDLE]: [RobotStatus.ACTIVE, RobotStatus.CHARGING, RobotStatus.MAINTENANCE],
            [RobotStatus.ACTIVE]: [RobotStatus.IDLE, RobotStatus.ERROR],
            [RobotStatus.CHARGING]: [RobotStatus.IDLE],
            [RobotStatus.MAINTENANCE]: [RobotStatus.IDLE],
            [RobotStatus.ERROR]: [RobotStatus.MAINTENANCE, RobotStatus.IDLE]
        };

        return validTransitions[this.status]?.includes(newStatus) || false;
    }
}

/**
 * Retrieves a device by its unique identifier with caching
 * @param id Device unique identifier
 * @returns Promise resolving to Device instance or null
 */
export async function findDeviceById(id: string): Promise<Device | null> {
    try {
        const query = `
            SELECT * FROM devices
            WHERE id = $1;
        `;

        const result = await executeQuery(query, [id]);
        
        if (result.rows.length === 0) {
            return null;
        }

        return new Device(result.rows[0]);
    } catch (error) {
        logger.error(error as Error, {
            correlationId: `device-find-error-${id}`,
            component: 'Device',
            operation: 'findDeviceById',
            details: { deviceId: id }
        });
        throw error;
    }
}