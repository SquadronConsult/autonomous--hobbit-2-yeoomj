/**
 * @fileoverview Enhanced device service for managing agricultural robot devices
 * Implements real-time telemetry streaming, device status management, and fleet coordination
 * @version 1.0.0
 */

import { ApiClient } from '../utils/api';
import { API_ENDPOINTS } from '../constants/apiEndpoints';
import { 
    IDevice, 
    DeviceType, 
    DeviceStatus, 
    DeviceCapability,
    IDeviceLocation 
} from '../interfaces/IDevice';

// Cache configuration
const DEVICE_CACHE_TTL = 5 * 60 * 1000; // 5 minutes
const TELEMETRY_BATCH_SIZE = 100;
const TELEMETRY_FLUSH_INTERVAL = 1000; // 1 second

/**
 * Device query options for filtering and pagination
 */
interface IDeviceQueryOptions {
    type?: DeviceType;
    status?: DeviceStatus;
    capabilities?: DeviceCapability[];
    page?: number;
    limit?: number;
    sortBy?: string;
    sortOrder?: 'asc' | 'desc';
}

/**
 * Paginated device response interface
 */
interface IPaginatedDevices {
    devices: IDevice[];
    total: number;
    page: number;
    limit: number;
    hasMore: boolean;
}

/**
 * Device telemetry subscription callback type
 */
type TelemetryCallback = (data: any) => void;

/**
 * Device service configuration options
 */
interface IDeviceServiceConfig {
    apiClient?: ApiClient;
    cacheTTL?: number;
    batchSize?: number;
    flushInterval?: number;
}

/**
 * Enhanced service class for managing agricultural robot devices
 * Provides comprehensive device management capabilities with real-time features
 */
export class DeviceService {
    private apiClient: ApiClient;
    private deviceCache: Map<string, { device: IDevice; timestamp: number }>;
    private telemetryBuffer: Map<string, any[]>;
    private flushInterval: NodeJS.Timeout | null;

    constructor(config: IDeviceServiceConfig = {}) {
        this.apiClient = config.apiClient || new ApiClient();
        this.deviceCache = new Map();
        this.telemetryBuffer = new Map();
        this.flushInterval = null;

        // Initialize telemetry buffer flush interval
        this.startTelemetryFlush(config.flushInterval || TELEMETRY_FLUSH_INTERVAL);
    }

    /**
     * Retrieves a paginated list of all available devices with filtering
     */
    public async getAllDevices(options: IDeviceQueryOptions = {}): Promise<IPaginatedDevices> {
        try {
            const response = await this.apiClient.request<IPaginatedDevices>({
                method: 'GET',
                url: API_ENDPOINTS.DEVICES.GET_ALL,
                params: {
                    type: options.type,
                    status: options.status,
                    capabilities: options.capabilities?.join(','),
                    page: options.page || 1,
                    limit: options.limit || 10,
                    sortBy: options.sortBy,
                    sortOrder: options.sortOrder || 'asc'
                }
            });

            // Update cache for each device
            response.devices.forEach(device => {
                this.updateDeviceCache(device);
            });

            return response;
        } catch (error) {
            console.error('Failed to fetch devices:', error);
            throw error;
        }
    }

    /**
     * Retrieves detailed information for a specific device
     */
    public async getDeviceById(deviceId: string): Promise<IDevice> {
        // Check cache first
        const cached = this.getFromCache(deviceId);
        if (cached) {
            return cached;
        }

        try {
            const device = await this.apiClient.request<IDevice>({
                method: 'GET',
                url: API_ENDPOINTS.DEVICES.GET_BY_ID.replace(':id', deviceId)
            });

            this.updateDeviceCache(device);
            return device;
        } catch (error) {
            console.error(`Failed to fetch device ${deviceId}:`, error);
            throw error;
        }
    }

    /**
     * Updates device status and configuration
     */
    public async updateDeviceStatus(
        deviceId: string, 
        status: DeviceStatus, 
        location?: IDeviceLocation
    ): Promise<IDevice> {
        try {
            const device = await this.apiClient.request<IDevice>({
                method: 'PUT',
                url: API_ENDPOINTS.DEVICES.STATUS.replace(':id', deviceId),
                data: { status, location }
            });

            this.updateDeviceCache(device);
            return device;
        } catch (error) {
            console.error(`Failed to update device ${deviceId} status:`, error);
            throw error;
        }
    }

    /**
     * Establishes real-time telemetry subscription for a device
     */
    public subscribeToTelemetry(deviceId: string, callback: TelemetryCallback): () => void {
        return this.apiClient.subscribe(
            API_ENDPOINTS.TELEMETRY.DEVICE_STREAM.replace(':deviceId', deviceId),
            {
                onMessage: (data) => {
                    this.bufferTelemetryData(deviceId, data);
                    callback(data);
                },
                onError: (error) => {
                    console.error(`Telemetry subscription error for device ${deviceId}:`, error);
                },
                onReconnect: () => {
                    console.log(`Reconnecting telemetry for device ${deviceId}`);
                }
            }
        );
    }

    /**
     * Performs batch update of multiple devices
     */
    public async batchUpdateDevices(
        updates: Array<{ deviceId: string; status: DeviceStatus; location?: IDeviceLocation }>
    ): Promise<IDevice[]> {
        try {
            const devices = await this.apiClient.request<IDevice[]>({
                method: 'PUT',
                url: API_ENDPOINTS.DEVICES.BATCH_STATUS,
                data: { updates }
            });

            devices.forEach(device => {
                this.updateDeviceCache(device);
            });

            return devices;
        } catch (error) {
            console.error('Failed to perform batch device update:', error);
            throw error;
        }
    }

    /**
     * Updates the device cache with new data
     */
    private updateDeviceCache(device: IDevice): void {
        this.deviceCache.set(device.id, {
            device,
            timestamp: Date.now()
        });
    }

    /**
     * Retrieves a device from cache if valid
     */
    private getFromCache(deviceId: string): IDevice | null {
        const cached = this.deviceCache.get(deviceId);
        if (cached && Date.now() - cached.timestamp < DEVICE_CACHE_TTL) {
            return cached.device;
        }
        return null;
    }

    /**
     * Buffers telemetry data for batch processing
     */
    private bufferTelemetryData(deviceId: string, data: any): void {
        const buffer = this.telemetryBuffer.get(deviceId) || [];
        buffer.push(data);
        
        if (buffer.length >= TELEMETRY_BATCH_SIZE) {
            this.flushTelemetryBuffer(deviceId);
        } else {
            this.telemetryBuffer.set(deviceId, buffer);
        }
    }

    /**
     * Starts the telemetry buffer flush interval
     */
    private startTelemetryFlush(interval: number): void {
        this.flushInterval = setInterval(() => {
            this.telemetryBuffer.forEach((_, deviceId) => {
                this.flushTelemetryBuffer(deviceId);
            });
        }, interval);
    }

    /**
     * Flushes the telemetry buffer for a device
     */
    private async flushTelemetryBuffer(deviceId: string): Promise<void> {
        const buffer = this.telemetryBuffer.get(deviceId);
        if (!buffer?.length) return;

        try {
            await this.apiClient.request({
                method: 'POST',
                url: API_ENDPOINTS.TELEMETRY.BATCH_METRICS,
                data: {
                    deviceId,
                    metrics: buffer
                }
            });

            this.telemetryBuffer.delete(deviceId);
        } catch (error) {
            console.error(`Failed to flush telemetry buffer for device ${deviceId}:`, error);
        }
    }

    /**
     * Cleanup method to be called when service is no longer needed
     */
    public dispose(): void {
        if (this.flushInterval) {
            clearInterval(this.flushInterval);
        }
        this.deviceCache.clear();
        this.telemetryBuffer.clear();
    }
}

export default DeviceService;