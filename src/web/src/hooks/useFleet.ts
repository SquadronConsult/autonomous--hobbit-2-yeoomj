/**
 * @fileoverview Enhanced React hook for managing agricultural robot fleets
 * Provides real-time monitoring, offline support, and optimized performance
 * @version 1.0.0
 */

import { useState, useEffect, useCallback, useMemo } from 'react';
import { IDevice, DeviceType, DeviceStatus } from '../interfaces/IDevice';
import { DeviceService } from '../services/device';
import { useWebSocket } from './useWebSocket';

// Constants for fleet management
const DEVICE_UPDATE_EVENT = 'device:update';
const DEVICE_STATUS_EVENT = 'device:status';
const DEVICE_LOCATION_EVENT = 'device:location';
const UPDATE_BATCH_SIZE = 10;
const UPDATE_INTERVAL_MS = 100;
const CACHE_TTL_MS = 300000; // 5 minutes
const MAX_RETRY_ATTEMPTS = 3;

/**
 * Interface for fleet filtering options
 */
interface IFleetFilters {
    deviceType?: DeviceType;
    status?: DeviceStatus;
    batteryThreshold?: number;
}

/**
 * Interface for device update message
 */
interface IDeviceUpdate {
    deviceId: string;
    updateType: 'status' | 'location' | 'telemetry';
    payload: Partial<IDevice>;
}

/**
 * Enhanced fleet management hook with comprehensive monitoring capabilities
 */
export function useFleet(filters: IFleetFilters = {}) {
    // Optimized state management using Map for O(1) lookups
    const [devices, setDevices] = useState<Map<string, IDevice>>(new Map());
    const [loading, setLoading] = useState<boolean>(true);
    const [error, setError] = useState<Error | null>(null);
    const [lastUpdate, setLastUpdate] = useState<Date>(new Date());

    // Initialize services
    const deviceService = useMemo(() => new DeviceService(), []);
    const { 
        isConnected, 
        connectionStatus, 
        sendMessage, 
        subscribe 
    } = useWebSocket({
        autoConnect: true,
        enableReconnect: true,
        reconnectInterval: 5000,
        maxReconnectAttempts: MAX_RETRY_ATTEMPTS
    });

    // Batch update queue for optimized state updates
    const updateQueue = useMemo(() => new Map<string, Partial<IDevice>>(), []);
    const updateTimeoutRef = useMemo(() => ({ current: null as NodeJS.Timeout | null }), []);

    /**
     * Fetches devices with caching and offline support
     */
    const fetchDevices = useCallback(async () => {
        try {
            setLoading(true);
            const response = await deviceService.getAllDevices({
                type: filters.deviceType,
                status: filters.status,
                page: 1,
                limit: 100
            });

            const deviceMap = new Map<string, IDevice>();
            response.devices
                .filter(device => !filters.batteryThreshold || device.batteryLevel >= filters.batteryThreshold)
                .forEach(device => deviceMap.set(device.id, device));

            setDevices(deviceMap);
            setLastUpdate(new Date());
            setError(null);
        } catch (err) {
            setError(err as Error);
            console.error('Failed to fetch devices:', err);
        } finally {
            setLoading(false);
        }
    }, [deviceService, filters]);

    /**
     * Handles real-time device updates with validation and batching
     */
    const handleDeviceUpdate = useCallback((message: IDeviceUpdate) => {
        const { deviceId, updateType, payload } = message;

        // Validate update before processing
        if (!deviceId || !updateType || !payload) {
            console.error('Invalid device update message:', message);
            return;
        }

        updateQueue.set(deviceId, {
            ...updateQueue.get(deviceId),
            ...payload
        });

        // Implement batched updates for performance
        if (updateTimeoutRef.current) {
            clearTimeout(updateTimeoutRef.current);
        }

        updateTimeoutRef.current = setTimeout(() => {
            setDevices(prevDevices => {
                const newDevices = new Map(prevDevices);
                
                updateQueue.forEach((updates, deviceId) => {
                    const device = newDevices.get(deviceId);
                    if (device) {
                        newDevices.set(deviceId, {
                            ...device,
                            ...updates,
                            lastUpdateTime: new Date()
                        });
                    }
                });

                updateQueue.clear();
                return newDevices;
            });
        }, UPDATE_INTERVAL_MS);
    }, [updateQueue]);

    /**
     * Updates device status with optimistic updates
     */
    const updateDeviceStatus = useCallback(async (
        deviceId: string, 
        status: DeviceStatus
    ): Promise<boolean> => {
        try {
            // Optimistic update
            setDevices(prev => {
                const newDevices = new Map(prev);
                const device = newDevices.get(deviceId);
                if (device) {
                    newDevices.set(deviceId, { ...device, status });
                }
                return newDevices;
            });

            await deviceService.updateDeviceStatus(deviceId, status);
            return true;
        } catch (err) {
            // Revert optimistic update on failure
            await fetchDevices();
            throw err;
        }
    }, [deviceService, fetchDevices]);

    /**
     * Performs batch update of multiple devices
     */
    const batchUpdateDevices = useCallback(async (
        updates: Array<{ deviceId: string; status: DeviceStatus }>
    ): Promise<boolean> => {
        try {
            // Optimistic batch update
            setDevices(prev => {
                const newDevices = new Map(prev);
                updates.forEach(({ deviceId, status }) => {
                    const device = newDevices.get(deviceId);
                    if (device) {
                        newDevices.set(deviceId, { ...device, status });
                    }
                });
                return newDevices;
            });

            await deviceService.batchUpdateDevices(updates);
            return true;
        } catch (err) {
            // Revert optimistic updates on failure
            await fetchDevices();
            throw err;
        }
    }, [deviceService, fetchDevices]);

    // Initialize WebSocket subscriptions
    useEffect(() => {
        const unsubscribeUpdate = subscribe(DEVICE_UPDATE_EVENT, handleDeviceUpdate);
        const unsubscribeStatus = subscribe(DEVICE_STATUS_EVENT, handleDeviceUpdate);
        const unsubscribeLocation = subscribe(DEVICE_LOCATION_EVENT, handleDeviceUpdate);

        return () => {
            unsubscribeUpdate();
            unsubscribeStatus();
            unsubscribeLocation();
        };
    }, [subscribe, handleDeviceUpdate]);

    // Fetch devices on mount and when filters change
    useEffect(() => {
        fetchDevices();
        const refreshInterval = setInterval(fetchDevices, CACHE_TTL_MS);
        
        return () => {
            clearInterval(refreshInterval);
            if (updateTimeoutRef.current) {
                clearTimeout(updateTimeoutRef.current);
            }
        };
    }, [fetchDevices]);

    return {
        devices,
        loading,
        error,
        isConnected,
        connectionQuality: connectionStatus?.connectionQuality,
        lastUpdate,
        updateDeviceStatus,
        batchUpdateDevices,
        refreshDevices: fetchDevices
    };
}