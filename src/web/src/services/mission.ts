/**
 * @fileoverview Mission service module for agricultural operations management
 * @version 1.0.0
 * 
 * Provides comprehensive mission management capabilities including:
 * - CRUD operations for missions
 * - Real-time mission status updates
 * - Offline support with sync
 * - Mission coordination and monitoring
 * - Robust error handling and validation
 */

import axios from 'axios'; // axios@1.4.0
import { IMission } from '../interfaces/IMission';
import { ApiService } from './api';
import { API_ENDPOINTS } from '../constants/apiEndpoints';
import { validateMission } from '../utils/validation';
import { MissionStatusCodes } from '../constants/statusCodes';

// WebSocket configuration for real-time updates
const WS_RECONNECT_DELAY = 3000;
const WS_MAX_RETRIES = 5;

/**
 * Mission service class providing comprehensive mission management capabilities
 */
export class MissionService {
    private readonly apiService: ApiService;
    private wsConnection: WebSocket | null = null;
    private missionSubscriptions: Map<string, Set<(status: any) => void>> = new Map();
    private offlineQueue: IMission[] = [];
    private wsRetryCount = 0;

    constructor(apiService: ApiService) {
        this.apiService = apiService;
        this.initializeWebSocket();
        this.setupOfflineSync();
    }

    /**
     * Retrieves all missions with support for offline mode and caching
     */
    public async getAllMissions(useCache: boolean = true): Promise<IMission[]> {
        try {
            const response = await this.apiService.request<IMission[]>({
                url: API_ENDPOINTS.MISSIONS.GET_ALL,
                method: 'GET',
                params: { useCache }
            });

            // Store missions for offline access
            if (navigator.onLine) {
                localStorage.setItem('cachedMissions', JSON.stringify(response));
            }

            return response;
        } catch (error) {
            // Return cached missions if offline
            if (!navigator.onLine) {
                const cachedMissions = localStorage.getItem('cachedMissions');
                if (cachedMissions) {
                    return JSON.parse(cachedMissions);
                }
            }
            throw error;
        }
    }

    /**
     * Retrieves a specific mission by ID with real-time updates
     */
    public async getMissionById(id: string): Promise<IMission> {
        const url = API_ENDPOINTS.MISSIONS.GET_BY_ID.replace(':id', id);
        const mission = await this.apiService.request<IMission>({
            url,
            method: 'GET'
        });

        // Setup real-time updates for this mission
        this.subscribeMissionUpdates(id);

        return mission;
    }

    /**
     * Creates a new mission with validation and offline support
     */
    public async createMission(missionData: Omit<IMission, 'id'>): Promise<IMission> {
        // Validate mission data
        validateMission(missionData as IMission);

        try {
            const mission = await this.apiService.request<IMission>({
                url: API_ENDPOINTS.MISSIONS.CREATE,
                method: 'POST',
                data: missionData
            });

            // Setup real-time monitoring for new mission
            this.subscribeMissionUpdates(mission.id);

            return mission;
        } catch (error) {
            if (!navigator.onLine) {
                // Queue for later sync
                const offlineMission = {
                    ...missionData,
                    id: `offline_${Date.now()}`,
                    status: MissionStatusCodes.PENDING
                } as IMission;
                this.offlineQueue.push(offlineMission);
                return offlineMission;
            }
            throw error;
        }
    }

    /**
     * Updates an existing mission with conflict resolution
     */
    public async updateMission(id: string, missionData: Partial<IMission>): Promise<IMission> {
        const url = API_ENDPOINTS.MISSIONS.UPDATE.replace(':id', id);
        
        // Validate update data
        if (Object.keys(missionData).length > 0) {
            validateMission({ ...await this.getMissionById(id), ...missionData });
        }

        return this.apiService.request<IMission>({
            url,
            method: 'PUT',
            data: missionData
        });
    }

    /**
     * Deletes a mission with cleanup
     */
    public async deleteMission(id: string): Promise<void> {
        const url = API_ENDPOINTS.MISSIONS.DELETE.replace(':id', id);
        
        await this.apiService.request({
            url,
            method: 'DELETE'
        });

        // Cleanup subscriptions and cached data
        this.unsubscribeMissionUpdates(id);
        this.cleanupMissionCache(id);
    }

    /**
     * Gets real-time mission status with offline fallback
     */
    public async getMissionStatus(id: string): Promise<{
        status: MissionStatusCodes;
        progress: number;
        metrics: Record<string, any>;
    }> {
        const url = API_ENDPOINTS.MISSIONS.STATUS.replace(':id', id);
        
        try {
            return await this.apiService.request({
                url,
                method: 'GET'
            });
        } catch (error) {
            if (!navigator.onLine) {
                return {
                    status: MissionStatusCodes.PENDING,
                    progress: 0,
                    metrics: {}
                };
            }
            throw error;
        }
    }

    /**
     * Subscribes to real-time mission updates
     */
    private subscribeMissionUpdates(missionId: string): void {
        if (this.wsConnection?.readyState === WebSocket.OPEN) {
            this.wsConnection.send(JSON.stringify({
                type: 'subscribe',
                missionId
            }));
        }
    }

    /**
     * Unsubscribes from mission updates
     */
    private unsubscribeMissionUpdates(missionId: string): void {
        if (this.wsConnection?.readyState === WebSocket.OPEN) {
            this.wsConnection.send(JSON.stringify({
                type: 'unsubscribe',
                missionId
            }));
        }
        this.missionSubscriptions.delete(missionId);
    }

    /**
     * Initializes WebSocket connection for real-time updates
     */
    private initializeWebSocket(): void {
        const wsUrl = `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.host}/ws/missions`;
        
        this.wsConnection = new WebSocket(wsUrl);
        
        this.wsConnection.onmessage = (event) => {
            const update = JSON.parse(event.data);
            const subscribers = this.missionSubscriptions.get(update.missionId);
            subscribers?.forEach(callback => callback(update));
        };

        this.wsConnection.onclose = () => {
            if (this.wsRetryCount < WS_MAX_RETRIES) {
                setTimeout(() => {
                    this.wsRetryCount++;
                    this.initializeWebSocket();
                }, WS_RECONNECT_DELAY);
            }
        };
    }

    /**
     * Sets up offline synchronization
     */
    private setupOfflineSync(): void {
        window.addEventListener('online', async () => {
            while (this.offlineQueue.length > 0) {
                const mission = this.offlineQueue.shift();
                if (mission) {
                    try {
                        const { id, ...missionData } = mission;
                        await this.createMission(missionData);
                    } catch (error) {
                        console.error('Failed to sync offline mission:', error);
                        this.offlineQueue.unshift(mission);
                        break;
                    }
                }
            }
        });
    }

    /**
     * Cleans up mission cache and subscriptions
     */
    private cleanupMissionCache(missionId: string): void {
        const cachedMissions = localStorage.getItem('cachedMissions');
        if (cachedMissions) {
            const missions = JSON.parse(cachedMissions);
            localStorage.setItem(
                'cachedMissions',
                JSON.stringify(missions.filter((m: IMission) => m.id !== missionId))
            );
        }
    }
}