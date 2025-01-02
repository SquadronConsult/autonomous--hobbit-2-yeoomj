/**
 * @fileoverview Analytics service for handling real-time video analytics and data processing
 * Implements frontend service layer for analytics dashboard with caching and validation
 * @version 1.0.0
 */

import { debounce } from 'lodash'; // v4.17.21
import { AxiosResponse } from 'axios'; // v1.4.0
import { Point } from '@turf/helpers'; // v6.5.0

import { ApiClient } from '../utils/api';
import { API_ENDPOINTS } from '../constants/apiEndpoints';
import { 
    IAnalytics, 
    IAnalyticsFilters, 
    AnalyticsType, 
    IDetection,
    MIN_CONFIDENCE_THRESHOLD 
} from '../interfaces/IAnalytics';

// Constants for analytics service configuration
const CACHE_DURATION = 300000; // 5 minutes
const MAX_CACHE_SIZE = 1000;
const DEBOUNCE_DELAY = 500;
const STREAM_RETRY_ATTEMPTS = 3;
const STREAM_RETRY_DELAY = 5000;

/**
 * Cache entry structure for analytics data
 */
interface CacheEntry {
    data: IAnalytics[];
    timestamp: number;
    expiresAt: number;
}

/**
 * Analytics service class for handling real-time and historical analytics data
 */
export class AnalyticsService {
    private apiClient: ApiClient;
    private cache: Map<string, CacheEntry>;
    private activeStreams: Map<string, () => void>;

    constructor(apiClient: ApiClient) {
        this.apiClient = apiClient;
        this.cache = new Map();
        this.activeStreams = new Map();
        
        // Initialize debounced methods
        this.debouncedGetAnalytics = debounce(this.getAnalytics, DEBOUNCE_DELAY);
    }

    /**
     * Retrieves analytics data based on provided filters
     * @param filters Analytics query filters
     * @returns Promise resolving to filtered analytics data
     */
    public async getAnalytics(filters: IAnalyticsFilters): Promise<IAnalytics[]> {
        try {
            // Validate confidence threshold
            if (filters.confidence && filters.confidence < MIN_CONFIDENCE_THRESHOLD) {
                filters.confidence = MIN_CONFIDENCE_THRESHOLD;
            }

            const cacheKey = this.generateCacheKey(filters);
            const cachedData = this.getCachedData(cacheKey);
            
            if (cachedData) {
                return cachedData;
            }

            const response = await this.apiClient.request<IAnalytics[]>({
                method: 'GET',
                url: API_ENDPOINTS.ANALYTICS.BASE,
                params: {
                    startDate: filters.startDate.toISOString(),
                    endDate: filters.endDate.toISOString(),
                    type: filters.type,
                    confidence: filters.confidence
                }
            });

            const validatedData = this.validateAnalyticsData(response);
            this.cacheData(cacheKey, validatedData);

            return validatedData;
        } catch (error) {
            console.error('Failed to fetch analytics:', error);
            throw error;
        }
    }

    /**
     * Establishes real-time analytics data stream
     * @param filters Stream filters
     * @param callback Callback function for handling incoming data
     * @returns Cleanup function for closing the stream
     */
    public async streamAnalytics(
        filters: IAnalyticsFilters,
        callback: (data: IAnalytics) => void
    ): Promise<() => void> {
        const streamId = this.generateStreamId(filters);
        
        if (this.activeStreams.has(streamId)) {
            throw new Error('Stream already exists for these filters');
        }

        const cleanup = await this.apiClient.subscribe(
            API_ENDPOINTS.ANALYTICS.BASE,
            {
                onMessage: (data: IAnalytics) => {
                    if (this.validateAnalyticsEntry(data)) {
                        callback(data);
                    }
                },
                onError: (error: Error) => {
                    console.error('Analytics stream error:', error);
                },
                retryAttempts: STREAM_RETRY_ATTEMPTS,
                retryDelay: STREAM_RETRY_DELAY
            }
        );

        this.activeStreams.set(streamId, cleanup);
        return () => this.closeStream(streamId);
    }

    /**
     * Retrieves system performance metrics
     * @returns Promise resolving to system metrics data
     */
    public async getSystemMetrics(): Promise<Record<string, number>> {
        try {
            const response = await this.apiClient.request<Record<string, number>>({
                method: 'GET',
                url: API_ENDPOINTS.ANALYTICS.SYSTEM_METRICS
            });

            return response;
        } catch (error) {
            console.error('Failed to fetch system metrics:', error);
            throw error;
        }
    }

    /**
     * Clears analytics cache
     */
    public clearCache(): void {
        this.cache.clear();
    }

    /**
     * Closes all active analytics streams
     */
    public closeAllStreams(): void {
        this.activeStreams.forEach(cleanup => cleanup());
        this.activeStreams.clear();
    }

    /**
     * Validates analytics data array
     */
    private validateAnalyticsData(data: IAnalytics[]): IAnalytics[] {
        return data.filter(entry => this.validateAnalyticsEntry(entry));
    }

    /**
     * Validates individual analytics entry
     */
    private validateAnalyticsEntry(entry: IAnalytics): boolean {
        return (
            entry &&
            typeof entry.id === 'string' &&
            typeof entry.missionId === 'string' &&
            entry.timestamp instanceof Date &&
            typeof entry.confidence === 'number' &&
            entry.confidence >= MIN_CONFIDENCE_THRESHOLD &&
            Object.values(AnalyticsType).includes(entry.type) &&
            Array.isArray(entry.detections) &&
            entry.detections.every(detection => this.validateDetection(detection))
        );
    }

    /**
     * Validates detection object
     */
    private validateDetection(detection: IDetection): boolean {
        return (
            detection &&
            typeof detection.type === 'string' &&
            typeof detection.confidence === 'number' &&
            detection.confidence >= MIN_CONFIDENCE_THRESHOLD &&
            this.validateBoundingBox(detection.boundingBox)
        );
    }

    /**
     * Validates bounding box coordinates
     */
    private validateBoundingBox(box: any): boolean {
        return (
            box &&
            typeof box.x === 'number' &&
            typeof box.y === 'number' &&
            typeof box.width === 'number' &&
            typeof box.height === 'number' &&
            box.x >= 0 && box.x <= 1 &&
            box.y >= 0 && box.y <= 1 &&
            box.width >= 0 && box.width <= 1 &&
            box.height >= 0 && box.height <= 1
        );
    }

    /**
     * Generates cache key from filters
     */
    private generateCacheKey(filters: IAnalyticsFilters): string {
        return `${filters.startDate.getTime()}-${filters.endDate.getTime()}-${filters.type}-${filters.confidence}`;
    }

    /**
     * Generates unique stream identifier
     */
    private generateStreamId(filters: IAnalyticsFilters): string {
        return `stream-${this.generateCacheKey(filters)}`;
    }

    /**
     * Retrieves cached data if valid
     */
    private getCachedData(key: string): IAnalytics[] | null {
        const cached = this.cache.get(key);
        if (cached && Date.now() < cached.expiresAt) {
            return cached.data;
        }
        this.cache.delete(key);
        return null;
    }

    /**
     * Caches analytics data with expiration
     */
    private cacheData(key: string, data: IAnalytics[]): void {
        if (this.cache.size >= MAX_CACHE_SIZE) {
            const oldestKey = Array.from(this.cache.keys())[0];
            this.cache.delete(oldestKey);
        }

        this.cache.set(key, {
            data,
            timestamp: Date.now(),
            expiresAt: Date.now() + CACHE_DURATION
        });
    }

    /**
     * Closes specific analytics stream
     */
    private closeStream(streamId: string): void {
        const cleanup = this.activeStreams.get(streamId);
        if (cleanup) {
            cleanup();
            this.activeStreams.delete(streamId);
        }
    }

    private debouncedGetAnalytics: (filters: IAnalyticsFilters) => Promise<IAnalytics[]>;
}

// Export singleton instance
export const analyticsService = new AnalyticsService(new ApiClient());