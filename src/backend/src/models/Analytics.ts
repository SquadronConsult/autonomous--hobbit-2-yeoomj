// @turf/helpers v6.5.0
import { Point } from '@turf/helpers';
// uuid v9.0.0
import { v4 as uuidv4 } from 'uuid';
// opossum v6.0.0
import CircuitBreaker from 'opossum';
// ioredis v5.3.0
import Redis from 'ioredis';

import { IAnalytics } from '../interfaces/IAnalytics';
import { executeQuery } from '../utils/database';
import { validateGeolocation } from '../utils/validation';
import { logger } from '../utils/logger';
import { ErrorCodes } from '../constants/errorCodes';

// Constants for analytics processing
const MIN_CONFIDENCE_THRESHOLD = 0.95;
const MAX_DETECTIONS_PER_RECORD = 100;
const CACHE_TTL_SECONDS = 300;

// Redis cache configuration
const REDIS_CONFIG = {
    host: process.env.REDIS_HOST || 'localhost',
    port: parseInt(process.env.REDIS_PORT || '6379'),
    password: process.env.REDIS_PASSWORD,
    db: 0,
    retryStrategy: (times: number) => Math.min(times * 50, 2000)
};

/**
 * High-performance analytics model class for agricultural data management
 * Implements real-time video analytics with caching and batch processing capabilities
 */
@CircuitBreaker({ timeout: 1000, resetTimeout: 5000 })
export class Analytics implements IAnalytics {
    public id: string;
    public missionId: string;
    public deviceId: string;
    public timestamp: Date;
    public location: Point;
    public type: string;
    public confidence: number;
    public detections: any[];
    public metadata: Record<string, any>;
    private cache: Redis;

    /**
     * Creates a new Analytics instance with data validation and caching setup
     * @param data Initial analytics data
     */
    constructor(data: IAnalytics) {
        // Validate and initialize data
        if (!data.confidence || data.confidence < MIN_CONFIDENCE_THRESHOLD) {
            throw new Error(`Confidence must be at least ${MIN_CONFIDENCE_THRESHOLD}`);
        }

        if (data.detections && data.detections.length > MAX_DETECTIONS_PER_RECORD) {
            throw new Error(`Maximum detections per record is ${MAX_DETECTIONS_PER_RECORD}`);
        }

        // Validate location data
        if (!validateGeolocation(data.location)) {
            throw new Error('Invalid location coordinates');
        }

        // Initialize properties
        this.id = data.id || uuidv4();
        this.missionId = data.missionId;
        this.deviceId = data.deviceId;
        this.timestamp = data.timestamp;
        this.location = data.location;
        this.type = data.type;
        this.confidence = data.confidence;
        this.detections = data.detections || [];
        this.metadata = data.metadata || {};

        // Initialize Redis cache
        this.cache = new Redis(REDIS_CONFIG);
        this.cache.on('error', (err) => {
            logger.error(err, {
                correlationId: `analytics-cache-${this.id}`,
                component: 'Analytics',
                operation: 'cache-error'
            });
        });
    }

    /**
     * Persists analytics data to TimescaleDB with caching
     * @returns Promise resolving on successful save
     */
    public async save(): Promise<void> {
        const query = `
            INSERT INTO analytics (
                id, mission_id, device_id, timestamp, location, type,
                confidence, detections, metadata
            ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
            ON CONFLICT (id) DO UPDATE SET
                confidence = EXCLUDED.confidence,
                detections = EXCLUDED.detections,
                metadata = EXCLUDED.metadata;
        `;

        const params = [
            this.id,
            this.missionId,
            this.deviceId,
            this.timestamp,
            this.location,
            this.type,
            this.confidence,
            JSON.stringify(this.detections),
            this.metadata
        ];

        try {
            await executeQuery(query, params);
            await this.cache.setex(
                `analytics:${this.id}`,
                CACHE_TTL_SECONDS,
                JSON.stringify(this)
            );
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `analytics-save-${this.id}`,
                component: 'Analytics',
                operation: 'save'
            });
            throw new Error(ErrorCodes.TELEMETRY_PROCESSING_ERROR.toString());
        }
    }

    /**
     * Retrieves analytics records for a specific mission with caching
     * @param missionId Mission identifier
     * @returns Promise resolving to array of Analytics instances
     */
    public static async findByMissionId(missionId: string): Promise<Analytics[]> {
        const cacheKey = `analytics:mission:${missionId}`;
        const cache = new Redis(REDIS_CONFIG);

        try {
            // Check cache first
            const cachedData = await cache.get(cacheKey);
            if (cachedData) {
                return JSON.parse(cachedData).map((data: IAnalytics) => new Analytics(data));
            }

            // Query database if cache miss
            const query = `
                SELECT * FROM analytics 
                WHERE mission_id = $1 
                ORDER BY timestamp DESC;
            `;
            const result = await executeQuery(query, [missionId]);
            const analytics = result.rows.map(row => new Analytics(row));

            // Update cache
            await cache.setex(
                cacheKey,
                CACHE_TTL_SECONDS,
                JSON.stringify(analytics)
            );

            return analytics;
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `analytics-find-mission-${missionId}`,
                component: 'Analytics',
                operation: 'findByMissionId'
            });
            throw new Error(ErrorCodes.TELEMETRY_PROCESSING_ERROR.toString());
        }
    }

    /**
     * Retrieves analytics records within a time range using TimescaleDB optimizations
     * @param startTime Range start time
     * @param endTime Range end time
     * @returns Promise resolving to array of Analytics instances
     */
    public static async findByTimeRange(startTime: Date, endTime: Date): Promise<Analytics[]> {
        const cacheKey = `analytics:timerange:${startTime.getTime()}-${endTime.getTime()}`;
        const cache = new Redis(REDIS_CONFIG);

        try {
            // Check cache first
            const cachedData = await cache.get(cacheKey);
            if (cachedData) {
                return JSON.parse(cachedData).map((data: IAnalytics) => new Analytics(data));
            }

            // Query database with time-series optimization
            const query = `
                SELECT * FROM analytics 
                WHERE timestamp >= $1 AND timestamp <= $2
                ORDER BY timestamp DESC;
            `;
            const result = await executeQuery(query, [startTime, endTime]);
            const analytics = result.rows.map(row => new Analytics(row));

            // Update cache
            await cache.setex(
                cacheKey,
                CACHE_TTL_SECONDS,
                JSON.stringify(analytics)
            );

            return analytics;
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `analytics-find-timerange-${startTime}-${endTime}`,
                component: 'Analytics',
                operation: 'findByTimeRange'
            });
            throw new Error(ErrorCodes.TELEMETRY_PROCESSING_ERROR.toString());
        }
    }
}