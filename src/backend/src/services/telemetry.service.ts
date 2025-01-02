/**
 * @fileoverview Service layer for managing real-time telemetry data from agricultural robots and drones
 * @version 1.0.0
 * @license MIT
 */

import Redis from 'ioredis'; // v5.3.2
import { ITelemetry } from '../interfaces/ITelemetry';
import { Telemetry } from '../models/Telemetry';
import { logger } from '../utils/logger';
import { metrics } from '../utils/metrics';

// Cache configuration constants
const CACHE_TTL = 60; // 60 seconds
const BATCH_SIZE = 100;
const RETRY_ATTEMPTS = 3;
const RETRY_DELAY = 1000; // 1 second

// Query options interface
interface QueryOptions {
    limit?: number;
    offset?: number;
    orderBy?: string;
    timeInterval?: string;
    aggregation?: 'avg' | 'min' | 'max' | 'count';
}

/**
 * Service class for managing telemetry data operations with Redis caching
 * and performance optimizations for sub-100ms latency
 */
export class TelemetryService {
    private _telemetryModel: Telemetry;
    private _cache: Redis.Cluster;
    private _batchSize: number;
    private _cacheTTL: number;
    private _batchQueue: ITelemetry[] = [];
    private _processingBatch: boolean = false;

    /**
     * Initializes telemetry service with model, cache, and performance configurations
     */
    constructor() {
        // Initialize Telemetry model with optimized batch size
        this._telemetryModel = new Telemetry(BATCH_SIZE);
        this._batchSize = BATCH_SIZE;
        this._cacheTTL = CACHE_TTL;

        // Initialize Redis cluster with failover support
        this._cache = new Redis.Cluster([
            {
                host: process.env.REDIS_HOST || 'localhost',
                port: parseInt(process.env.REDIS_PORT || '6379')
            }
        ], {
            redisOptions: {
                enableReadyCheck: true,
                maxRetriesPerRequest: RETRY_ATTEMPTS
            },
            clusterRetryStrategy: (times: number) => {
                return Math.min(times * RETRY_DELAY, 5000);
            }
        });

        // Set up cache error handling
        this._cache.on('error', (error: Error) => {
            logger.error(error, {
                correlationId: `cache-error-${Date.now()}`,
                component: 'TelemetryService',
                operation: 'cache-error'
            });
        });

        // Initialize batch processing
        this._initializeBatchProcessing();
    }

    /**
     * Creates new telemetry records with batching and caching
     * @param dataArray Array of telemetry data to create
     * @returns Promise<ITelemetry[]> Created telemetry records
     */
    public async createTelemetry(dataArray: Partial<ITelemetry>[]): Promise<ITelemetry[]> {
        try {
            const startTime = Date.now();
            const createdRecords: ITelemetry[] = [];

            // Process each telemetry record
            for (const data of dataArray) {
                const record = await this._telemetryModel.create(data);
                createdRecords.push(record);

                // Update cache with new record
                await this._updateCache(record);

                // Add to batch queue
                this._batchQueue.push(record);
            }

            // Update device metrics
            metrics.updateDeviceCount('all', 'active', 'all', dataArray.length);

            // Record processing latency
            const latency = (Date.now() - startTime) / 1000;
            metrics.recordVideoLatency('telemetry', 'processing', latency);

            logger.info('Telemetry records created', {
                correlationId: `telemetry-create-${Date.now()}`,
                component: 'TelemetryService',
                operation: 'createTelemetry',
                details: { count: createdRecords.length, latency }
            });

            return createdRecords;
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `telemetry-create-${Date.now()}`,
                component: 'TelemetryService',
                operation: 'createTelemetry'
            });
            throw error;
        }
    }

    /**
     * Retrieves telemetry data for devices with caching and pagination
     * @param deviceIds Array of device IDs
     * @param options Query options for filtering and pagination
     * @returns Promise<ITelemetry[]> Array of telemetry records
     */
    public async getTelemetryByDevice(
        deviceIds: string[],
        options: QueryOptions = {}
    ): Promise<ITelemetry[]> {
        try {
            const startTime = Date.now();
            const results: ITelemetry[] = [];

            // Check cache for each device
            for (const deviceId of deviceIds) {
                const cacheKey = `telemetry:${deviceId}`;
                const cachedData = await this._cache.get(cacheKey);

                if (cachedData) {
                    results.push(...JSON.parse(cachedData));
                } else {
                    // Cache miss - fetch from database
                    const dbData = await this._telemetryModel.findByDeviceId(deviceId, options);
                    results.push(...dbData);

                    // Update cache
                    await this._updateCache(dbData[0]);
                }
            }

            // Record latency
            const latency = (Date.now() - startTime) / 1000;
            metrics.recordVideoLatency('telemetry', 'retrieval', latency);

            return this._applyQueryOptions(results, options);
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `telemetry-get-${Date.now()}`,
                component: 'TelemetryService',
                operation: 'getTelemetryByDevice'
            });
            throw error;
        }
    }

    /**
     * Retrieves time-series telemetry data with optimization
     * @param startTime Start of time range
     * @param endTime End of time range
     * @param options Query options for filtering and aggregation
     * @returns Promise<ITelemetry[]> Array of telemetry records
     */
    public async getTelemetryInTimeRange(
        startTime: Date,
        endTime: Date,
        options: QueryOptions = {}
    ): Promise<ITelemetry[]> {
        try {
            const queryStartTime = Date.now();
            
            // Get data from database with time-series optimization
            const results = await this._telemetryModel.findByTimeRange(
                startTime,
                endTime,
                options
            );

            // Record resource usage
            metrics.recordResourceUsage('database', 'telemetry', 
                process.memoryUsage().heapUsed / process.memoryUsage().heapTotal * 100);

            // Record query latency
            const latency = (Date.now() - queryStartTime) / 1000;
            metrics.recordVideoLatency('telemetry', 'timerange', latency);

            return results;
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `telemetry-timerange-${Date.now()}`,
                component: 'TelemetryService',
                operation: 'getTelemetryInTimeRange'
            });
            throw error;
        }
    }

    /**
     * Updates cache with telemetry data
     * @param data Telemetry data to cache
     * @private
     */
    private async _updateCache(data: ITelemetry): Promise<void> {
        try {
            const cacheKey = `telemetry:${data.deviceId}`;
            await this._cache.setex(
                cacheKey,
                this._cacheTTL,
                JSON.stringify(data)
            );
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `cache-update-${Date.now()}`,
                component: 'TelemetryService',
                operation: '_updateCache'
            });
        }
    }

    /**
     * Initializes batch processing for telemetry data
     * @private
     */
    private _initializeBatchProcessing(): void {
        setInterval(async () => {
            if (this._batchQueue.length >= this._batchSize && !this._processingBatch) {
                this._processingBatch = true;
                try {
                    const batch = this._batchQueue.splice(0, this._batchSize);
                    await this._telemetryModel.create(batch[0]);
                } catch (error) {
                    logger.error(error as Error, {
                        correlationId: `batch-process-${Date.now()}`,
                        component: 'TelemetryService',
                        operation: '_processBatch'
                    });
                } finally {
                    this._processingBatch = false;
                }
            }
        }, 1000);
    }

    /**
     * Applies query options to results
     * @param results Raw query results
     * @param options Query options
     * @private
     */
    private _applyQueryOptions(
        results: ITelemetry[],
        options: QueryOptions
    ): ITelemetry[] {
        const { limit = 100, offset = 0, orderBy = 'timestamp' } = options;

        // Apply sorting
        results.sort((a, b) => {
            return orderBy === 'timestamp' 
                ? b.timestamp.getTime() - a.timestamp.getTime()
                : 0;
        });

        // Apply pagination
        return results.slice(offset, offset + limit);
    }
}