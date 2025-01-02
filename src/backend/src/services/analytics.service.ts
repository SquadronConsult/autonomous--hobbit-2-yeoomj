import { Injectable } from '@nestjs/common';
import { Point } from '@turf/helpers'; // v6.5.0
import * as promClient from 'prom-client'; // v14.2.0
import Redis from 'redis'; // v4.6.7
import CircuitBreaker from 'circuit-breaker-js'; // v0.0.1

import { IAnalytics, AnalyticsType, MIN_CONFIDENCE_THRESHOLD } from '../interfaces/IAnalytics';
import { Analytics } from '../models/Analytics';
import { logger } from '../utils/logger';
import { ErrorCodes } from '../constants/errorCodes';
import { validateGeolocation } from '../utils/validation';

// Constants for analytics processing
const BATCH_SIZE = 100;
const PROCESSING_TIMEOUT = 5000;
const CACHE_TTL = 300; // 5 minutes
const MAX_RETRIES = 3;

// Redis configuration
const REDIS_CONFIG = {
    host: process.env.REDIS_HOST || 'localhost',
    port: parseInt(process.env.REDIS_PORT || '6379'),
    password: process.env.REDIS_PASSWORD,
    db: 0
};

// Prometheus metrics
const analyticsCounter = new promClient.Counter({
    name: 'analytics_processed_total',
    help: 'Total number of analytics records processed',
    labelNames: ['type', 'status']
});

const processingLatencyHistogram = new promClient.Histogram({
    name: 'analytics_processing_latency_seconds',
    help: 'Analytics processing latency in seconds',
    labelNames: ['type', 'status'],
    buckets: [0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1]
});

const cacheHitRatio = new promClient.Gauge({
    name: 'analytics_cache_hit_ratio',
    help: 'Cache hit ratio for analytics queries',
    labelNames: ['operation']
});

@Injectable()
export class AnalyticsService {
    private readonly cache: Redis;
    private readonly circuitBreaker: CircuitBreaker;

    constructor() {
        // Initialize Redis cache
        this.cache = new Redis(REDIS_CONFIG);
        this.cache.on('error', (err) => {
            logger.error(err, {
                correlationId: `analytics-cache-${Date.now()}`,
                component: 'AnalyticsService',
                operation: 'cache-error'
            });
        });

        // Initialize circuit breaker
        this.circuitBreaker = new CircuitBreaker({
            windowDuration: 10000,
            numBuckets: 10,
            timeoutDuration: PROCESSING_TIMEOUT,
            errorThreshold: 50,
            volumeThreshold: 10
        });
    }

    /**
     * Creates a new analytics record with validation and metrics tracking
     * @param analyticsData Analytics data to process
     * @returns Created analytics record
     */
    public async createAnalytics(analyticsData: IAnalytics): Promise<Analytics> {
        const startTime = process.hrtime();

        try {
            // Validate confidence threshold
            if (analyticsData.confidence < MIN_CONFIDENCE_THRESHOLD) {
                throw new Error(`Confidence must be at least ${MIN_CONFIDENCE_THRESHOLD}`);
            }

            // Validate location data
            if (!validateGeolocation(analyticsData.location)) {
                throw new Error('Invalid location coordinates');
            }

            // Create and save analytics record
            const analytics = new Analytics(analyticsData);
            await analytics.save();

            // Update cache
            await this.cache.setex(
                `analytics:${analytics.id}`,
                CACHE_TTL,
                JSON.stringify(analytics)
            );

            // Record metrics
            const [seconds, nanoseconds] = process.hrtime(startTime);
            const duration = seconds + nanoseconds / 1e9;
            
            analyticsCounter.inc({ type: analyticsData.type, status: 'success' });
            processingLatencyHistogram.observe(
                { type: analyticsData.type, status: 'success' },
                duration
            );

            return analytics;
        } catch (error) {
            analyticsCounter.inc({ type: analyticsData.type, status: 'error' });
            logger.error(error as Error, {
                correlationId: `analytics-create-${Date.now()}`,
                component: 'AnalyticsService',
                operation: 'createAnalytics'
            });
            throw new Error(ErrorCodes.TELEMETRY_PROCESSING_ERROR.toString());
        }
    }

    /**
     * Processes real-time video analytics with performance monitoring
     * @param analyticsData Stream of analytics data to process
     */
    public async processRealTimeAnalytics(analyticsData: IAnalytics): Promise<void> {
        const startTime = process.hrtime();

        try {
            await this.circuitBreaker.execute(async () => {
                // Process analytics in batches for performance
                const batchPromises = [];
                for (let i = 0; i < analyticsData.detections.length; i += BATCH_SIZE) {
                    const batch = analyticsData.detections.slice(i, i + BATCH_SIZE);
                    batchPromises.push(this.processBatch(batch, analyticsData));
                }

                await Promise.all(batchPromises);

                // Record processing metrics
                const [seconds, nanoseconds] = process.hrtime(startTime);
                const duration = seconds + nanoseconds / 1e9;
                
                processingLatencyHistogram.observe(
                    { type: 'real_time', status: 'success' },
                    duration
                );
            });
        } catch (error) {
            analyticsCounter.inc({ type: 'real_time', status: 'error' });
            logger.error(error as Error, {
                correlationId: `analytics-realtime-${Date.now()}`,
                component: 'AnalyticsService',
                operation: 'processRealTimeAnalytics'
            });
            throw error;
        }
    }

    /**
     * Retrieves analytics by mission ID with caching
     * @param missionId Mission identifier
     * @returns Array of analytics records
     */
    public async getAnalyticsByMission(missionId: string): Promise<Analytics[]> {
        const cacheKey = `analytics:mission:${missionId}`;

        try {
            // Check cache first
            const cachedData = await this.cache.get(cacheKey);
            if (cachedData) {
                cacheHitRatio.inc({ operation: 'mission_lookup' });
                return JSON.parse(cachedData);
            }

            // Query database if cache miss
            const analytics = await Analytics.findByMissionId(missionId);
            
            // Update cache
            await this.cache.setex(
                cacheKey,
                CACHE_TTL,
                JSON.stringify(analytics)
            );

            return analytics;
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `analytics-mission-${missionId}`,
                component: 'AnalyticsService',
                operation: 'getAnalyticsByMission'
            });
            throw error;
        }
    }

    /**
     * Processes a batch of analytics detections
     * @param batch Batch of detections to process
     * @param baseData Base analytics data
     */
    private async processBatch(
        batch: any[],
        baseData: IAnalytics
    ): Promise<void> {
        try {
            const batchAnalytics = batch.map(detection => ({
                ...baseData,
                detections: [detection],
                confidence: detection.confidence
            }));

            await Promise.all(
                batchAnalytics.map(analytics => this.createAnalytics(analytics))
            );
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `analytics-batch-${Date.now()}`,
                component: 'AnalyticsService',
                operation: 'processBatch'
            });
            throw error;
        }
    }

    /**
     * Retrieves current analytics metrics
     * @returns Object containing current metric values
     */
    public async getMetrics(): Promise<any> {
        return {
            processedTotal: await analyticsCounter.get(),
            latencyHistogram: await processingLatencyHistogram.get(),
            cacheHitRatio: await cacheHitRatio.get()
        };
    }
}

export default AnalyticsService;