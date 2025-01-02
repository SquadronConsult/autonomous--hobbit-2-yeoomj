/**
 * @fileoverview Custom React hook for managing analytics data with real-time updates
 * Implements analytics data fetching, state management and validation for the agricultural management system
 * @version 1.0.0
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import { AnalyticsService } from '../services/analytics';
import { IAnalytics, IAnalyticsFilters, MIN_CONFIDENCE_THRESHOLD } from '../interfaces/IAnalytics';

// Constants for analytics hook configuration
const DEFAULT_REFRESH_INTERVAL = 30000; // 30 seconds
const REQUEST_TIMEOUT = 5000; // 5 seconds
const MAX_CACHE_SIZE = 1000;
const ERROR_RETRY_ATTEMPTS = 3;

/**
 * Options interface for useAnalytics hook configuration
 */
interface UseAnalyticsOptions {
    missionId?: string;
    refreshInterval?: number;
    filters: IAnalyticsFilters;
    enableAutoRefresh?: boolean;
    confidenceThreshold?: number;
}

/**
 * Error interface for analytics-specific errors
 */
interface AnalyticsError extends Error {
    code: string;
    context?: Record<string, any>;
}

/**
 * Return type interface for useAnalytics hook
 */
interface UseAnalyticsReturn {
    analytics: IAnalytics[];
    loading: boolean;
    error: AnalyticsError | null;
    refetch: () => Promise<void>;
    pauseAutoRefresh: () => void;
    resumeAutoRefresh: () => void;
}

/**
 * Custom hook for managing analytics data with real-time updates
 * @param options Hook configuration options
 * @returns Analytics data and control functions
 */
export const useAnalytics = ({
    missionId,
    refreshInterval = DEFAULT_REFRESH_INTERVAL,
    filters,
    enableAutoRefresh = true,
    confidenceThreshold = MIN_CONFIDENCE_THRESHOLD
}: UseAnalyticsOptions): UseAnalyticsReturn => {
    // Initialize state
    const [analytics, setAnalytics] = useState<IAnalytics[]>([]);
    const [loading, setLoading] = useState<boolean>(true);
    const [error, setError] = useState<AnalyticsError | null>(null);
    const [isPaused, setIsPaused] = useState<boolean>(!enableAutoRefresh);

    // Service and cleanup references
    const analyticsService = useRef(new AnalyticsService());
    const abortController = useRef<AbortController>();
    const refreshTimeout = useRef<NodeJS.Timeout>();
    const retryCount = useRef<number>(0);

    /**
     * Fetches analytics data with validation and error handling
     */
    const fetchAnalytics = useCallback(async () => {
        try {
            // Cancel any pending requests
            abortController.current?.abort();
            abortController.current = new AbortController();

            setLoading(true);
            setError(null);

            // Validate confidence threshold
            const validatedThreshold = Math.max(confidenceThreshold, MIN_CONFIDENCE_THRESHOLD);

            let data: IAnalytics[];
            if (missionId) {
                data = await analyticsService.current.getMissionAnalytics(
                    missionId,
                    filters,
                    validatedThreshold,
                    { signal: abortController.current.signal }
                );
            } else {
                data = await analyticsService.current.getAnalytics(
                    filters,
                    validatedThreshold,
                    { signal: abortController.current.signal }
                );
            }

            // Validate response data
            const validatedData = data.filter(entry => 
                analyticsService.current.validateAnalyticsResponse(entry, validatedThreshold)
            );

            setAnalytics(validatedData);
            retryCount.current = 0;
        } catch (err) {
            const analyticsError: AnalyticsError = {
                name: 'AnalyticsError',
                message: err instanceof Error ? err.message : 'Failed to fetch analytics',
                code: 'FETCH_ERROR',
                context: {
                    missionId,
                    filters,
                    retryCount: retryCount.current
                }
            };

            setError(analyticsError);

            // Implement retry logic
            if (retryCount.current < ERROR_RETRY_ATTEMPTS) {
                retryCount.current++;
                setTimeout(fetchAnalytics, refreshInterval / 2);
            }
        } finally {
            setLoading(false);
        }
    }, [missionId, filters, confidenceThreshold, refreshInterval]);

    /**
     * Sets up auto-refresh functionality
     */
    useEffect(() => {
        const setupRefresh = () => {
            if (!isPaused && enableAutoRefresh) {
                refreshTimeout.current = setTimeout(fetchAnalytics, refreshInterval);
            }
        };

        // Initial fetch
        fetchAnalytics().then(setupRefresh);

        return () => {
            refreshTimeout.current && clearTimeout(refreshTimeout.current);
            abortController.current?.abort();
        };
    }, [fetchAnalytics, refreshInterval, isPaused, enableAutoRefresh]);

    /**
     * Pauses auto-refresh functionality
     */
    const pauseAutoRefresh = useCallback(() => {
        setIsPaused(true);
        refreshTimeout.current && clearTimeout(refreshTimeout.current);
    }, []);

    /**
     * Resumes auto-refresh functionality
     */
    const resumeAutoRefresh = useCallback(() => {
        setIsPaused(false);
        fetchAnalytics();
    }, [fetchAnalytics]);

    /**
     * Manual refetch function
     */
    const refetch = useCallback(async () => {
        refreshTimeout.current && clearTimeout(refreshTimeout.current);
        await fetchAnalytics();
        if (!isPaused && enableAutoRefresh) {
            refreshTimeout.current = setTimeout(fetchAnalytics, refreshInterval);
        }
    }, [fetchAnalytics, refreshInterval, isPaused, enableAutoRefresh]);

    return {
        analytics,
        loading,
        error,
        refetch,
        pauseAutoRefresh,
        resumeAutoRefresh
    };
};

export type { UseAnalyticsOptions, UseAnalyticsReturn, AnalyticsError };