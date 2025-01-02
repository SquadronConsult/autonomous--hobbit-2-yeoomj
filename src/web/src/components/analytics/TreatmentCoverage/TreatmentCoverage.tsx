import React, { useEffect, useMemo, useCallback } from 'react';
import { PieChart } from '../../common/Chart/PieChart';
import { useAnalytics } from '../../../hooks/useAnalytics';
import { IAnalytics, AnalyticsType } from '../../../interfaces/IAnalytics';

// Constants for component configuration
const DEFAULT_REFRESH_INTERVAL = 30000; // 30 seconds
const CHART_LABELS = ['Treated Area', 'Untreated Area'];
const ERROR_MESSAGES = {
    INVALID_DATA: 'Invalid analytics data received',
    CALCULATION_ERROR: 'Error calculating coverage data',
    NETWORK_ERROR: 'Failed to fetch analytics data'
} as const;
const ARIA_DESCRIPTIONS = {
    CHART: 'Pie chart showing treatment coverage statistics',
    LOADING: 'Loading treatment coverage data',
    ERROR: 'Error loading treatment coverage data'
} as const;

/**
 * Props interface for TreatmentCoverage component
 */
interface ITreatmentCoverageProps {
    /** ID of the mission to display coverage for */
    missionId: string;
    /** Interval for data refresh in milliseconds */
    refreshInterval?: number;
    /** Optional CSS class name for styling */
    className?: string;
    /** Optional error callback handler */
    onError?: (error: Error) => void;
    /** Accessibility label for the chart */
    ariaLabel?: string;
}

/**
 * Calculates coverage data from analytics
 * @param analytics Array of analytics data
 * @returns Processed coverage data for visualization
 */
const calculateCoverageData = (analytics: IAnalytics[]) => {
    try {
        // Filter for treatment coverage analytics
        const coverageData = analytics.filter(
            entry => entry.type === AnalyticsType.TREATMENT_COVERAGE
        );

        if (!coverageData.length) {
            return null;
        }

        // Calculate total and treated areas
        const totalArea = coverageData.reduce((sum, entry) => 
            sum + (entry.metadata.totalArea || 0), 0);
        const treatedArea = coverageData.reduce((sum, entry) => 
            sum + (entry.metadata.treatedArea || 0), 0);

        if (totalArea <= 0) {
            return null;
        }

        // Calculate percentages
        const treatedPercentage = (treatedArea / totalArea) * 100;
        const untreatedPercentage = 100 - treatedPercentage;

        return [
            {
                label: CHART_LABELS[0],
                value: treatedPercentage
            },
            {
                label: CHART_LABELS[1],
                value: untreatedPercentage
            }
        ];
    } catch (error) {
        console.error('Error calculating coverage data:', error);
        return null;
    }
};

/**
 * TreatmentCoverage component for visualizing treatment coverage analytics
 * @component
 */
const TreatmentCoverage: React.FC<ITreatmentCoverageProps> = React.memo(({
    missionId,
    refreshInterval = DEFAULT_REFRESH_INTERVAL,
    className,
    onError,
    ariaLabel = ARIA_DESCRIPTIONS.CHART
}) => {
    // Initialize analytics hook with mission-specific filters
    const { analytics, loading, error, refetch } = useAnalytics({
        missionId,
        refreshInterval,
        filters: {
            type: AnalyticsType.TREATMENT_COVERAGE,
            startDate: new Date(Date.now() - 24 * 60 * 60 * 1000), // Last 24 hours
            endDate: new Date(),
            deviceId: '*' // All devices
        },
        enableAutoRefresh: true
    });

    // Set up automatic refresh interval
    useEffect(() => {
        const refreshTimer = setInterval(refetch, refreshInterval);
        return () => clearInterval(refreshTimer);
    }, [refreshInterval, refetch]);

    // Process analytics data for visualization
    const coverageData = useMemo(() => {
        if (!analytics.length) return null;
        return calculateCoverageData(analytics);
    }, [analytics]);

    // Error handling callback
    const handleError = useCallback((err: Error) => {
        console.error('Treatment coverage error:', err);
        onError?.(err);
    }, [onError]);

    // Loading state
    if (loading) {
        return (
            <div 
                className={className}
                role="status"
                aria-label={ARIA_DESCRIPTIONS.LOADING}
            >
                <div className="loading-spinner" />
            </div>
        );
    }

    // Error state
    if (error || !coverageData) {
        return (
            <div 
                className={`${className} error-container`}
                role="alert"
                aria-label={ARIA_DESCRIPTIONS.ERROR}
            >
                <p className="error-message">
                    {error?.message || ERROR_MESSAGES.INVALID_DATA}
                </p>
            </div>
        );
    }

    // Render pie chart with coverage data
    return (
        <PieChart
            data={coverageData}
            className={className}
            colorScheme="success"
            title="Treatment Coverage Analysis"
            animate={true}
            onError={handleError}
            aria-label={ariaLabel}
        />
    );
});

// Display name for debugging
TreatmentCoverage.displayName = 'TreatmentCoverage';

export default TreatmentCoverage;