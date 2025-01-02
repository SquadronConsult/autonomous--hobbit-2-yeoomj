import React, { useCallback, useEffect, useMemo, useState } from 'react';
import styled from '@emotion/styled';
import { useMediaQuery } from '@mui/material';
import { ErrorBoundary } from 'react-error-boundary';
import Layout from '../../components/common/Layout/Layout';
import PerformanceGraph from '../../components/analytics/PerformanceGraph/PerformanceGraph';
import { useAnalytics } from '../../hooks/useAnalytics';
import { useAuth } from '../../hooks/useAuth';
import { AnalyticsType } from '../../interfaces/IAnalytics';
import Loading from '../../components/common/Loading/Loading';

// Constants
const REFRESH_INTERVAL = 30000; // 30 seconds
const ERROR_MESSAGES = {
  LOAD_ERROR: 'Failed to load analytics data',
  UPDATE_ERROR: 'Failed to update analytics data',
  NETWORK_ERROR: 'Network connection error'
} as const;

const ACCESSIBILITY_LABELS = {
  MAIN_SECTION: 'Analytics Dashboard',
  PERFORMANCE_SECTION: 'System Performance Metrics',
  DETECTION_SECTION: 'Pest Detection Analytics',
  TREATMENT_SECTION: 'Treatment Coverage Data'
} as const;

// Styled Components
const AnalyticsContainer = styled.div`
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: ${props => props.theme.spacing(3)};
  padding: ${props => props.theme.spacing(3)};
  max-width: 1440px;
  margin: 0 auto;

  @media (max-width: ${props => props.theme.breakpoints.md}) {
    grid-template-columns: 1fr;
    padding: ${props => props.theme.spacing(2)};
  }
`;

const AnalyticsCard = styled.section`
  background-color: ${props => props.theme.colors.background.paper};
  border-radius: ${props => props.theme.shape.borderRadius}px;
  padding: ${props => props.theme.spacing(3)};
  box-shadow: ${props => props.theme.shadows[1]};
  transition: box-shadow 0.3s ease-in-out;
  position: relative;
  overflow: hidden;

  &:hover {
    box-shadow: ${props => props.theme.shadows[3]};
  }

  @media (prefers-reduced-motion: reduce) {
    transition: none;
  }
`;

const ErrorContainer = styled.div`
  padding: ${props => props.theme.spacing(2)};
  color: ${props => props.theme.colors.status.error};
  background-color: ${props => props.theme.colors.background.elevated};
  border-radius: ${props => props.theme.shape.borderRadius}px;
  margin-bottom: ${props => props.theme.spacing(2)};
`;

// Types
interface AnalyticsProps {
  className?: string;
  refreshInterval?: number;
}

interface AnalyticsState {
  loading: boolean;
  error: Error | null;
}

// Error Fallback Component
const ErrorFallback: React.FC<{ error: Error; resetErrorBoundary: () => void }> = ({
  error,
  resetErrorBoundary
}) => (
  <ErrorContainer role="alert">
    <h2>Error Loading Analytics</h2>
    <p>{error.message}</p>
    <button onClick={resetErrorBoundary}>Retry</button>
  </ErrorContainer>
);

// Main Component
const Analytics: React.FC<AnalyticsProps> = ({
  className,
  refreshInterval = REFRESH_INTERVAL
}) => {
  const { hasRole } = useAuth();
  const isMobile = useMediaQuery('(max-width: 768px)');
  const [state, setState] = useState<AnalyticsState>({
    loading: true,
    error: null
  });

  // Analytics hook for real-time data
  const { analytics, loading, error, refetch } = useAnalytics({
    filters: {
      startDate: new Date(Date.now() - 24 * 60 * 60 * 1000), // Last 24 hours
      endDate: new Date(),
      type: AnalyticsType.PEST_DETECTION,
      deviceId: 'all'
    },
    refreshInterval,
    enableAutoRefresh: true,
    confidenceThreshold: 0.95
  });

  // Error handling callback
  const handleError = useCallback((error: Error) => {
    console.error('Analytics error:', error);
    setState(prev => ({ ...prev, error }));
  }, []);

  // Refresh data handler
  const handleRefresh = useCallback(async () => {
    try {
      setState(prev => ({ ...prev, loading: true, error: null }));
      await refetch();
    } catch (error) {
      handleError(error as Error);
    } finally {
      setState(prev => ({ ...prev, loading: false }));
    }
  }, [refetch, handleError]);

  // Initial data load
  useEffect(() => {
    handleRefresh();
  }, [handleRefresh]);

  // Memoized analytics metrics
  const metrics = useMemo(() => {
    if (!analytics.length) return null;
    return {
      pestDetections: analytics.filter(a => a.type === AnalyticsType.PEST_DETECTION).length,
      averageConfidence: analytics.reduce((acc, curr) => acc + curr.confidence, 0) / analytics.length,
      totalCoverage: analytics.filter(a => a.type === AnalyticsType.TREATMENT_COVERAGE).length
    };
  }, [analytics]);

  return (
    <Layout>
      <ErrorBoundary
        FallbackComponent={ErrorFallback}
        onReset={handleRefresh}
      >
        <main
          className={className}
          role="main"
          aria-label={ACCESSIBILITY_LABELS.MAIN_SECTION}
        >
          <AnalyticsContainer>
            {/* Performance Metrics Section */}
            <AnalyticsCard
              role="region"
              aria-label={ACCESSIBILITY_LABELS.PERFORMANCE_SECTION}
            >
              <h2>System Performance</h2>
              <PerformanceGraph
                refreshInterval={refreshInterval}
                height={isMobile ? 200 : 300}
                onError={handleError}
              />
            </AnalyticsCard>

            {/* Pest Detection Analytics */}
            {hasRole('ANALYST') && (
              <AnalyticsCard
                role="region"
                aria-label={ACCESSIBILITY_LABELS.DETECTION_SECTION}
              >
                <h2>Pest Detection Analytics</h2>
                {loading ? (
                  <Loading size="lg" label="Loading pest detection data..." />
                ) : (
                  metrics && (
                    <div>
                      <p>Total Detections: {metrics.pestDetections}</p>
                      <p>Average Confidence: {(metrics.averageConfidence * 100).toFixed(1)}%</p>
                    </div>
                  )
                )}
              </AnalyticsCard>
            )}

            {/* Treatment Coverage Analytics */}
            {hasRole('ANALYST') && (
              <AnalyticsCard
                role="region"
                aria-label={ACCESSIBILITY_LABELS.TREATMENT_SECTION}
              >
                <h2>Treatment Coverage</h2>
                {loading ? (
                  <Loading size="lg" label="Loading treatment coverage data..." />
                ) : (
                  metrics && (
                    <div>
                      <p>Total Coverage: {metrics.totalCoverage} zones</p>
                    </div>
                  )
                )}
              </AnalyticsCard>
            )}
          </AnalyticsContainer>
        </main>
      </ErrorBoundary>
    </Layout>
  );
};

Analytics.displayName = 'Analytics';

export default Analytics;