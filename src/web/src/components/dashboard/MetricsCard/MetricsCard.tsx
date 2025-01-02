import React, { useCallback, useEffect } from 'react';
import styled from '@emotion/styled';
import { Card } from '../../common/Card/Card';
import { useAnalytics } from '../../../hooks/useAnalytics';
import { AnalyticsType } from '../../../interfaces/IAnalytics';

// Props interface for MetricsCard component
interface MetricsCardProps {
  title: string;
  type: AnalyticsType;
  value: number;
  unit?: string;
  trend?: number;
  refreshInterval?: number;
  onClick?: (event: React.MouseEvent) => void;
}

// Styled components with Material Design 3.0 compliance
const MetricsContainer = styled.div<{ isClickable: boolean }>`
  display: flex;
  flex-direction: column;
  gap: 8px;
  padding: 16px;
  min-width: 200px;
  transition: all 0.2s ease-in-out;
  cursor: ${props => props.isClickable ? 'pointer' : 'default'};

  @media (max-width: 768px) {
    padding: 12px;
  }
`;

const MetricsTitle = styled.h3`
  font-size: 16px;
  font-weight: 500;
  color: ${props => props.theme.text.primary};
  margin: 0;
  line-height: 1.5;
  font-family: 'Roboto', 'Noto Sans', sans-serif;
`;

const MetricsValue = styled.p`
  font-size: clamp(20px, 4vw, 24px);
  font-weight: bold;
  color: ${props => props.theme.text.primary};
  margin: 0;
  line-height: 1.2;
  font-family: 'Roboto', 'Noto Sans', sans-serif;
`;

const MetricsTrend = styled.span<{ trend: number }>`
  font-size: 14px;
  font-weight: 500;
  color: ${props => props.trend > 0 ? props.theme.status.success : props.theme.status.error};
  display: flex;
  align-items: center;
  gap: 4px;
  font-family: 'Roboto', 'Noto Sans', sans-serif;

  &::before {
    content: '${props => props.trend > 0 ? '↑' : '↓'}';
  }
`;

const LoadingOverlay = styled.div`
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(255, 255, 255, 0.7);
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 8px;
`;

/**
 * Formats metric values with appropriate units and separators
 */
const formatValue = (value: number, unit?: string): string => {
  const formattedValue = new Intl.NumberFormat('en-US', {
    maximumFractionDigits: 2,
    minimumFractionDigits: 0
  }).format(value);
  
  return unit ? `${formattedValue} ${unit}` : formattedValue;
};

/**
 * MetricsCard component for displaying real-time analytics metrics
 * Implements Material Design 3.0 standards and WCAG 2.1 Level AA accessibility
 */
const MetricsCard: React.FC<MetricsCardProps> = React.memo(({
  title,
  type,
  value,
  unit,
  trend,
  refreshInterval = 5000,
  onClick
}) => {
  // Analytics hook for real-time data
  const { analytics, loading, error, refetch } = useAnalytics({
    filters: {
      type,
      startDate: new Date(Date.now() - 3600000), // Last hour
      endDate: new Date(),
      deviceId: 'all'
    },
    refreshInterval,
    enableAutoRefresh: true
  });

  // Handle automatic refresh
  useEffect(() => {
    const refreshTimer = setInterval(() => {
      refetch();
    }, refreshInterval);

    return () => {
      clearInterval(refreshTimer);
    };
  }, [refreshInterval, refetch]);

  // Handle click events
  const handleClick = useCallback((event: React.MouseEvent) => {
    if (onClick && !loading) {
      onClick(event);
    }
  }, [onClick, loading]);

  // Generate ARIA label
  const ariaLabel = `${title}: ${formatValue(value, unit)}${trend ? `, ${trend > 0 ? 'up' : 'down'} ${Math.abs(trend)}%` : ''}`;

  return (
    <Card
      variant="elevated"
      onClick={handleClick}
      ariaLabel={ariaLabel}
    >
      <MetricsContainer isClickable={Boolean(onClick)}>
        <MetricsTitle>{title}</MetricsTitle>
        <MetricsValue>
          {formatValue(value, unit)}
        </MetricsValue>
        {trend !== undefined && (
          <MetricsTrend trend={trend}>
            {Math.abs(trend)}%
          </MetricsTrend>
        )}
        {loading && (
          <LoadingOverlay role="progressbar" aria-label="Loading metrics">
            {/* Loading indicator would be implemented here */}
          </LoadingOverlay>
        )}
        {error && (
          <span role="alert" style={{ color: 'error.main', fontSize: '12px' }}>
            Failed to load metrics
          </span>
        )}
      </MetricsContainer>
    </Card>
  );
});

// Set display name for dev tools
MetricsCard.displayName = 'MetricsCard';

export default MetricsCard;
export type { MetricsCardProps };