import React, { useCallback, useEffect, useMemo } from 'react';
import styled from '@emotion/styled';
import { LineChart } from '../../common/Chart/LineChart';
import { useAnalytics } from '../../../hooks/useAnalytics';
import { IAnalytics } from '../../../interfaces/IAnalytics';
import { CHART_COLORS } from '../../../constants/chartColors';

// Constants
const DEFAULT_REFRESH_INTERVAL = 30000; // 30 seconds
const MAX_DATA_POINTS = 100; // Limit data points for performance

// Styled Components
const GraphContainer = styled.div<{
  width?: number;
  height?: number;
  theme: any;
}>`
  width: ${props => props.width ? `${props.width}px` : '100%'};
  height: ${props => props.height ? `${props.height}px` : '300px'};
  padding: ${props => props.theme.spacing(2)};
  background-color: ${props => props.theme.palette.background.paper};
  border-radius: ${props => props.theme.shape.borderRadius}px;
  box-shadow: ${props => props.theme.shadows[1]};
  transition: all 0.3s ease;

  @media (max-width: ${props => props.theme.breakpoints.md}) {
    height: 200px;
  }
`;

// Types
interface PerformanceGraphProps {
  refreshInterval?: number;
  height?: number;
  width?: number;
  theme: any;
  onError?: (error: Error) => void;
}

/**
 * Transforms analytics data into chart format with performance optimization
 */
const transformAnalyticsData = (analytics: IAnalytics[]) => {
  const sortedData = [...analytics]
    .sort((a, b) => a.timestamp.getTime() - b.timestamp.getTime())
    .slice(-MAX_DATA_POINTS);

  const labels = sortedData.map(entry => entry.timestamp);
  const cpuData = sortedData.map(entry => entry.metadata.cpuUsage || 0);
  const gpuData = sortedData.map(entry => entry.metadata.gpuUtilization || 0);

  return {
    labels,
    datasets: [
      {
        label: 'CPU Usage',
        data: cpuData,
        borderColor: CHART_COLORS.primary[0],
        backgroundColor: CHART_COLORS.primary[4],
        fill: true,
        tension: 0.4
      },
      {
        label: 'GPU Utilization',
        data: gpuData,
        borderColor: CHART_COLORS.secondary[0],
        backgroundColor: CHART_COLORS.secondary[4],
        fill: true,
        tension: 0.4
      }
    ]
  };
};

/**
 * Performance Graph Component
 * Displays real-time system performance metrics using Chart.js
 */
const PerformanceGraph: React.FC<PerformanceGraphProps> = React.memo(({
  refreshInterval = DEFAULT_REFRESH_INTERVAL,
  height,
  width,
  theme,
  onError
}) => {
  // Analytics hook for real-time data
  const { analytics, loading, error } = useAnalytics({
    refreshInterval,
    filters: {
      type: 'SYSTEM_METRICS',
      startDate: new Date(Date.now() - 3600000), // Last hour
      endDate: new Date(),
      deviceId: 'system'
    },
    enableAutoRefresh: true
  });

  // Error handling
  useEffect(() => {
    if (error && onError) {
      onError(error);
    }
  }, [error, onError]);

  // Transform data for chart
  const chartData = useMemo(() => {
    return transformAnalyticsData(analytics);
  }, [analytics]);

  // Chart options with theme integration
  const chartOptions = useMemo(() => ({
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      legend: {
        position: 'top' as const,
        align: 'end' as const,
        labels: {
          usePointStyle: true,
          color: theme.palette.text.primary
        }
      },
      tooltip: {
        mode: 'index' as const,
        intersect: false,
        backgroundColor: theme.palette.background.paper,
        titleColor: theme.palette.text.primary,
        bodyColor: theme.palette.text.secondary,
        borderColor: theme.palette.divider,
        borderWidth: 1
      }
    },
    scales: {
      x: {
        type: 'time',
        time: {
          unit: 'minute',
          displayFormats: {
            minute: 'HH:mm'
          }
        },
        grid: {
          display: false
        },
        ticks: {
          color: theme.palette.text.secondary
        }
      },
      y: {
        beginAtZero: true,
        max: 100,
        ticks: {
          callback: (value: number) => `${value}%`,
          color: theme.palette.text.secondary
        },
        grid: {
          color: theme.palette.divider
        }
      }
    }
  }), [theme]);

  return (
    <GraphContainer
      width={width}
      height={height}
      theme={theme}
      role="region"
      aria-label="System Performance Graph"
    >
      <LineChart
        data={chartData}
        options={chartOptions}
        loading={loading}
        height={height}
        width={width}
        theme={theme.palette.mode}
        aria-label="Real-time system performance metrics"
        gradient={true}
        reduceMotion={false}
      />
    </GraphContainer>
  );
});

PerformanceGraph.displayName = 'PerformanceGraph';

export default PerformanceGraph;