import React, { useMemo, useCallback } from 'react';
import styled from '@emotion/styled';
import { debounce } from 'lodash'; // v4.17.21
import { ErrorBoundary } from 'react-error-boundary'; // v4.0.0

import LineChart from '../../common/Chart/LineChart';
import Card from '../../common/Card/Card';
import { useAnalytics } from '../../../hooks/useAnalytics';
import { CHART_COLORS } from '../../../constants/chartColors';

// Constants for threshold values and refresh intervals
const DEFAULT_REFRESH_INTERVAL = 30000; // 30 seconds
const CPU_THRESHOLD = 90;
const GPU_THRESHOLD = 85;
const MEMORY_THRESHOLD = 85;
const TEMPERATURE_THRESHOLD = 80;

// Styled components for layout
const MetricsContainer = styled.div`
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 1rem;
  padding: 1rem;
  width: 100%;
  max-width: 1440px;
  margin: 0 auto;

  @media (max-width: 768px) {
    grid-template-columns: 1fr;
  }
`;

const MetricCard = styled(Card)<{ hasAlert: boolean }>`
  min-height: 300px;
  padding: 1rem;
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  position: relative;
  background: var(--card-background);
  border-radius: 8px;
  transition: all 0.3s ease;

  &:focus-within {
    outline: 2px solid var(--focus-color);
  }

  ${props => props.hasAlert && `
    border: 2px solid ${CHART_COLORS.error[0]};
  `}
`;

// Component props interface
interface SystemMetricsProps {
  refreshInterval?: number;
  timeRange?: string;
  showThresholds?: boolean;
  alertCallback?: (metric: string, value: number) => void;
}

// Chart options configuration
const CHART_OPTIONS = {
  responsive: true,
  maintainAspectRatio: false,
  animation: {
    duration: 300
  },
  scales: {
    y: {
      beginAtZero: true,
      max: 100,
      title: {
        display: true,
        text: 'Utilization (%)'
      }
    },
    x: {
      type: 'time',
      time: {
        unit: 'minute'
      },
      title: {
        display: true,
        text: 'Time'
      }
    }
  },
  plugins: {
    tooltip: {
      enabled: true,
      mode: 'index'
    },
    legend: {
      display: true,
      position: 'top'
    }
  }
};

const SystemMetrics: React.FC<SystemMetricsProps> = React.memo(({
  refreshInterval = DEFAULT_REFRESH_INTERVAL,
  timeRange = '1h',
  showThresholds = true,
  alertCallback
}) => {
  // Analytics hook for fetching system metrics
  const { analytics, loading, error } = useAnalytics({
    refreshInterval,
    filters: {
      startDate: new Date(Date.now() - (timeRange === '1h' ? 3600000 : 86400000)),
      endDate: new Date(),
      type: 'SYSTEM_METRICS',
      deviceId: 'edge-system'
    },
    enableAutoRefresh: true
  });

  // Process metrics data for charts
  const processedData = useMemo(() => {
    if (!analytics.length) return null;

    const timestamps = analytics.map(entry => entry.timestamp);
    const cpuData = analytics.map(entry => entry.metadata.cpuUsage);
    const gpuData = analytics.map(entry => entry.metadata.gpuUsage);
    const memoryData = analytics.map(entry => entry.metadata.memoryUsage);
    const temperatureData = analytics.map(entry => entry.metadata.temperature);

    // Check thresholds and trigger alerts
    const debouncedAlert = debounce((metric: string, value: number) => {
      alertCallback?.(metric, value);
    }, 1000);

    const checkThresholds = (data: number[], threshold: number, metricName: string) => {
      const currentValue = data[data.length - 1];
      if (currentValue > threshold) {
        debouncedAlert(metricName, currentValue);
      }
    };

    if (showThresholds) {
      checkThresholds(cpuData, CPU_THRESHOLD, 'CPU Usage');
      checkThresholds(gpuData, GPU_THRESHOLD, 'GPU Usage');
      checkThresholds(memoryData, MEMORY_THRESHOLD, 'Memory Usage');
      checkThresholds(temperatureData, TEMPERATURE_THRESHOLD, 'Temperature');
    }

    return {
      timestamps,
      metrics: {
        cpu: cpuData,
        gpu: gpuData,
        memory: memoryData,
        temperature: temperatureData
      }
    };
  }, [analytics, showThresholds, alertCallback]);

  // Error handler for error boundary
  const handleError = useCallback((error: Error) => {
    console.error('SystemMetrics component error:', error);
  }, []);

  if (error) {
    return (
      <MetricsContainer>
        <MetricCard variant="elevated" hasAlert={true}>
          <div role="alert">Error loading system metrics: {error.message}</div>
        </MetricCard>
      </MetricsContainer>
    );
  }

  return (
    <ErrorBoundary fallback={<div>Error loading metrics</div>} onError={handleError}>
      <MetricsContainer>
        <MetricCard 
          variant="elevated" 
          hasAlert={processedData?.metrics.cpu.slice(-1)[0] > CPU_THRESHOLD}
          aria-label="CPU Usage Metrics"
        >
          <LineChart
            data={{
              labels: processedData?.timestamps.map(t => t.toISOString()) || [],
              datasets: [{
                label: 'CPU Usage',
                data: processedData?.metrics.cpu || [],
                borderColor: CHART_COLORS.primary[0],
                backgroundColor: CHART_COLORS.primary[4],
                fill: true
              }]
            }}
            options={CHART_OPTIONS}
            loading={loading}
            aria-label="CPU usage over time"
          />
        </MetricCard>

        <MetricCard 
          variant="elevated" 
          hasAlert={processedData?.metrics.gpu.slice(-1)[0] > GPU_THRESHOLD}
          aria-label="GPU Usage Metrics"
        >
          <LineChart
            data={{
              labels: processedData?.timestamps.map(t => t.toISOString()) || [],
              datasets: [{
                label: 'GPU Usage',
                data: processedData?.metrics.gpu || [],
                borderColor: CHART_COLORS.secondary[0],
                backgroundColor: CHART_COLORS.secondary[4],
                fill: true
              }]
            }}
            options={CHART_OPTIONS}
            loading={loading}
            aria-label="GPU usage over time"
          />
        </MetricCard>

        <MetricCard 
          variant="elevated" 
          hasAlert={processedData?.metrics.memory.slice(-1)[0] > MEMORY_THRESHOLD}
          aria-label="Memory Usage Metrics"
        >
          <LineChart
            data={{
              labels: processedData?.timestamps.map(t => t.toISOString()) || [],
              datasets: [{
                label: 'Memory Usage',
                data: processedData?.metrics.memory || [],
                borderColor: CHART_COLORS.success[0],
                backgroundColor: CHART_COLORS.success[4],
                fill: true
              }]
            }}
            options={CHART_OPTIONS}
            loading={loading}
            aria-label="Memory usage over time"
          />
        </MetricCard>

        <MetricCard 
          variant="elevated" 
          hasAlert={processedData?.metrics.temperature.slice(-1)[0] > TEMPERATURE_THRESHOLD}
          aria-label="Temperature Metrics"
        >
          <LineChart
            data={{
              labels: processedData?.timestamps.map(t => t.toISOString()) || [],
              datasets: [{
                label: 'Temperature',
                data: processedData?.metrics.temperature || [],
                borderColor: CHART_COLORS.warning[0],
                backgroundColor: CHART_COLORS.warning[4],
                fill: true
              }]
            }}
            options={CHART_OPTIONS}
            loading={loading}
            aria-label="System temperature over time"
          />
        </MetricCard>
      </MetricsContainer>
    </ErrorBoundary>
  );
});

SystemMetrics.displayName = 'SystemMetrics';

export default SystemMetrics;