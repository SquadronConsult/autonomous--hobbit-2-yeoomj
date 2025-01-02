import React, { memo, useMemo, useEffect } from 'react';
import styled from '@emotion/styled';
import LineChart from '../../common/Chart/LineChart';
import { useAnalytics } from '../../../hooks/useAnalytics';
import { AnalyticsType } from '../../../interfaces/IAnalytics';
import Loading from '../../common/Loading/Loading';
import { CHART_COLORS } from '../../../constants/chartColors';

// Constants
const DEFAULT_REFRESH_INTERVAL = 30000;
const CONFIDENCE_THRESHOLD = 0.95;

// Interfaces
interface PestDetectionProps {
  missionId?: string;
  refreshInterval?: number;
  reduceMotion?: boolean;
}

// Styled Components
const Container = styled.div`
  display: flex;
  flex-direction: column;
  gap: ${props => props.theme.spacing(2)};
  padding: ${props => props.theme.spacing(3)};
  background-color: ${props => props.theme.palette.background.paper};
  border-radius: ${props => props.theme.shape.borderRadius}px;
  box-shadow: ${props => props.theme.shadows[2]};

  @media (prefers-reduced-motion: reduce) {
    transition: none;
  }

  @media (max-width: ${props => props.theme.breakpoints.md}) {
    padding: ${props => props.theme.spacing(2)};
  }
`;

const Header = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
`;

const Title = styled.h2`
  font-family: var(--font-family-primary);
  font-size: var(--font-size-xl);
  font-weight: var(--font-weight-medium);
  color: ${props => props.theme.palette.text.primary};
  margin: 0;
`;

const MetricContainer = styled.div`
  display: flex;
  gap: ${props => props.theme.spacing(2)};
`;

const Metric = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: ${props => props.theme.spacing(2)};
  background-color: ${props => props.theme.palette.background.default};
  border-radius: ${props => props.theme.shape.borderRadius}px;
  min-width: 120px;
`;

const MetricLabel = styled.span`
  font-size: var(--font-size-sm);
  color: ${props => props.theme.palette.text.secondary};
  margin-bottom: ${props => props.theme.spacing(1)};
`;

const MetricValue = styled.span`
  font-size: var(--font-size-lg);
  font-weight: var(--font-weight-bold);
  color: ${props => props.theme.palette.text.primary};
`;

// Main Component
const PestDetection: React.FC<PestDetectionProps> = memo(({
  missionId,
  refreshInterval = DEFAULT_REFRESH_INTERVAL,
  reduceMotion = false
}) => {
  // Analytics hook with confidence threshold validation
  const { analytics, loading, error } = useAnalytics({
    missionId,
    refreshInterval,
    filters: {
      type: AnalyticsType.PEST_DETECTION,
      confidence: CONFIDENCE_THRESHOLD
    },
    enableAutoRefresh: true
  });

  // Process analytics data for visualization
  const { chartData, metrics } = useMemo(() => {
    if (!analytics.length) {
      return { chartData: null, metrics: { total: 0, confidence: 0 } };
    }

    const validDetections = analytics.filter(a => a.confidence >= CONFIDENCE_THRESHOLD);
    const timeLabels = validDetections.map(a => a.timestamp.toISOString());
    const confidenceValues = validDetections.map(a => a.confidence);

    const averageConfidence = confidenceValues.reduce((acc, val) => acc + val, 0) / confidenceValues.length;

    return {
      chartData: {
        labels: timeLabels,
        datasets: [{
          label: 'Detection Confidence',
          data: confidenceValues,
          borderColor: CHART_COLORS.primary[0],
          backgroundColor: CHART_COLORS.primary[4],
          fill: true,
          tension: 0.4
        }]
      },
      metrics: {
        total: validDetections.length,
        confidence: averageConfidence
      }
    };
  }, [analytics]);

  // Chart options with accessibility considerations
  const chartOptions = useMemo(() => ({
    responsive: true,
    maintainAspectRatio: false,
    interaction: {
      intersect: false,
      mode: 'index'
    },
    plugins: {
      legend: {
        display: true,
        position: 'top' as const,
        labels: {
          font: {
            family: 'var(--font-family-primary)'
          }
        }
      },
      tooltip: {
        enabled: true,
        mode: 'index' as const,
        intersect: false,
        backgroundColor: 'rgba(0, 0, 0, 0.8)',
        titleFont: {
          family: 'var(--font-family-primary)'
        },
        bodyFont: {
          family: 'var(--font-family-primary)'
        }
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
        title: {
          display: true,
          text: 'Time'
        }
      },
      y: {
        beginAtZero: true,
        max: 1,
        title: {
          display: true,
          text: 'Confidence Level'
        }
      }
    }
  }), []);

  if (error) {
    return (
      <Container role="alert" aria-live="polite">
        <Title>Error loading pest detection data</Title>
        <p>{error.message}</p>
      </Container>
    );
  }

  return (
    <Container role="region" aria-label="Pest Detection Analytics">
      <Header>
        <Title>Pest Detection Analytics</Title>
        <MetricContainer>
          <Metric>
            <MetricLabel>Total Detections</MetricLabel>
            <MetricValue>{metrics?.total || 0}</MetricValue>
          </Metric>
          <Metric>
            <MetricLabel>Avg. Confidence</MetricLabel>
            <MetricValue>
              {metrics?.confidence ? `${(metrics.confidence * 100).toFixed(1)}%` : 'N/A'}
            </MetricValue>
          </Metric>
        </MetricContainer>
      </Header>

      {loading ? (
        <Loading 
          size="lg" 
          label="Loading pest detection data..."
        />
      ) : (
        <LineChart
          data={chartData}
          options={chartOptions}
          height={400}
          loading={loading}
          gradient={true}
          responsive={true}
          aria-label="Pest detection confidence over time"
          reduceMotion={reduceMotion}
        />
      )}
    </Container>
  );
});

PestDetection.displayName = 'PestDetection';

export default PestDetection;