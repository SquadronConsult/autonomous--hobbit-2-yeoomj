import React, { useMemo, useRef, useEffect } from 'react';
import styled from '@emotion/styled';
import { Chart as ChartJS, CategoryScale, LinearScale, PointElement, LineElement, Title, Tooltip, Legend, ChartOptions } from 'chart.js';
import { Line } from 'react-chartjs-2';
import { CHART_COLORS, CHART_GRADIENTS } from '../../constants/chartColors';
import Loading from '../Loading/Loading';

// Register required Chart.js components
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

// Types and Interfaces
interface LineChartProps {
  data: {
    labels: string[];
    datasets: Array<{
      label: string;
      data: number[];
      borderColor?: string;
      backgroundColor?: string;
      fill?: boolean;
      tension?: number;
    }>;
  };
  options?: ChartOptions<'line'>;
  height?: number;
  width?: number;
  loading?: boolean;
  gradient?: boolean;
  responsive?: boolean;
  theme?: 'light' | 'dark';
  'aria-label'?: string;
  reduceMotion?: boolean;
}

// Styled Components
const ChartContainer = styled.div<{
  width?: number;
  height?: number;
  theme?: 'light' | 'dark';
}>`
  position: relative;
  width: ${props => props.width ? `${props.width}px` : '100%'};
  height: ${props => props.height ? `${props.height}px` : '300px'};
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: ${props => props.theme === 'dark' ? '#1a1a1a' : '#ffffff'};
  border-radius: var(--border-radius-lg);
  padding: var(--spacing-md);
  box-shadow: ${props => props.theme === 'dark' ? 'none' : 'var(--shadow-md)'};
  transition: box-shadow var(--transition-duration-normal) var(--transition-timing-ease-in-out);

  @media (max-width: var(--breakpoint-mobile)) {
    height: ${props => props.height ? `${props.height * 0.8}px` : '240px'};
    padding: var(--spacing-sm);
  }
`;

// Constants
const DEFAULT_OPTIONS: ChartOptions<'line'> = {
  responsive: true,
  maintainAspectRatio: false,
  animation: {
    duration: 750,
    easing: 'easeInOutQuart'
  },
  interaction: {
    intersect: false,
    mode: 'index'
  },
  plugins: {
    legend: {
      position: 'top',
      align: 'start',
      labels: {
        usePointStyle: true,
        padding: 16,
        font: {
          family: 'var(--font-family-primary)',
          size: 12
        }
      }
    },
    tooltip: {
      enabled: true,
      mode: 'index',
      intersect: false,
      backgroundColor: 'rgba(0,0,0,0.8)',
      padding: 12,
      cornerRadius: 4,
      titleFont: {
        family: 'var(--font-family-primary)',
        size: 14,
        weight: 'bold'
      },
      bodyFont: {
        family: 'var(--font-family-primary)',
        size: 12
      }
    }
  },
  scales: {
    x: {
      grid: {
        display: false
      },
      ticks: {
        maxRotation: 0,
        font: {
          family: 'var(--font-family-primary)',
          size: 12
        }
      }
    },
    y: {
      beginAtZero: true,
      grid: {
        color: 'rgba(0,0,0,0.1)'
      },
      ticks: {
        font: {
          family: 'var(--font-family-primary)',
          size: 12
        }
      }
    }
  }
};

// Helper function to create gradient
const createGradient = (
  canvas: HTMLCanvasElement,
  colors: string[],
  theme: 'light' | 'dark'
): CanvasGradient | null => {
  const ctx = canvas.getContext('2d');
  if (!ctx) return null;

  const gradient = ctx.createLinearGradient(0, 0, 0, canvas.height);
  const opacity = theme === 'dark' ? 0.7 : 0.8;
  
  gradient.addColorStop(0, colors[0].replace('0.8', opacity.toString()));
  gradient.addColorStop(1, colors[1]);
  
  return gradient;
};

// Main Component
const LineChart: React.FC<LineChartProps> = React.memo(({
  data,
  options = {},
  height = 300,
  width,
  loading = false,
  gradient = true,
  responsive = true,
  theme = 'light',
  'aria-label': ariaLabel = 'Line chart',
  reduceMotion = false
}) => {
  const chartRef = useRef<ChartJS<'line'>>(null);

  // Merge default options with provided options
  const chartOptions = useMemo(() => ({
    ...DEFAULT_OPTIONS,
    ...options,
    animation: {
      ...DEFAULT_OPTIONS.animation,
      duration: reduceMotion ? 0 : DEFAULT_OPTIONS.animation?.duration
    }
  }), [options, reduceMotion]);

  // Apply gradients when canvas is ready
  useEffect(() => {
    if (gradient && chartRef.current) {
      const chart = chartRef.current;
      const canvas = chart.canvas;

      data.datasets = data.datasets.map((dataset, index) => {
        const colors = index === 0 ? CHART_GRADIENTS.performance : CHART_GRADIENTS.telemetry;
        const gradientFill = createGradient(canvas, colors, theme);
        
        return {
          ...dataset,
          backgroundColor: dataset.fill ? gradientFill : dataset.backgroundColor || CHART_COLORS.primary[index],
          borderColor: dataset.borderColor || CHART_COLORS.primary[index],
          tension: dataset.tension || 0.4
        };
      });

      chart.update();
    }
  }, [data, gradient, theme]);

  if (loading) {
    return (
      <ChartContainer width={width} height={height} theme={theme}>
        <Loading size="lg" label="Loading chart data..." />
      </ChartContainer>
    );
  }

  return (
    <ChartContainer 
      width={width} 
      height={height} 
      theme={theme}
      role="region" 
      aria-label={ariaLabel}
    >
      <Line
        ref={chartRef}
        data={data}
        options={chartOptions}
        aria-label={ariaLabel}
        role="img"
      />
    </ChartContainer>
  );
});

LineChart.displayName = 'LineChart';

export default LineChart;