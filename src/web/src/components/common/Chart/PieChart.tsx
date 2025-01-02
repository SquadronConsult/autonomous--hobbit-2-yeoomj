import React, { useEffect, useRef } from 'react';
import { Chart, ChartConfiguration, ChartData } from 'chart.js/auto'; // v4.0.0
import { debounce } from 'lodash'; // v4.17.21
import { CHART_COLORS } from '../../../constants/chartColors';

interface ChartDataItem {
  label: string;
  value: number;
}

interface IPieChartProps {
  data: ChartDataItem[];
  title?: string;
  className?: string;
  colorScheme?: 'primary' | 'secondary' | 'success' | 'warning' | 'error';
  height?: number;
  width?: number;
  animate?: boolean;
  onError?: (error: Error) => void;
}

const DEFAULT_OPTIONS: Partial<ChartConfiguration['options']> = {
  responsive: true,
  maintainAspectRatio: false,
  plugins: {
    legend: {
      position: 'bottom',
      labels: {
        padding: 20,
        usePointStyle: true,
        font: {
          family: 'Roboto, sans-serif',
          size: 12
        },
        generateLabels: (chart) => {
          const labels = Chart.defaults.plugins.legend.labels.generateLabels(chart);
          return labels.map(label => ({
            ...label,
            text: label.text.length > 25 ? `${label.text.substring(0, 22)}...` : label.text
          }));
        }
      }
    },
    tooltip: {
      enabled: true,
      mode: 'index',
      intersect: false,
      padding: 12,
      backgroundColor: 'rgba(33, 33, 33, 0.95)',
      titleFont: {
        family: 'Roboto, sans-serif',
        size: 14,
        weight: 'bold'
      },
      bodyFont: {
        family: 'Roboto, sans-serif',
        size: 12
      },
      callbacks: {
        label: (context) => {
          const value = context.raw as number;
          const total = context.chart.data.datasets[0].data.reduce((a, b) => (a as number) + (b as number), 0);
          const percentage = ((value as number / total as number) * 100).toFixed(1);
          return `${context.label}: ${value} (${percentage}%)`;
        }
      }
    },
    accessibility: {
      enabled: true,
      type: 'pie',
      description: 'Interactive pie chart visualization',
      announceNewData: {
        enabled: true,
        announcementFormatter: (data) => {
          if (data.type === 'data') {
            return `New data ${data.label}: ${data.value}`;
          }
          return null;
        }
      }
    }
  },
  animation: {
    duration: 750,
    easing: 'easeInOutQuart',
    animateRotate: true,
    animateScale: true
  }
};

const getChartConfig = (props: IPieChartProps): ChartConfiguration => {
  const { data, colorScheme = 'primary', animate = true } = props;
  const colors = CHART_COLORS[colorScheme];

  const chartData: ChartData = {
    labels: data.map(item => item.label),
    datasets: [{
      data: data.map(item => item.value),
      backgroundColor: data.map((_, index) => colors[index % colors.length]),
      borderColor: 'rgba(255, 255, 255, 0.8)',
      borderWidth: 2,
      hoverOffset: 4,
      hoverBorderWidth: 3,
      hoverBorderColor: 'rgba(255, 255, 255, 1)'
    }]
  };

  return {
    type: 'pie',
    data: chartData,
    options: {
      ...DEFAULT_OPTIONS,
      animation: animate ? DEFAULT_OPTIONS.animation : false
    }
  };
};

const PieChart: React.FC<IPieChartProps> = React.memo((props) => {
  const {
    className,
    height = 300,
    width = 300,
    title,
    onError
  } = props;

  const canvasRef = useRef<HTMLCanvasElement>(null);
  const chartRef = useRef<Chart | null>(null);

  const debouncedUpdate = debounce(() => {
    if (chartRef.current) {
      try {
        chartRef.current.update();
      } catch (error) {
        onError?.(error as Error);
      }
    }
  }, 100);

  useEffect(() => {
    if (!canvasRef.current) return;

    try {
      const ctx = canvasRef.current.getContext('2d');
      if (!ctx) throw new Error('Canvas context not available');

      const config = getChartConfig(props);
      chartRef.current = new Chart(ctx, config);

      // Add keyboard navigation support
      canvasRef.current.tabIndex = 0;
      canvasRef.current.setAttribute('role', 'img');
      canvasRef.current.setAttribute('aria-label', title || 'Pie chart visualization');

      return () => {
        debouncedUpdate.cancel();
        if (chartRef.current) {
          chartRef.current.destroy();
          chartRef.current = null;
        }
      };
    } catch (error) {
      onError?.(error as Error);
    }
  }, []);

  useEffect(() => {
    if (!chartRef.current) return;

    try {
      const config = getChartConfig(props);
      chartRef.current.data = config.data;
      debouncedUpdate();
    } catch (error) {
      onError?.(error as Error);
    }
  }, [props.data, props.colorScheme]);

  return (
    <div 
      className={className}
      style={{ 
        width, 
        height,
        position: 'relative'
      }}
    >
      <canvas
        ref={canvasRef}
        role="img"
        aria-label={title || 'Pie chart visualization'}
      />
    </div>
  );
});

PieChart.displayName = 'PieChart';

export default PieChart;