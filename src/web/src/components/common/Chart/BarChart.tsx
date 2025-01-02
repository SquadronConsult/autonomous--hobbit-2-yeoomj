import React, { useMemo } from 'react';
import { Chart as ChartJS, CategoryScale, LinearScale, BarElement, Title, Tooltip, Legend } from 'chart.js';
import { Bar } from 'react-chartjs-2';
import { CHART_COLORS } from '../../../constants/chartColors';

// Register required Chart.js components
ChartJS.register(
  CategoryScale,
  LinearScale,
  BarElement,
  Title,
  Tooltip,
  Legend
);

interface Dataset {
  label: string;
  data: number[];
  backgroundColor: string | string[];
  borderColor: string | string[];
  borderWidth: number;
  hoverBackgroundColor?: string | string[];
  hoverBorderColor?: string | string[];
}

interface ChartData {
  labels: string[];
  datasets: Dataset[];
}

interface IBarChartProps {
  data: ChartData;
  options?: any;
  height?: number | string;
  width?: number | string;
  className?: string;
  theme?: 'light' | 'dark';
  ariaLabel?: string;
  onDataPointClick?: (event: MouseEvent, element: any) => void;
}

const DEFAULT_OPTIONS = {
  responsive: true,
  maintainAspectRatio: false,
  animation: {
    duration: 750,
    easing: 'easeInOutQuart',
    mode: 'progressive'
  },
  plugins: {
    legend: {
      position: 'top' as const,
      align: 'start' as const,
      labels: {
        usePointStyle: true,
        padding: 20,
        font: {
          family: 'Roboto, sans-serif',
          size: 12
        },
        color: '#666666'
      }
    },
    tooltip: {
      enabled: true,
      mode: 'index' as const,
      intersect: false,
      backgroundColor: 'rgba(0,0,0,0.8)',
      titleFont: {
        family: 'Roboto, sans-serif',
        size: 14,
        weight: 'bold'
      },
      bodyFont: {
        family: 'Roboto, sans-serif',
        size: 13
      },
      padding: 12,
      cornerRadius: 4,
      displayColors: true
    }
  },
  scales: {
    y: {
      beginAtZero: true,
      grid: {
        drawBorder: false,
        color: 'rgba(0,0,0,0.1)'
      },
      ticks: {
        padding: 10,
        font: {
          family: 'Roboto, sans-serif',
          size: 11
        },
        color: '#666666'
      }
    },
    x: {
      grid: {
        display: false
      },
      ticks: {
        padding: 10,
        font: {
          family: 'Roboto, sans-serif',
          size: 11
        },
        color: '#666666'
      }
    }
  }
};

const ACCESSIBILITY_CONFIG = {
  'aria-label': 'Bar chart visualization',
  role: 'img',
  tabIndex: 0,
  'aria-live': 'polite',
  'aria-atomic': true,
  'aria-relevant': 'all'
};

const BarChart: React.FC<IBarChartProps> = React.memo(({
  data,
  options = {},
  height = 300,
  width = '100%',
  className = '',
  theme = 'light',
  ariaLabel,
  onDataPointClick
}) => {
  // Apply theme-specific colors and styles
  const themedData = useMemo(() => {
    return {
      ...data,
      datasets: data.datasets.map((dataset, index) => ({
        ...dataset,
        backgroundColor: dataset.backgroundColor || CHART_COLORS.primary[index % CHART_COLORS.primary.length],
        borderColor: dataset.borderColor || CHART_COLORS.primary[index % CHART_COLORS.primary.length],
        borderWidth: dataset.borderWidth || 1,
        hoverBackgroundColor: dataset.hoverBackgroundColor || CHART_COLORS.secondary[index % CHART_COLORS.secondary.length],
        hoverBorderColor: dataset.hoverBorderColor || CHART_COLORS.secondary[index % CHART_COLORS.secondary.length]
      }))
    };
  }, [data]);

  // Merge default options with custom options and theme-specific adjustments
  const chartOptions = useMemo(() => {
    const themeColors = {
      dark: {
        color: '#E0E0E0',
        gridColor: 'rgba(255,255,255,0.1)',
        tooltipBackground: 'rgba(48,48,48,0.95)'
      },
      light: {
        color: '#666666',
        gridColor: 'rgba(0,0,0,0.1)',
        tooltipBackground: 'rgba(0,0,0,0.8)'
      }
    };

    return {
      ...DEFAULT_OPTIONS,
      ...options,
      plugins: {
        ...DEFAULT_OPTIONS.plugins,
        ...options.plugins,
        tooltip: {
          ...DEFAULT_OPTIONS.plugins.tooltip,
          ...options.plugins?.tooltip,
          backgroundColor: themeColors[theme].tooltipBackground
        }
      },
      scales: {
        ...DEFAULT_OPTIONS.scales,
        ...options.scales,
        y: {
          ...DEFAULT_OPTIONS.scales.y,
          ...options.scales?.y,
          grid: {
            ...DEFAULT_OPTIONS.scales.y.grid,
            color: themeColors[theme].gridColor
          },
          ticks: {
            ...DEFAULT_OPTIONS.scales.y.ticks,
            color: themeColors[theme].color
          }
        },
        x: {
          ...DEFAULT_OPTIONS.scales.x,
          ...options.scales?.x,
          ticks: {
            ...DEFAULT_OPTIONS.scales.x.ticks,
            color: themeColors[theme].color
          }
        }
      },
      onClick: onDataPointClick
    };
  }, [options, theme, onDataPointClick]);

  return (
    <div
      style={{ height, width }}
      className={`bar-chart-container ${className}`}
      {...ACCESSIBILITY_CONFIG}
      aria-label={ariaLabel || ACCESSIBILITY_CONFIG['aria-label']}
    >
      <Bar
        data={themedData}
        options={chartOptions}
        plugins={[{
          id: 'customCanvasBackgroundColor',
          beforeDraw: (chart) => {
            const ctx = chart.canvas.getContext('2d');
            if (ctx) {
              ctx.save();
              ctx.globalCompositeOperation = 'destination-over';
              ctx.fillStyle = theme === 'dark' ? '#1A1A1A' : '#FFFFFF';
              ctx.fillRect(0, 0, chart.width, chart.height);
              ctx.restore();
            }
          }
        }]}
      />
    </div>
  );
});

BarChart.displayName = 'BarChart';

export default BarChart;