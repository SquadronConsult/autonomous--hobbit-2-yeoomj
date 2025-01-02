import React from 'react';
import { render, screen, fireEvent, waitFor, within } from '@testing-library/react';
import { Chart } from 'chart.js';
import { jest, describe, it, expect, beforeEach, afterEach } from '@jest/globals';

import BarChart from '../../../../src/components/common/Chart/BarChart';
import LineChart from '../../../../src/components/common/Chart/LineChart';
import PieChart from '../../../../src/components/common/Chart/PieChart';
import { CHART_COLORS } from '../../../../src/constants/chartColors';

// Mock Chart.js to prevent canvas rendering issues
jest.mock('chart.js', () => ({
  Chart: jest.fn(),
  register: jest.fn(),
  defaults: {
    plugins: {
      legend: {
        labels: {
          generateLabels: jest.fn()
        }
      }
    }
  }
}));

// Mock ResizeObserver
global.ResizeObserver = jest.fn().mockImplementation(() => ({
  observe: jest.fn(),
  unobserve: jest.fn(),
  disconnect: jest.fn(),
}));

// Mock data constants
const MOCK_BAR_DATA = {
  labels: ['CPU Usage', 'Memory Usage', 'GPU Usage', 'Storage Usage'],
  datasets: [{
    label: 'System Resources',
    data: [75, 60, 82, 45],
    backgroundColor: 'rgba(75,192,192,0.4)',
    borderColor: 'rgba(75,192,192,1)',
    borderWidth: 1
  }]
};

const MOCK_LINE_DATA = {
  labels: ['00:00', '04:00', '08:00', '12:00', '16:00', '20:00'],
  datasets: [{
    label: 'Performance Metrics',
    data: [65, 75, 85, 80, 90, 85],
    borderColor: '#4CAF50',
    backgroundColor: 'rgba(76,175,80,0.1)',
    fill: true
  }]
};

const MOCK_PIE_DATA = [
  { label: 'Active Drones', value: 60 },
  { label: 'Charging', value: 15 },
  { label: 'Maintenance', value: 10 },
  { label: 'Standby', value: 15 }
];

describe('BarChart Component', () => {
  beforeEach(() => {
    Chart.mockClear();
  });

  it('renders with required props and correct accessibility attributes', () => {
    render(<BarChart data={MOCK_BAR_DATA} />);
    const chartContainer = screen.getByRole('img');
    
    expect(chartContainer).toHaveAttribute('aria-label');
    expect(chartContainer).toHaveAttribute('tabIndex', '0');
  });

  it('applies Material Design 3.0 color scheme correctly', () => {
    render(<BarChart data={MOCK_BAR_DATA} theme="light" />);
    expect(Chart).toHaveBeenCalledWith(
      expect.any(Object),
      expect.objectContaining({
        data: expect.objectContaining({
          datasets: expect.arrayContaining([
            expect.objectContaining({
              backgroundColor: expect.stringContaining(CHART_COLORS.primary[0])
            })
          ])
        })
      })
    );
  });

  it('handles responsive layout changes', async () => {
    const { rerender } = render(<BarChart data={MOCK_BAR_DATA} width={500} />);
    rerender(<BarChart data={MOCK_BAR_DATA} width={300} />);
    
    await waitFor(() => {
      expect(Chart.prototype.update).toHaveBeenCalled();
    });
  });

  it('supports keyboard navigation', () => {
    render(<BarChart data={MOCK_BAR_DATA} />);
    const chart = screen.getByRole('img');
    
    fireEvent.keyDown(chart, { key: 'Enter' });
    expect(chart).toHaveFocus();
  });
});

describe('LineChart Component', () => {
  beforeEach(() => {
    Chart.mockClear();
  });

  it('renders with loading state', () => {
    render(<LineChart data={MOCK_LINE_DATA} loading={true} />);
    expect(screen.getByText('Loading chart data...')).toBeInTheDocument();
  });

  it('applies gradient background when enabled', () => {
    render(<LineChart data={MOCK_LINE_DATA} gradient={true} />);
    expect(Chart).toHaveBeenCalledWith(
      expect.any(Object),
      expect.objectContaining({
        data: expect.objectContaining({
          datasets: expect.arrayContaining([
            expect.objectContaining({
              fill: true
            })
          ])
        })
      })
    );
  });

  it('handles theme changes correctly', async () => {
    const { rerender } = render(<LineChart data={MOCK_LINE_DATA} theme="light" />);
    rerender(<LineChart data={MOCK_LINE_DATA} theme="dark" />);
    
    await waitFor(() => {
      expect(Chart.prototype.update).toHaveBeenCalled();
    });
  });

  it('supports reduced motion preferences', () => {
    render(<LineChart data={MOCK_LINE_DATA} reduceMotion={true} />);
    expect(Chart).toHaveBeenCalledWith(
      expect.any(Object),
      expect.objectContaining({
        options: expect.objectContaining({
          animation: expect.objectContaining({
            duration: 0
          })
        })
      })
    );
  });
});

describe('PieChart Component', () => {
  beforeEach(() => {
    Chart.mockClear();
  });

  it('renders with correct data structure', () => {
    render(<PieChart data={MOCK_PIE_DATA} />);
    expect(Chart).toHaveBeenCalledWith(
      expect.any(Object),
      expect.objectContaining({
        type: 'pie',
        data: expect.objectContaining({
          labels: expect.arrayContaining(['Active Drones', 'Charging', 'Maintenance', 'Standby'])
        })
      })
    );
  });

  it('handles error states gracefully', () => {
    const onError = jest.fn();
    Chart.mockImplementationOnce(() => {
      throw new Error('Canvas error');
    });
    
    render(<PieChart data={MOCK_PIE_DATA} onError={onError} />);
    expect(onError).toHaveBeenCalledWith(expect.any(Error));
  });

  it('updates chart on data changes', async () => {
    const { rerender } = render(<PieChart data={MOCK_PIE_DATA} />);
    const updatedData = [...MOCK_PIE_DATA];
    updatedData[0].value = 70;
    
    rerender(<PieChart data={updatedData} />);
    await waitFor(() => {
      expect(Chart.prototype.update).toHaveBeenCalled();
    });
  });

  it('supports custom color schemes', () => {
    render(<PieChart data={MOCK_PIE_DATA} colorScheme="success" />);
    expect(Chart).toHaveBeenCalledWith(
      expect.any(Object),
      expect.objectContaining({
        data: expect.objectContaining({
          datasets: expect.arrayContaining([
            expect.objectContaining({
              backgroundColor: expect.arrayContaining([
                expect.stringContaining(CHART_COLORS.success[0])
              ])
            })
          ])
        })
      })
    );
  });
});