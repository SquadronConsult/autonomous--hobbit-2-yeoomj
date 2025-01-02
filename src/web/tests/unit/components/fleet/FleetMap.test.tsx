import React from 'react';
import { render, screen, fireEvent, waitFor, act } from '@testing-library/react';
import { jest, describe, beforeEach, it, expect } from '@jest/globals';
import FleetMap from '../../../../src/components/fleet/FleetMap/FleetMap';
import { useFleet } from '../../../../src/hooks/useFleet';
import { createMockDevice } from '../../../mocks/handlers';
import { DeviceType, DeviceStatus } from '../../../../src/interfaces/IDevice';

// Mock maplibre-gl to avoid actual map rendering
jest.mock('maplibre-gl', () => ({
  Map: jest.fn(() => ({
    on: jest.fn(),
    remove: jest.fn(),
    addControl: jest.fn(),
  })),
}));

// Mock useFleet hook
jest.mock('../../../../src/hooks/useFleet');

describe('FleetMap Component', () => {
  // Default props for testing
  const defaultProps = {
    center: { lat: 0, lng: 0 },
    zoom: 12,
    onDeviceSelect: jest.fn(),
    clusterThreshold: 8,
    offlineMode: false,
  };

  // Mock devices for testing
  const mockDevices = [
    createMockDevice({
      id: 'drone-1',
      type: DeviceType.DRONE,
      status: DeviceStatus.ACTIVE,
      batteryLevel: 85,
      location: { latitude: 1, longitude: 1, altitude: 100, heading: 0, speed: 5, accuracy: 1 }
    }),
    createMockDevice({
      id: 'ground-1',
      type: DeviceType.GROUND_ROBOT,
      status: DeviceStatus.IDLE,
      batteryLevel: 45,
      location: { latitude: 2, longitude: 2, altitude: 0, heading: 90, speed: 2, accuracy: 0.5 }
    })
  ];

  beforeEach(() => {
    // Reset all mocks before each test
    jest.clearAllMocks();
    
    // Mock useFleet hook implementation
    (useFleet as jest.Mock).mockReturnValue({
      devices: new Map(mockDevices.map(device => [device.id, device])),
      loading: false,
      error: null,
      isConnected: true,
      connectionQuality: 'excellent',
      lastUpdate: new Date(),
      updateDeviceStatus: jest.fn(),
    });
  });

  it('renders loading state correctly', async () => {
    (useFleet as jest.Mock).mockReturnValue({
      devices: new Map(),
      loading: true,
      error: null,
    });

    render(<FleetMap {...defaultProps} />);
    
    expect(screen.getByRole('status')).toBeInTheDocument();
    expect(screen.getByText('Loading...')).toBeInTheDocument();
  });

  it('renders error state with message', async () => {
    const errorMessage = 'Failed to load fleet data';
    (useFleet as jest.Mock).mockReturnValue({
      devices: new Map(),
      loading: false,
      error: new Error(errorMessage),
    });

    render(<FleetMap {...defaultProps} />);
    
    expect(screen.getByText(errorMessage)).toBeInTheDocument();
  });

  it('renders device markers with correct styling', async () => {
    render(<FleetMap {...defaultProps} />);

    await waitFor(() => {
      const markers = screen.getAllByRole('button');
      expect(markers).toHaveLength(mockDevices.length);
      
      // Verify drone marker
      const droneMarker = screen.getByLabelText(/drone.*active/i);
      expect(droneMarker).toHaveStyle({
        backgroundColor: '#4CAF50',
        opacity: 1,
        transform: 'scale(1.2)',
      });

      // Verify ground robot marker
      const groundMarker = screen.getByLabelText(/ground.*idle/i);
      expect(groundMarker).toHaveStyle({
        backgroundColor: '#2196F3',
        opacity: 1,
        transform: 'scale(1)',
      });
    });
  });

  it('handles device selection correctly', async () => {
    render(<FleetMap {...defaultProps} />);

    await waitFor(() => {
      const droneMarker = screen.getByLabelText(/drone.*active/i);
      fireEvent.click(droneMarker);
      
      expect(defaultProps.onDeviceSelect).toHaveBeenCalledWith(mockDevices[0]);
      expect(droneMarker).toHaveStyle({
        boxShadow: '0 0 0 3px #FFC107',
      });
    });
  });

  it('updates device positions in real-time', async () => {
    const { rerender } = render(<FleetMap {...defaultProps} />);

    // Simulate device position update
    const updatedDevices = new Map(mockDevices.map(device => [
      device.id,
      {
        ...device,
        location: {
          ...device.location,
          latitude: device.location.latitude + 1,
          longitude: device.location.longitude + 1,
        }
      }
    ]));

    // Measure update time
    const startTime = performance.now();
    
    act(() => {
      (useFleet as jest.Mock).mockReturnValue({
        devices: updatedDevices,
        loading: false,
        error: null,
        isConnected: true,
        connectionQuality: 'excellent',
        lastUpdate: new Date(),
      });
    });

    rerender(<FleetMap {...defaultProps} />);

    const updateTime = performance.now() - startTime;
    expect(updateTime).toBeLessThan(100); // Verify update latency requirement
  });

  it('handles offline mode correctly', async () => {
    render(<FleetMap {...defaultProps} offlineMode={true} />);

    // Simulate offline device
    const offlineDevice = {
      ...mockDevices[0],
      status: DeviceStatus.OFFLINE,
    };

    await waitFor(() => {
      const marker = screen.getByLabelText(/drone.*offline/i);
      fireEvent.click(marker);
      
      // Should still allow selection in offline mode
      expect(defaultProps.onDeviceSelect).toHaveBeenCalledWith(offlineDevice);
    });
  });

  it('supports keyboard navigation', async () => {
    render(<FleetMap {...defaultProps} />);

    await waitFor(() => {
      const markers = screen.getAllByRole('button');
      
      // Test keyboard focus
      markers[0].focus();
      expect(document.activeElement).toBe(markers[0]);

      // Test Enter key selection
      fireEvent.keyPress(markers[0], { key: 'Enter', code: 13, charCode: 13 });
      expect(defaultProps.onDeviceSelect).toHaveBeenCalledWith(mockDevices[0]);
    });
  });

  it('handles clustering when device count exceeds threshold', async () => {
    // Create more devices than the clustering threshold
    const manyDevices = Array(10).fill(null).map((_, index) => 
      createMockDevice({
        id: `device-${index}`,
        type: index % 2 === 0 ? DeviceType.DRONE : DeviceType.GROUND_ROBOT,
      })
    );

    (useFleet as jest.Mock).mockReturnValue({
      devices: new Map(manyDevices.map(device => [device.id, device])),
      loading: false,
      error: null,
    });

    render(<FleetMap {...defaultProps} />);

    await waitFor(() => {
      expect(screen.getAllByRole('button').length).toBeLessThan(manyDevices.length);
    });
  });

  it('displays connection quality indicator', async () => {
    render(<FleetMap {...defaultProps} />);

    await waitFor(() => {
      const indicator = screen.getByText(/excellent/i);
      expect(indicator).toBeInTheDocument();
      expect(indicator).toHaveStyle({
        backgroundColor: '#4CAF50',
      });
    });
  });

  it('shows device details in popup on selection', async () => {
    render(<FleetMap {...defaultProps} />);

    await waitFor(() => {
      const droneMarker = screen.getByLabelText(/drone.*active/i);
      fireEvent.click(droneMarker);
      
      expect(screen.getByText(mockDevices[0].name)).toBeInTheDocument();
      expect(screen.getByText(`Battery: ${mockDevices[0].batteryLevel}%`)).toBeInTheDocument();
    });
  });
});