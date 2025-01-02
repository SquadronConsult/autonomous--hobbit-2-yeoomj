import React from 'react';
import { render, screen, waitFor, fireEvent } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import WS from 'jest-websocket-mock';
import '@testing-library/jest-dom/extend-expect';

import FleetMap from '../../src/components/fleet/FleetMap/FleetMap';
import { DeviceService } from '../../src/services/device';
import { DeviceType, DeviceStatus } from '../../src/interfaces/IDevice';

// Mock WebSocket server
const WS_URL = 'ws://localhost:8080';
let wsServer: WS;

// Mock device data
const mockDevices = [
  {
    id: 'drone-1',
    name: 'Drone Alpha',
    type: DeviceType.DRONE,
    status: DeviceStatus.ACTIVE,
    batteryLevel: 85,
    location: {
      latitude: 34.0522,
      longitude: -118.2437,
      altitude: 100,
      heading: 90,
      speed: 5,
      accuracy: 2
    },
    lastActive: new Date(),
    capabilities: ['SURVEILLANCE', 'TREATMENT'],
    subType: 'DJI-M300',
    metadata: {
      model: 'M300-RTK',
      firmware: '1.0.0',
      lastMaintenance: new Date(),
      operationalHours: 150
    },
    currentMission: null,
    errorCode: null
  },
  {
    id: 'robot-1',
    name: 'Ground Bot 1',
    type: DeviceType.GROUND_ROBOT,
    status: DeviceStatus.IDLE,
    batteryLevel: 92,
    location: {
      latitude: 34.0523,
      longitude: -118.2438,
      altitude: 0,
      heading: 180,
      speed: 0,
      accuracy: 1
    },
    lastActive: new Date(),
    capabilities: ['TREATMENT', 'SAMPLING'],
    subType: 'AGV-200',
    metadata: {
      model: 'AGV-200',
      firmware: '2.1.0',
      lastMaintenance: new Date(),
      operationalHours: 300
    },
    currentMission: null,
    errorCode: null
  }
];

// Performance thresholds
const PERFORMANCE_THRESHOLDS = {
  updateLatency: 100, // ms
  renderTime: 50, // ms
  maxDevices: 8
};

// Mock DeviceService
jest.mock('../../src/services/device', () => ({
  DeviceService: jest.fn().mockImplementation(() => ({
    getAllDevices: jest.fn().mockResolvedValue({ devices: mockDevices, total: mockDevices.length }),
    getDeviceStatus: jest.fn().mockResolvedValue(mockDevices[0]),
    updateDeviceStatus: jest.fn().mockResolvedValue(mockDevices[0]),
    subscribeToDeviceUpdates: jest.fn()
  }))
}));

describe('Fleet Management Integration', () => {
  let deviceService: jest.Mocked<DeviceService>;

  beforeEach(async () => {
    // Setup WebSocket mock server
    wsServer = new WS(WS_URL);
    deviceService = new DeviceService() as jest.Mocked<DeviceService>;
  });

  afterEach(() => {
    WS.clean();
    jest.clearAllMocks();
  });

  describe('Device Visualization', () => {
    it('should render fleet map with multiple devices', async () => {
      const startTime = performance.now();

      render(
        <FleetMap
          center={{ lat: 34.0522, lng: -118.2437 }}
          zoom={15}
          onDeviceSelect={jest.fn()}
        />
      );

      await waitFor(() => {
        expect(screen.getByRole('application')).toBeInTheDocument();
      });

      // Verify render performance
      const renderTime = performance.now() - startTime;
      expect(renderTime).toBeLessThan(PERFORMANCE_THRESHOLDS.renderTime);

      // Verify device markers
      const markers = screen.getAllByRole('button');
      expect(markers).toHaveLength(mockDevices.length);

      // Verify device status indicators
      const activeMarker = screen.getByLabelText(/Drone Alpha - Status: ACTIVE/i);
      expect(activeMarker).toHaveAttribute('data-status', 'ACTIVE');
    });

    it('should handle device selection and display details', async () => {
      const onDeviceSelect = jest.fn();

      render(
        <FleetMap
          center={{ lat: 34.0522, lng: -118.2437 }}
          zoom={15}
          onDeviceSelect={onDeviceSelect}
        />
      );

      await waitFor(() => {
        const droneMarker = screen.getByLabelText(/Drone Alpha/i);
        fireEvent.click(droneMarker);
      });

      expect(onDeviceSelect).toHaveBeenCalledWith(
        expect.objectContaining({
          id: 'drone-1',
          type: DeviceType.DRONE
        })
      );

      // Verify popup content
      expect(screen.getByText('Drone Alpha')).toBeInTheDocument();
      expect(screen.getByText('Battery: 85%')).toBeInTheDocument();
    });
  });

  describe('Real-time Updates', () => {
    it('should handle WebSocket device updates within latency requirements', async () => {
      render(
        <FleetMap
          center={{ lat: 34.0522, lng: -118.2437 }}
          zoom={15}
          onDeviceSelect={jest.fn()}
        />
      );

      await waitFor(() => {
        expect(wsServer.connected).toBe(true);
      });

      const updateStartTime = performance.now();

      // Simulate device update
      wsServer.send(JSON.stringify({
        type: 'device:update',
        payload: {
          deviceId: 'drone-1',
          status: DeviceStatus.ACTIVE,
          location: {
            latitude: 34.0525,
            longitude: -118.2440
          }
        }
      }));

      await waitFor(() => {
        const marker = screen.getByLabelText(/Drone Alpha/i);
        expect(marker).toHaveAttribute('data-status', 'ACTIVE');
      });

      const updateLatency = performance.now() - updateStartTime;
      expect(updateLatency).toBeLessThan(PERFORMANCE_THRESHOLDS.updateLatency);
    });

    it('should maintain performance with maximum device load', async () => {
      // Generate maximum device load
      const maxDevices = Array.from({ length: PERFORMANCE_THRESHOLDS.maxDevices }, (_, i) => ({
        ...mockDevices[0],
        id: `drone-${i}`,
        name: `Drone ${i}`
      }));

      deviceService.getAllDevices.mockResolvedValue({
        devices: maxDevices,
        total: maxDevices.length
      });

      const startTime = performance.now();

      render(
        <FleetMap
          center={{ lat: 34.0522, lng: -118.2437 }}
          zoom={15}
          onDeviceSelect={jest.fn()}
        />
      );

      await waitFor(() => {
        const markers = screen.getAllByRole('button');
        expect(markers).toHaveLength(PERFORMANCE_THRESHOLDS.maxDevices);
      });

      const loadTime = performance.now() - startTime;
      expect(loadTime).toBeLessThan(PERFORMANCE_THRESHOLDS.renderTime * 2);
    });
  });

  describe('Error Handling', () => {
    it('should handle WebSocket disconnection gracefully', async () => {
      render(
        <FleetMap
          center={{ lat: 34.0522, lng: -118.2437 }}
          zoom={15}
          onDeviceSelect={jest.fn()}
        />
      );

      await waitFor(() => {
        expect(wsServer.connected).toBe(true);
      });

      wsServer.close();

      await waitFor(() => {
        expect(screen.getByText(/Connection lost/i)).toBeInTheDocument();
      });

      // Verify devices still visible with offline indicator
      const offlineMarker = screen.getByLabelText(/Drone Alpha/i);
      expect(offlineMarker).toHaveAttribute('data-status', 'OFFLINE');
    });

    it('should recover from API errors', async () => {
      deviceService.getAllDevices.mockRejectedValueOnce(new Error('API Error'));

      render(
        <FleetMap
          center={{ lat: 34.0522, lng: -118.2437 }}
          zoom={15}
          onDeviceSelect={jest.fn()}
        />
      );

      await waitFor(() => {
        expect(screen.getByText(/Error loading devices/i)).toBeInTheDocument();
      });

      // Simulate retry
      deviceService.getAllDevices.mockResolvedValueOnce({
        devices: mockDevices,
        total: mockDevices.length
      });

      fireEvent.click(screen.getByText(/Retry/i));

      await waitFor(() => {
        const markers = screen.getAllByRole('button');
        expect(markers).toHaveLength(mockDevices.length);
      });
    });
  });
});