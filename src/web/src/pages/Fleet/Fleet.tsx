import React, { useState, useEffect, useCallback, useMemo } from 'react';
import styled from '@emotion/styled';
import { IDevice, DeviceType, DeviceStatus } from '../../interfaces/IDevice';
import FleetMap from '../../components/fleet/FleetMap/FleetMap';
import DroneCard from '../../components/fleet/DroneCard/DroneCard';
import RobotCard from '../../components/fleet/RobotCard/RobotCard';
import TelemetryPanel from '../../components/fleet/TelemetryPanel/TelemetryPanel';
import { useFleet } from '../../hooks/useFleet';

// Styled components for layout
const FleetContainer = styled.div`
  display: grid;
  grid-template-columns: 320px 1fr 320px;
  gap: var(--spacing-md);
  height: 100%;
  padding: var(--spacing-md);
  background-color: var(--theme-background-default);

  @media (max-width: 1200px) {
    grid-template-columns: 280px 1fr;
  }

  @media (max-width: 768px) {
    grid-template-columns: 1fr;
    grid-template-rows: auto 1fr auto;
  }
`;

const DeviceList = styled.div`
  display: flex;
  flex-direction: column;
  gap: var(--spacing-sm);
  overflow-y: auto;
  padding-right: var(--spacing-xs);
  height: 100%;
  scrollbar-width: thin;
`;

const MapContainer = styled.div`
  position: relative;
  height: 100%;
  min-height: 500px;
  border-radius: var(--border-radius-md);
  overflow: hidden;
  box-shadow: var(--shadow-sm);
`;

const TelemetryContainer = styled.div`
  display: flex;
  flex-direction: column;
  gap: var(--spacing-md);
  height: 100%;
  overflow-y: auto;

  @media (max-width: 1200px) {
    display: none;
  }
`;

const ErrorBanner = styled.div`
  background-color: var(--theme-status-error);
  color: white;
  padding: var(--spacing-sm);
  border-radius: var(--border-radius-sm);
  margin-bottom: var(--spacing-md);
`;

/**
 * Fleet management page component providing comprehensive interface for
 * monitoring and controlling agricultural robots and drones.
 */
const Fleet: React.FC = () => {
  // Fleet management state
  const {
    devices,
    loading,
    error,
    isConnected,
    connectionQuality,
    lastUpdate,
    updateDeviceStatus,
    batchUpdateDevices,
  } = useFleet();

  // Local state
  const [selectedDeviceId, setSelectedDeviceId] = useState<string | null>(null);
  const [mapCenter, setMapCenter] = useState({ lat: 0, lng: 0 });
  const [mapZoom, setMapZoom] = useState(12);

  // Memoized device lists
  const { drones, groundRobots } = useMemo(() => {
    const deviceArray = Array.from(devices.values());
    return {
      drones: deviceArray.filter(device => device.type === DeviceType.DRONE),
      groundRobots: deviceArray.filter(device => device.type === DeviceType.GROUND_ROBOT)
    };
  }, [devices]);

  // Selected device
  const selectedDevice = useMemo(() => 
    selectedDeviceId ? devices.get(selectedDeviceId) : null,
    [devices, selectedDeviceId]
  );

  // Handle device selection with map centering
  const handleDeviceSelect = useCallback((deviceId: string) => {
    const device = devices.get(deviceId);
    if (device) {
      setSelectedDeviceId(deviceId);
      setMapCenter({
        lat: device.location.latitude,
        lng: device.location.longitude
      });
      setMapZoom(16);
    }
  }, [devices]);

  // Handle device commands with optimistic updates
  const handleDeviceCommand = useCallback(async (
    deviceId: string,
    command: string
  ) => {
    try {
      const device = devices.get(deviceId);
      if (!device) return;

      // Optimistic update
      const newStatus = command === 'start' ? DeviceStatus.ACTIVE :
                       command === 'stop' ? DeviceStatus.IDLE :
                       command === 'recall' ? DeviceStatus.MAINTENANCE :
                       device.status;

      await updateDeviceStatus(deviceId, newStatus);
    } catch (error) {
      console.error('Command failed:', error);
      throw error;
    }
  }, [devices, updateDeviceStatus]);

  // Initialize map center on first device load
  useEffect(() => {
    if (devices.size > 0 && !selectedDeviceId) {
      const firstDevice = Array.from(devices.values())[0];
      setMapCenter({
        lat: firstDevice.location.latitude,
        lng: firstDevice.location.longitude
      });
    }
  }, [devices, selectedDeviceId]);

  return (
    <FleetContainer>
      {error && (
        <ErrorBanner role="alert">
          Error loading fleet data: {error.message}
        </ErrorBanner>
      )}

      <DeviceList>
        {drones.map(drone => (
          <DroneCard
            key={drone.id}
            device={drone}
            onSelect={handleDeviceSelect}
            onError={console.error}
          />
        ))}
        {groundRobots.map(robot => (
          <RobotCard
            key={robot.id}
            robot={robot}
            onSelect={handleDeviceSelect}
            onCommand={handleDeviceCommand}
            selected={robot.id === selectedDeviceId}
            isLoading={loading}
          />
        ))}
      </DeviceList>

      <MapContainer>
        <FleetMap
          center={mapCenter}
          zoom={mapZoom}
          onDeviceSelect={handleDeviceSelect}
          className="fleet-map"
          clusterThreshold={8}
          offlineMode={!isConnected}
        />
      </MapContainer>

      {selectedDevice && (
        <TelemetryContainer>
          <TelemetryPanel
            deviceId={selectedDevice.id}
            height={400}
            updateInterval={100}
            historyLimit={1000}
            offlineSupport={true}
            onError={console.error}
          />
        </TelemetryContainer>
      )}
    </FleetContainer>
  );
};

Fleet.displayName = 'Fleet';

export default Fleet;