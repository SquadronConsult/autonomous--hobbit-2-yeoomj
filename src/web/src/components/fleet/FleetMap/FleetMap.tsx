import React, { useCallback, useEffect, useMemo, useState } from 'react';
import styled from 'styled-components';
import { IDevice, DeviceType, DeviceStatus } from '../../../interfaces/IDevice';
import Map from '../../common/Map/Map';
import { useFleet } from '../../../hooks/useFleet';

/**
 * Props interface for FleetMap component
 */
interface FleetMapProps {
  /** Initial map center coordinates */
  center: { lat: number; lng: number };
  /** Initial map zoom level */
  zoom: number;
  /** Callback when device is selected */
  onDeviceSelect?: (device: IDevice) => void;
  /** Optional CSS class name */
  className?: string;
  /** Device count threshold for clustering */
  clusterThreshold?: number;
  /** Flag for offline operation mode */
  offlineMode?: boolean;
}

/**
 * Enhanced map component for visualizing and managing agricultural robot fleets
 * Provides real-time position tracking and status monitoring for both aerial and ground robots
 * 
 * @version 1.0.0
 */
const FleetMap: React.FC<FleetMapProps> = ({
  center,
  zoom,
  onDeviceSelect,
  className = '',
  clusterThreshold = 8,
  offlineMode = false,
}) => {
  // Fleet management hook with offline support
  const {
    devices,
    loading,
    error,
    isConnected,
    connectionQuality,
    lastUpdate,
    updateDeviceStatus,
  } = useFleet();

  // Local state for selected device
  const [selectedDeviceId, setSelectedDeviceId] = useState<string | null>(null);

  // Convert devices Map to array for rendering
  const deviceArray = useMemo(() => Array.from(devices.values()), [devices]);

  // Filter active devices for map display
  const activeDevices = useMemo(() => 
    deviceArray.filter(device => device.status !== DeviceStatus.OFFLINE),
    [deviceArray]
  );

  /**
   * Enhanced device click handler with accessibility support
   */
  const handleDeviceClick = useCallback((device: IDevice, event: React.MouseEvent | React.KeyboardEvent) => {
    event.stopPropagation();
    
    if (device.status === DeviceStatus.OFFLINE && !offlineMode) {
      return;
    }

    setSelectedDeviceId(device.id);
    onDeviceSelect?.(device);
  }, [offlineMode, onDeviceSelect]);

  /**
   * Generates comprehensive marker style based on device properties
   */
  const getDeviceMarkerStyle = useCallback((device: IDevice) => {
    const baseStyle = {
      backgroundColor: device.type === DeviceType.DRONE ? '#4CAF50' : '#2196F3',
      opacity: device.status === DeviceStatus.OFFLINE ? 0.5 : 1,
      transform: `scale(${device.status === DeviceStatus.ACTIVE ? 1.2 : 1})`,
      boxShadow: selectedDeviceId === device.id ? '0 0 0 3px #FFC107' : 'none',
    };

    // Add battery level indicator
    const batteryColor = device.batteryLevel > 50 ? '#4CAF50' : 
                        device.batteryLevel > 20 ? '#FFC107' : '#F44336';

    // Add connection quality indicator
    const connectionColor = device.connectionQuality > 80 ? '#4CAF50' :
                          device.connectionQuality > 40 ? '#FFC107' : '#F44336';

    return {
      ...baseStyle,
      '&::after': {
        content: '""',
        position: 'absolute',
        bottom: '-4px',
        right: '-4px',
        width: '8px',
        height: '8px',
        borderRadius: '50%',
        backgroundColor: batteryColor,
        border: `2px solid ${connectionColor}`,
      }
    };
  }, [selectedDeviceId]);

  /**
   * Renders device information popup
   */
  const renderDevicePopup = useCallback((device: IDevice) => (
    <MarkerPopup>
      <h4>{device.name}</h4>
      <p>Status: {device.status}</p>
      <p>Battery: {device.batteryLevel}%</p>
      <p>Last Update: {new Date(device.lastSyncTime).toLocaleTimeString()}</p>
    </MarkerPopup>
  ), []);

  // Update map when connection status changes
  useEffect(() => {
    if (!isConnected && !offlineMode) {
      console.warn('Connection lost, switching to offline mode');
    }
  }, [isConnected, offlineMode]);

  return (
    <StyledFleetMap className={className}>
      <Map
        center={center}
        zoom={zoom}
        devices={activeDevices}
        onDeviceClick={handleDeviceClick}
        enableClustering={activeDevices.length > clusterThreshold}
        offlineSupport={offlineMode}
        accessibilityMode={true}
      >
        {/* Connection status indicator */}
        <ConnectionIndicator 
          status={connectionQuality}
          lastUpdate={lastUpdate}
          isOffline={!isConnected}
        />

        {/* Device markers with status indicators */}
        {activeDevices.map(device => (
          <DeviceMarker
            key={device.id}
            style={getDeviceMarkerStyle(device)}
            onClick={(e) => handleDeviceClick(device, e)}
            onKeyPress={(e) => e.key === 'Enter' && handleDeviceClick(device, e)}
            tabIndex={0}
            role="button"
            aria-label={`${device.type} ${device.name} - Status: ${device.status}`}
          >
            {selectedDeviceId === device.id && renderDevicePopup(device)}
          </DeviceMarker>
        ))}
      </Map>
    </StyledFleetMap>
  );
};

// Styled components
const StyledFleetMap = styled.div`
  width: 100%;
  height: 100%;
  min-height: 500px;
  position: relative;
  overflow: hidden;
  touch-action: none;
`;

const DeviceMarker = styled.div`
  width: 24px;
  height: 24px;
  border-radius: 50%;
  border: 2px solid var(--theme-background-paper);
  transition: all 0.3s ease-in-out;
  cursor: pointer;
  transform-origin: center;
  will-change: transform;

  &:focus {
    outline: none;
    box-shadow: 0 0 0 3px var(--theme-primary-main);
  }

  &:hover {
    transform: scale(1.1);
  }
`;

const MarkerPopup = styled.div`
  padding: 12px;
  background: var(--theme-background-paper);
  border-radius: 4px;
  box-shadow: var(--theme-elevation-2);
  min-width: 200px;
  z-index: 1000;
`;

const ConnectionIndicator = styled.div<{ status: string; isOffline: boolean }>`
  position: absolute;
  top: 16px;
  right: 16px;
  padding: 8px 16px;
  border-radius: 4px;
  background: ${props => props.isOffline ? '#F44336' : 
    props.status === 'excellent' ? '#4CAF50' :
    props.status === 'good' ? '#FFC107' : '#F44336'};
  color: white;
  font-size: 14px;
  font-weight: 500;
  z-index: 1000;
`;

export default FleetMap;