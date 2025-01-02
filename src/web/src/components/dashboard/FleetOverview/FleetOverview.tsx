/**
 * @fileoverview Enhanced FleetOverview component for real-time agricultural robot fleet monitoring
 * Provides comprehensive status visualization and performance metrics for drone and ground robot fleets
 * @version 1.0.0
 */

import React, { useMemo, useEffect, useCallback } from 'react';
import styled from '@emotion/styled';
import { FixedSizeList as VirtualList } from 'react-window'; // v1.8.9
import { withErrorBoundary } from 'react-error-boundary'; // v4.0.0
import { IDevice, DeviceType, DeviceStatus } from '../../../interfaces/IDevice';
import { useFleet } from '../../../hooks/useFleet';

// Constants for component configuration
const REFRESH_INTERVAL = 1000; // 1 second refresh rate
const BATTERY_WARNING_THRESHOLD = 20;
const BATTERY_CRITICAL_THRESHOLD = 10;
const VIRTUAL_LIST_ITEM_SIZE = 80;
const VIRTUAL_LIST_HEIGHT = 600;

// Styled components for enhanced visualization
const FleetContainer = styled.div`
  display: flex;
  flex-direction: column;
  gap: 1rem;
  padding: 1rem;
  background: var(--background-primary);
  border-radius: 8px;
  box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);
`;

const FleetHeader = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding-bottom: 1rem;
  border-bottom: 1px solid var(--border-color);
`;

const StatsContainer = styled.div`
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 1rem;
  margin-bottom: 1rem;
`;

const StatCard = styled.div<{ status?: 'warning' | 'critical' | 'normal' }>`
  padding: 1rem;
  background: ${({ status }) => 
    status === 'warning' ? 'var(--warning-bg)' :
    status === 'critical' ? 'var(--error-bg)' :
    'var(--background-secondary)'};
  border-radius: 6px;
  transition: background-color 0.3s ease;
`;

const DeviceListContainer = styled.div`
  border: 1px solid var(--border-color);
  border-radius: 6px;
  overflow: hidden;
`;

const DeviceItem = styled.div<{ status: DeviceStatus }>`
  display: grid;
  grid-template-columns: 1fr 2fr 1fr 1fr 1fr;
  gap: 1rem;
  padding: 1rem;
  align-items: center;
  background: ${({ status }) => 
    status === DeviceStatus.ERROR ? 'var(--error-bg)' :
    status === DeviceStatus.MAINTENANCE ? 'var(--warning-bg)' :
    'var(--background-secondary)'};
  border-bottom: 1px solid var(--border-color);
`;

const ConnectionStatus = styled.div<{ connected: boolean }>`
  display: flex;
  align-items: center;
  gap: 0.5rem;
  color: ${({ connected }) => connected ? 'var(--success-color)' : 'var(--error-color)'};
`;

// Interface for fleet statistics
interface FleetStats {
  totalDevices: number;
  activeDevices: number;
  averageBatteryLevel: number;
  criticalBatteryCount: number;
  errorCount: number;
  maintenanceCount: number;
  drones: {
    total: number;
    active: number;
  };
  groundRobots: {
    total: number;
    active: number;
  };
}

/**
 * Enhanced FleetOverview component with real-time monitoring capabilities
 */
const FleetOverviewComponent: React.FC = () => {
  const {
    devices,
    loading,
    error,
    isConnected,
    connectionQuality,
    refreshDevices
  } = useFleet();

  // Calculate fleet statistics with memoization
  const fleetStats = useMemo((): FleetStats => {
    const deviceArray = Array.from(devices.values());
    
    return {
      totalDevices: deviceArray.length,
      activeDevices: deviceArray.filter(d => d.status === DeviceStatus.ACTIVE).length,
      averageBatteryLevel: deviceArray.reduce((acc, d) => acc + d.batteryLevel, 0) / deviceArray.length || 0,
      criticalBatteryCount: deviceArray.filter(d => d.batteryLevel <= BATTERY_CRITICAL_THRESHOLD).length,
      errorCount: deviceArray.filter(d => d.status === DeviceStatus.ERROR).length,
      maintenanceCount: deviceArray.filter(d => d.status === DeviceStatus.MAINTENANCE).length,
      drones: {
        total: deviceArray.filter(d => d.type === DeviceType.DRONE).length,
        active: deviceArray.filter(d => d.type === DeviceType.DRONE && d.status === DeviceStatus.ACTIVE).length
      },
      groundRobots: {
        total: deviceArray.filter(d => d.type === DeviceType.GROUND_ROBOT).length,
        active: deviceArray.filter(d => d.type === DeviceType.GROUND_ROBOT && d.status === DeviceStatus.ACTIVE).length
      }
    };
  }, [devices]);

  // Auto-refresh implementation
  useEffect(() => {
    const intervalId = setInterval(refreshDevices, REFRESH_INTERVAL);
    return () => clearInterval(intervalId);
  }, [refreshDevices]);

  // Virtual list row renderer
  const renderDeviceRow = useCallback(({ index, style }: { index: number; style: React.CSSProperties }) => {
    const device = Array.from(devices.values())[index];
    if (!device) return null;

    return (
      <DeviceItem status={device.status} style={style}>
        <span>{device.id}</span>
        <span>{device.type === DeviceType.DRONE ? 'Drone' : 'Ground Robot'}</span>
        <span>{device.status}</span>
        <span>{`${device.batteryLevel}%`}</span>
        <span>{device.lastUpdate.toLocaleTimeString()}</span>
      </DeviceItem>
    );
  }, [devices]);

  if (error) {
    throw error;
  }

  return (
    <FleetContainer>
      <FleetHeader>
        <h2>Fleet Overview</h2>
        <ConnectionStatus connected={isConnected}>
          <span>●</span>
          {isConnected ? `Connected (${connectionQuality})` : 'Disconnected'}
        </ConnectionStatus>
      </FleetHeader>

      <StatsContainer>
        <StatCard>
          <h3>Total Devices</h3>
          <p>{fleetStats.totalDevices}</p>
          <p>Active: {fleetStats.activeDevices}</p>
        </StatCard>

        <StatCard status={fleetStats.averageBatteryLevel <= BATTERY_WARNING_THRESHOLD ? 'warning' : 'normal'}>
          <h3>Battery Status</h3>
          <p>{`${Math.round(fleetStats.averageBatteryLevel)}% Average`}</p>
          <p>{fleetStats.criticalBatteryCount} Critical</p>
        </StatCard>

        <StatCard status={fleetStats.errorCount > 0 ? 'critical' : 'normal'}>
          <h3>System Status</h3>
          <p>{fleetStats.errorCount} Errors</p>
          <p>{fleetStats.maintenanceCount} In Maintenance</p>
        </StatCard>

        <StatCard>
          <h3>Drones</h3>
          <p>{fleetStats.drones.active} / {fleetStats.drones.total} Active</p>
        </StatCard>

        <StatCard>
          <h3>Ground Robots</h3>
          <p>{fleetStats.groundRobots.active} / {fleetStats.groundRobots.total} Active</p>
        </StatCard>
      </StatsContainer>

      <DeviceListContainer>
        {loading ? (
          <div>Loading fleet data...</div>
        ) : (
          <VirtualList
            height={VIRTUAL_LIST_HEIGHT}
            itemCount={devices.size}
            itemSize={VIRTUAL_LIST_ITEM_SIZE}
            width="100%"
          >
            {renderDeviceRow}
          </VirtualList>
        )}
      </DeviceListContainer>
    </FleetContainer>
  );
};

// Export with error boundary wrapper
export const FleetOverview = withErrorBoundary(FleetOverviewComponent, {
  fallback: <div>Error loading fleet overview. Please try refreshing the page.</div>,
  onError: (error) => {
    console.error('Fleet Overview Error:', error);
    // Additional error reporting logic could be added here
  }
});