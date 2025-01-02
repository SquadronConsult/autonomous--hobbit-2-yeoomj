import React, { useMemo, useCallback } from 'react';
import styled from '@emotion/styled';
import { useTheme } from '@mui/material';
import debounce from 'lodash/debounce';

import { IDevice } from '../../../interfaces/IDevice';
import { useFleet } from '../../../hooks/useFleet';
import Card from '../../common/Card/Card';

// Constants for battery level thresholds and update interval
const CRITICAL_BATTERY = 20;
const WARNING_BATTERY = 40;
const DEFAULT_UPDATE_INTERVAL = 1000;

// Styled components with theme support
const BatteryContainer = styled(Card)`
  min-width: 250px;
  margin: 8px;
`;

const BatteryIndicator = styled.div<{ level: number; theme: any }>`
  width: 100%;
  height: 8px;
  border-radius: 4px;
  background-color: ${props => getBatteryColor(props.level, props.theme)};
  transition: all 0.3s ease-in-out;
  position: relative;
  overflow: hidden;

  &::before {
    content: '';
    position: absolute;
    top: 0;
    left: 0;
    width: ${props => props.level}%;
    height: 100%;
    background-color: ${props => getBatteryColor(props.level, props.theme)};
  }
`;

const BatteryWarning = styled.div`
  padding: 8px;
  margin: 8px 0;
  border-radius: 4px;
  background-color: ${props => props.theme.status.error};
  color: ${props => props.theme.text.primary};
  font-size: 14px;
  font-weight: 500;
`;

const DeviceInfo = styled.div`
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 12px;
`;

const DeviceName = styled.span`
  font-weight: 500;
  color: ${props => props.theme.text.primary};
`;

const BatteryLevel = styled.span`
  color: ${props => props.theme.text.secondary};
`;

const LastUpdate = styled.div`
  font-size: 12px;
  color: ${props => props.theme.text.disabled};
  margin-top: 8px;
  text-align: right;
`;

// Props interface
interface BatteryStatusProps {
  deviceId?: string;
  className?: string;
  showWarnings?: boolean;
  updateInterval?: number;
}

// Helper function to determine battery color based on level
const getBatteryColor = (level: number, theme: any): string => {
  if (level <= CRITICAL_BATTERY) {
    return theme.status.error;
  }
  if (level <= WARNING_BATTERY) {
    return theme.status.warning;
  }
  return theme.status.success;
};

// Format timestamp for last update display
const formatLastUpdate = (date: Date): string => {
  return new Intl.RelativeTimeFormat('en', { numeric: 'auto' }).format(
    Math.round((date.getTime() - Date.now()) / 1000 / 60),
    'minute'
  );
};

const BatteryStatus: React.FC<BatteryStatusProps> = React.memo(({
  deviceId,
  className,
  showWarnings = true,
  updateInterval = DEFAULT_UPDATE_INTERVAL
}) => {
  const theme = useTheme();
  const { devices, loading, error } = useFleet();

  // Debounced update handler for performance
  const handleBatteryUpdate = useMemo(
    () => debounce((device: IDevice) => {
      console.log(`Battery update for device ${device.id}: ${device.batteryLevel}%`);
    }, updateInterval),
    [updateInterval]
  );

  // Filter and sort devices based on battery level
  const relevantDevices = useMemo(() => {
    if (deviceId) {
      return devices.filter(device => device.id === deviceId);
    }
    return Array.from(devices.values())
      .sort((a, b) => a.batteryLevel - b.batteryLevel);
  }, [devices, deviceId]);

  // Render battery warning if needed
  const renderBatteryWarning = useCallback((device: IDevice) => {
    if (!showWarnings || device.batteryLevel > CRITICAL_BATTERY) {
      return null;
    }

    return (
      <BatteryWarning
        role="alert"
        aria-live="polite"
      >
        Critical battery level for {device.name}!
      </BatteryWarning>
    );
  }, [showWarnings]);

  if (loading) {
    return <BatteryContainer>Loading battery status...</BatteryContainer>;
  }

  if (error) {
    return <BatteryContainer>Error loading battery status</BatteryContainer>;
  }

  if (relevantDevices.length === 0) {
    return <BatteryContainer>No devices found</BatteryContainer>;
  }

  return (
    <>
      {relevantDevices.map(device => (
        <BatteryContainer
          key={device.id}
          className={className}
          variant="elevated"
          aria-label={`Battery status for ${device.name}`}
        >
          <DeviceInfo>
            <DeviceName>{device.name}</DeviceName>
            <BatteryLevel>
              {device.batteryLevel}%
            </BatteryLevel>
          </DeviceInfo>

          <BatteryIndicator
            level={device.batteryLevel}
            theme={theme}
            role="progressbar"
            aria-valuemin={0}
            aria-valuemax={100}
            aria-valuenow={device.batteryLevel}
            onTransitionEnd={() => handleBatteryUpdate(device)}
          />

          {renderBatteryWarning(device)}

          <LastUpdate>
            Last updated: {formatLastUpdate(device.lastActive)}
          </LastUpdate>
        </BatteryContainer>
      ))}
    </>
  );
});

BatteryStatus.displayName = 'BatteryStatus';

export default BatteryStatus;