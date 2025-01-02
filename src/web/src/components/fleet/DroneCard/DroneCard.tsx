import React, { memo, useCallback, useEffect, useState } from 'react';
import { Card, CardContent, CardHeader, Typography, LinearProgress, IconButton, Box, useTheme, Tooltip } from '@mui/material';
import { styled } from '@mui/material/styles';
import BatteryFullIcon from '@mui/icons-material/BatteryFull';
import BatteryAlertIcon from '@mui/icons-material/BatteryAlert';
import FlightIcon from '@mui/icons-material/Flight';
import ErrorIcon from '@mui/icons-material/Error';
import { debounce } from 'lodash';

import { IDevice, DeviceStatus } from '../../../interfaces/IDevice';
import { useWebSocket } from '../../../hooks/useWebSocket';

// Enhanced styled components with theme integration
const DroneCardContainer = styled(Card)(({ theme }) => ({
  width: '300px',
  margin: '8px',
  cursor: 'pointer',
  transition: 'all 0.2s ease-in-out',
  '&:hover': {
    transform: 'translateY(-4px)',
    boxShadow: theme.shadows[8],
  },
  '&:focus': {
    outline: `2px solid ${theme.palette.primary.main}`,
    outlineOffset: '2px',
  },
}));

const StatusIndicator = styled(Box)<{ status: DeviceStatus }>(({ theme, status }) => ({
  width: '12px',
  height: '12px',
  borderRadius: '50%',
  backgroundColor: status === DeviceStatus.ACTIVE ? theme.palette.success.main :
                  status === DeviceStatus.ERROR ? theme.palette.error.main :
                  status === DeviceStatus.MAINTENANCE ? theme.palette.warning.main :
                  theme.palette.grey[400],
  marginRight: theme.spacing(1),
}));

interface DroneCardProps {
  device: IDevice;
  onSelect?: (id: string) => void;
  onError?: (error: Error) => void;
  className?: string;
}

const DroneCard = memo(({ device, onSelect, onError, className }: DroneCardProps) => {
  const theme = useTheme();
  const [localDevice, setLocalDevice] = useState<IDevice>(device);
  const { subscribe } = useWebSocket({
    enableReconnect: true,
    onError: (error) => onError?.(error as Error),
  });

  // Optimistic update handler with debounce
  const updateDeviceData = useCallback(
    debounce((newData: Partial<IDevice>) => {
      setLocalDevice((prev) => ({
        ...prev,
        ...newData,
        lastUpdate: new Date(),
      }));
    }, 100),
    []
  );

  // WebSocket subscription for real-time updates
  useEffect(() => {
    const unsubscribe = subscribe(`device.${device.id}`, (message: any) => {
      try {
        updateDeviceData(message.payload);
      } catch (error) {
        onError?.(error as Error);
      }
    });

    return () => {
      unsubscribe();
    };
  }, [device.id, subscribe, updateDeviceData, onError]);

  // Battery status indicator with accessibility
  const BatteryIndicator = useCallback(() => {
    const isLowBattery = localDevice.batteryLevel < 20;
    return (
      <Tooltip title={`Battery Level: ${localDevice.batteryLevel}%`}>
        <Box display="flex" alignItems="center" aria-label={`Battery level ${localDevice.batteryLevel}%`}>
          {isLowBattery ? (
            <BatteryAlertIcon color="error" />
          ) : (
            <BatteryFullIcon color="success" />
          )}
          <Typography variant="body2" color={isLowBattery ? 'error' : 'textSecondary'}>
            {localDevice.batteryLevel}%
          </Typography>
        </Box>
      </Tooltip>
    );
  }, [localDevice.batteryLevel]);

  // Click handler with keyboard support
  const handleClick = useCallback(() => {
    onSelect?.(localDevice.id);
  }, [localDevice.id, onSelect]);

  const handleKeyPress = useCallback((event: React.KeyboardEvent) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      handleClick();
    }
  }, [handleClick]);

  return (
    <DroneCardContainer
      className={className}
      onClick={handleClick}
      onKeyPress={handleKeyPress}
      tabIndex={0}
      role="button"
      aria-label={`Drone ${localDevice.name} - Status: ${localDevice.status}`}
    >
      <CardHeader
        avatar={
          <StatusIndicator 
            status={localDevice.status} 
            aria-label={`Status: ${localDevice.status}`}
          />
        }
        title={
          <Typography variant="h6" component="h3">
            {localDevice.name}
          </Typography>
        }
        action={
          <IconButton
            aria-label="Drone flight status"
            size="small"
            color={localDevice.status === DeviceStatus.ERROR ? 'error' : 'default'}
          >
            {localDevice.status === DeviceStatus.ERROR ? <ErrorIcon /> : <FlightIcon />}
          </IconButton>
        }
      />
      <CardContent>
        <Box display="flex" flexDirection="column" gap={1}>
          <Box display="flex" justifyContent="space-between" alignItems="center">
            <Typography variant="body2" color="textSecondary">
              Status: {localDevice.status}
            </Typography>
            <BatteryIndicator />
          </Box>
          
          <Box>
            <Typography variant="body2" color="textSecondary" gutterBottom>
              Location
            </Typography>
            <Typography variant="body2">
              {`Lat: ${localDevice.location.latitude.toFixed(6)}`}
              <br />
              {`Long: ${localDevice.location.longitude.toFixed(6)}`}
              <br />
              {`Alt: ${localDevice.location.altitude.toFixed(1)}m`}
            </Typography>
          </Box>

          {localDevice.currentMission && (
            <Box>
              <Typography variant="body2" color="textSecondary">
                Mission Progress
              </Typography>
              <LinearProgress
                variant="determinate"
                value={75}
                aria-label="Mission progress"
              />
            </Box>
          )}

          {localDevice.errorCode && (
            <Typography 
              variant="body2" 
              color="error"
              role="alert"
              aria-live="polite"
            >
              Error: {localDevice.errorCode}
            </Typography>
          )}
        </Box>
      </CardContent>
    </DroneCardContainer>
  );
});

DroneCard.displayName = 'DroneCard';

export default DroneCard;