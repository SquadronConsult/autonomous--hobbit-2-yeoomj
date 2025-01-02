import React, { useCallback, useMemo } from 'react';
import styled from '@emotion/styled';
import { debounce } from 'lodash'; // ^4.17.21
import Card from '../../common/Card/Card';
import Icon from '../../common/Icons/Icons';
import Button from '../../common/Button/Button';
import { IDevice, DeviceStatus } from '../../../interfaces/IDevice';

// Styled components with responsive design and theme support
const CardContent = styled.div`
  display: flex;
  flex-direction: column;
  gap: 12px;
  width: 100%;
  padding: 16px;

  @media (max-width: 768px) {
    padding: 12px;
  }
`;

const StatusBadge = styled.span<{ status: string; error: boolean }>`
  display: inline-flex;
  align-items: center;
  gap: 4px;
  padding: 4px 8px;
  border-radius: 16px;
  font-size: 12px;
  font-weight: 500;
  background-color: ${({ status, error, theme }) => 
    error ? theme.status.error :
    status === DeviceStatus.ACTIVE ? theme.status.success :
    status === DeviceStatus.CHARGING ? theme.status.warning :
    status === DeviceStatus.MAINTENANCE ? theme.status.info :
    theme.text.disabled};
  color: ${({ theme }) => theme.background.paper};
  transition: background-color 0.3s ease;
`;

const BatteryIndicator = styled.div<{ level: number }>`
  display: flex;
  align-items: center;
  gap: 8px;
  color: ${({ level, theme }) => 
    level <= 10 ? theme.status.error :
    level <= 20 ? theme.status.warning :
    theme.text.primary};
  transition: color 0.3s ease;
`;

const LocationInfo = styled.div`
  display: flex;
  flex-direction: column;
  gap: 4px;
  font-size: 14px;
  color: ${({ theme }) => theme.text.secondary};
`;

const ErrorMessage = styled.div`
  color: ${({ theme }) => theme.status.error};
  font-size: 12px;
  margin-top: 8px;
  padding: 8px;
  border-radius: 4px;
  background-color: ${({ theme }) => `${theme.status.error}14`};
`;

const Controls = styled.div`
  display: flex;
  gap: 8px;
  margin-top: 8px;
`;

// Props interface with comprehensive type safety
interface RobotCardProps {
  robot: IDevice;
  onSelect: (id: string) => void;
  onCommand: (id: string, command: string) => Promise<void>;
  selected: boolean;
  isLoading?: boolean;
}

// Utility functions for formatting and status handling
const formatBatteryLevel = (level: number): string => {
  if (level < 0 || level > 100) return 'Invalid';
  return `${Math.round(level)}%${level <= 20 ? ' ⚠️' : ''}`;
};

const formatLocation = (location: IDevice['location']): string => {
  return `${location.latitude.toFixed(6)}, ${location.longitude.toFixed(6)}`;
};

export const RobotCard: React.FC<RobotCardProps> = ({
  robot,
  onSelect,
  onCommand,
  selected,
  isLoading = false
}) => {
  // Debounced command handler to prevent rapid repeated calls
  const debouncedCommand = useCallback(
    debounce(async (command: string) => {
      try {
        await onCommand(robot.id, command);
      } catch (error) {
        console.error(`Command failed for robot ${robot.id}:`, error);
      }
    }, 300),
    [robot.id, onCommand]
  );

  // Memoized status check for performance
  const hasError = useMemo(() => 
    robot.status === DeviceStatus.ERROR || Boolean(robot.errorCode),
    [robot.status, robot.errorCode]
  );

  return (
    <Card
      variant="elevated"
      onClick={() => onSelect(robot.id)}
      aria-label={`Robot ${robot.name} control card`}
      aria-selected={selected}
      aria-busy={isLoading}
      aria-invalid={hasError}
    >
      <CardContent>
        <div role="heading" aria-level={3}>
          <Icon 
            name="drone"
            size={24}
            aria-hidden="true"
          />
          {robot.name}
        </div>

        <StatusBadge 
          status={robot.status}
          error={hasError}
          aria-label={`Status: ${robot.status}${hasError ? ' with error' : ''}`}
        >
          {robot.status}
        </StatusBadge>

        <BatteryIndicator 
          level={robot.batteryLevel}
          aria-label={`Battery level: ${formatBatteryLevel(robot.batteryLevel)}`}
        >
          Battery: {formatBatteryLevel(robot.batteryLevel)}
        </BatteryIndicator>

        <LocationInfo aria-label="Current location">
          <span>Location:</span>
          <span>{formatLocation(robot.location)}</span>
        </LocationInfo>

        {hasError && (
          <ErrorMessage
            role="alert"
            aria-live="polite"
          >
            {robot.errorCode || 'Unknown error occurred'}
          </ErrorMessage>
        )}

        <Controls>
          <Button
            variant="contained"
            color="primary"
            size="small"
            disabled={isLoading || hasError}
            onClick={() => debouncedCommand('start')}
            aria-label="Start mission"
          >
            Start
          </Button>
          <Button
            variant="outlined"
            color="warning"
            size="small"
            disabled={isLoading || hasError}
            onClick={() => debouncedCommand('stop')}
            aria-label="Stop mission"
          >
            Stop
          </Button>
          <Button
            variant="outlined"
            color="secondary"
            size="small"
            disabled={isLoading || hasError}
            onClick={() => debouncedCommand('recall')}
            aria-label="Recall robot"
          >
            Recall
          </Button>
        </Controls>
      </CardContent>
    </Card>
  );
};

RobotCard.displayName = 'RobotCard';

export default RobotCard;