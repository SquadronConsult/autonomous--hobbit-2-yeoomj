import React, { useCallback, useEffect, useMemo } from 'react';
import styled from '@emotion/styled';
import { useIntersectionObserver } from 'react-intersection-observer';
import { IMission } from '../../../interfaces/IMission';
import Card from '../../common/Card/Card';
import { useMission } from '../../../hooks/useMission';
import { MissionTypes } from '../../../constants/missionTypes';

// Styled components with enhanced accessibility and animations
const StatusContainer = styled.div`
  display: grid;
  grid-gap: 16px;
  padding: 16px;
  width: 100%;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  transition: all 0.3s ease;

  @media (max-width: 768px) {
    grid-template-columns: 1fr;
  }
`;

const ProgressBar = styled.div<{ progress: number; status: string }>`
  width: 100%;
  height: 4px;
  background-color: ${props => props.theme.background.default};
  border-radius: 2px;
  overflow: hidden;
  position: relative;

  &::after {
    content: '';
    position: absolute;
    top: 0;
    left: 0;
    height: 100%;
    width: ${props => props.progress}%;
    background-color: ${props => getStatusColor(props.status)};
    transition: width 0.3s ease-in-out;
  }
`;

const StatusIndicator = styled.span<{ status: string }>`
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 12px;
  font-weight: 500;
  display: flex;
  align-items: center;
  gap: 4px;
  background-color: ${props => getStatusColor(props.status)};
  color: ${props => props.theme.text.onPrimary};
`;

const MissionInfo = styled.div`
  display: flex;
  flex-direction: column;
  gap: 8px;
`;

const MissionHeader = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
`;

const MissionTitle = styled.h3`
  margin: 0;
  font-size: 16px;
  font-weight: 600;
  color: ${props => props.theme.text.primary};
`;

const MissionDetails = styled.div`
  display: flex;
  flex-direction: column;
  gap: 4px;
  font-size: 14px;
  color: ${props => props.theme.text.secondary};
`;

const OfflineIndicator = styled.div`
  display: flex;
  align-items: center;
  gap: 4px;
  color: ${props => props.theme.status.warning};
  font-size: 12px;
`;

// Props interface
interface MissionStatusProps {
  className?: string;
  filterType?: MissionTypes;
  sortOrder?: 'asc' | 'desc';
}

// Helper function for status-specific colors
const getStatusColor = (status: string): string => {
  switch (status) {
    case 'IN_PROGRESS':
      return '#2196F3';
    case 'COMPLETED':
      return '#4CAF50';
    case 'FAILED':
      return '#F44336';
    case 'QUEUED':
      return '#9E9E9E';
    case 'PAUSED':
      return '#FFC107';
    default:
      return '#757575';
  }
};

// Enhanced MissionStatus component with real-time updates and offline support
const MissionStatus: React.FC<MissionStatusProps> = React.memo(({
  className,
  filterType,
  sortOrder = 'desc'
}) => {
  const {
    missions,
    loading,
    error,
    connectionStatus,
    refreshMissions,
    metrics
  } = useMission();

  // Intersection observer for progressive loading
  const { ref, inView } = useIntersectionObserver({
    threshold: 0.1,
    triggerOnce: true
  });

  // Filter and sort missions
  const filteredMissions = useMemo(() => {
    let result = missions;

    if (filterType) {
      result = result.filter(mission => mission.type === filterType);
    }

    return result.sort((a, b) => {
      const dateA = new Date(a.startTime).getTime();
      const dateB = new Date(b.startTime).getTime();
      return sortOrder === 'asc' ? dateA - dateB : dateB - dateA;
    });
  }, [missions, filterType, sortOrder]);

  // Handle refresh with error tracking
  const handleRefresh = useCallback(async () => {
    try {
      await refreshMissions();
    } catch (error) {
      console.error('Failed to refresh missions:', error);
    }
  }, [refreshMissions]);

  // Auto-refresh on visibility change
  useEffect(() => {
    if (inView) {
      handleRefresh();
    }
  }, [inView, handleRefresh]);

  // Error state
  if (error) {
    return (
      <StatusContainer className={className} role="alert">
        <Card variant="flat">
          <MissionInfo>
            <MissionTitle>Error Loading Missions</MissionTitle>
            <MissionDetails>{error.message}</MissionDetails>
          </MissionInfo>
        </Card>
      </StatusContainer>
    );
  }

  // Loading state
  if (loading) {
    return (
      <StatusContainer className={className} aria-busy="true">
        {[1, 2, 3].map(i => (
          <Card key={i} variant="flat">
            <MissionInfo>
              <MissionHeader>
                <div style={{ height: 20, width: 200, background: '#f0f0f0' }} />
              </MissionHeader>
              <div style={{ height: 16, width: '100%', background: '#f0f0f0' }} />
            </MissionInfo>
          </Card>
        ))}
      </StatusContainer>
    );
  }

  return (
    <StatusContainer 
      className={className}
      ref={ref}
      role="region"
      aria-label="Mission Status Overview"
    >
      {connectionStatus !== 'connected' && (
        <OfflineIndicator role="status" aria-live="polite">
          <span>⚠️</span>
          <span>Working offline - Updates will sync when connection is restored</span>
        </OfflineIndicator>
      )}

      {filteredMissions.map(mission => (
        <Card
          key={mission.id}
          variant="elevated"
          aria-label={`Mission ${mission.name}`}
        >
          <MissionInfo>
            <MissionHeader>
              <MissionTitle>{mission.name}</MissionTitle>
              <StatusIndicator 
                status={mission.status}
                role="status"
                aria-label={`Status: ${mission.status}`}
              >
                {mission.status}
              </StatusIndicator>
            </MissionHeader>

            <MissionDetails>
              <span>Type: {mission.type}</span>
              <span>Start: {new Date(mission.startTime).toLocaleString()}</span>
              {mission.endTime && (
                <span>End: {new Date(mission.endTime).toLocaleString()}</span>
              )}
            </MissionDetails>

            <ProgressBar
              progress={mission.progress}
              status={mission.status}
              role="progressbar"
              aria-valuenow={mission.progress}
              aria-valuemin={0}
              aria-valuemax={100}
              aria-label={`Mission progress: ${mission.progress}%`}
            />

            {mission.errorDetails && (
              <div 
                role="alert"
                aria-label="Mission Error Details"
                style={{ color: getStatusColor('FAILED') }}
              >
                {mission.errorDetails.message}
              </div>
            )}
          </MissionInfo>
        </Card>
      ))}

      {metrics && (
        <div aria-hidden="true">
          <small>Last sync: {metrics.lastSyncTime?.toLocaleString()}</small>
        </div>
      )}
    </StatusContainer>
  );
});

MissionStatus.displayName = 'MissionStatus';

export default MissionStatus;