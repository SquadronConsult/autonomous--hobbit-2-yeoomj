import React, { useState, useEffect, useCallback, useMemo } from 'react';
import styled from '@emotion/styled';
import { useVirtualizer } from '@tanstack/react-virtual';
import { Table, TableProps } from '../../common/Table/Table';
import { useWebSocket } from '../../../hooks/useWebSocket';
import { useMission } from '../../../hooks/useMission';
import { IMission } from '../../../interfaces/IMission';
import { MissionTypes, MissionTypeLabels } from '../../../constants/missionTypes';
import { MissionStatusCodes } from '../../../constants/statusCodes';

// Version comments for dependencies
// @emotion/styled: ^11.11.0
// @tanstack/react-virtual: ^3.0.0
// react: ^18.2.0

interface MissionListProps {
  onMissionSelect?: (mission: IMission) => void;
  sortable?: boolean;
  pageSize?: number;
  className?: string;
  offlineMode?: boolean;
  updateInterval?: number;
  virtualScrolling?: boolean;
}

const ListContainer = styled.div`
  width: 100%;
  background: var(--theme-background-paper);
  border-radius: var(--border-radius-md);
  box-shadow: var(--shadow-sm);
  transition: all 0.2s ease-in-out;
  position: relative;
  overflow: hidden;

  &.offline {
    opacity: 0.9;
    &::after {
      content: 'Offline Mode';
      position: absolute;
      top: var(--spacing-xs);
      right: var(--spacing-xs);
      background: var(--theme-status-warning);
      color: white;
      padding: var(--spacing-2xs) var(--spacing-xs);
      border-radius: var(--border-radius-sm);
      font-size: var(--font-size-xs);
    }
  }

  @media (max-width: 768px) {
    padding: var(--spacing-sm);
  }

  @media (prefers-reduced-motion) {
    transition: none;
  }
`;

const StatusIndicator = styled.span<{ status: MissionStatusCodes }>`
  display: inline-flex;
  align-items: center;
  padding: var(--spacing-2xs) var(--spacing-xs);
  border-radius: var(--border-radius-sm);
  font-size: var(--font-size-xs);
  font-weight: var(--font-weight-medium);

  ${({ status }) => {
    switch (status) {
      case MissionStatusCodes.IN_PROGRESS:
        return 'background: var(--theme-status-info)14; color: var(--theme-status-info);';
      case MissionStatusCodes.COMPLETED:
        return 'background: var(--theme-status-success)14; color: var(--theme-status-success);';
      case MissionStatusCodes.FAILED:
        return 'background: var(--theme-status-error)14; color: var(--theme-status-error);';
      default:
        return 'background: var(--theme-status-warning)14; color: var(--theme-status-warning);';
    }
  }}
`;

export const MissionList: React.FC<MissionListProps> = ({
  onMissionSelect,
  sortable = true,
  pageSize = 10,
  className = '',
  offlineMode = false,
  updateInterval = 30000,
  virtualScrolling = true,
}) => {
  const {
    missions,
    loading,
    error,
    selectMission,
    updateMission,
    connectionStatus,
    syncStatus,
    refreshMissions
  } = useMission();

  const { subscribe, connectionStatus: wsStatus } = useWebSocket({
    autoConnect: true,
    enableReconnect: true,
    onMessage: handleWebSocketMessage
  });

  // Memoized table columns configuration
  const columns = useMemo(() => [
    {
      id: 'name',
      label: 'Mission Name',
      accessor: 'name',
      sortable: true,
      width: '25%',
      priority: 'high'
    },
    {
      id: 'type',
      label: 'Type',
      accessor: 'type',
      sortable: true,
      width: '15%',
      priority: 'high',
      render: (value: MissionTypes) => MissionTypeLabels[value]
    },
    {
      id: 'status',
      label: 'Status',
      accessor: 'status',
      sortable: true,
      width: '15%',
      priority: 'high',
      render: (value: MissionStatusCodes) => (
        <StatusIndicator status={value}>
          {value.replace('_', ' ')}
        </StatusIndicator>
      )
    },
    {
      id: 'progress',
      label: 'Progress',
      accessor: 'progress',
      sortable: true,
      width: '15%',
      priority: 'medium',
      render: (value: number) => `${Math.round(value)}%`
    },
    {
      id: 'startTime',
      label: 'Start Time',
      accessor: 'startTime',
      sortable: true,
      width: '15%',
      priority: 'low',
      render: (value: Date) => new Date(value).toLocaleString()
    },
    {
      id: 'assignedDevices',
      label: 'Devices',
      accessor: 'assignedDevices',
      sortable: false,
      width: '15%',
      priority: 'low',
      render: (value: string[]) => value.length
    }
  ], []);

  // Handle real-time mission updates
  function handleWebSocketMessage(message: any) {
    if (message.type === 'mission:update') {
      const { missionId, updates } = message.payload;
      updateMission(missionId, updates);
    }
  }

  // Setup periodic refresh
  useEffect(() => {
    if (!offlineMode && updateInterval > 0) {
      const interval = setInterval(refreshMissions, updateInterval);
      return () => clearInterval(interval);
    }
  }, [offlineMode, updateInterval, refreshMissions]);

  // Handle mission selection
  const handleMissionSelect = useCallback((mission: IMission) => {
    selectMission(mission);
    onMissionSelect?.(mission);
  }, [selectMission, onMissionSelect]);

  // Enhanced table props with accessibility
  const tableProps: TableProps = {
    data: missions,
    columns,
    isLoading: loading,
    isOffline: offlineMode || connectionStatus === 'disconnected',
    sortable,
    selectable: true,
    pagination: !virtualScrolling,
    pageSize,
    onSelect: handleMissionSelect,
    refreshInterval: updateInterval,
    'aria-label': 'Mission List',
    'aria-busy': loading,
    'aria-live': 'polite'
  };

  if (error) {
    return (
      <ListContainer className={`${className} error`} role="alert">
        <div>Error loading missions: {error.message}</div>
      </ListContainer>
    );
  }

  return (
    <ListContainer 
      className={`${className} ${offlineMode ? 'offline' : ''}`}
      role="region"
      aria-label="Mission Management"
    >
      <Table {...tableProps} />
    </ListContainer>
  );
};

export type { MissionListProps };