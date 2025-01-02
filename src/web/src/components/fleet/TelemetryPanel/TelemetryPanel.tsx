/**
 * @fileoverview High-performance telemetry visualization component with offline support
 * @version 1.0.0
 */

import React, { useState, useEffect, useCallback, useMemo, useRef } from 'react';
import styled from '@emotion/styled';
import { useVirtualizer } from '@tanstack/react-virtual';
import { ITelemetry } from '../../interfaces/ITelemetry';
import { useWebSocket } from '../../hooks/useWebSocket';
import { TELEMETRY } from '../../../constants/apiEndpoints';

// Constants for component configuration
const TELEMETRY_HISTORY_LIMIT = 1000;
const CHART_UPDATE_INTERVAL = 100;
const BATCH_UPDATE_SIZE = 50;
const RECONNECTION_ATTEMPTS = 5;
const OFFLINE_STORAGE_KEY = 'telemetry_offline_data';

// WebSocket event types
const WEBSOCKET_EVENTS = {
  TELEMETRY_UPDATE: 'telemetry:update',
  TELEMETRY_ERROR: 'telemetry:error',
  TELEMETRY_RECONNECT: 'telemetry:reconnect',
  TELEMETRY_OFFLINE: 'telemetry:offline'
} as const;

// Component props interface
interface TelemetryPanelProps {
  deviceId: string;
  height?: number;
  width?: number;
  className?: string;
  updateInterval?: number;
  historyLimit?: number;
  onError?: (error: Error) => void;
  offlineSupport?: boolean;
}

// Styled components
const PanelContainer = styled.div<{ width?: number; height?: number }>`
  display: flex;
  flex-direction: column;
  gap: 1rem;
  padding: 1rem;
  background-color: ${({ theme }) => theme.colors.background.secondary};
  border-radius: ${({ theme }) => theme.borderRadius.medium};
  box-shadow: ${({ theme }) => theme.shadows.medium};
  width: ${({ width }) => (width ? `${width}px` : '100%')};
  height: ${({ height }) => (height ? `${height}px` : '400px')};
  position: relative;
  outline: none;
  transition: all 0.2s ease;

  &:focus-visible {
    outline: 2px solid ${({ theme }) => theme.colors.primary.main};
  }

  @media (prefers-reduced-motion) {
    transition: none;
  }
`;

const TelemetryList = styled.div`
  flex: 1;
  overflow: auto;
  scrollbar-width: thin;
  scrollbar-color: ${({ theme }) => `${theme.colors.scrollbar.thumb} ${theme.colors.scrollbar.track}`};
`;

const TelemetryItem = styled.div<{ status: string }>`
  padding: 0.5rem;
  border-bottom: 1px solid ${({ theme }) => theme.colors.border.light};
  display: grid;
  grid-template-columns: auto 1fr auto;
  gap: 1rem;
  align-items: center;
  background-color: ${({ status, theme }) => 
    status === 'error' ? theme.colors.status.error.background :
    status === 'warning' ? theme.colors.status.warning.background :
    'transparent'
  };
`;

const StatusIndicator = styled.div<{ status: string }>`
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background-color: ${({ status, theme }) => theme.colors.status[status].main};
`;

/**
 * High-performance telemetry visualization component with offline support
 */
const TelemetryPanel: React.FC<TelemetryPanelProps> = React.memo(({
  deviceId,
  height = 400,
  width,
  className,
  updateInterval = CHART_UPDATE_INTERVAL,
  historyLimit = TELEMETRY_HISTORY_LIMIT,
  onError,
  offlineSupport = true
}) => {
  // State management
  const [telemetryData, setTelemetryData] = useState<ITelemetry[]>([]);
  const [isOffline, setIsOffline] = useState(false);
  
  // Refs for performance optimization
  const telemetryQueueRef = useRef<ITelemetry[]>([]);
  const updateTimeoutRef = useRef<NodeJS.Timeout>();
  
  // WebSocket connection management
  const { isConnected, connectionStatus, subscribe, reconnect } = useWebSocket({
    autoConnect: true,
    enableReconnect: true,
    maxReconnectAttempts: RECONNECTION_ATTEMPTS,
    onError: (error: Error) => onError?.(error)
  });

  // Virtual list configuration for performance
  const parentRef = useRef<HTMLDivElement>(null);
  const rowVirtualizer = useVirtualizer({
    count: telemetryData.length,
    getScrollElement: () => parentRef.current,
    estimateSize: () => 40,
    overscan: 5
  });

  // Offline storage management
  const saveToOfflineStorage = useCallback((data: ITelemetry[]) => {
    if (offlineSupport) {
      try {
        localStorage.setItem(
          `${OFFLINE_STORAGE_KEY}_${deviceId}`,
          JSON.stringify(data.slice(-historyLimit))
        );
      } catch (error) {
        console.error('Failed to save offline data:', error);
      }
    }
  }, [deviceId, historyLimit, offlineSupport]);

  const loadFromOfflineStorage = useCallback(() => {
    if (offlineSupport) {
      try {
        const stored = localStorage.getItem(`${OFFLINE_STORAGE_KEY}_${deviceId}`);
        return stored ? JSON.parse(stored) : [];
      } catch (error) {
        console.error('Failed to load offline data:', error);
        return [];
      }
    }
    return [];
  }, [deviceId, offlineSupport]);

  // Batch update management
  const processTelemetryQueue = useCallback(() => {
    if (telemetryQueueRef.current.length > 0) {
      setTelemetryData(current => {
        const newData = [...current, ...telemetryQueueRef.current]
          .slice(-historyLimit)
          .sort((a, b) => new Date(a.timestamp).getTime() - new Date(b.timestamp).getTime());
        
        if (offlineSupport) {
          saveToOfflineStorage(newData);
        }
        
        return newData;
      });
      
      telemetryQueueRef.current = [];
    }
    
    updateTimeoutRef.current = setTimeout(processTelemetryQueue, updateInterval);
  }, [historyLimit, updateInterval, offlineSupport, saveToOfflineStorage]);

  // WebSocket subscription setup
  useEffect(() => {
    const unsubscribe = subscribe(WEBSOCKET_EVENTS.TELEMETRY_UPDATE, (data: ITelemetry) => {
      if (data.deviceId === deviceId) {
        telemetryQueueRef.current.push(data);
      }
    });

    processTelemetryQueue();

    return () => {
      unsubscribe();
      if (updateTimeoutRef.current) {
        clearTimeout(updateTimeoutRef.current);
      }
    };
  }, [deviceId, subscribe, processTelemetryQueue]);

  // Offline mode management
  useEffect(() => {
    const handleOffline = () => {
      setIsOffline(true);
      if (offlineSupport) {
        const offlineData = loadFromOfflineStorage();
        setTelemetryData(offlineData);
      }
    };

    const handleOnline = () => {
      setIsOffline(false);
      reconnect();
    };

    window.addEventListener('offline', handleOffline);
    window.addEventListener('online', handleOnline);

    return () => {
      window.removeEventListener('offline', handleOffline);
      window.removeEventListener('online', handleOnline);
    };
  }, [offlineSupport, loadFromOfflineStorage, reconnect]);

  // Virtual list rendering
  const virtualItems = useMemo(() => rowVirtualizer.getVirtualItems(), [rowVirtualizer]);

  return (
    <PanelContainer
      ref={parentRef}
      width={width}
      height={height}
      className={className}
      tabIndex={0}
      role="region"
      aria-label="Telemetry Data Panel"
    >
      <TelemetryList
        style={{
          height: `${rowVirtualizer.getTotalSize()}px`,
          width: '100%',
          position: 'relative'
        }}
      >
        {virtualItems.map(virtualRow => {
          const item = telemetryData[virtualRow.index];
          return (
            <TelemetryItem
              key={item.id}
              style={{
                position: 'absolute',
                top: 0,
                left: 0,
                width: '100%',
                transform: `translateY(${virtualRow.start}px)`
              }}
              status={item.status}
              role="listitem"
              aria-label={`Telemetry reading from ${new Date(item.timestamp).toLocaleString()}`}
            >
              <StatusIndicator status={item.status} aria-hidden="true" />
              <span>{item.value} {item.unit}</span>
              <span>{new Date(item.timestamp).toLocaleTimeString()}</span>
            </TelemetryItem>
          );
        })}
      </TelemetryList>
      {isOffline && (
        <div role="alert" aria-live="polite">
          Operating in offline mode
        </div>
      )}
    </PanelContainer>
  );
});

TelemetryPanel.displayName = 'TelemetryPanel';

export default TelemetryPanel;