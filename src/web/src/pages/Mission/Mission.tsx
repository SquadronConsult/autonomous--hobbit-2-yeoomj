/**
 * @fileoverview Mission management page component with real-time updates and offline support
 * @version 1.0.0
 * 
 * Implements comprehensive mission management interface with accessibility features,
 * real-time updates, and offline capabilities as specified in the technical requirements.
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import styled from '@emotion/styled';
import { ErrorBoundary } from 'react-error-boundary';

import { IMission } from '../../interfaces/IMission';
import { useWebSocket } from '../../hooks/useWebSocket';
import { MissionTypes, MissionTypeLabels } from '../../constants/missionTypes';
import { MissionStatusCodes } from '../../constants/statusCodes';
import { API_ENDPOINTS } from '../../constants/apiEndpoints';

// Styled components with accessibility support
const PageContainer = styled.div`
  display: flex;
  flex-direction: column;
  gap: 1.5rem;
  padding: 2rem;
  height: 100%;
  overflow: auto;
  position: relative;

  @media (prefers-reduced-motion: reduce) {
    transition: none;
  }

  @media (max-width: 768px) {
    padding: 1rem;
  }
`;

const MissionList = styled.div`
  display: grid;
  gap: 1rem;
  grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
`;

const MissionCard = styled.div<{ isSelected: boolean }>`
  border: 1px solid ${props => props.isSelected ? '#0066cc' : '#e0e0e0'};
  border-radius: 8px;
  padding: 1rem;
  background: ${props => props.isSelected ? '#f0f7ff' : '#ffffff'};
  cursor: pointer;
  transition: all 0.2s ease;

  &:hover {
    border-color: #0066cc;
  }

  &:focus-within {
    outline: 2px solid #0066cc;
    outline-offset: 2px;
  }
`;

const ActionBar = styled.div`
  display: flex;
  gap: 1rem;
  align-items: center;
  padding: 1rem 0;

  @media (max-width: 768px) {
    flex-direction: column;
    align-items: stretch;
  }
`;

const Button = styled.button`
  padding: 0.5rem 1rem;
  border-radius: 4px;
  border: none;
  background: #0066cc;
  color: white;
  cursor: pointer;

  &:disabled {
    background: #cccccc;
    cursor: not-allowed;
  }

  &:focus-visible {
    outline: 2px solid #0066cc;
    outline-offset: 2px;
  }
`;

// Error fallback component
const ErrorFallback = ({ error, resetErrorBoundary }: { error: Error; resetErrorBoundary: () => void }) => (
  <div role="alert" aria-live="assertive">
    <h2>Something went wrong:</h2>
    <pre>{error.message}</pre>
    <Button onClick={resetErrorBoundary}>Try again</Button>
  </div>
);

const Mission: React.FC = () => {
  const [missions, setMissions] = useState<IMission[]>([]);
  const [selectedMission, setSelectedMission] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);

  const { 
    isConnected, 
    connect, 
    disconnect, 
    subscribe 
  } = useWebSocket({
    autoConnect: true,
    enableReconnect: true,
    onError: (err: Error) => setError(err)
  });

  // Fetch missions on component mount
  useEffect(() => {
    const fetchMissions = async () => {
      try {
        const response = await fetch(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}${API_ENDPOINTS.MISSIONS.GET_ALL}`);
        if (!response.ok) throw new Error('Failed to fetch missions');
        const data = await response.json();
        setMissions(data);
      } catch (err) {
        setError(err as Error);
      } finally {
        setIsLoading(false);
      }
    };

    fetchMissions();
  }, []);

  // Subscribe to real-time mission updates
  useEffect(() => {
    if (!isConnected) return;

    const unsubscribe = subscribe('mission-update', (message: any) => {
      setMissions(prev => prev.map(mission => 
        mission.id === message.missionId ? { ...mission, ...message.updates } : mission
      ));
    });

    return () => {
      unsubscribe();
      disconnect();
    };
  }, [isConnected, subscribe, disconnect]);

  // Handle mission selection with keyboard support
  const handleMissionSelect = useCallback((missionId: string) => {
    setSelectedMission(prev => prev === missionId ? null : missionId);
  }, []);

  // Create new mission
  const handleCreateMission = useCallback(async () => {
    try {
      const response = await fetch(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}${API_ENDPOINTS.MISSIONS.CREATE}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          type: MissionTypes.SURVEY,
          status: MissionStatusCodes.PENDING,
          parameters: {
            altitude: 50,
            speed: 5,
            scanResolution: 2
          }
        })
      });

      if (!response.ok) throw new Error('Failed to create mission');
      const newMission = await response.json();
      setMissions(prev => [...prev, newMission]);
    } catch (err) {
      setError(err as Error);
    }
  }, []);

  // Delete selected mission
  const handleDeleteMission = useCallback(async () => {
    if (!selectedMission) return;

    try {
      const response = await fetch(
        `${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}${API_ENDPOINTS.MISSIONS.DELETE.replace(':id', selectedMission)}`,
        { method: 'DELETE' }
      );

      if (!response.ok) throw new Error('Failed to delete mission');
      setMissions(prev => prev.filter(mission => mission.id !== selectedMission));
      setSelectedMission(null);
    } catch (err) {
      setError(err as Error);
    }
  }, [selectedMission]);

  if (isLoading) {
    return <div aria-live="polite">Loading missions...</div>;
  }

  return (
    <ErrorBoundary FallbackComponent={ErrorFallback} onReset={() => setError(null)}>
      <PageContainer>
        <h1>Mission Management</h1>
        
        <ActionBar>
          <Button 
            onClick={handleCreateMission}
            aria-label="Create new mission"
          >
            New Mission
          </Button>
          <Button 
            onClick={handleDeleteMission}
            disabled={!selectedMission}
            aria-label="Delete selected mission"
          >
            Delete Mission
          </Button>
        </ActionBar>

        <MissionList role="list" aria-label="Mission list">
          {missions.map(mission => (
            <MissionCard
              key={mission.id}
              isSelected={selectedMission === mission.id}
              onClick={() => handleMissionSelect(mission.id)}
              onKeyDown={e => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault();
                  handleMissionSelect(mission.id);
                }
              }}
              role="listitem"
              tabIndex={0}
              aria-selected={selectedMission === mission.id}
            >
              <h3>{mission.name}</h3>
              <p>Type: {MissionTypeLabels[mission.type]}</p>
              <p>Status: {mission.status}</p>
              <p>Progress: {mission.progress}%</p>
            </MissionCard>
          ))}
        </MissionList>

        {error && (
          <div role="alert" aria-live="assertive">
            Error: {error.message}
          </div>
        )}
      </PageContainer>
    </ErrorBoundary>
  );
};

export default Mission;