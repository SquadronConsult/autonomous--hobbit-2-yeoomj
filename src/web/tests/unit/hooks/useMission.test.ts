/**
 * @fileoverview Comprehensive unit test suite for useMission hook
 * @version 1.0.0
 */

import { renderHook, act, waitFor } from '@testing-library/react-hooks'; // v8.0.1
import { describe, it, expect, beforeEach, afterEach } from '@jest/globals'; // v29.5.0
import { mockPerformanceObserver } from '@testing-library/jest-dom'; // v5.16.5
import { useMission } from '../../../src/hooks/useMission';
import { mockWebSocket } from '../../mocks/websocket';
import { MissionTypes } from '../../../src/constants/missionTypes';
import { MissionStatusCodes } from '../../../src/constants/statusCodes';

// Test constants
const TEST_MISSION = {
  id: 'test-mission-1',
  name: 'Test Survey Mission',
  description: 'Automated test mission',
  type: MissionTypes.SURVEY,
  status: MissionStatusCodes.PENDING,
  assignedDevices: ['drone-1', 'drone-2'],
  coverageArea: {
    type: 'Polygon',
    coordinates: [[[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]]
  },
  startTime: new Date(),
  endTime: null,
  progress: 0,
  parameters: {
    altitude: 50,
    speed: 5,
    scanResolution: 2,
    treatmentType: '',
    treatmentDensity: 0
  }
};

const MOCK_ERROR = new Error('Test error');
const PERFORMANCE_THRESHOLDS = {
  initTime: 100,
  updateLatency: 50,
  memoryLimit: 50000
};

describe('useMission Hook', () => {
  // Setup and cleanup
  beforeEach(() => {
    mockPerformanceObserver.resetMocks();
    localStorage.clear();
    jest.useFakeTimers();
  });

  afterEach(() => {
    jest.clearAllMocks();
    jest.useRealTimers();
  });

  describe('Initialization and Loading', () => {
    it('should initialize with default state', () => {
      const { result } = renderHook(() => useMission());

      expect(result.current.missions).toEqual([]);
      expect(result.current.loading).toBe(true);
      expect(result.current.error).toBeNull();
      expect(result.current.selectedMission).toBeNull();
    });

    it('should load missions on mount', async () => {
      const { result } = renderHook(() => useMission());

      await waitFor(() => {
        expect(result.current.loading).toBe(false);
        expect(result.current.missions.length).toBeGreaterThan(0);
      });
    });

    it('should measure initialization performance', () => {
      const { result } = renderHook(() => useMission());

      expect(mockPerformanceObserver.getEntries()).toContainEqual(
        expect.objectContaining({
          name: 'mission-hook-init',
          duration: expect.any(Number)
        })
      );
      expect(mockPerformanceObserver.getEntries()[0].duration).toBeLessThan(PERFORMANCE_THRESHOLDS.initTime);
    });
  });

  describe('Mission Management', () => {
    it('should create new mission with optimistic update', async () => {
      const { result } = renderHook(() => useMission());

      await act(async () => {
        await result.current.createMission(TEST_MISSION);
      });

      expect(result.current.missions).toContainEqual(
        expect.objectContaining({ name: TEST_MISSION.name })
      );
    });

    it('should update existing mission', async () => {
      const { result } = renderHook(() => useMission());
      const updatedProgress = 50;

      await act(async () => {
        await result.current.createMission(TEST_MISSION);
        await result.current.updateMission(TEST_MISSION.id, { progress: updatedProgress });
      });

      expect(result.current.missions.find(m => m.id === TEST_MISSION.id)?.progress).toBe(updatedProgress);
    });

    it('should delete mission with cleanup', async () => {
      const { result } = renderHook(() => useMission());

      await act(async () => {
        await result.current.createMission(TEST_MISSION);
        await result.current.deleteMission(TEST_MISSION.id);
      });

      expect(result.current.missions).not.toContainEqual(
        expect.objectContaining({ id: TEST_MISSION.id })
      );
    });
  });

  describe('Real-time Updates', () => {
    it('should handle WebSocket mission updates', async () => {
      const { result } = renderHook(() => useMission());
      const updateData = { progress: 75, status: MissionStatusCodes.IN_PROGRESS };

      await act(async () => {
        await result.current.createMission(TEST_MISSION);
        mockWebSocket.simulateMessage('mission:update', {
          missionId: TEST_MISSION.id,
          updates: updateData,
          timestamp: new Date().toISOString()
        });
      });

      expect(result.current.missions.find(m => m.id === TEST_MISSION.id)).toMatchObject(updateData);
    });

    it('should handle WebSocket reconnection', async () => {
      const { result } = renderHook(() => useMission());

      await act(async () => {
        mockWebSocket.simulateDisconnect();
        await waitFor(() => expect(result.current.connectionStatus).toBe('disconnected'));
        mockWebSocket.simulateReconnect();
        await waitFor(() => expect(result.current.connectionStatus).toBe('connected'));
      });
    });

    it('should measure update latency', async () => {
      const { result } = renderHook(() => useMission());

      await act(async () => {
        const start = performance.now();
        await result.current.updateMission(TEST_MISSION.id, { progress: 25 });
        const latency = performance.now() - start;
        expect(latency).toBeLessThan(PERFORMANCE_THRESHOLDS.updateLatency);
      });
    });
  });

  describe('Offline Support', () => {
    it('should handle mission creation in offline mode', async () => {
      const { result } = renderHook(() => useMission());

      await act(async () => {
        // Simulate offline mode
        Object.defineProperty(navigator, 'onLine', { value: false });
        await result.current.createMission(TEST_MISSION);
      });

      expect(result.current.missions).toContainEqual(
        expect.objectContaining({
          name: TEST_MISSION.name,
          status: MissionStatusCodes.PENDING
        })
      );
      expect(localStorage.getItem('cachedMissions')).toBeTruthy();
    });

    it('should sync offline changes when back online', async () => {
      const { result } = renderHook(() => useMission());

      await act(async () => {
        Object.defineProperty(navigator, 'onLine', { value: false });
        await result.current.createMission(TEST_MISSION);
        Object.defineProperty(navigator, 'onLine', { value: true });
        // Trigger online event
        window.dispatchEvent(new Event('online'));
      });

      await waitFor(() => {
        expect(result.current.syncStatus).toBe('synced');
      });
    });

    it('should handle conflict resolution during sync', async () => {
      const { result } = renderHook(() => useMission());
      const offlineUpdate = { progress: 30 };
      const serverUpdate = { progress: 50 };

      await act(async () => {
        Object.defineProperty(navigator, 'onLine', { value: false });
        await result.current.updateMission(TEST_MISSION.id, offlineUpdate);
        mockWebSocket.simulateMessage('mission:update', {
          missionId: TEST_MISSION.id,
          updates: serverUpdate,
          timestamp: new Date().toISOString()
        });
        Object.defineProperty(navigator, 'onLine', { value: true });
        window.dispatchEvent(new Event('online'));
      });

      await waitFor(() => {
        const mission = result.current.missions.find(m => m.id === TEST_MISSION.id);
        expect(mission?.progress).toBe(serverUpdate.progress);
      });
    });
  });

  describe('Error Handling', () => {
    it('should handle mission creation failure', async () => {
      const { result } = renderHook(() => useMission());

      await act(async () => {
        mockWebSocket.simulateError(MOCK_ERROR);
        await result.current.createMission(TEST_MISSION);
      });

      expect(result.current.error).toBeTruthy();
      expect(result.current.missions).not.toContainEqual(
        expect.objectContaining({ id: TEST_MISSION.id })
      );
    });

    it('should retry failed operations with backoff', async () => {
      const { result } = renderHook(() => useMission());
      let retryCount = 0;

      await act(async () => {
        mockWebSocket.simulateError(MOCK_ERROR);
        await result.current.refreshMissions().catch(() => retryCount++);
        jest.advanceTimersByTime(5000);
      });

      expect(retryCount).toBeGreaterThan(0);
      expect(retryCount).toBeLessThanOrEqual(3);
    });

    it('should cleanup resources on error', async () => {
      const { result, unmount } = renderHook(() => useMission());

      await act(async () => {
        mockWebSocket.simulateError(MOCK_ERROR);
        unmount();
      });

      expect(mockWebSocket.getSubscriptions().size).toBe(0);
    });
  });

  describe('Performance Monitoring', () => {
    it('should track memory usage', () => {
      const { result } = renderHook(() => useMission());

      expect(result.current.metrics.averageResponseTime).toBeDefined();
      expect(performance.memory.usedJSHeapSize).toBeLessThan(PERFORMANCE_THRESHOLDS.memoryLimit);
    });

    it('should measure operation success rate', async () => {
      const { result } = renderHook(() => useMission());

      await act(async () => {
        await result.current.createMission(TEST_MISSION);
        await result.current.updateMission(TEST_MISSION.id, { progress: 25 });
      });

      expect(result.current.metrics.successRate).toBeGreaterThanOrEqual(95);
    });

    it('should track pending operations', async () => {
      const { result } = renderHook(() => useMission());

      await act(async () => {
        mockWebSocket.simulateDisconnect();
        await result.current.createMission(TEST_MISSION);
      });

      expect(result.current.metrics.pendingOperations).toBeGreaterThan(0);
    });
  });
});