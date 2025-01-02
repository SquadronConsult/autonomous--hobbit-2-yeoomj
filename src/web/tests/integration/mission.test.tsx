/**
 * @fileoverview Integration tests for Mission management functionality
 * @version 1.0.0
 * 
 * Tests mission management features including real-time updates, offline capabilities,
 * and accessibility compliance as specified in the technical requirements.
 */

import React from 'react';
import { render, screen, fireEvent, waitFor, within } from '@testing-library/react';
import { rest } from 'msw';
import { setupServer } from 'msw/node';
import { vi, describe, it, expect, beforeEach, afterEach } from 'vitest';
import axe from '@axe-core/react';

import Mission from '../../src/pages/Mission/Mission';
import { IMission } from '../../src/interfaces/IMission';
import { MissionTypes } from '../../src/constants/missionTypes';
import { MissionStatusCodes } from '../../src/constants/statusCodes';
import { API_ENDPOINTS } from '../../src/constants/apiEndpoints';

// WebSocket event constants
const WS_EVENTS = {
  MISSION_UPDATE: 'mission:update',
  STATUS_UPDATE: 'mission:status',
  FLEET_UPDATE: 'fleet:update'
} as const;

// Mock mission data generator
const createMockMission = (id: string): IMission => ({
  id,
  name: `Mission ${id}`,
  description: `Test mission ${id}`,
  type: MissionTypes.SURVEY,
  status: MissionStatusCodes.PENDING,
  assignedDevices: [`drone-${id}`, `ground-${id}`],
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
});

// Create mock missions array
const mockMissions = Array.from({ length: 5 }, (_, i) => createMockMission(`m${i}`));

// MSW server setup
const server = setupServer(
  // GET missions
  rest.get(
    `${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}${API_ENDPOINTS.MISSIONS.GET_ALL}`,
    (req, res, ctx) => res(ctx.json(mockMissions))
  ),

  // POST new mission
  rest.post(
    `${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}${API_ENDPOINTS.MISSIONS.CREATE}`,
    async (req, res, ctx) => {
      const newMission = createMockMission(`m${mockMissions.length}`);
      mockMissions.push(newMission);
      return res(ctx.json(newMission));
    }
  ),

  // DELETE mission
  rest.delete(
    `${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}${API_ENDPOINTS.MISSIONS.DELETE}`,
    (req, res, ctx) => res(ctx.status(200))
  )
);

// Mock WebSocket setup
const mockWebSocket = {
  send: vi.fn(),
  close: vi.fn(),
  addEventListener: vi.fn(),
  removeEventListener: vi.fn()
};

// Setup and teardown
beforeEach(() => {
  server.listen();
  // Mock WebSocket
  global.WebSocket = vi.fn().mockImplementation(() => mockWebSocket);
  // Mock IndexedDB for offline support
  const indexedDB = {
    open: vi.fn(),
    deleteDatabase: vi.fn()
  };
  global.indexedDB = indexedDB as any;
});

afterEach(() => {
  server.resetHandlers();
  vi.clearAllMocks();
});

describe('Mission Management Integration Tests', () => {
  describe('Mission Loading and Display', () => {
    it('should load and display missions correctly', async () => {
      render(<Mission />);
      
      // Check loading state
      expect(screen.getByText('Loading missions...')).toBeInTheDocument();
      
      // Wait for missions to load
      await waitFor(() => {
        mockMissions.forEach(mission => {
          expect(screen.getByText(mission.name)).toBeInTheDocument();
        });
      });
    });

    it('should handle loading errors gracefully', async () => {
      server.use(
        rest.get(
          `${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}${API_ENDPOINTS.MISSIONS.GET_ALL}`,
          (req, res, ctx) => res(ctx.status(500))
        )
      );

      render(<Mission />);
      await waitFor(() => {
        expect(screen.getByRole('alert')).toHaveTextContent('Failed to fetch missions');
      });
    });
  });

  describe('Mission Creation and Deletion', () => {
    it('should create a new mission successfully', async () => {
      render(<Mission />);
      
      const createButton = await screen.findByRole('button', { name: /create new mission/i });
      fireEvent.click(createButton);

      await waitFor(() => {
        expect(screen.getByText(`Mission m${mockMissions.length - 1}`)).toBeInTheDocument();
      });
    });

    it('should delete a mission after confirmation', async () => {
      render(<Mission />);
      
      // Wait for missions to load and select one
      const firstMission = await screen.findByText('Mission m0');
      fireEvent.click(firstMission);

      // Click delete button
      const deleteButton = screen.getByRole('button', { name: /delete mission/i });
      fireEvent.click(deleteButton);

      await waitFor(() => {
        expect(screen.queryByText('Mission m0')).not.toBeInTheDocument();
      });
    });
  });

  describe('Real-time Updates', () => {
    it('should handle WebSocket mission updates', async () => {
      render(<Mission />);
      
      // Simulate WebSocket connection
      const wsHandlers = mockWebSocket.addEventListener.mock.calls
        .find(call => call[0] === 'message')[1];

      // Simulate mission update
      wsHandlers({
        data: JSON.stringify({
          type: WS_EVENTS.MISSION_UPDATE,
          payload: {
            missionId: 'm0',
            updates: {
              status: MissionStatusCodes.IN_PROGRESS,
              progress: 50
            }
          }
        })
      });

      await waitFor(() => {
        expect(screen.getByText('50%')).toBeInTheDocument();
      });
    });

    it('should maintain state during WebSocket disconnection', async () => {
      render(<Mission />);
      
      // Wait for initial load
      await waitFor(() => {
        expect(screen.getByText('Mission m0')).toBeInTheDocument();
      });

      // Simulate WebSocket disconnection
      const wsHandlers = mockWebSocket.addEventListener.mock.calls
        .find(call => call[0] === 'close')[1];
      wsHandlers();

      // Verify state is maintained
      expect(screen.getByText('Mission m0')).toBeInTheDocument();
    });
  });

  describe('Offline Support', () => {
    it('should work offline using IndexedDB', async () => {
      // Simulate offline mode
      server.use(
        rest.get('*', (req, res, ctx) => res(ctx.status(503)))
      );

      render(<Mission />);

      // Verify offline indicator
      await waitFor(() => {
        expect(screen.getByText(/offline mode/i)).toBeInTheDocument();
      });

      // Verify cached missions are displayed
      mockMissions.forEach(mission => {
        expect(screen.getByText(mission.name)).toBeInTheDocument();
      });
    });
  });

  describe('Performance Tests', () => {
    it('should handle multiple mission updates efficiently', async () => {
      const startTime = performance.now();
      render(<Mission />);

      // Simulate multiple rapid updates
      const wsHandlers = mockWebSocket.addEventListener.mock.calls
        .find(call => call[0] === 'message')[1];

      for (let i = 0; i < 100; i++) {
        wsHandlers({
          data: JSON.stringify({
            type: WS_EVENTS.MISSION_UPDATE,
            payload: {
              missionId: `m${i % 5}`,
              updates: { progress: i }
            }
          })
        });
      }

      await waitFor(() => {
        const endTime = performance.now();
        expect(endTime - startTime).toBeLessThan(100); // 100ms performance requirement
      });
    });
  });

  describe('Accessibility Tests', () => {
    it('should meet WCAG 2.1 Level AA standards', async () => {
      const { container } = render(<Mission />);
      
      await waitFor(() => {
        expect(screen.getByText('Mission m0')).toBeInTheDocument();
      });

      const results = await axe.run(container);
      expect(results.violations).toHaveLength(0);
    });

    it('should support keyboard navigation', async () => {
      render(<Mission />);
      
      // Wait for missions to load
      await waitFor(() => {
        expect(screen.getByText('Mission m0')).toBeInTheDocument();
      });

      // Test keyboard navigation
      const firstMission = screen.getByText('Mission m0').closest('[role="listitem"]');
      firstMission?.focus();
      expect(document.activeElement).toBe(firstMission);

      // Test keyboard selection
      fireEvent.keyDown(firstMission!, { key: 'Enter' });
      expect(firstMission).toHaveAttribute('aria-selected', 'true');
    });
  });
});