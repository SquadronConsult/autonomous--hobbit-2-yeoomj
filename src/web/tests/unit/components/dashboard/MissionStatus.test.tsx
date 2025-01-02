import React from 'react';
import { render, screen, fireEvent, waitFor, within } from '@testing-library/react';
import { axe, toHaveNoViolations } from 'jest-axe';
import { ThemeProvider } from '@emotion/react';
import { act } from 'react-dom/test-utils';
import MissionStatus from '../../../../src/components/dashboard/MissionStatus/MissionStatus';
import { IMission } from '../../../../src/interfaces/IMission';
import { MissionTypes } from '../../../../src/constants/missionTypes';
import { MissionStatusCodes } from '../../../../src/constants/statusCodes';
import { lightTheme } from '../../../../src/constants/theme';
import { useMission } from '../../../../src/hooks/useMission';

// Extend Jest matchers
expect.extend(toHaveNoViolations);

// Mock useMission hook
jest.mock('../../../../src/hooks/useMission');
const mockUseMission = useMission as jest.MockedFunction<typeof useMission>;

// Mock mission data
const mockMissions: IMission[] = [
  {
    id: 'mission-1',
    name: 'Area Survey Alpha',
    description: 'Aerial survey of northern fields',
    type: MissionTypes.SURVEY,
    status: MissionStatusCodes.IN_PROGRESS,
    assignedDevices: ['drone-1', 'drone-2'],
    coverageArea: {
      type: 'Polygon',
      coordinates: [[[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]]
    },
    startTime: new Date('2024-02-20T08:00:00Z'),
    endTime: null,
    progress: 45,
    parameters: {
      altitude: 50,
      speed: 5,
      scanResolution: 2,
      treatmentType: '',
      treatmentDensity: 0
    }
  },
  {
    id: 'mission-2',
    name: 'Pest Treatment Beta',
    description: 'Targeted pest control operation',
    type: MissionTypes.TREATMENT,
    status: MissionStatusCodes.QUEUED,
    assignedDevices: ['drone-3'],
    coverageArea: {
      type: 'Polygon',
      coordinates: [[[1, 1], [2, 1], [2, 2], [1, 2], [1, 1]]]
    },
    startTime: new Date('2024-02-20T09:00:00Z'),
    endTime: null,
    progress: 0,
    parameters: {
      altitude: 30,
      speed: 3,
      scanResolution: 0,
      treatmentType: 'pesticide-a',
      treatmentDensity: 2.5
    }
  }
];

describe('MissionStatus Component', () => {
  // Setup before each test
  beforeEach(() => {
    jest.useFakeTimers();
    mockUseMission.mockReturnValue({
      missions: mockMissions,
      loading: false,
      error: null,
      connectionStatus: 'connected',
      syncStatus: 'synced',
      metrics: {
        averageResponseTime: 50,
        successRate: 100,
        pendingOperations: 0,
        lastSyncTime: new Date()
      },
      refreshMissions: jest.fn(),
      createMission: jest.fn(),
      updateMission: jest.fn(),
      deleteMission: jest.fn(),
      selectMission: jest.fn()
    });
  });

  // Cleanup after each test
  afterEach(() => {
    jest.clearAllMocks();
    jest.useRealTimers();
  });

  describe('Rendering States', () => {
    test('renders loading state correctly', () => {
      mockUseMission.mockReturnValueOnce({
        ...mockUseMission(),
        loading: true
      });

      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus />
        </ThemeProvider>
      );

      expect(screen.getByRole('region')).toHaveAttribute('aria-busy', 'true');
      expect(screen.getAllByRole('article')).toHaveLength(3); // Loading skeletons
    });

    test('renders error state correctly', () => {
      const errorMessage = 'Failed to load missions';
      mockUseMission.mockReturnValueOnce({
        ...mockUseMission(),
        error: new Error(errorMessage)
      });

      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus />
        </ThemeProvider>
      );

      expect(screen.getByRole('alert')).toHaveTextContent(errorMessage);
    });

    test('renders missions correctly', () => {
      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus />
        </ThemeProvider>
      );

      mockMissions.forEach(mission => {
        const missionElement = screen.getByRole('article', { name: `Mission ${mission.name}` });
        expect(missionElement).toBeInTheDocument();
        expect(within(missionElement).getByText(mission.name)).toBeInTheDocument();
        expect(within(missionElement).getByText(`Type: ${mission.type}`)).toBeInTheDocument();
        expect(within(missionElement).getByRole('progressbar')).toHaveAttribute('aria-valuenow', mission.progress.toString());
      });
    });
  });

  describe('Real-time Updates', () => {
    test('updates mission progress in real-time', async () => {
      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus />
        </ThemeProvider>
      );

      const updatedMissions = [...mockMissions];
      updatedMissions[0].progress = 75;

      act(() => {
        mockUseMission.mockReturnValueOnce({
          ...mockUseMission(),
          missions: updatedMissions
        });
      });

      await waitFor(() => {
        const progressBar = screen.getAllByRole('progressbar')[0];
        expect(progressBar).toHaveAttribute('aria-valuenow', '75');
      });
    });

    test('handles offline mode correctly', () => {
      mockUseMission.mockReturnValueOnce({
        ...mockUseMission(),
        connectionStatus: 'disconnected'
      });

      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus />
        </ThemeProvider>
      );

      expect(screen.getByRole('status')).toHaveTextContent(/working offline/i);
    });
  });

  describe('Filtering and Sorting', () => {
    test('filters missions by type correctly', () => {
      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus filterType={MissionTypes.SURVEY} />
        </ThemeProvider>
      );

      const missionElements = screen.getAllByRole('article');
      expect(missionElements).toHaveLength(1);
      expect(within(missionElements[0]).getByText(/Area Survey Alpha/)).toBeInTheDocument();
    });

    test('sorts missions by start time correctly', () => {
      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus sortOrder="asc" />
        </ThemeProvider>
      );

      const missionElements = screen.getAllByRole('article');
      expect(within(missionElements[0]).getByText(/Area Survey Alpha/)).toBeInTheDocument();
      expect(within(missionElements[1]).getByText(/Pest Treatment Beta/)).toBeInTheDocument();
    });
  });

  describe('Accessibility', () => {
    test('meets WCAG accessibility guidelines', async () => {
      const { container } = render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus />
        </ThemeProvider>
      );

      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    test('supports keyboard navigation', () => {
      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus />
        </ThemeProvider>
      );

      const missionCards = screen.getAllByRole('article');
      missionCards.forEach(card => {
        fireEvent.focus(card);
        expect(card).toHaveFocus();
      });
    });

    test('provides appropriate ARIA labels', () => {
      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus />
        </ThemeProvider>
      );

      expect(screen.getByRole('region')).toHaveAttribute('aria-label', 'Mission Status Overview');
      expect(screen.getAllByRole('progressbar')).toHaveLength(mockMissions.length);
      screen.getAllByRole('progressbar').forEach(progressBar => {
        expect(progressBar).toHaveAttribute('aria-valuemin', '0');
        expect(progressBar).toHaveAttribute('aria-valuemax', '100');
      });
    });
  });

  describe('Performance', () => {
    test('renders within performance budget', async () => {
      const startTime = performance.now();
      
      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus />
        </ThemeProvider>
      );

      const endTime = performance.now();
      const renderTime = endTime - startTime;
      
      expect(renderTime).toBeLessThan(100); // 100ms budget
    });

    test('handles large mission lists efficiently', () => {
      const largeMissionList = Array(100).fill(null).map((_, index) => ({
        ...mockMissions[0],
        id: `mission-${index}`,
        name: `Mission ${index}`
      }));

      mockUseMission.mockReturnValueOnce({
        ...mockUseMission(),
        missions: largeMissionList
      });

      const startTime = performance.now();
      
      render(
        <ThemeProvider theme={lightTheme}>
          <MissionStatus />
        </ThemeProvider>
      );

      const endTime = performance.now();
      const renderTime = endTime - startTime;
      
      expect(renderTime).toBeLessThan(500); // 500ms budget for large lists
    });
  });
});