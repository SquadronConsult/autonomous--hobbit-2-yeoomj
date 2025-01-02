/**
 * @fileoverview Global test setup configuration for the Agricultural Management System
 * Configures Jest testing environment with MSW, WebSocket mocks, and test lifecycle management
 * @version 1.0.0
 */

import { setupServer } from 'msw/node'; // v1.2.0
import '@testing-library/jest-dom'; // v5.16.5
import 'jest-environment-jsdom'; // v29.5.0

import { 
  missionHandlers, 
  deviceHandlers, 
  analyticsHandlers, 
  telemetryHandlers 
} from './mocks/handlers';

import { 
  MockWebSocketManager,
  MOCK_EVENT_TYPES,
  MOCK_RECONNECT_INTERVAL 
} from './mocks/websocket';

// Configure MSW server with all request handlers
export const server = setupServer(
  ...missionHandlers,
  ...deviceHandlers,
  ...analyticsHandlers,
  ...telemetryHandlers
);

// Initialize WebSocket mock manager with default configuration
export const mockWebSocket = new MockWebSocketManager();

/**
 * Global test environment setup
 * Configures JSDOM, MSW, WebSocket mocks, and testing utilities
 */
beforeAll(async () => {
  // Configure custom JSDOM environment
  Object.defineProperty(window, 'matchMedia', {
    writable: true,
    value: jest.fn().mockImplementation(query => ({
      matches: false,
      media: query,
      onchange: null,
      addListener: jest.fn(),
      removeListener: jest.fn(),
      addEventListener: jest.fn(),
      removeEventListener: jest.fn(),
      dispatchEvent: jest.fn(),
    })),
  });

  // Configure viewport and user agent
  Object.defineProperty(window, 'innerWidth', { writable: true, value: 1024 });
  Object.defineProperty(window, 'innerHeight', { writable: true, value: 768 });
  Object.defineProperty(window.navigator, 'userAgent', {
    value: 'jest-test-environment'
  });

  // Start MSW server with response delay simulation
  server.listen({ 
    onUnhandledRequest: 'warn',
  });

  // Initialize WebSocket mock
  await mockWebSocket.connect();

  // Configure global error boundary
  const originalError = console.error;
  console.error = (...args) => {
    if (/Warning.*not wrapped in act/.test(args[0])) {
      return;
    }
    originalError.call(console, ...args);
  };

  // Configure performance monitoring
  jest.spyOn(window.performance, 'now').mockImplementation(() => Date.now());
});

/**
 * Global test environment cleanup
 * Resets all mocks and cleans up test environment
 */
afterAll(() => {
  // Stop MSW server and clear handlers
  server.close();

  // Disconnect WebSocket mock
  mockWebSocket.disconnect();

  // Reset all mocks
  jest.clearAllMocks();
  jest.restoreAllMocks();

  // Clear any remaining intervals/timeouts
  jest.useRealTimers();

  // Reset console error override
  jest.spyOn(console, 'error').mockRestore();

  // Clear performance monitoring
  window.performance.now.mockRestore?.();
});

/**
 * Reset test state before each test
 * Clears request history, WebSocket state, and DOM
 */
beforeEach(() => {
  // Reset MSW request handlers
  server.resetHandlers();

  // Clear WebSocket message queue and subscriptions
  mockWebSocket.disconnect();
  mockWebSocket.connect();

  // Reset localStorage and sessionStorage
  window.localStorage.clear();
  window.sessionStorage.clear();

  // Reset document body
  document.body.innerHTML = '';

  // Configure mock timers
  jest.useFakeTimers();

  // Reset network conditions
  server.use(
    ...missionHandlers,
    ...deviceHandlers,
    ...analyticsHandlers,
    ...telemetryHandlers
  );
});

/**
 * Clean up after each test
 * Resets DOM state and clears mocks
 */
afterEach(() => {
  // Reset document
  document.body.innerHTML = '';

  // Clear any remaining timeouts
  jest.clearAllTimers();

  // Reset all mocks
  jest.clearAllMocks();

  // Clear any error boundaries
  console.error.mockClear?.();
});

// Configure additional Jest matchers
expect.extend({
  toBeValidResponse(received) {
    const pass = received && 
      typeof received === 'object' && 
      !Array.isArray(received) &&
      'data' in received;
    
    return {
      pass,
      message: () => pass
        ? 'Expected response not to be a valid API response'
        : 'Expected response to be a valid API response with data property',
    };
  },
  toHaveValidPagination(received) {
    const pass = received && 
      typeof received === 'object' && 
      'pagination' in received &&
      typeof received.pagination === 'object' &&
      'page' in received.pagination &&
      'pageSize' in received.pagination &&
      'total' in received.pagination;
    
    return {
      pass,
      message: () => pass
        ? 'Expected response not to have valid pagination'
        : 'Expected response to have valid pagination object',
    };
  },
});