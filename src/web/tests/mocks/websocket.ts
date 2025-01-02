/**
 * @fileoverview Mock WebSocket implementation for testing real-time communication features
 * Provides comprehensive simulation of WebSocket connections, message handling, and error scenarios
 * @version 1.0.0
 */

import { EventEmitter } from 'events';
import { jest } from '@jest/globals';
import { API_ENDPOINTS } from '../../src/constants/apiEndpoints';

// Mock WebSocket global
export const MockWebSocket = jest.fn().mockImplementation(() => ({
  close: jest.fn(),
  send: jest.fn()
}));

// Configuration constants
export const MOCK_RECONNECT_INTERVAL = 1000;
export const MOCK_MAX_RECONNECT_ATTEMPTS = 3;

// Event type definitions
export const MOCK_EVENT_TYPES = {
  CONNECT: 'connect',
  DISCONNECT: 'disconnect',
  MESSAGE: 'message',
  ERROR: 'error'
} as const;

type EventType = typeof MOCK_EVENT_TYPES[keyof typeof MOCK_EVENT_TYPES];
type MessageCallback = (data: any) => void;
type ErrorCallback = (error: Error) => void;

/**
 * Mock WebSocket manager for testing real-time communication features
 * Supports connection management, event handling, and message simulation
 */
export class MockWebSocketManager {
  private eventEmitter: EventEmitter;
  private isConnected: boolean;
  private reconnectAttempts: number;
  private subscriptions: Map<string, Function[]>;
  private wsUrl: string;
  private reconnectTimer: NodeJS.Timeout | null;

  /**
   * Initialize mock WebSocket manager with test configuration
   * @param wsUrl - WebSocket endpoint URL
   */
  constructor(wsUrl: string = `ws://${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/ws`) {
    this.eventEmitter = new EventEmitter();
    this.isConnected = false;
    this.reconnectAttempts = 0;
    this.subscriptions = new Map();
    this.wsUrl = wsUrl;
    this.reconnectTimer = null;
  }

  /**
   * Simulate WebSocket connection establishment
   * @returns Promise resolving when mock connection is established
   */
  async connect(): Promise<void> {
    if (this.isConnected) {
      return;
    }

    this.reconnectAttempts++;

    if (this.reconnectAttempts > MOCK_MAX_RECONNECT_ATTEMPTS) {
      throw new Error('Maximum reconnection attempts exceeded');
    }

    return new Promise<void>((resolve) => {
      setTimeout(() => {
        this.isConnected = true;
        this.reconnectAttempts = 0;
        this.eventEmitter.emit(MOCK_EVENT_TYPES.CONNECT);
        
        if (this.reconnectTimer) {
          clearTimeout(this.reconnectTimer);
          this.reconnectTimer = null;
        }
        
        resolve();
      }, 100); // Simulate connection delay
    });
  }

  /**
   * Simulate WebSocket disconnection with cleanup
   */
  disconnect(): void {
    this.isConnected = false;
    this.subscriptions.clear();
    
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    
    this.reconnectAttempts = 0;
    this.eventEmitter.emit(MOCK_EVENT_TYPES.DISCONNECT);
    this.eventEmitter.removeAllListeners();
  }

  /**
   * Simulate sending data through WebSocket
   * @param data - Message data to send
   * @returns Promise resolving with send success status
   */
  async send(data: object): Promise<boolean> {
    if (!this.isConnected) {
      throw new Error('WebSocket is not connected');
    }

    try {
      const message = JSON.stringify(data);
      this.eventEmitter.emit(MOCK_EVENT_TYPES.MESSAGE, message);
      return true;
    } catch (error) {
      this.simulateError(error as Error);
      return false;
    }
  }

  /**
   * Subscribe to WebSocket events with type safety
   * @param event - Event type to subscribe to
   * @param callback - Event handler callback
   * @returns Unsubscribe function for cleanup
   */
  subscribe(event: EventType, callback: MessageCallback | ErrorCallback): () => void {
    if (!this.subscriptions.has(event)) {
      this.subscriptions.set(event, []);
    }

    const subscribers = this.subscriptions.get(event)!;
    subscribers.push(callback);

    return () => {
      const index = subscribers.indexOf(callback);
      if (index !== -1) {
        subscribers.splice(index, 1);
      }
    };
  }

  /**
   * Test utility to simulate incoming WebSocket messages
   * @param event - Event type for the simulated message
   * @param data - Message data to simulate
   */
  simulateMessage(event: EventType, data: any): void {
    if (!this.isConnected) {
      throw new Error('Cannot simulate message while disconnected');
    }

    const subscribers = this.subscriptions.get(event) || [];
    const message = typeof data === 'string' ? data : JSON.stringify(data);

    subscribers.forEach(callback => {
      try {
        callback(message);
      } catch (error) {
        this.simulateError(error as Error);
      }
    });

    this.eventEmitter.emit(MOCK_EVENT_TYPES.MESSAGE, message);
  }

  /**
   * Test utility to simulate WebSocket errors
   * @param error - Error to simulate
   */
  simulateError(error: Error): void {
    this.isConnected = false;
    this.eventEmitter.emit(MOCK_EVENT_TYPES.ERROR, error);

    if (this.reconnectAttempts < MOCK_MAX_RECONNECT_ATTEMPTS) {
      this.reconnectTimer = setTimeout(() => {
        this.connect().catch(() => {
          if (this.reconnectAttempts >= MOCK_MAX_RECONNECT_ATTEMPTS) {
            this.eventEmitter.emit(MOCK_EVENT_TYPES.ERROR, new Error('Connection permanently failed'));
          }
        });
      }, MOCK_RECONNECT_INTERVAL);
    } else {
      this.disconnect();
    }
  }

  /**
   * Get current connection status
   * @returns Boolean indicating connection status
   */
  getConnectionStatus(): boolean {
    return this.isConnected;
  }

  /**
   * Get current number of reconnection attempts
   * @returns Number of reconnection attempts
   */
  getReconnectAttempts(): number {
    return this.reconnectAttempts;
  }
}