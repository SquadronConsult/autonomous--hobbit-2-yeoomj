/**
 * @fileoverview React Context provider for WebSocket functionality with enhanced security and reliability
 * @version 1.0.0
 */

import React, { createContext, useContext, useState, useEffect, useCallback, useRef, ReactNode } from 'react';
import { WebSocketManager, WebSocketMessage } from '../services/websocket';
import { API_ENDPOINTS } from '../constants/apiEndpoints';

// Rate limiting configuration
interface RateLimit {
  maxRequests: number;
  timeWindow: number; // in milliseconds
}

// WebSocket configuration interface
interface WebSocketConfig {
  reconnectLimit: number;
  heartbeatInterval: number;
  messageTimeout: number;
  enableEncryption: boolean;
  rateLimit: RateLimit;
}

// Default configuration
const DEFAULT_CONFIG: WebSocketConfig = {
  reconnectLimit: 5,
  heartbeatInterval: 30000,
  messageTimeout: 5000,
  enableEncryption: true,
  rateLimit: {
    maxRequests: 100,
    timeWindow: 60000, // 1 minute
  },
};

// Connection statistics interface
interface ConnectionStats {
  latency: number;
  messagesSent: number;
  messagesReceived: number;
  errors: number;
  lastHeartbeat: Date;
}

// Generic WebSocket context type
interface WebSocketContextType<T> {
  isConnected: boolean;
  isReconnecting: boolean;
  reconnectAttempts: number;
  lastHeartbeat: Date;
  wsManager: WebSocketManager | null;
  connect: () => Promise<void>;
  disconnect: () => void;
  sendMessage: (message: WebSocketMessage) => Promise<boolean>;
  subscribe: (event: string, callback: (message: T) => void) => () => void;
  getConnectionStats: () => ConnectionStats;
  validateConnection: () => boolean;
}

// Provider props interface
interface WebSocketProviderProps {
  children: ReactNode;
  config?: Partial<WebSocketConfig>;
}

// Create context with type safety
const WebSocketContext = createContext<WebSocketContextType<unknown> | null>(null);

/**
 * WebSocket Provider Component with enhanced security and monitoring
 */
export const WebSocketProvider: React.FC<WebSocketProviderProps> = ({ 
  children, 
  config: userConfig 
}) => {
  const [isConnected, setIsConnected] = useState(false);
  const [isReconnecting, setIsReconnecting] = useState(false);
  const [reconnectAttempts, setReconnectAttempts] = useState(0);
  const [lastHeartbeat, setLastHeartbeat] = useState<Date>(new Date());
  
  const wsManagerRef = useRef<WebSocketManager | null>(null);
  const configRef = useRef<WebSocketConfig>({ ...DEFAULT_CONFIG, ...userConfig });
  const requestCountRef = useRef<number>(0);
  const lastRequestTimeRef = useRef<number>(Date.now());

  // Initialize WebSocket manager
  useEffect(() => {
    wsManagerRef.current = new WebSocketManager({
      autoReconnect: true,
      maxReconnectAttempts: configRef.current.reconnectLimit,
      reconnectInterval: configRef.current.heartbeatInterval,
      protocols: ['v1.agricultural.protocol'],
      headers: {
        'X-Client-Version': '1.0.0',
      },
    });

    return () => {
      if (wsManagerRef.current) {
        wsManagerRef.current.disconnect();
      }
    };
  }, []);

  // Rate limiting check
  const checkRateLimit = useCallback(() => {
    const now = Date.now();
    if (now - lastRequestTimeRef.current > configRef.current.rateLimit.timeWindow) {
      requestCountRef.current = 0;
      lastRequestTimeRef.current = now;
    }

    if (requestCountRef.current >= configRef.current.rateLimit.maxRequests) {
      throw new Error('Rate limit exceeded');
    }

    requestCountRef.current++;
  }, []);

  // Connect to WebSocket server
  const connect = useCallback(async () => {
    if (!wsManagerRef.current || isConnected) return;

    try {
      setIsReconnecting(true);
      await wsManagerRef.current.connect();
      setIsConnected(true);
      setReconnectAttempts(0);
    } catch (error) {
      setReconnectAttempts(prev => prev + 1);
      throw error;
    } finally {
      setIsReconnecting(false);
    }
  }, [isConnected]);

  // Disconnect from WebSocket server
  const disconnect = useCallback(() => {
    if (!wsManagerRef.current) return;
    wsManagerRef.current.disconnect();
    setIsConnected(false);
  }, []);

  // Send message with rate limiting and encryption
  const sendMessage = useCallback(async (message: WebSocketMessage): Promise<boolean> => {
    if (!wsManagerRef.current || !isConnected) return false;

    try {
      checkRateLimit();
      
      const enhancedMessage = {
        ...message,
        timestamp: new Date().toISOString(),
        encrypted: configRef.current.enableEncryption,
      };

      return await wsManagerRef.current.send(enhancedMessage);
    } catch (error) {
      console.error('Failed to send message:', error);
      return false;
    }
  }, [isConnected, checkRateLimit]);

  // Subscribe to WebSocket events
  const subscribe = useCallback(<T,>(event: string, callback: (message: T) => void) => {
    if (!wsManagerRef.current) return () => {};

    return wsManagerRef.current.subscribe(event, callback, {
      filter: (message: WebSocketMessage) => {
        return !message.encrypted || configRef.current.enableEncryption;
      },
      maxRetries: configRef.current.reconnectLimit,
      priority: 'high',
    });
  }, []);

  // Get connection statistics
  const getConnectionStats = useCallback((): ConnectionStats => {
    if (!wsManagerRef.current) {
      return {
        latency: 0,
        messagesSent: 0,
        messagesReceived: 0,
        errors: 0,
        lastHeartbeat,
      };
    }

    const metrics = wsManagerRef.current.getMetrics();
    return {
      latency: metrics.connectionState.latency,
      messagesSent: metrics.messagesSent,
      messagesReceived: metrics.messagesReceived,
      errors: metrics.errors,
      lastHeartbeat,
    };
  }, [lastHeartbeat]);

  // Validate connection health
  const validateConnection = useCallback((): boolean => {
    if (!wsManagerRef.current || !isConnected) return false;

    const now = Date.now();
    const lastHeartbeatTime = lastHeartbeat.getTime();
    return (now - lastHeartbeatTime) <= configRef.current.heartbeatInterval * 2;
  }, [isConnected, lastHeartbeat]);

  // Context value
  const contextValue = {
    isConnected,
    isReconnecting,
    reconnectAttempts,
    lastHeartbeat,
    wsManager: wsManagerRef.current,
    connect,
    disconnect,
    sendMessage,
    subscribe,
    getConnectionStats,
    validateConnection,
  };

  return (
    <WebSocketContext.Provider value={contextValue}>
      {children}
    </WebSocketContext.Provider>
  );
};

/**
 * Custom hook for accessing WebSocket context with type safety
 */
export function useWebSocketContext<T>(): WebSocketContextType<T> {
  const context = useContext(WebSocketContext);
  if (!context) {
    throw new Error('useWebSocketContext must be used within a WebSocketProvider');
  }
  return context as WebSocketContextType<T>;
}