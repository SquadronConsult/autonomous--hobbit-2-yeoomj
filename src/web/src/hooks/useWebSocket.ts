/**
 * @fileoverview Enhanced WebSocket hook for real-time communication with agricultural system
 * @version 1.0.0
 */

import { useState, useEffect, useCallback, useRef } from 'react'; // v18.2.0
import { useWebSocketContext } from '../contexts/WebSocketContext';

// WebSocket hook configuration options
interface WebSocketOptions {
  autoConnect?: boolean;
  enableReconnect?: boolean;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
  onMessage?: (message: any) => void;
  onConnect?: () => void;
  onDisconnect?: () => void;
  onReconnect?: () => void;
  onError?: (error: Error) => void;
}

// Extended connection status information
interface ConnectionStatus {
  isConnected: boolean;
  latency: number;
  reconnectAttempts: number;
  connectionQuality: 'excellent' | 'good' | 'poor' | 'disconnected';
  queuedMessages: number;
}

/**
 * Enhanced WebSocket hook with automatic reconnection and monitoring
 */
export function useWebSocket(options: WebSocketOptions = {}) {
  const {
    isConnected: contextConnected,
    connect: contextConnect,
    disconnect: contextDisconnect,
    sendMessage: contextSendMessage,
    subscribe: contextSubscribe,
    getConnectionStats,
    getQueuedMessages
  } = useWebSocketContext();

  const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>({
    isConnected: false,
    latency: 0,
    reconnectAttempts: 0,
    connectionQuality: 'disconnected',
    queuedMessages: 0
  });

  const optionsRef = useRef(options);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout>();
  const monitoringIntervalRef = useRef<NodeJS.Timeout>();

  // Update connection quality based on latency
  const updateConnectionQuality = useCallback((latency: number): 'excellent' | 'good' | 'poor' | 'disconnected' => {
    if (!contextConnected) return 'disconnected';
    if (latency < 100) return 'excellent';
    if (latency < 300) return 'good';
    return 'poor';
  }, [contextConnected]);

  // Monitor connection health
  const monitorConnection = useCallback(() => {
    const stats = getConnectionStats();
    const queuedMessages = getQueuedMessages().length;

    setConnectionStatus({
      isConnected: contextConnected,
      latency: stats.latency,
      reconnectAttempts: stats.reconnectAttempts,
      connectionQuality: updateConnectionQuality(stats.latency),
      queuedMessages
    });
  }, [contextConnected, getConnectionStats, getQueuedMessages, updateConnectionQuality]);

  // Handle reconnection
  const handleReconnect = useCallback(async () => {
    if (!optionsRef.current.enableReconnect || 
        connectionStatus.reconnectAttempts >= (optionsRef.current.maxReconnectAttempts || 5)) {
      return;
    }

    try {
      await contextConnect();
      optionsRef.current.onReconnect?.();
    } catch (error) {
      optionsRef.current.onError?.(error as Error);
      reconnectTimeoutRef.current = setTimeout(
        handleReconnect,
        optionsRef.current.reconnectInterval || 5000
      );
    }
  }, [contextConnect, connectionStatus.reconnectAttempts]);

  // Initialize connection monitoring
  useEffect(() => {
    monitoringIntervalRef.current = setInterval(monitorConnection, 1000);
    return () => {
      if (monitoringIntervalRef.current) {
        clearInterval(monitoringIntervalRef.current);
      }
    };
  }, [monitorConnection]);

  // Handle automatic connection
  useEffect(() => {
    if (optionsRef.current.autoConnect) {
      contextConnect()
        .then(() => optionsRef.current.onConnect?.())
        .catch(error => optionsRef.current.onError?.(error));
    }

    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
    };
  }, [contextConnect]);

  // Handle connection state changes
  useEffect(() => {
    if (contextConnected) {
      optionsRef.current.onConnect?.();
    } else {
      optionsRef.current.onDisconnect?.();
      if (optionsRef.current.enableReconnect) {
        handleReconnect();
      }
    }
  }, [contextConnected, handleReconnect]);

  // Enhanced message sending with queue management
  const sendMessage = useCallback(async (data: any): Promise<boolean> => {
    try {
      return await contextSendMessage(data);
    } catch (error) {
      optionsRef.current.onError?.(error as Error);
      return false;
    }
  }, [contextSendMessage]);

  // Enhanced subscription with type safety
  const subscribe = useCallback((event: string, callback: Function) => {
    return contextSubscribe(event, (message: any) => {
      try {
        callback(message);
        optionsRef.current.onMessage?.(message);
      } catch (error) {
        optionsRef.current.onError?.(error as Error);
      }
    });
  }, [contextSubscribe]);

  // Clear message queue
  const clearQueue = useCallback(() => {
    while (getQueuedMessages().length > 0) {
      getQueuedMessages().shift();
    }
  }, [getQueuedMessages]);

  // Manual reconnection trigger
  const reconnect = useCallback(async () => {
    await contextDisconnect();
    return handleReconnect();
  }, [contextDisconnect, handleReconnect]);

  return {
    isConnected: contextConnected,
    connectionStatus,
    connect: contextConnect,
    disconnect: contextDisconnect,
    sendMessage,
    subscribe,
    getQueuedMessages,
    clearQueue,
    reconnect
  };
}