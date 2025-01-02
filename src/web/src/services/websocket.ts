/**
 * @fileoverview Enhanced WebSocket service implementation for real-time agricultural system communication
 * @version 1.0.0
 * Handles real-time telemetry, mission updates, and system metrics with reliability and monitoring
 */

import ReconnectingWebSocket from 'reconnecting-websocket'; // v4.4.0
import { API_ENDPOINTS } from '../constants/apiEndpoints';

// Message compression threshold in bytes
const COMPRESSION_THRESHOLD = 1024;
// Maximum message queue size
const MAX_QUEUE_SIZE = 1000;
// Heartbeat interval in milliseconds
const HEARTBEAT_INTERVAL = 30000;
// Acknowledgment timeout in milliseconds
const ACK_TIMEOUT = 5000;

/**
 * Type-safe interface for WebSocket messages
 */
export interface WebSocketMessage {
    type: string;
    payload: any;
    timestamp: string;
    messageId: string;
    requiresAck: boolean;
    compressed?: boolean;
}

/**
 * Interface for WebSocket connection state tracking
 */
interface ConnectionState {
    isConnected: boolean;
    reconnectAttempts: number;
    lastHeartbeat: Date;
    latency: number;
}

/**
 * Interface for subscription options
 */
interface SubscriptionOptions {
    filter?: (message: WebSocketMessage) => boolean;
    maxRetries?: number;
    batchSize?: number;
    priority?: 'high' | 'normal' | 'low';
}

/**
 * Interface for WebSocket configuration
 */
interface WebSocketConfig {
    autoReconnect?: boolean;
    maxReconnectAttempts?: number;
    reconnectInterval?: number;
    protocols?: string[];
    headers?: Record<string, string>;
}

/**
 * Enhanced WebSocket manager with advanced features
 */
export class WebSocketManager {
    private ws: ReconnectingWebSocket;
    private subscribers: Map<string, Set<Function>>;
    private connectionState: ConnectionState;
    private messageQueue: WebSocketMessage[];
    private pendingAcks: Map<string, { message: WebSocketMessage; timeout: NodeJS.Timeout }>;
    private metrics: {
        messagesSent: number;
        messagesReceived: number;
        errors: number;
        averageLatency: number;
    };

    /**
     * Initializes the WebSocket manager with enhanced configuration
     */
    constructor(private config: WebSocketConfig = {}) {
        const wsUrl = `${window.location.protocol === 'https:' ? 'wss:' : 'ws:'}//${window.location.host}${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/ws`;
        
        this.ws = new ReconnectingWebSocket(wsUrl, config.protocols, {
            maxRetries: config.maxReconnectAttempts || 10,
            reconnectionDelayGrowFactor: 1.3,
            maxReconnectionDelay: 10000,
            minReconnectionDelay: 1000,
            connectionTimeout: 4000,
        });

        this.subscribers = new Map();
        this.connectionState = {
            isConnected: false,
            reconnectAttempts: 0,
            lastHeartbeat: new Date(),
            latency: 0,
        };
        this.messageQueue = [];
        this.pendingAcks = new Map();
        this.metrics = {
            messagesSent: 0,
            messagesReceived: 0,
            errors: 0,
            averageLatency: 0,
        };

        this.initializeEventListeners();
    }

    /**
     * Establishes WebSocket connection with enhanced features
     */
    public async connect(): Promise<void> {
        return new Promise((resolve, reject) => {
            const connectionTimeout = setTimeout(() => {
                reject(new Error('Connection timeout'));
            }, 5000);

            this.ws.addEventListener('open', () => {
                clearTimeout(connectionTimeout);
                this.connectionState.isConnected = true;
                this.startHeartbeat();
                this.processQueuedMessages();
                resolve();
            }, { once: true });

            this.ws.addEventListener('error', (error) => {
                clearTimeout(connectionTimeout);
                reject(error);
            }, { once: true });
        });
    }

    /**
     * Sends message with reliability guarantees
     */
    public async send(message: WebSocketMessage): Promise<boolean> {
        try {
            if (!this.connectionState.isConnected) {
                if (this.messageQueue.length >= MAX_QUEUE_SIZE) {
                    throw new Error('Message queue full');
                }
                this.messageQueue.push(message);
                return false;
            }

            const enhancedMessage = {
                ...message,
                messageId: crypto.randomUUID(),
                timestamp: new Date().toISOString(),
            };

            let messageData = JSON.stringify(enhancedMessage);
            
            if (messageData.length > COMPRESSION_THRESHOLD) {
                messageData = await this.compressMessage(messageData);
                enhancedMessage.compressed = true;
            }

            if (message.requiresAck) {
                this.setupMessageAcknowledgment(enhancedMessage);
            }

            this.ws.send(messageData);
            this.metrics.messagesSent++;
            return true;
        } catch (error) {
            this.metrics.errors++;
            throw error;
        }
    }

    /**
     * Enhanced subscription management with filtering
     */
    public subscribe(event: string, callback: Function, options: SubscriptionOptions = {}): () => void {
        if (!this.subscribers.has(event)) {
            this.subscribers.set(event, new Set());
        }

        const enhancedCallback = (message: WebSocketMessage) => {
            if (options.filter && !options.filter(message)) {
                return;
            }
            callback(message);
        };

        this.subscribers.get(event)!.add(enhancedCallback);

        return () => {
            this.subscribers.get(event)?.delete(enhancedCallback);
            if (this.subscribers.get(event)?.size === 0) {
                this.subscribers.delete(event);
            }
        };
    }

    /**
     * Retrieves current connection metrics
     */
    public getMetrics() {
        return {
            ...this.metrics,
            connectionState: { ...this.connectionState },
            queueSize: this.messageQueue.length,
            pendingAcks: this.pendingAcks.size,
        };
    }

    /**
     * Gracefully disconnects the WebSocket connection
     */
    public disconnect(): void {
        this.ws.close();
        this.connectionState.isConnected = false;
        this.clearPendingAcks();
    }

    private initializeEventListeners(): void {
        this.ws.addEventListener('message', this.handleMessage.bind(this));
        this.ws.addEventListener('close', () => {
            this.connectionState.isConnected = false;
            this.connectionState.reconnectAttempts++;
        });
        this.ws.addEventListener('error', (error) => {
            this.metrics.errors++;
            console.error('WebSocket error:', error);
        });
    }

    private async handleMessage(event: MessageEvent): Promise<void> {
        try {
            let data = event.data;
            if (typeof data === 'string') {
                const message: WebSocketMessage = JSON.parse(data);
                
                if (message.compressed) {
                    data = await this.decompressMessage(data);
                }

                if (message.type === 'ack') {
                    this.handleAcknowledgment(message);
                    return;
                }

                this.metrics.messagesReceived++;
                this.subscribers.get(message.type)?.forEach(callback => callback(message));
            }
        } catch (error) {
            this.metrics.errors++;
            console.error('Message handling error:', error);
        }
    }

    private async compressMessage(message: string): Promise<string> {
        // Implement message compression logic here
        // This is a placeholder for actual compression implementation
        return message;
    }

    private async decompressMessage(message: string): Promise<string> {
        // Implement message decompression logic here
        // This is a placeholder for actual decompression implementation
        return message;
    }

    private setupMessageAcknowledgment(message: WebSocketMessage): void {
        const timeout = setTimeout(() => {
            this.handleAckTimeout(message);
        }, ACK_TIMEOUT);

        this.pendingAcks.set(message.messageId, { message, timeout });
    }

    private handleAcknowledgment(ack: WebSocketMessage): void {
        const pending = this.pendingAcks.get(ack.messageId);
        if (pending) {
            clearTimeout(pending.timeout);
            this.pendingAcks.delete(ack.messageId);
        }
    }

    private handleAckTimeout(message: WebSocketMessage): void {
        this.pendingAcks.delete(message.messageId);
        this.messageQueue.push(message);
        this.metrics.errors++;
    }

    private startHeartbeat(): void {
        setInterval(() => {
            if (this.connectionState.isConnected) {
                const start = Date.now();
                this.send({
                    type: 'heartbeat',
                    payload: { timestamp: start },
                    timestamp: new Date().toISOString(),
                    messageId: crypto.randomUUID(),
                    requiresAck: false,
                }).then(() => {
                    this.connectionState.latency = Date.now() - start;
                    this.connectionState.lastHeartbeat = new Date();
                });
            }
        }, HEARTBEAT_INTERVAL);
    }

    private async processQueuedMessages(): Promise<void> {
        while (this.messageQueue.length > 0 && this.connectionState.isConnected) {
            const message = this.messageQueue.shift();
            if (message) {
                try {
                    await this.send(message);
                } catch (error) {
                    this.messageQueue.unshift(message);
                    break;
                }
            }
        }
    }

    private clearPendingAcks(): void {
        this.pendingAcks.forEach(({ timeout }) => clearTimeout(timeout));
        this.pendingAcks.clear();
    }
}