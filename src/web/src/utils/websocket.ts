/**
 * @fileoverview WebSocket Manager for Agricultural Management System
 * Provides real-time communication capabilities with advanced connection management,
 * message handling, and event subscription features.
 * @version 1.0.0
 */

import { EventEmitter } from 'events'; // ^3.3.0
import { API_ENDPOINTS } from '../constants/apiEndpoints';

// WebSocket message types
type MessageType = 'telemetry' | 'mission' | 'device' | 'alert' | 'system';

interface WebSocketMessage {
    type: MessageType;
    payload: unknown;
    timestamp: number;
    id: string;
}

interface WebSocketConfig {
    autoReconnect?: boolean;
    reconnectInterval?: number;
    maxReconnectAttempts?: number;
    connectionTimeout?: number;
    protocols?: string[];
}

interface ConnectionResult {
    success: boolean;
    timestamp: number;
    connectionId: string;
    error?: Error;
}

interface SendResult {
    success: boolean;
    messageId: string;
    timestamp: number;
    error?: Error;
}

interface NetworkQuality {
    latency: number;
    stability: number;
    lastCheck: number;
}

interface Subscription {
    unsubscribe: () => void;
    id: string;
}

type EventCallback = (data: unknown) => void;

/**
 * WebSocket Manager class providing comprehensive real-time communication capabilities
 */
export class WebSocketManager {
    private ws: WebSocket | null = null;
    private readonly eventEmitter: EventEmitter;
    private reconnectAttempts: number = 0;
    private isConnecting: boolean = false;
    private readonly subscriptions: Map<string, Function[]>;
    private reconnectTimer?: NodeJS.Timeout;
    private connectionTimer?: NodeJS.Timeout;
    private lastReconnectDelay: number = RECONNECT_INTERVAL;
    private networkStatus: NetworkQuality = {
        latency: 0,
        stability: 1,
        lastCheck: Date.now()
    };
    private readonly config: Required<WebSocketConfig>;

    /**
     * Initialize WebSocket manager with configuration
     * @param config WebSocket configuration options
     */
    constructor(config: WebSocketConfig = {}) {
        this.eventEmitter = new EventEmitter();
        this.eventEmitter.setMaxListeners(50); // Prevent memory leaks
        this.subscriptions = new Map();

        this.config = {
            autoReconnect: true,
            reconnectInterval: RECONNECT_INTERVAL,
            maxReconnectAttempts: MAX_RECONNECT_ATTEMPTS,
            connectionTimeout: CONNECTION_TIMEOUT,
            protocols: ['v1.agricultural.protocol'],
            ...config
        };

        // Handle process cleanup
        if (typeof window !== 'undefined') {
            window.addEventListener('beforeunload', () => this.disconnect());
        }
    }

    /**
     * Establish WebSocket connection with reconnection logic
     * @returns Promise<ConnectionResult>
     */
    public async connect(): Promise<ConnectionResult> {
        if (this.ws?.readyState === WebSocket.OPEN) {
            return {
                success: true,
                timestamp: Date.now(),
                connectionId: this.generateConnectionId()
            };
        }

        if (this.isConnecting) {
            throw new Error('Connection attempt already in progress');
        }

        this.isConnecting = true;

        return new Promise((resolve, reject) => {
            try {
                const wsUrl = `ws://${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/ws`;
                this.ws = new WebSocket(wsUrl, this.config.protocols);

                // Set connection timeout
                this.connectionTimer = setTimeout(() => {
                    if (this.ws?.readyState !== WebSocket.OPEN) {
                        this.ws?.close();
                        reject(new Error('Connection timeout'));
                    }
                }, this.config.connectionTimeout);

                this.ws.onopen = () => {
                    clearTimeout(this.connectionTimer);
                    this.isConnecting = false;
                    this.reconnectAttempts = 0;
                    this.lastReconnectDelay = RECONNECT_INTERVAL;
                    
                    const result: ConnectionResult = {
                        success: true,
                        timestamp: Date.now(),
                        connectionId: this.generateConnectionId()
                    };
                    
                    this.eventEmitter.emit('connected', result);
                    resolve(result);
                };

                this.setupEventHandlers();

            } catch (error) {
                this.isConnecting = false;
                reject(error);
            }
        });
    }

    /**
     * Gracefully disconnect WebSocket
     */
    public async disconnect(): Promise<void> {
        clearTimeout(this.reconnectTimer);
        clearTimeout(this.connectionTimer);

        if (this.ws) {
            this.ws.onclose = null; // Prevent reconnection attempt
            this.ws.close(1000, 'Client disconnected');
            this.ws = null;
        }

        this.eventEmitter.emit('disconnected');
    }

    /**
     * Send message through WebSocket with retry logic
     * @param message WebSocket message to send
     */
    public async send(message: WebSocketMessage): Promise<SendResult> {
        if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
            throw new Error('WebSocket is not connected');
        }

        if (JSON.stringify(message).length > MAX_MESSAGE_SIZE) {
            throw new Error('Message size exceeds maximum allowed size');
        }

        return new Promise((resolve, reject) => {
            try {
                this.ws!.send(JSON.stringify(message));
                resolve({
                    success: true,
                    messageId: message.id,
                    timestamp: Date.now()
                });
            } catch (error) {
                reject({
                    success: false,
                    messageId: message.id,
                    timestamp: Date.now(),
                    error
                });
            }
        });
    }

    /**
     * Subscribe to WebSocket events with type safety
     * @param eventType Event type to subscribe to
     * @param callback Callback function for event
     */
    public subscribe(eventType: MessageType, callback: EventCallback): Subscription {
        const subscriptionId = this.generateSubscriptionId();
        
        if (!this.subscriptions.has(eventType)) {
            this.subscriptions.set(eventType, []);
        }
        
        this.subscriptions.get(eventType)!.push(callback);
        
        return {
            unsubscribe: () => {
                const callbacks = this.subscriptions.get(eventType);
                if (callbacks) {
                    this.subscriptions.set(
                        eventType,
                        callbacks.filter(cb => cb !== callback)
                    );
                }
            },
            id: subscriptionId
        };
    }

    /**
     * Get current connection status
     */
    public getConnectionStatus(): boolean {
        return this.ws?.readyState === WebSocket.OPEN;
    }

    /**
     * Get network quality metrics
     */
    public getNetworkQuality(): NetworkQuality {
        return { ...this.networkStatus };
    }

    private setupEventHandlers(): void {
        if (!this.ws) return;

        this.ws.onmessage = (event: MessageEvent) => {
            try {
                const message: WebSocketMessage = JSON.parse(event.data);
                const callbacks = this.subscriptions.get(message.type);
                callbacks?.forEach(callback => callback(message.payload));
                this.eventEmitter.emit('message', message);
            } catch (error) {
                this.eventEmitter.emit('error', error);
            }
        };

        this.ws.onclose = (event: CloseEvent) => {
            this.handleDisconnection(event);
        };

        this.ws.onerror = (error: Event) => {
            this.eventEmitter.emit('error', error);
        };
    }

    private handleDisconnection(event: CloseEvent): void {
        this.isConnecting = false;
        this.eventEmitter.emit('disconnected', event);

        if (this.config.autoReconnect && this.reconnectAttempts < this.config.maxReconnectAttempts) {
            this.reconnectTimer = setTimeout(() => {
                this.reconnectAttempts++;
                this.lastReconnectDelay *= BACKOFF_MULTIPLIER;
                this.connect().catch(error => {
                    this.eventEmitter.emit('error', error);
                });
            }, this.lastReconnectDelay);
        }
    }

    private generateConnectionId(): string {
        return `conn_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    }

    private generateSubscriptionId(): string {
        return `sub_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    }
}

// Constants
const RECONNECT_INTERVAL = 5000;
const MAX_RECONNECT_ATTEMPTS = 5;
const CONNECTION_TIMEOUT = 10000;
const BACKOFF_MULTIPLIER = 1.5;
const MAX_MESSAGE_SIZE = 1048576; // 1MB