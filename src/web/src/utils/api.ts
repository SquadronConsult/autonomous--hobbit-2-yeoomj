/**
 * @fileoverview Enhanced API client utility with real-time capabilities and security features
 * Implements standardized HTTP client functionality with comprehensive error handling
 * @version 1.0.0
 */

import axios, { AxiosInstance, AxiosError, AxiosRequestConfig } from 'axios'; // v1.4.0
import { EventSourceParser, ParseEvent } from 'eventsource-parser'; // v1.0.0
import { BASE_URL, API_VERSION, STREAM_ENDPOINTS } from '../constants/apiEndpoints';
import { HttpStatusCodes } from '../constants/statusCodes';
import { useAuth } from '../contexts/AuthContext';

// Configuration constants
const REQUEST_TIMEOUT_MS = 30000;
const MAX_RETRIES = 3;
const RATE_LIMIT_WINDOW_MS = 60000;
const MAX_REQUESTS_PER_WINDOW = 100;
const CACHE_TTL_MS = 300000;

/**
 * Cache entry structure for request deduplication
 */
interface CacheEntry<T = any> {
    data: T;
    timestamp: number;
    expiresAt: number;
}

/**
 * API client configuration options
 */
interface ClientOptions {
    baseURL?: string;
    timeout?: number;
    enableCache?: boolean;
    maxRetries?: number;
    rateLimit?: number;
}

/**
 * Subscription options for real-time data streams
 */
interface SubscriptionOptions {
    onMessage: (data: any) => void;
    onError?: (error: Error) => void;
    onReconnect?: () => void;
    retryAttempts?: number;
    retryDelay?: number;
}

/**
 * Enhanced error type with additional context
 */
interface ApiError extends Error {
    code: string;
    status?: number;
    timestamp: number;
    context?: Record<string, any>;
}

/**
 * Enhanced API client with real-time capabilities and security features
 */
export class ApiClient {
    private client: AxiosInstance;
    private requestCache: Map<string, CacheEntry>;
    private rateLimitCounter: number;
    private rateLimitResetTime: number;

    constructor(options: ClientOptions = {}) {
        this.client = axios.create({
            baseURL: `${options.baseURL || BASE_URL}/${API_VERSION}`,
            timeout: options.timeout || REQUEST_TIMEOUT_MS,
            headers: {
                'Content-Type': 'application/json',
                'X-Client-Version': '1.0.0',
            },
        });

        this.requestCache = new Map();
        this.rateLimitCounter = 0;
        this.rateLimitResetTime = Date.now() + RATE_LIMIT_WINDOW_MS;

        this.setupInterceptors();
    }

    /**
     * Configures request and response interceptors with enhanced security
     */
    private setupInterceptors(): void {
        // Request interceptor
        this.client.interceptors.request.use(
            async (config) => {
                if (this.isRateLimited()) {
                    throw new Error('Rate limit exceeded');
                }

                const { getAccessToken, getSessionFingerprint } = useAuth();
                const token = await getAccessToken();
                const fingerprint = getSessionFingerprint();

                config.headers.Authorization = `Bearer ${token}`;
                config.headers['X-Session-Fingerprint'] = fingerprint;
                config.headers['X-Request-ID'] = crypto.randomUUID();

                this.incrementRateLimit();
                return config;
            },
            (error) => Promise.reject(error)
        );

        // Response interceptor
        this.client.interceptors.response.use(
            (response) => {
                const cacheKey = this.getCacheKey(response.config);
                if (response.config.method === 'GET' && response.status === 200) {
                    this.cacheResponse(cacheKey, response.data);
                }
                return response;
            },
            (error) => this.handleApiError(error)
        );
    }

    /**
     * Makes a type-safe API request with enhanced error handling
     */
    public async request<T = any>(config: AxiosRequestConfig): Promise<T> {
        const cacheKey = this.getCacheKey(config);
        const cachedResponse = this.getCachedResponse<T>(cacheKey);
        
        if (cachedResponse) {
            return cachedResponse;
        }

        try {
            const response = await this.client.request<T>(config);
            return response.data;
        } catch (error) {
            throw await this.handleApiError(error as AxiosError);
        }
    }

    /**
     * Creates a real-time data subscription with automatic reconnection
     */
    public subscribe(endpoint: string, options: SubscriptionOptions): () => void {
        let eventSource: EventSource | null = null;
        let retryCount = 0;
        const maxRetries = options.retryAttempts || 3;
        const retryDelay = options.retryDelay || 5000;

        const connect = async () => {
            try {
                const { getAccessToken } = useAuth();
                const token = await getAccessToken();
                
                const url = new URL(`${this.client.defaults.baseURL}${endpoint}`);
                url.searchParams.append('access_token', token);

                eventSource = new EventSource(url.toString());
                
                eventSource.onmessage = (event) => {
                    try {
                        const data = JSON.parse(event.data);
                        options.onMessage(data);
                    } catch (error) {
                        options.onError?.(error as Error);
                    }
                };

                eventSource.onerror = async () => {
                    eventSource?.close();
                    if (retryCount < maxRetries) {
                        retryCount++;
                        options.onReconnect?.();
                        setTimeout(connect, retryDelay);
                    } else {
                        options.onError?.(new Error('Max retry attempts reached'));
                    }
                };
            } catch (error) {
                options.onError?.(error as Error);
            }
        };

        connect();

        return () => {
            eventSource?.close();
        };
    }

    /**
     * Handles API errors with enhanced context and monitoring
     */
    private async handleApiError(error: AxiosError): Promise<never> {
        const apiError: ApiError = {
            name: 'ApiError',
            message: error.message,
            code: error.code || 'UNKNOWN_ERROR',
            status: error.response?.status,
            timestamp: Date.now(),
            context: {
                url: error.config?.url,
                method: error.config?.method,
                requestId: error.config?.headers['X-Request-ID'],
            },
        };

        if (error.response?.status === HttpStatusCodes.UNAUTHORIZED) {
            // Force reauthentication
            const { logout } = useAuth();
            await logout();
        }

        // Log error for monitoring
        console.error('[API Error]', {
            ...apiError,
            stack: error.stack,
        });

        throw apiError;
    }

    /**
     * Checks if current request would exceed rate limit
     */
    private isRateLimited(): boolean {
        if (Date.now() > this.rateLimitResetTime) {
            this.rateLimitCounter = 0;
            this.rateLimitResetTime = Date.now() + RATE_LIMIT_WINDOW_MS;
            return false;
        }
        return this.rateLimitCounter >= MAX_REQUESTS_PER_WINDOW;
    }

    /**
     * Increments the rate limit counter
     */
    private incrementRateLimit(): void {
        this.rateLimitCounter++;
    }

    /**
     * Generates a cache key for a request
     */
    private getCacheKey(config: AxiosRequestConfig): string {
        return `${config.method}-${config.url}-${JSON.stringify(config.params)}`;
    }

    /**
     * Retrieves a cached response if valid
     */
    private getCachedResponse<T>(key: string): T | null {
        const cached = this.requestCache.get(key);
        if (cached && Date.now() < cached.expiresAt) {
            return cached.data as T;
        }
        this.requestCache.delete(key);
        return null;
    }

    /**
     * Caches a response with expiration
     */
    private cacheResponse(key: string, data: any): void {
        this.requestCache.set(key, {
            data,
            timestamp: Date.now(),
            expiresAt: Date.now() + CACHE_TTL_MS,
        });
    }
}

/**
 * Creates a configured API client instance
 */
export const createApiClient = (options?: ClientOptions): ApiClient => {
    return new ApiClient(options);
};

/**
 * Standalone error handler for API errors
 */
export const handleApiError = async (error: AxiosError): Promise<never> => {
    const client = new ApiClient();
    return client.handleApiError(error);
};