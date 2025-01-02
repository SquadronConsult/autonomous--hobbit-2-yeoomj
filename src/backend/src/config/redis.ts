import { Redis } from 'ioredis'; // v5.3.2
import { LogLevel } from './logging';

// Redis Node interface for cluster configuration
interface RedisNode {
  host: string;
  port: number;
}

// Main Redis configuration interface
interface RedisConfig {
  connection: {
    host: string;
    port: number;
    password: string | null;
    db: number;
    keyPrefix: string;
    tls: boolean;
    connectTimeout: number;
    commandTimeout: number;
    keepAlive: number;
  };
  options: {
    retryStrategy: (retries: number) => number | null;
    maxRetriesPerRequest: number;
    enableReadyCheck: boolean;
    autoResubscribe: boolean;
    autoResendUnfulfilledCommands: boolean;
    lazyConnect: boolean;
  };
  cache: {
    ttl: number;
    maxKeys: number;
    evictionPolicy: string;
    compressionEnabled: boolean;
  };
  cluster: {
    enabled: boolean;
    nodes: RedisNode[];
    options: {
      clusterRetryStrategy: (retries: number) => number | null;
      enableOfflineQueue: boolean;
      enableReadyCheck: boolean;
      scaleReads: string;
      maxRedirections: number;
    };
  };
}

// Environment variables with defaults
const REDIS_HOST = process.env.REDIS_HOST || 'localhost';
const REDIS_PORT = parseInt(process.env.REDIS_PORT || '6379');
const REDIS_PASSWORD = process.env.REDIS_PASSWORD;
const REDIS_DB = parseInt(process.env.REDIS_DB || '0');
const REDIS_KEY_PREFIX = process.env.REDIS_KEY_PREFIX || 'agri:';
const REDIS_CLUSTER_ENABLED = process.env.REDIS_CLUSTER_ENABLED === 'true';

// Constants for Redis configuration
const DEFAULT_TTL = 900; // 15 minutes in seconds
const MAX_RETRY_ATTEMPTS = 10;
const MAX_RETRY_DELAY = 5000; // 5 seconds
const MAX_CACHE_KEYS = 10000;
const CONNECT_TIMEOUT = 10000; // 10 seconds
const COMMAND_TIMEOUT = 5000; // 5 seconds

/**
 * Creates retry strategy for Redis connection failures with exponential backoff
 * @param retries - Number of retry attempts made
 * @returns Delay in milliseconds before next retry or null to stop retrying
 */
const createRetryStrategy = (retries: number): number | null => {
  if (retries > MAX_RETRY_ATTEMPTS) {
    console.log(LogLevel.ERROR, `Redis connection failed after ${MAX_RETRY_ATTEMPTS} attempts`);
    return null; // Stop retrying
  }

  // Exponential backoff with jitter
  const delay = Math.min(
    Math.floor(Math.random() * Math.pow(2, retries) * 1000),
    MAX_RETRY_DELAY
  );

  console.log(LogLevel.ERROR, `Retrying Redis connection in ${delay}ms (attempt ${retries})`);
  return delay;
};

/**
 * Creates Redis configuration based on environment settings and system requirements
 * @returns Complete Redis configuration object
 */
const createRedisConfig = (): RedisConfig => {
  const config: RedisConfig = {
    connection: {
      host: REDIS_HOST,
      port: REDIS_PORT,
      password: REDIS_PASSWORD || null,
      db: REDIS_DB,
      keyPrefix: REDIS_KEY_PREFIX,
      tls: process.env.NODE_ENV === 'production',
      connectTimeout: CONNECT_TIMEOUT,
      commandTimeout: COMMAND_TIMEOUT,
      keepAlive: 10000,
    },
    options: {
      retryStrategy: createRetryStrategy,
      maxRetriesPerRequest: 3,
      enableReadyCheck: true,
      autoResubscribe: true,
      autoResendUnfulfilledCommands: true,
      lazyConnect: false,
    },
    cache: {
      ttl: DEFAULT_TTL,
      maxKeys: MAX_CACHE_KEYS,
      evictionPolicy: 'volatile-lru',
      compressionEnabled: true,
    },
    cluster: {
      enabled: REDIS_CLUSTER_ENABLED,
      nodes: [
        { host: REDIS_HOST, port: REDIS_PORT },
        // Additional nodes can be configured through environment variables
      ],
      options: {
        clusterRetryStrategy: createRetryStrategy,
        enableOfflineQueue: true,
        enableReadyCheck: true,
        scaleReads: 'slave',
        maxRedirections: 16,
      },
    },
  };

  return config;
};

// Export the Redis configuration
export const redisConfig = createRedisConfig();

// Export individual configuration sections for granular access
export const {
  connection,
  options,
  cache,
  cluster
} = redisConfig;

export default redisConfig;