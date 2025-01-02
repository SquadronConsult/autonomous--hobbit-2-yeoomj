import { Pool, PoolConfig } from 'pg'; // v8.11.0
import { ssl } from 'node:tls';
import { logger } from '../utils/logger';

// Global constants for pool configuration
const MAX_POOL_SIZE = 20;
const MIN_POOL_SIZE = 5;
const CONNECTION_TIMEOUT = 30000; // 30 seconds
const IDLE_TIMEOUT = 10000; // 10 seconds
const RETRY_ATTEMPTS = 3;
const RETRY_DELAY = 5000; // 5 seconds
const HEALTH_CHECK_INTERVAL = 60000; // 1 minute

// Interface for database configuration
interface DatabaseConfig extends PoolConfig {
  retryAttempts: number;
  healthCheckInterval: number;
  ssl?: {
    rejectUnauthorized: boolean;
    ca?: string;
    key?: string;
    cert?: string;
  };
}

// Retry decorator for connection attempts
function retry(attempts: number, delay: number) {
  return function (target: any, propertyKey: string, descriptor: PropertyDescriptor) {
    const originalMethod = descriptor.value;

    descriptor.value = async function (...args: any[]) {
      let lastError: Error;
      
      for (let i = 0; i < attempts; i++) {
        try {
          return await originalMethod.apply(this, args);
        } catch (error) {
          lastError = error as Error;
          logger.warn(`Database connection attempt ${i + 1}/${attempts} failed`, {
            correlationId: `db-retry-${Date.now()}`,
            component: 'database',
            operation: 'connect',
            details: { error: error.message }
          });
          
          if (i < attempts - 1) {
            await new Promise(resolve => setTimeout(resolve, delay));
          }
        }
      }
      
      throw lastError!;
    };
    
    return descriptor;
  };
}

// Validate database configuration
function validateConfig(config: DatabaseConfig): boolean {
  const requiredFields = ['host', 'port', 'database', 'user', 'password'];
  
  for (const field of requiredFields) {
    if (!config[field]) {
      logger.error(`Missing required database configuration field: ${field}`, {
        correlationId: `config-validation-${Date.now()}`,
        component: 'database',
        operation: 'validate'
      });
      return false;
    }
  }

  if (process.env.NODE_ENV === 'production' && !config.ssl) {
    logger.error('SSL configuration is required in production environment', {
      correlationId: `config-validation-${Date.now()}`,
      component: 'database',
      operation: 'validate'
    });
    return false;
  }

  return true;
}

// Monitor pool health
function monitorPoolHealth(pool: Pool): void {
  setInterval(() => {
    const metrics = {
      totalCount: pool.totalCount,
      idleCount: pool.idleCount,
      waitingCount: pool.waitingCount
    };

    logger.debug('Database pool health check', {
      correlationId: `pool-health-${Date.now()}`,
      component: 'database',
      operation: 'monitor',
      details: metrics
    });

    // Implement connection recycling if needed
    if (pool.idleCount > MAX_POOL_SIZE / 2) {
      pool.on('remove', client => {
        logger.info('Recycling idle connection', {
          correlationId: `connection-recycle-${Date.now()}`,
          component: 'database',
          operation: 'recycle'
        });
      });
    }
  }, HEALTH_CHECK_INTERVAL);
}

// Initialize database pool
@retry(RETRY_ATTEMPTS, RETRY_DELAY)
async function initializePool(config: DatabaseConfig): Promise<Pool> {
  if (!validateConfig(config)) {
    throw new Error('Invalid database configuration');
  }

  const poolConfig: PoolConfig = {
    host: config.host,
    port: config.port,
    database: config.database,
    user: config.user,
    password: config.password,
    max: MAX_POOL_SIZE,
    min: MIN_POOL_SIZE,
    idleTimeoutMillis: IDLE_TIMEOUT,
    connectionTimeoutMillis: CONNECTION_TIMEOUT,
    ssl: config.ssl
  };

  const pool = new Pool(poolConfig);

  // Set up error handling
  pool.on('error', (err, client) => {
    logger.error('Unexpected database pool error', {
      correlationId: `pool-error-${Date.now()}`,
      component: 'database',
      operation: 'pool-error',
      details: { error: err.message }
    });
  });

  // Set up connection handling
  pool.on('connect', client => {
    logger.info('New database connection established', {
      correlationId: `connection-${Date.now()}`,
      component: 'database',
      operation: 'connect'
    });
  });

  // Initialize health monitoring
  monitorPoolHealth(pool);

  return pool;
}

// Export database configuration
export const databaseConfig: DatabaseConfig = {
  host: process.env.DB_HOST || 'localhost',
  port: parseInt(process.env.DB_PORT || '5432'),
  database: process.env.DB_NAME || 'agricultural_system',
  user: process.env.DB_USER || 'postgres',
  password: process.env.DB_PASSWORD || '',
  retryAttempts: RETRY_ATTEMPTS,
  healthCheckInterval: HEALTH_CHECK_INTERVAL,
  ssl: process.env.NODE_ENV === 'production' ? {
    rejectUnauthorized: true,
    ca: process.env.DB_SSL_CA,
    key: process.env.DB_SSL_KEY,
    cert: process.env.DB_SSL_CERT
  } : undefined
};

// Initialize and export pool instance
export const pool = await initializePool(databaseConfig);