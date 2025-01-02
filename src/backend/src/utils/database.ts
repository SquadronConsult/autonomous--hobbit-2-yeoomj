import { Pool, QueryResult } from 'pg'; // v8.11.0
import * as promClient from 'prom-client'; // v14.2.0
import { databaseConfig } from '../config/database';
import { logger } from './logger';

// Global connection pool
let pool: Pool | null = null;

// Constants for query execution
const QUERY_TIMEOUT = 30000; // 30 seconds
const MAX_RETRIES = 3;
const RETRY_DELAY = 1000; // 1 second

// Prometheus metrics
const queryLatencyHistogram = new promClient.Histogram({
  name: 'database_query_latency_seconds',
  help: 'Database query latency in seconds',
  labelNames: ['query_type', 'status'],
  buckets: [0.1, 0.5, 1, 2, 5]
});

const connectionGauge = new promClient.Gauge({
  name: 'database_connections_active',
  help: 'Number of active database connections'
});

const queryErrorCounter = new promClient.Counter({
  name: 'database_query_errors_total',
  help: 'Total number of database query errors',
  labelNames: ['error_type']
});

// SQL injection prevention regex
const SQL_INJECTION_PATTERN = /('|--|;|\/\*|\*\/|xp_|sp_|exec|execute|insert|select|delete|update|drop|union|into|load_file|outfile)/i;

/**
 * Initializes the database connection pool and verifies TimescaleDB setup
 */
async function initializeDatabase(): Promise<void> {
  try {
    pool = new Pool({
      host: databaseConfig.host,
      port: databaseConfig.port,
      database: databaseConfig.database,
      max: databaseConfig.max,
      min: databaseConfig.min,
      idleTimeoutMillis: databaseConfig.idleTimeoutMillis,
      ssl: databaseConfig.ssl
    });

    // Set up event handlers
    pool.on('connect', () => {
      connectionGauge.inc();
      logger.info('New database connection established', {
        correlationId: `db-connect-${Date.now()}`,
        component: 'database',
        operation: 'connect'
      });
    });

    pool.on('remove', () => {
      connectionGauge.dec();
      logger.info('Database connection removed from pool', {
        correlationId: `db-remove-${Date.now()}`,
        component: 'database',
        operation: 'remove'
      });
    });

    pool.on('error', (err: Error) => {
      queryErrorCounter.inc({ error_type: err.name });
      logger.error(err, {
        correlationId: `db-error-${Date.now()}`,
        component: 'database',
        operation: 'pool-error'
      });
    });

    // Verify TimescaleDB extension
    await executeQuery('CREATE EXTENSION IF NOT EXISTS timescaledb CASCADE;', []);
    await executeQuery('SELECT extversion FROM pg_extension WHERE extname = $1;', ['timescaledb']);

    logger.info('Database initialization completed successfully', {
      correlationId: `db-init-${Date.now()}`,
      component: 'database',
      operation: 'initialize'
    });
  } catch (error) {
    logger.error(error as Error, {
      correlationId: `db-init-error-${Date.now()}`,
      component: 'database',
      operation: 'initialize'
    });
    throw error;
  }
}

/**
 * Executes a database query with retries and monitoring
 */
async function executeQuery<T = any>(
  query: string,
  params: any[],
  options: { 
    timeout?: number;
    retries?: number;
    label?: string;
  } = {}
): Promise<QueryResult<T>> {
  if (!pool) {
    throw new Error('Database not initialized');
  }

  // Validate query for SQL injection
  if (SQL_INJECTION_PATTERN.test(query)) {
    const error = new Error('Potential SQL injection detected');
    queryErrorCounter.inc({ error_type: 'sql_injection' });
    throw error;
  }

  const startTime = process.hrtime();
  const queryType = options.label || query.split(' ')[0].toLowerCase();
  let attempts = 0;
  
  while (attempts < (options.retries || MAX_RETRIES)) {
    try {
      const client = await pool.connect();
      try {
        const result = await Promise.race([
          client.query(query, params),
          new Promise((_, reject) => 
            setTimeout(() => reject(new Error('Query timeout')), 
            options.timeout || QUERY_TIMEOUT)
          )
        ]) as QueryResult<T>;

        const [seconds, nanoseconds] = process.hrtime(startTime);
        const duration = seconds + nanoseconds / 1e9;
        queryLatencyHistogram.observe({ query_type: queryType, status: 'success' }, duration);

        return result;
      } finally {
        client.release();
      }
    } catch (error) {
      attempts++;
      queryErrorCounter.inc({ error_type: (error as Error).name });
      
      if (attempts === (options.retries || MAX_RETRIES)) {
        logger.error(error as Error, {
          correlationId: `query-error-${Date.now()}`,
          component: 'database',
          operation: 'query',
          details: { query, params, attempts }
        });
        throw error;
      }

      await new Promise(resolve => setTimeout(resolve, RETRY_DELAY * attempts));
    }
  }

  throw new Error('Query failed after maximum retries');
}

/**
 * Executes multiple queries within a transaction
 */
async function executeTransaction<T = any>(
  queries: Array<{ query: string; params: any[] }>
): Promise<QueryResult<T>[]> {
  if (!pool) {
    throw new Error('Database not initialized');
  }

  const client = await pool.connect();
  const results: QueryResult<T>[] = [];
  const startTime = process.hrtime();

  try {
    await client.query('BEGIN');

    for (const { query, params } of queries) {
      if (SQL_INJECTION_PATTERN.test(query)) {
        throw new Error('Potential SQL injection detected');
      }
      const result = await client.query(query, params);
      results.push(result);
    }

    await client.query('COMMIT');

    const [seconds, nanoseconds] = process.hrtime(startTime);
    const duration = seconds + nanoseconds / 1e9;
    queryLatencyHistogram.observe({ query_type: 'transaction', status: 'success' }, duration);

    return results;
  } catch (error) {
    await client.query('ROLLBACK');
    queryErrorCounter.inc({ error_type: 'transaction_error' });
    logger.error(error as Error, {
      correlationId: `transaction-error-${Date.now()}`,
      component: 'database',
      operation: 'transaction'
    });
    throw error;
  } finally {
    client.release();
  }
}

/**
 * Gracefully closes database connections
 */
async function closeDatabase(): Promise<void> {
  if (pool) {
    await pool.end();
    pool = null;
    logger.info('Database connections closed', {
      correlationId: `db-close-${Date.now()}`,
      component: 'database',
      operation: 'close'
    });
  }
}

// Export database interface
export const db = {
  initializeDatabase,
  executeQuery,
  executeTransaction,
  closeDatabase
};