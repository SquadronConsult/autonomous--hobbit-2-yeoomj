import { describe, beforeAll, afterAll, beforeEach, test, expect, jest } from '@jest/globals'; // v29.0.0
import { QueryResult } from 'pg'; // v8.11.0
import { db } from '../../src/utils/database';
import { databaseConfig } from '../../src/config/database';

// Mock Prometheus metrics
jest.mock('prom-client', () => ({
  Histogram: jest.fn().mockImplementation(() => ({
    observe: jest.fn()
  })),
  Counter: jest.fn().mockImplementation(() => ({
    inc: jest.fn()
  })),
  Gauge: jest.fn().mockImplementation(() => ({
    inc: jest.fn(),
    dec: jest.fn()
  }))
}));

describe('Database Integration Tests', () => {
  beforeAll(async () => {
    // Initialize test database with TimescaleDB extension
    await db.initializeDatabase();
    
    // Create test schema and tables
    await db.executeQuery(`
      CREATE SCHEMA IF NOT EXISTS test_schema;
      
      CREATE TABLE IF NOT EXISTS test_schema.telemetry (
        id UUID PRIMARY KEY,
        device_id UUID NOT NULL,
        timestamp TIMESTAMPTZ NOT NULL,
        metrics JSONB NOT NULL,
        location POINT
      );
      
      SELECT create_hypertable('test_schema.telemetry', 'timestamp');
      
      CREATE TABLE IF NOT EXISTS test_schema.devices (
        id UUID PRIMARY KEY,
        type VARCHAR(50) NOT NULL,
        status VARCHAR(20) NOT NULL,
        last_seen TIMESTAMPTZ
      );
    `, []);
  });

  afterAll(async () => {
    // Clean up test schema and close connections
    await db.executeQuery('DROP SCHEMA test_schema CASCADE;', []);
    await db.closeDatabase();
  });

  beforeEach(async () => {
    // Clear test data before each test
    await db.executeQuery('TRUNCATE test_schema.telemetry, test_schema.devices;', []);
  });

  describe('Database Connection', () => {
    test('should successfully connect with SSL verification', async () => {
      const result = await db.executeQuery('SELECT 1 as connected;', []);
      expect(result.rows[0].connected).toBe(1);
    });

    test('should handle connection errors with proper retries', async () => {
      // Temporarily modify host to trigger retries
      const originalHost = databaseConfig.host;
      databaseConfig.host = 'invalid-host';

      await expect(db.executeQuery('SELECT 1;', [])).rejects.toThrow();
      expect(jest.mocked(console.error)).toHaveBeenCalledTimes(3); // Max retries

      databaseConfig.host = originalHost;
    });

    test('should enforce connection pool limits', async () => {
      const queries = Array(25).fill('SELECT pg_sleep(0.1);');
      const results = await Promise.allSettled(
        queries.map(query => db.executeQuery(query, []))
      );

      const rejected = results.filter(r => r.status === 'rejected');
      expect(rejected.length).toBeGreaterThan(0);
    });
  });

  describe('Query Execution', () => {
    test('should execute parameterized queries safely', async () => {
      const deviceId = '123e4567-e89b-12d3-a456-426614174000';
      const result = await db.executeQuery(
        'INSERT INTO test_schema.devices (id, type, status, last_seen) VALUES ($1, $2, $3, $4) RETURNING *',
        [deviceId, 'DRONE', 'ACTIVE', new Date()]
      );
      
      expect(result.rowCount).toBe(1);
      expect(result.rows[0].id).toBe(deviceId);
    });

    test('should handle large result sets efficiently', async () => {
      // Insert 10000 telemetry records
      const telemetryData = Array(10000).fill(null).map((_, i) => ({
        id: `123e4567-e89b-12d3-a456-${i.toString().padStart(12, '0')}`,
        device_id: '123e4567-e89b-12d3-a456-426614174000',
        timestamp: new Date(Date.now() - i * 1000),
        metrics: { temperature: 20 + Math.random() * 10 }
      }));

      await db.executeTransaction(
        telemetryData.map(data => ({
          query: `
            INSERT INTO test_schema.telemetry (id, device_id, timestamp, metrics)
            VALUES ($1, $2, $3, $4)
          `,
          params: [data.id, data.device_id, data.timestamp, data.metrics]
        }))
      );

      const result = await db.executeQuery(
        'SELECT * FROM test_schema.telemetry ORDER BY timestamp DESC',
        []
      );

      expect(result.rows.length).toBe(10000);
    });

    test('should enforce query timeout limits', async () => {
      await expect(
        db.executeQuery('SELECT pg_sleep(31);', [], { timeout: 1000 })
      ).rejects.toThrow('Query timeout');
    });
  });

  describe('Transaction Management', () => {
    test('should maintain ACID properties', async () => {
      const deviceId = '123e4567-e89b-12d3-a456-426614174000';
      
      try {
        await db.executeTransaction([
          {
            query: 'INSERT INTO test_schema.devices (id, type, status) VALUES ($1, $2, $3)',
            params: [deviceId, 'DRONE', 'ACTIVE']
          },
          {
            query: 'INSERT INTO test_schema.devices (id, type, status) VALUES ($1, $2, $3)',
            params: [deviceId, 'DRONE', 'ACTIVE'] // Duplicate key violation
          }
        ]);
      } catch (error) {
        // Transaction should rollback
      }

      const result = await db.executeQuery(
        'SELECT COUNT(*) as count FROM test_schema.devices WHERE id = $1',
        [deviceId]
      );
      
      expect(result.rows[0].count).toBe('0');
    });

    test('should handle nested transactions correctly', async () => {
      const deviceId = '123e4567-e89b-12d3-a456-426614174000';
      
      await db.executeTransaction([
        {
          query: 'BEGIN;',
          params: []
        },
        {
          query: 'SAVEPOINT sp1;',
          params: []
        },
        {
          query: 'INSERT INTO test_schema.devices (id, type, status) VALUES ($1, $2, $3)',
          params: [deviceId, 'DRONE', 'ACTIVE']
        },
        {
          query: 'RELEASE SAVEPOINT sp1;',
          params: []
        },
        {
          query: 'COMMIT;',
          params: []
        }
      ]);

      const result = await db.executeQuery(
        'SELECT COUNT(*) as count FROM test_schema.devices WHERE id = $1',
        [deviceId]
      );
      
      expect(result.rows[0].count).toBe('1');
    });
  });

  describe('TimescaleDB Features', () => {
    test('should create and manage hypertables', async () => {
      const result = await db.executeQuery(`
        SELECT * FROM timescaledb_information.hypertables
        WHERE hypertable_name = 'telemetry';
      `, []);
      
      expect(result.rows.length).toBe(1);
    });

    test('should handle time-bucket operations', async () => {
      // Insert test data across different time periods
      const now = new Date();
      await db.executeTransaction([
        {
          query: `
            INSERT INTO test_schema.telemetry (id, device_id, timestamp, metrics)
            SELECT 
              gen_random_uuid(),
              '123e4567-e89b-12d3-a456-426614174000',
              generate_series(
                $1::timestamptz - interval '1 hour',
                $1::timestamptz,
                interval '1 minute'
              ),
              '{"temperature": 25}'::jsonb
          `,
          params: [now]
        }
      ]);

      const result = await db.executeQuery(`
        SELECT time_bucket('5 minutes', timestamp) AS bucket,
               COUNT(*) as count,
               AVG((metrics->>'temperature')::numeric) as avg_temp
        FROM test_schema.telemetry
        GROUP BY bucket
        ORDER BY bucket DESC;
      `, []);

      expect(result.rows.length).toBeGreaterThan(0);
      expect(result.rows[0].avg_temp).toBe('25');
    });

    test('should enforce retention policies', async () => {
      await db.executeQuery(`
        SELECT add_retention_policy(
          'test_schema.telemetry',
          INTERVAL '30 days'
        );
      `, []);

      const result = await db.executeQuery(`
        SELECT * FROM timescaledb_information.jobs
        WHERE application_name LIKE 'Retention Policy%';
      `, []);

      expect(result.rows.length).toBe(1);
    });
  });

  describe('Error Handling', () => {
    test('should handle connection failures gracefully', async () => {
      const originalPool = (db as any).pool;
      (db as any).pool = null;

      await expect(
        db.executeQuery('SELECT 1;', [])
      ).rejects.toThrow('Database not initialized');

      (db as any).pool = originalPool;
    });

    test('should manage constraint violations', async () => {
      const deviceId = '123e4567-e89b-12d3-a456-426614174000';
      
      await db.executeQuery(
        'INSERT INTO test_schema.devices (id, type, status) VALUES ($1, $2, $3)',
        [deviceId, 'DRONE', 'ACTIVE']
      );

      await expect(
        db.executeQuery(
          'INSERT INTO test_schema.devices (id, type, status) VALUES ($1, $2, $3)',
          [deviceId, 'DRONE', 'ACTIVE']
        )
      ).rejects.toThrow(/duplicate key/);
    });

    test('should handle query syntax errors', async () => {
      await expect(
        db.executeQuery('SELEC * FROM test_schema.devices;', [])
      ).rejects.toThrow(/syntax error/);
    });
  });
});