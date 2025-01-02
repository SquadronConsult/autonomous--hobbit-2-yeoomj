/**
 * @fileoverview Implements the Telemetry model for managing real-time telemetry data
 * from agricultural robots and drones using TimescaleDB with time-series optimizations
 * @version 1.0.0
 * @license MIT
 */

import { v4 as uuidv4 } from 'uuid'; // v9.0.0
import { ITelemetry } from '../interfaces/ITelemetry';
import { TelemetryType } from '../constants/telemetryTypes';
import { pool } from '../config/database';
import { logger } from '../utils/logger';

/**
 * Interface for query options when retrieving telemetry data
 */
interface QueryOptions {
  limit?: number;
  offset?: number;
  orderBy?: string;
  timeInterval?: string;
  aggregation?: 'avg' | 'min' | 'max' | 'count';
}

/**
 * Telemetry model class for managing time-series telemetry data with optimized
 * batch processing and caching capabilities
 */
export class Telemetry {
  private readonly tableName: string = 'telemetry';
  private readonly timeColumn: string = 'timestamp';
  private readonly deviceColumn: string = 'device_id';
  private readonly batchSize: number;
  private readonly retryAttempts: number;
  private batchQueue: ITelemetry[] = [];

  /**
   * Initialize the Telemetry model with configuration parameters
   * @param batchSize - Size of batch for bulk inserts
   * @param retryAttempts - Number of retry attempts for failed operations
   */
  constructor(batchSize: number = 100, retryAttempts: number = 3) {
    this.batchSize = batchSize;
    this.retryAttempts = retryAttempts;
  }

  /**
   * Creates a new telemetry record with optimized batch processing
   * @param data - Partial telemetry data to create
   * @returns Promise<ITelemetry> - Created telemetry record
   */
  async create(data: Partial<ITelemetry>): Promise<ITelemetry> {
    try {
      const telemetryRecord: ITelemetry = {
        id: uuidv4(),
        deviceId: data.deviceId!,
        timestamp: data.timestamp || new Date(),
        type: data.type!,
        value: data.value!,
        unit: data.unit!,
        metadata: data.metadata || {}
      };

      // Add to batch queue
      this.batchQueue.push(telemetryRecord);

      // Process batch if queue is full
      if (this.batchQueue.length >= this.batchSize) {
        await this.processBatch();
      }

      logger.info('Telemetry record created', {
        correlationId: telemetryRecord.id,
        component: 'TelemetryModel',
        operation: 'create',
        details: { deviceId: telemetryRecord.deviceId, type: telemetryRecord.type }
      });

      return telemetryRecord;
    } catch (error) {
      logger.error(error as Error, {
        correlationId: `telemetry-create-${Date.now()}`,
        component: 'TelemetryModel',
        operation: 'create'
      });
      throw error;
    }
  }

  /**
   * Retrieves telemetry records for a specific device with time-series optimizations
   * @param deviceId - Device identifier
   * @param options - Query options for filtering and pagination
   * @returns Promise<ITelemetry[]> - Array of telemetry records
   */
  async findByDeviceId(deviceId: string, options: QueryOptions = {}): Promise<ITelemetry[]> {
    try {
      const {
        limit = 100,
        offset = 0,
        timeInterval = '1 hour',
        aggregation = 'avg'
      } = options;

      const query = `
        SELECT 
          id,
          ${this.deviceColumn},
          time_bucket($1, ${this.timeColumn}) AS timestamp,
          type,
          ${aggregation}(CASE WHEN value::text ~ '^[0-9]+\.?[0-9]*$' THEN value::text::numeric ELSE null END) as value,
          unit,
          jsonb_agg(metadata) as metadata
        FROM ${this.tableName}
        WHERE ${this.deviceColumn} = $2
        GROUP BY id, ${this.deviceColumn}, time_bucket($1, ${this.timeColumn}), type, unit
        ORDER BY timestamp DESC
        LIMIT $3 OFFSET $4
      `;

      const result = await pool.query(query, [timeInterval, deviceId, limit, offset]);

      logger.debug('Retrieved telemetry records', {
        correlationId: `telemetry-find-${deviceId}`,
        component: 'TelemetryModel',
        operation: 'findByDeviceId',
        details: { recordCount: result.rows.length }
      });

      return result.rows;
    } catch (error) {
      logger.error(error as Error, {
        correlationId: `telemetry-find-${deviceId}`,
        component: 'TelemetryModel',
        operation: 'findByDeviceId'
      });
      throw error;
    }
  }

  /**
   * Retrieves telemetry records within a specified time range using materialized views
   * @param startTime - Start of time range
   * @param endTime - End of time range
   * @param options - Query options for filtering and aggregation
   * @returns Promise<ITelemetry[]> - Array of telemetry records
   */
  async findByTimeRange(
    startTime: Date,
    endTime: Date,
    options: QueryOptions = {}
  ): Promise<ITelemetry[]> {
    try {
      const {
        limit = 1000,
        offset = 0,
        aggregation = 'avg'
      } = options;

      const query = `
        SELECT 
          id,
          ${this.deviceColumn},
          ${this.timeColumn},
          type,
          ${aggregation}(CASE WHEN value::text ~ '^[0-9]+\.?[0-9]*$' THEN value::text::numeric ELSE null END) as value,
          unit,
          metadata
        FROM ${this.tableName}
        WHERE ${this.timeColumn} BETWEEN $1 AND $2
        GROUP BY id, ${this.deviceColumn}, ${this.timeColumn}, type, unit, metadata
        ORDER BY ${this.timeColumn} DESC
        LIMIT $3 OFFSET $4
      `;

      const result = await pool.query(query, [startTime, endTime, limit, offset]);

      logger.debug('Retrieved telemetry by time range', {
        correlationId: `telemetry-timerange-${Date.now()}`,
        component: 'TelemetryModel',
        operation: 'findByTimeRange',
        details: { recordCount: result.rows.length }
      });

      return result.rows;
    } catch (error) {
      logger.error(error as Error, {
        correlationId: `telemetry-timerange-${Date.now()}`,
        component: 'TelemetryModel',
        operation: 'findByTimeRange'
      });
      throw error;
    }
  }

  /**
   * Gets the most recent telemetry record for a device and type
   * @param deviceId - Device identifier
   * @param type - Type of telemetry data
   * @returns Promise<ITelemetry> - Latest telemetry record
   */
  async getLatestByDeviceId(deviceId: string, type: TelemetryType): Promise<ITelemetry> {
    try {
      const query = `
        SELECT *
        FROM ${this.tableName}
        WHERE ${this.deviceColumn} = $1 AND type = $2
        ORDER BY ${this.timeColumn} DESC
        LIMIT 1
      `;

      const result = await pool.query(query, [deviceId, type]);

      if (result.rows.length === 0) {
        throw new Error(`No telemetry found for device ${deviceId} and type ${type}`);
      }

      logger.debug('Retrieved latest telemetry', {
        correlationId: `telemetry-latest-${deviceId}`,
        component: 'TelemetryModel',
        operation: 'getLatestByDeviceId',
        details: { type }
      });

      return result.rows[0];
    } catch (error) {
      logger.error(error as Error, {
        correlationId: `telemetry-latest-${deviceId}`,
        component: 'TelemetryModel',
        operation: 'getLatestByDeviceId'
      });
      throw error;
    }
  }

  /**
   * Processes the batch queue of telemetry records
   * @private
   */
  private async processBatch(): Promise<void> {
    if (this.batchQueue.length === 0) return;

    try {
      const values = this.batchQueue.map((record, index) => `(
        $${index * 7 + 1}, $${index * 7 + 2}, $${index * 7 + 3}, 
        $${index * 7 + 4}, $${index * 7 + 5}, $${index * 7 + 6}, 
        $${index * 7 + 7}
      )`).join(',');

      const params = this.batchQueue.flatMap(record => [
        record.id,
        record.deviceId,
        record.timestamp,
        record.type,
        record.value,
        record.unit,
        record.metadata
      ]);

      const query = `
        INSERT INTO ${this.tableName} (
          id, ${this.deviceColumn}, ${this.timeColumn}, 
          type, value, unit, metadata
        ) VALUES ${values}
      `;

      await pool.query(query, params);

      logger.info('Processed telemetry batch', {
        correlationId: `batch-${Date.now()}`,
        component: 'TelemetryModel',
        operation: 'processBatch',
        details: { batchSize: this.batchQueue.length }
      });

      // Clear the batch queue
      this.batchQueue = [];
    } catch (error) {
      logger.error(error as Error, {
        correlationId: `batch-${Date.now()}`,
        component: 'TelemetryModel',
        operation: 'processBatch'
      });
      throw error;
    }
  }
}