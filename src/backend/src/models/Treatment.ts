import { QueryResult, PoolClient } from 'pg'; // v8.11.0
import { ITreatment, ITreatmentLocation, ITreatmentCoverage } from '../interfaces/ITreatment';
import { pool } from '../config/database';
import { logger } from '../utils/logger';
import { MissionStatus } from '../constants/missionStatus';

/**
 * Configuration options for the Treatment model
 */
interface TreatmentOptions {
    retryAttempts?: number;
    batchSize?: number;
    enableMetrics?: boolean;
}

/**
 * Query options for treatment retrieval
 */
interface TreatmentQueryOptions {
    startDate?: Date;
    endDate?: Date;
    limit?: number;
    offset?: number;
    sortBy?: string;
    sortOrder?: 'ASC' | 'DESC';
}

/**
 * Enhanced Treatment model class for managing agricultural treatment operations
 * with optimized time-series data handling and geospatial queries
 */
export class Treatment {
    private readonly pool;
    private readonly retryAttempts: number;
    private readonly batchSize: number;
    private readonly metricsEnabled: boolean;

    constructor(options: TreatmentOptions = {}) {
        this.pool = pool;
        this.retryAttempts = options.retryAttempts || 3;
        this.batchSize = options.batchSize || 100;
        this.metricsEnabled = options.enableMetrics || true;
        this.initializeDatabase();
    }

    /**
     * Initializes the database schema and required indexes
     */
    private async initializeDatabase(): Promise<void> {
        const client = await this.pool.connect();
        try {
            await client.query('BEGIN');

            // Create hypertable for time-series optimization
            await client.query(`
                CREATE TABLE IF NOT EXISTS treatments (
                    id UUID PRIMARY KEY,
                    mission_id UUID NOT NULL,
                    device_id UUID NOT NULL,
                    type VARCHAR(50) NOT NULL,
                    status VARCHAR(20) NOT NULL,
                    location POINT NOT NULL,
                    applied_at TIMESTAMPTZ NOT NULL,
                    quantity DECIMAL(10,2) NOT NULL,
                    coverage GEOMETRY(POLYGON, 4326) NOT NULL,
                    parameters JSONB,
                    robot_type VARCHAR(20) NOT NULL,
                    concentration DECIMAL(10,2),
                    weather_conditions JSONB,
                    effectiveness JSONB,
                    created_at TIMESTAMPTZ DEFAULT CURRENT_TIMESTAMP
                );
            `);

            // Convert to hypertable
            await client.query(`
                SELECT create_hypertable('treatments', 'applied_at', 
                    chunk_time_interval => INTERVAL '1 day',
                    if_not_exists => TRUE
                );
            `);

            // Create indexes for optimized queries
            await client.query(`
                CREATE INDEX IF NOT EXISTS idx_treatments_mission_id ON treatments(mission_id);
                CREATE INDEX IF NOT EXISTS idx_treatments_device_id ON treatments(device_id);
                CREATE INDEX IF NOT EXISTS idx_treatments_applied_at ON treatments(applied_at DESC);
                CREATE INDEX IF NOT EXISTS idx_treatments_location USING GIST(location);
                CREATE INDEX IF NOT EXISTS idx_treatments_coverage USING GIST(coverage);
            `);

            await client.query('COMMIT');
            
            logger.info('Treatment model initialized successfully', {
                correlationId: `init-${Date.now()}`,
                component: 'TreatmentModel',
                operation: 'initialize'
            });
        } catch (error) {
            await client.query('ROLLBACK');
            logger.error(error as Error, {
                correlationId: `init-error-${Date.now()}`,
                component: 'TreatmentModel',
                operation: 'initialize'
            });
            throw error;
        } finally {
            client.release();
        }
    }

    /**
     * Creates a new treatment record with enhanced validation and error handling
     */
    public async createTreatment(treatment: ITreatment): Promise<ITreatment> {
        const client = await this.pool.connect();
        try {
            await client.query('BEGIN');

            const result = await client.query(`
                INSERT INTO treatments (
                    id, mission_id, device_id, type, status, location, 
                    applied_at, quantity, coverage, parameters, robot_type,
                    concentration, weather_conditions, effectiveness
                ) VALUES (
                    $1, $2, $3, $4, $5, point($6, $7),
                    $8, $9, ST_GeomFromGeoJSON($10), $11, $12,
                    $13, $14, $15
                ) RETURNING *;
            `, [
                treatment.id,
                treatment.missionId,
                treatment.deviceId,
                treatment.type,
                treatment.status,
                treatment.location.latitude,
                treatment.location.longitude,
                treatment.appliedAt,
                treatment.quantity,
                JSON.stringify(treatment.coverage),
                treatment.parameters,
                treatment.robotType,
                treatment.concentration,
                treatment.weatherConditions,
                treatment.effectiveness
            ]);

            await client.query('COMMIT');

            if (this.metricsEnabled) {
                logger.info('Treatment record created', {
                    correlationId: `treatment-${treatment.id}`,
                    component: 'TreatmentModel',
                    operation: 'create',
                    details: {
                        missionId: treatment.missionId,
                        deviceId: treatment.deviceId,
                        type: treatment.type
                    }
                });
            }

            return this.transformTreatmentRecord(result.rows[0]);
        } catch (error) {
            await client.query('ROLLBACK');
            logger.error(error as Error, {
                correlationId: `treatment-error-${treatment.id}`,
                component: 'TreatmentModel',
                operation: 'create'
            });
            throw error;
        } finally {
            client.release();
        }
    }

    /**
     * Retrieves treatments by mission ID with optimized query performance
     */
    public async getTreatmentsByMission(
        missionId: string,
        options: TreatmentQueryOptions = {}
    ): Promise<ITreatment[]> {
        const {
            startDate,
            endDate,
            limit = 100,
            offset = 0,
            sortBy = 'applied_at',
            sortOrder = 'DESC'
        } = options;

        const queryParams: any[] = [missionId];
        let queryConditions = 'WHERE mission_id = $1';
        let paramCount = 1;

        if (startDate) {
            paramCount++;
            queryConditions += ` AND applied_at >= $${paramCount}`;
            queryParams.push(startDate);
        }

        if (endDate) {
            paramCount++;
            queryConditions += ` AND applied_at <= $${paramCount}`;
            queryParams.push(endDate);
        }

        try {
            const result = await this.pool.query(`
                SELECT *,
                    ST_AsGeoJSON(coverage) as coverage_geojson,
                    ST_X(location) as latitude,
                    ST_Y(location) as longitude
                FROM treatments
                ${queryConditions}
                ORDER BY ${sortBy} ${sortOrder}
                LIMIT $${paramCount + 1} OFFSET $${paramCount + 2};
            `, [...queryParams, limit, offset]);

            return result.rows.map(this.transformTreatmentRecord);
        } catch (error) {
            logger.error(error as Error, {
                correlationId: `query-error-${Date.now()}`,
                component: 'TreatmentModel',
                operation: 'getTreatmentsByMission'
            });
            throw error;
        }
    }

    /**
     * Transforms a database record into an ITreatment interface
     */
    private transformTreatmentRecord(record: any): ITreatment {
        return {
            id: record.id,
            missionId: record.mission_id,
            deviceId: record.device_id,
            type: record.type,
            status: record.status as MissionStatus,
            location: {
                latitude: record.latitude,
                longitude: record.longitude,
                altitude: 0 // Altitude is not stored in the current schema
            },
            appliedAt: record.applied_at,
            quantity: record.quantity,
            coverage: JSON.parse(record.coverage_geojson),
            parameters: record.parameters,
            robotType: record.robot_type,
            concentration: record.concentration,
            weatherConditions: record.weather_conditions,
            effectiveness: record.effectiveness
        };
    }
}

export default Treatment;