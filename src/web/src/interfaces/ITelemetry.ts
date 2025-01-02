/**
 * Interface defining the structure of telemetry data points received from agricultural robots and drones.
 * Used for real-time monitoring and visualization in the web dashboard.
 * @version 1.0.0
 */
export interface ITelemetry {
    /** Unique identifier for the telemetry data point */
    id: string;

    /** Identifier of the device sending the telemetry */
    deviceId: string;

    /** Timestamp when the telemetry data was recorded */
    timestamp: Date;

    /** Type of telemetry data (e.g., 'temperature', 'speed', 'position') */
    type: string;

    /** Value of the telemetry measurement. Can be various types depending on the metric */
    value: number | string | boolean | object;

    /** Unit of measurement for the telemetry value (e.g., 'celsius', 'm/s', 'degrees') */
    unit: string;

    /** Additional metadata associated with the telemetry reading */
    metadata: Record<string, any>;

    /** Geographic location where the telemetry was recorded */
    location: {
        latitude: number;
        longitude: number;
        altitude?: number;
    };

    /** Current operational status of the device */
    status: 'active' | 'inactive' | 'error' | 'maintenance';

    /** Current battery level as a percentage (0-100) */
    batteryLevel: number;

    /** ID of the current mission if device is assigned to one, null otherwise */
    missionId: string | null;

    /** Type of device sending the telemetry */
    deviceType: 'drone' | 'ground_robot';
}