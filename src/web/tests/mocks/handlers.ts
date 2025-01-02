/**
 * @fileoverview Mock request handlers for MSW (Mock Service Worker)
 * Provides comprehensive API mocking for testing with realistic response patterns
 * @version 1.0.0
 */

import { rest } from 'msw';
import { API_ENDPOINTS } from '../../src/constants/apiEndpoints';
import { HttpStatusCodes } from '../../src/constants/statusCodes';

// Configuration constants
const MOCK_DELAY = 100; // Simulated network delay in ms
const DEFAULT_PAGE_SIZE = 10;
const MAX_PAGE_SIZE = 100;
const ERROR_SIMULATION_RATE = 0.1; // 10% chance of error response

// Types for mock data
interface IMission {
    id: string;
    type: 'survey' | 'treatment' | 'monitoring';
    status: string;
    progress: number;
    parameters: Record<string, unknown>;
    coverage_area: {
        type: 'Polygon';
        coordinates: number[][][];
    };
    created_at: string;
    updated_at: string;
}

interface IDevice {
    id: string;
    type: 'drone' | 'ground';
    model: string;
    status: string;
    battery_level: number;
    location: {
        type: 'Point';
        coordinates: [number, number];
    };
    last_telemetry: string;
}

interface IAnalytics {
    id: string;
    mission_id?: string;
    metrics: {
        pest_detection: Record<string, number>;
        treatment_coverage: number;
        system_performance: Record<string, number>;
    };
    timestamp: string;
}

interface ITelemetry {
    device_id: string;
    mission_id?: string;
    metrics: {
        battery: number;
        speed: number;
        altitude?: number;
        sensor_readings: Record<string, number>;
    };
    location: {
        type: 'Point';
        coordinates: [number, number];
    };
    timestamp: string;
}

// Mock data generators
const createMockMission = (overrides: Partial<IMission> = {}): IMission => ({
    id: Math.random().toString(36).substr(2, 9),
    type: 'survey',
    status: 'pending',
    progress: 0,
    parameters: {
        altitude: 50,
        speed: 5,
        scan_resolution: 'high'
    },
    coverage_area: {
        type: 'Polygon',
        coordinates: [[[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]]
    },
    created_at: new Date().toISOString(),
    updated_at: new Date().toISOString(),
    ...overrides
});

const createMockDevice = (overrides: Partial<IDevice> = {}): IDevice => ({
    id: Math.random().toString(36).substr(2, 9),
    type: 'drone',
    model: 'DJI-M300-RTK',
    status: 'online',
    battery_level: Math.random() * 100,
    location: {
        type: 'Point',
        coordinates: [Math.random() * 180 - 90, Math.random() * 360 - 180]
    },
    last_telemetry: new Date().toISOString(),
    ...overrides
});

const createMockAnalytics = (overrides: Partial<IAnalytics> = {}): IAnalytics => ({
    id: Math.random().toString(36).substr(2, 9),
    metrics: {
        pest_detection: {
            aphids: Math.random() * 100,
            beetles: Math.random() * 100
        },
        treatment_coverage: Math.random() * 100,
        system_performance: {
            cpu_usage: Math.random() * 100,
            memory_usage: Math.random() * 100
        }
    },
    timestamp: new Date().toISOString(),
    ...overrides
});

const createMockTelemetry = (overrides: Partial<ITelemetry> = {}): ITelemetry => ({
    device_id: Math.random().toString(36).substr(2, 9),
    metrics: {
        battery: Math.random() * 100,
        speed: Math.random() * 10,
        altitude: Math.random() * 100,
        sensor_readings: {
            temperature: 20 + Math.random() * 10,
            humidity: 40 + Math.random() * 20
        }
    },
    location: {
        type: 'Point',
        coordinates: [Math.random() * 180 - 90, Math.random() * 360 - 180]
    },
    timestamp: new Date().toISOString(),
    ...overrides
});

// Utility functions
const simulateNetworkDelay = async () => {
    await new Promise(resolve => setTimeout(resolve, MOCK_DELAY));
};

const shouldSimulateError = () => Math.random() < ERROR_SIMULATION_RATE;

// Mission handlers
const missionHandlers = [
    rest.get(API_ENDPOINTS.MISSIONS.GET_ALL, async (req, res, ctx) => {
        await simulateNetworkDelay();
        
        if (shouldSimulateError()) {
            return res(ctx.status(HttpStatusCodes.INTERNAL_SERVER_ERROR));
        }

        const page = parseInt(req.url.searchParams.get('page') || '1');
        const pageSize = Math.min(
            parseInt(req.url.searchParams.get('pageSize') || String(DEFAULT_PAGE_SIZE)),
            MAX_PAGE_SIZE
        );

        const missions = Array(pageSize).fill(null).map(() => createMockMission());

        return res(
            ctx.status(HttpStatusCodes.OK),
            ctx.json({
                data: missions,
                pagination: {
                    page,
                    pageSize,
                    total: 100
                }
            })
        );
    }),

    rest.post(API_ENDPOINTS.MISSIONS.CREATE, async (req, res, ctx) => {
        await simulateNetworkDelay();

        if (shouldSimulateError()) {
            return res(ctx.status(HttpStatusCodes.BAD_REQUEST));
        }

        const mission = createMockMission(await req.json());
        return res(ctx.status(HttpStatusCodes.CREATED), ctx.json(mission));
    })
];

// Device handlers
const deviceHandlers = [
    rest.get(API_ENDPOINTS.DEVICES.GET_ALL, async (req, res, ctx) => {
        await simulateNetworkDelay();

        if (shouldSimulateError()) {
            return res(ctx.status(HttpStatusCodes.INTERNAL_SERVER_ERROR));
        }

        const devices = Array(10).fill(null).map(() => createMockDevice());
        return res(ctx.status(HttpStatusCodes.OK), ctx.json(devices));
    }),

    rest.get(API_ENDPOINTS.DEVICES.STATUS, async (req, res, ctx) => {
        await simulateNetworkDelay();

        if (shouldSimulateError()) {
            return res(ctx.status(HttpStatusCodes.NOT_FOUND));
        }

        const device = createMockDevice();
        return res(ctx.status(HttpStatusCodes.OK), ctx.json(device));
    })
];

// Analytics handlers
const analyticsHandlers = [
    rest.get(API_ENDPOINTS.ANALYTICS.PERFORMANCE, async (req, res, ctx) => {
        await simulateNetworkDelay();

        if (shouldSimulateError()) {
            return res(ctx.status(HttpStatusCodes.INTERNAL_SERVER_ERROR));
        }

        const analytics = createMockAnalytics();
        return res(ctx.status(HttpStatusCodes.OK), ctx.json(analytics));
    })
];

// Telemetry handlers
const telemetryHandlers = [
    rest.get(API_ENDPOINTS.TELEMETRY.STREAM, async (req, res, ctx) => {
        await simulateNetworkDelay();

        if (shouldSimulateError()) {
            return res(ctx.status(HttpStatusCodes.INTERNAL_SERVER_ERROR));
        }

        const telemetry = createMockTelemetry();
        return res(ctx.status(HttpStatusCodes.OK), ctx.json(telemetry));
    })
];

// Export all handlers
export const handlers = [
    ...missionHandlers,
    ...deviceHandlers,
    ...analyticsHandlers,
    ...telemetryHandlers
];