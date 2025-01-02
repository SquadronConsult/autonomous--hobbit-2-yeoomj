/**
 * @fileoverview Unit tests for the core API service
 * @version 1.0.0
 */

import { describe, test, expect, beforeAll, afterAll, beforeEach, jest } from '@jest/globals'; // jest@29.5.0
import { setupServer } from 'msw/node'; // msw@1.2.0
import { rest } from 'msw'; // msw@1.2.0
import { ApiService } from '../../src/services/api';
import { API_ENDPOINTS } from '../../src/constants/apiEndpoints';
import { handlers } from '../../mocks/handlers';
import { HttpStatusCodes } from '../../src/constants/statusCodes';

// Test configuration
const TEST_TIMEOUT = 5000;
const MOCK_AUTH_TOKEN = 'mock-jwt-token';
const server = setupServer(...handlers);
let apiService: ApiService;

// Mock response data
const mockMissionResponse = {
    id: 'mission-123',
    name: 'Test Mission',
    type: 'SURVEY',
    status: 'PENDING',
    progress: 0
};

const mockDeviceResponse = {
    id: 'device-123',
    type: 'DRONE',
    status: 'ACTIVE',
    batteryLevel: 85
};

// Test setup and teardown
beforeAll(() => {
    server.listen({ onUnhandledRequest: 'error' });
});

afterAll(() => {
    server.close();
});

beforeEach(() => {
    server.resetHandlers();
    apiService = new ApiService();
});

describe('ApiService HTTP Methods', () => {
    test('should make successful GET request with proper response transformation', async () => {
        server.use(
            rest.get(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/missions`, (req, res, ctx) => {
                return res(ctx.status(200), ctx.json(mockMissionResponse));
            })
        );

        const response = await apiService.request({
            method: 'GET',
            url: '/missions'
        });

        expect(response).toEqual(mockMissionResponse);
    }, TEST_TIMEOUT);

    test('should make successful POST request with data validation', async () => {
        const requestData = {
            name: 'New Mission',
            type: 'SURVEY'
        };

        server.use(
            rest.post(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/missions`, async (req, res, ctx) => {
                const body = await req.json();
                expect(body).toEqual(requestData);
                return res(ctx.status(201), ctx.json({ ...mockMissionResponse, ...requestData }));
            })
        );

        const response = await apiService.request({
            method: 'POST',
            url: '/missions',
            data: requestData
        });

        expect(response).toMatchObject(requestData);
    }, TEST_TIMEOUT);

    test('should make successful PUT request with optimistic updates', async () => {
        const updateData = {
            status: 'IN_PROGRESS',
            progress: 50
        };

        server.use(
            rest.put(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/missions/mission-123`, async (req, res, ctx) => {
                const body = await req.json();
                expect(body).toEqual(updateData);
                return res(ctx.status(200), ctx.json({ ...mockMissionResponse, ...updateData }));
            })
        );

        const response = await apiService.request({
            method: 'PUT',
            url: '/missions/mission-123',
            data: updateData
        });

        expect(response.status).toBe('IN_PROGRESS');
        expect(response.progress).toBe(50);
    }, TEST_TIMEOUT);

    test('should handle query parameters correctly', async () => {
        server.use(
            rest.get(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/devices`, (req, res, ctx) => {
                expect(req.url.searchParams.get('status')).toBe('ACTIVE');
                expect(req.url.searchParams.get('type')).toBe('DRONE');
                return res(ctx.status(200), ctx.json([mockDeviceResponse]));
            })
        );

        await apiService.request({
            method: 'GET',
            url: '/devices',
            params: {
                status: 'ACTIVE',
                type: 'DRONE'
            }
        });
    }, TEST_TIMEOUT);
});

describe('ApiService Authentication', () => {
    test('should set authentication token correctly', () => {
        apiService.setAuthToken(MOCK_AUTH_TOKEN);
        expect(apiService['apiInstance'].defaults.headers.common['Authorization'])
            .toBe(`Bearer ${MOCK_AUTH_TOKEN}`);
    });

    test('should include auth token in request headers', async () => {
        apiService.setAuthToken(MOCK_AUTH_TOKEN);

        server.use(
            rest.get(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/missions`, (req, res, ctx) => {
                expect(req.headers.get('Authorization')).toBe(`Bearer ${MOCK_AUTH_TOKEN}`);
                return res(ctx.status(200), ctx.json(mockMissionResponse));
            })
        );

        await apiService.request({
            method: 'GET',
            url: '/missions'
        });
    }, TEST_TIMEOUT);

    test('should handle unauthorized responses', async () => {
        server.use(
            rest.get(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/missions`, (req, res, ctx) => {
                return res(ctx.status(401), ctx.json({ message: 'Unauthorized' }));
            })
        );

        await expect(apiService.request({
            method: 'GET',
            url: '/missions'
        })).rejects.toThrow('Unauthorized');
    }, TEST_TIMEOUT);
});

describe('ApiService Error Handling', () => {
    test('should handle network errors with retry', async () => {
        let attempts = 0;
        server.use(
            rest.get(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/missions`, (req, res, ctx) => {
                attempts++;
                if (attempts < 3) {
                    return res(ctx.status(503));
                }
                return res(ctx.status(200), ctx.json(mockMissionResponse));
            })
        );

        const response = await apiService.request({
            method: 'GET',
            url: '/missions'
        });

        expect(attempts).toBe(3);
        expect(response).toEqual(mockMissionResponse);
    }, TEST_TIMEOUT);

    test('should implement circuit breaker pattern', async () => {
        let failures = 0;
        server.use(
            rest.get(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/missions`, (req, res, ctx) => {
                failures++;
                return res(ctx.status(500));
            })
        );

        for (let i = 0; i < 5; i++) {
            try {
                await apiService.request({
                    method: 'GET',
                    url: '/missions'
                });
            } catch (error) {
                expect(error).toBeDefined();
            }
        }

        expect(failures).toBeLessThan(5); // Circuit breaker should prevent all attempts
    }, TEST_TIMEOUT);

    test('should queue requests when offline', async () => {
        const originalOnline = window.navigator.onLine;
        Object.defineProperty(window.navigator, 'onLine', { value: false, configurable: true });

        try {
            await apiService.request({
                method: 'GET',
                url: '/missions'
            });
        } catch (error: any) {
            expect(error.message).toContain('offline');
            expect(apiService['requestQueue'].size()).toBe(1);
        }

        Object.defineProperty(window.navigator, 'onLine', { value: originalOnline, configurable: true });
    }, TEST_TIMEOUT);
});

describe('ApiService Request Configuration', () => {
    test('should respect request timeout settings', async () => {
        server.use(
            rest.get(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/missions`, async (req, res, ctx) => {
                await new Promise(resolve => setTimeout(resolve, 2000));
                return res(ctx.status(200), ctx.json(mockMissionResponse));
            })
        );

        await expect(apiService.request({
            method: 'GET',
            url: '/missions',
            timeout: 1000
        })).rejects.toThrow('timeout');
    }, TEST_TIMEOUT);

    test('should implement request caching', async () => {
        let calls = 0;
        server.use(
            rest.get(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/missions`, (req, res, ctx) => {
                calls++;
                return res(ctx.status(200), ctx.json(mockMissionResponse));
            })
        );

        // Make two identical requests
        await apiService.request({ method: 'GET', url: '/missions' });
        await apiService.request({ method: 'GET', url: '/missions' });

        expect(calls).toBe(1); // Second request should use cache
    }, TEST_TIMEOUT);

    test('should track performance metrics', async () => {
        server.use(
            rest.get(`${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/missions`, (req, res, ctx) => {
                return res(ctx.status(200), ctx.json(mockMissionResponse));
            })
        );

        await apiService.request({ method: 'GET', url: '/missions' });
        const metrics = apiService.getMetrics();

        expect(metrics['/missions']).toBeDefined();
        expect(typeof metrics['/missions']).toBe('number');
    }, TEST_TIMEOUT);
});