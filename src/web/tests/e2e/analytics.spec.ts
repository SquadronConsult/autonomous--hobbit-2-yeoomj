/**
 * @fileoverview End-to-end test suite for analytics dashboard functionality
 * Implements comprehensive testing of real-time analytics features with strict performance requirements
 * @version 1.0.0
 */

import { test, expect } from '@playwright/test';
import { server, mockWebSocket } from '../setup';
import { AnalyticsService } from '../../src/services/analytics';
import { AnalyticsType, MIN_CONFIDENCE_THRESHOLD } from '../../src/interfaces/IAnalytics';
import { UserRole } from '../../src/interfaces/IUser';

// Test configuration constants
const TEST_TIMEOUT = 30000;
const REFRESH_INTERVAL = 1000;
const MIN_CONFIDENCE_THRESHOLD = 0.95;
const MAX_LATENCY_MS = 100;
const RECONNECTION_TIMEOUT = 5000;

test.describe('Analytics Dashboard Tests', () => {
    let analyticsService: AnalyticsService;

    test.beforeEach(async ({ page }) => {
        // Reset server state and initialize mocks
        server.resetHandlers();
        await mockWebSocket.connect();

        // Initialize analytics service
        analyticsService = new AnalyticsService(page);

        // Navigate to analytics dashboard
        await page.goto('/analytics');
        
        // Wait for initial data load
        await page.waitForSelector('[data-testid="analytics-dashboard"]', {
            timeout: TEST_TIMEOUT,
            state: 'visible'
        });

        // Verify authentication state
        const isAuthenticated = await page.evaluate(() => {
            return window.localStorage.getItem('isAuthenticated') === 'true';
        });
        expect(isAuthenticated).toBeTruthy();
    });

    test.afterEach(async () => {
        // Clean up subscriptions and connections
        mockWebSocket.disconnect();
        await analyticsService.closeAllStreams();
        server.resetHandlers();
    });

    test('should load analytics dashboard with correct layout', async ({ page }) => {
        // Verify dashboard structure
        await expect(page.locator('[data-testid="pest-detection-chart"]')).toBeVisible();
        await expect(page.locator('[data-testid="treatment-coverage-map"]')).toBeVisible();
        await expect(page.locator('[data-testid="system-metrics"]')).toBeVisible();

        // Verify accessibility requirements
        await expect(page).toHaveScreenshot('analytics-dashboard.png');
        const accessibilityScore = await page.evaluate(async () => {
            const response = await fetch('/api/v1/accessibility-check');
            return response.json();
        });
        expect(accessibilityScore.score).toBeGreaterThanOrEqual(90);

        // Test responsive layout
        await page.setViewportSize({ width: 768, height: 1024 });
        await expect(page.locator('.analytics-grid')).toHaveClass(/mobile-layout/);
    });

    test('should handle real-time updates with sub-100ms latency', async ({ page }) => {
        // Initialize WebSocket connection
        const latencyResults: number[] = [];
        
        // Subscribe to real-time updates
        const unsubscribe = await analyticsService.streamAnalytics(
            {
                startDate: new Date(),
                endDate: new Date(),
                type: AnalyticsType.PEST_DETECTION,
                deviceId: 'test-device'
            },
            async (data) => {
                const latency = await mockWebSocket.measureLatency();
                latencyResults.push(latency);
            }
        );

        // Simulate real-time updates
        for (let i = 0; i < 10; i++) {
            await mockWebSocket.simulateMessage('analytics', {
                id: `test-${i}`,
                type: AnalyticsType.PEST_DETECTION,
                confidence: 0.97,
                timestamp: new Date()
            });
            await page.waitForTimeout(REFRESH_INTERVAL);
        }

        // Test reconnection handling
        await mockWebSocket.simulateReconnection(RECONNECTION_TIMEOUT);
        
        // Verify latency requirements
        const averageLatency = latencyResults.reduce((a, b) => a + b, 0) / latencyResults.length;
        expect(averageLatency).toBeLessThan(MAX_LATENCY_MS);
        
        unsubscribe();
    });

    test('should validate pest detection accuracy threshold', async ({ page }) => {
        // Mock detection data
        const detectionData = {
            id: 'test-detection',
            type: AnalyticsType.PEST_DETECTION,
            confidence: 0.96,
            detections: [
                { type: 'aphids', confidence: 0.97, boundingBox: { x: 0.1, y: 0.1, width: 0.2, height: 0.2 } },
                { type: 'beetles', confidence: 0.98, boundingBox: { x: 0.5, y: 0.5, width: 0.2, height: 0.2 } }
            ]
        };

        // Simulate detection updates
        await mockWebSocket.simulateMessage('detection', detectionData);
        
        // Verify confidence indicators
        const confidenceIndicator = page.locator('[data-testid="confidence-indicator"]');
        await expect(confidenceIndicator).toHaveText(/96%/);
        await expect(confidenceIndicator).toHaveClass(/above-threshold/);

        // Test threshold validation
        const belowThresholdData = { ...detectionData, confidence: 0.94 };
        await mockWebSocket.simulateMessage('detection', belowThresholdData);
        await expect(confidenceIndicator).toHaveClass(/below-threshold/);
        
        // Verify alert generation
        await expect(page.locator('[data-testid="accuracy-alert"]')).toBeVisible();
    });

    test('should visualize treatment coverage metrics', async ({ page }) => {
        // Mock coverage data
        const coverageData = {
            id: 'test-coverage',
            type: AnalyticsType.TREATMENT_COVERAGE,
            coverage: 85.5,
            timestamp: new Date(),
            zones: [
                { id: 'zone-1', coverage: 90.0, area: 1000 },
                { id: 'zone-2', coverage: 81.0, area: 1500 }
            ]
        };

        // Update coverage visualization
        await mockWebSocket.simulateMessage('coverage', coverageData);
        
        // Verify coverage map
        const coverageMap = page.locator('[data-testid="coverage-map"]');
        await expect(coverageMap).toBeVisible();
        
        // Test zone highlighting
        await page.click('[data-testid="zone-1"]');
        await expect(page.locator('[data-testid="zone-details"]')).toContainText('90.0%');
        
        // Verify coverage calculations
        const totalCoverage = page.locator('[data-testid="total-coverage"]');
        await expect(totalCoverage).toContainText('85.5%');
    });
});