/**
 * @fileoverview End-to-end tests for fleet management interface
 * Validates real-time monitoring, device control, telemetry visualization, and responsive design
 * @version 1.0.0
 */

import { test, expect, Page } from '@playwright/test';
import { mockWebSocket } from '../mocks/websocket';
import { handlers } from '../mocks/handlers';
import { HttpStatusCodes, DeviceStatusCodes } from '../../src/constants/statusCodes';
import { API_ENDPOINTS } from '../../src/constants/apiEndpoints';

// Test configuration constants
const VIEWPORT_BREAKPOINTS = {
  MOBILE: { width: 320, height: 568 },
  TABLET: { width: 768, height: 1024 },
  DESKTOP: { width: 1024, height: 768 },
  LARGE: { width: 1440, height: 900 }
};

const PERFORMANCE_THRESHOLDS = {
  MAX_LATENCY: 100, // Maximum acceptable latency in ms
  MIN_DRONE_FEEDS: 8, // Minimum simultaneous drone feeds
  RENDER_TIME: 16 // Maximum frame render time in ms
};

let mockWs: typeof mockWebSocket;

test.describe('Fleet Management Page', () => {
  test.beforeEach(async ({ page }) => {
    // Initialize mock WebSocket
    mockWs = new mockWebSocket(`ws://${API_ENDPOINTS.BASE_URL}/${API_ENDPOINTS.API_VERSION}/ws`);
    await mockWs.connect();
    mockWs.simulateLatency(50); // Set 50ms baseline latency

    // Reset mock API handlers
    await page.route(`${API_ENDPOINTS.BASE_URL}/*`, async (route) => {
      const handler = handlers.find(h => h.test(route.request().url()));
      if (handler) {
        await handler(route.request(), route, route.continue);
      } else {
        await route.continue();
      }
    });

    // Navigate to fleet page
    await page.goto('/fleet');
    await page.waitForLoadState('networkidle');
  });

  test('validates initial page load and responsive design', async ({ page }) => {
    // Test Material Design 3.0 compliance
    const materialTheme = await page.evaluate(() => {
      const styles = window.getComputedStyle(document.documentElement);
      return {
        primaryColor: styles.getPropertyValue('--md-sys-color-primary'),
        typography: styles.getPropertyValue('--md-sys-typescale-body-large-font')
      };
    });
    expect(materialTheme.typography).toContain('Roboto');

    // Test responsive layout at different breakpoints
    for (const [name, dimensions] of Object.entries(VIEWPORT_BREAKPOINTS)) {
      await page.setViewportSize(dimensions);
      await page.waitForTimeout(500); // Allow for layout recalculation

      // Verify component visibility and layout
      const fleetGrid = await page.locator('.fleet-grid');
      const isGridVisible = await fleetGrid.isVisible();
      expect(isGridVisible).toBeTruthy();

      // Check responsive grid columns
      const gridColumns = await fleetGrid.evaluate((el) => 
        window.getComputedStyle(el).getPropertyValue('grid-template-columns')
      );
      
      switch (name) {
        case 'MOBILE':
          expect(gridColumns).toContain('1fr');
          break;
        case 'TABLET':
          expect(gridColumns).toContain('repeat(2, 1fr)');
          break;
        default:
          expect(gridColumns).toContain('repeat(3, 1fr)');
      }
    }

    // Verify accessibility compliance
    const accessibilityReport = await page.accessibility.snapshot();
    expect(accessibilityReport).toBeTruthy();
  });

  test('validates real-time updates and performance', async ({ page }) => {
    // Setup performance monitoring
    await page.evaluate(() => {
      window.performance.mark('telemetryStart');
    });

    // Simulate multiple drone feeds
    const numDrones = PERFORMANCE_THRESHOLDS.MIN_DRONE_FEEDS;
    for (let i = 0; i < numDrones; i++) {
      mockWs.simulateMessage('telemetry', {
        device_id: `drone-${i}`,
        status: DeviceStatusCodes.ONLINE,
        location: { type: 'Point', coordinates: [Math.random() * 180 - 90, Math.random() * 360 - 180] },
        battery_level: Math.random() * 100
      });
    }

    // Verify update frequency
    const updateLatency = await page.evaluate(() => {
      const mark = window.performance.getEntriesByName('telemetryStart')[0];
      return performance.now() - mark.startTime;
    });
    expect(updateLatency).toBeLessThan(PERFORMANCE_THRESHOLDS.MAX_LATENCY);

    // Verify map markers
    const mapMarkers = await page.locator('.device-marker');
    expect(await mapMarkers.count()).toBeGreaterThanOrEqual(numDrones);

    // Test telemetry graph performance
    const telemetryGraph = await page.locator('.telemetry-graph');
    const graphPerformance = await telemetryGraph.evaluate((el) => {
      const start = performance.now();
      el.style.transform = 'scale(1.01)';
      return performance.now() - start;
    });
    expect(graphPerformance).toBeLessThan(PERFORMANCE_THRESHOLDS.RENDER_TIME);
  });

  test('validates device control functionality', async ({ page }) => {
    // Test keyboard navigation
    await page.keyboard.press('Tab');
    const focusedElement = await page.evaluate(() => document.activeElement?.className);
    expect(focusedElement).toContain('device-list-item');

    // Test command button states
    const commandButton = await page.locator('.command-button').first();
    expect(await commandButton.isEnabled()).toBeTruthy();

    // Test command execution
    await commandButton.click();
    const confirmDialog = await page.locator('.confirmation-dialog');
    expect(await confirmDialog.isVisible()).toBeTruthy();

    // Verify command feedback
    await page.locator('.confirm-button').click();
    const successToast = await page.locator('.toast-success');
    expect(await successToast.isVisible()).toBeTruthy();

    // Test multi-device selection
    await page.keyboard.down('Control');
    await page.click('.device-list-item:nth-child(1)');
    await page.click('.device-list-item:nth-child(2)');
    await page.keyboard.up('Control');

    const selectedDevices = await page.locator('.device-list-item.selected');
    expect(await selectedDevices.count()).toBe(2);
  });

  test('handles error scenarios gracefully', async ({ page }) => {
    // Test connection loss
    mockWs.simulateError(new Error('Connection lost'));
    const reconnectBanner = await page.locator('.reconnect-banner');
    expect(await reconnectBanner.isVisible()).toBeTruthy();

    // Test data fetch error
    await page.route(`${API_ENDPOINTS.BASE_URL}${API_ENDPOINTS.DEVICES.GET_ALL}`, (route) => {
      route.fulfill({
        status: HttpStatusCodes.INTERNAL_SERVER_ERROR,
        body: JSON.stringify({ error: 'Internal server error' })
      });
    });

    await page.reload();
    const errorState = await page.locator('.error-state');
    expect(await errorState.isVisible()).toBeTruthy();

    // Test recovery
    mockWs.connect();
    await page.waitForTimeout(1000);
    expect(await reconnectBanner.isVisible()).toBeFalsy();
  });
});