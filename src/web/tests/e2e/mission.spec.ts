/**
 * @fileoverview End-to-end tests for mission management functionality
 * @version 1.0.0
 * 
 * Provides comprehensive test coverage for mission operations including:
 * - Mission creation, monitoring, and management
 * - Status tracking and updates
 * - Error handling and validation
 * - Accessibility and performance testing
 */

import { test, expect, type Page } from '@playwright/test'; // @playwright/test@1.35.0
import { IMission } from '../../src/interfaces/IMission';
import { API_ENDPOINTS } from '../../src/constants/apiEndpoints';
import { MissionTypes } from '../../src/constants/missionTypes';
import { MissionStatusCodes } from '../../src/constants/statusCodes';
import { v4 as uuidv4 } from 'uuid'; // uuid@9.0.0

/**
 * Creates a test mission object with configurable properties
 * @param overrides - Optional partial mission properties to override defaults
 * @returns Complete test mission object
 */
export const createTestMission = (overrides: Partial<IMission> = {}): IMission => {
    const defaultMission: IMission = {
        id: uuidv4(),
        name: `Test Mission ${Date.now()}`,
        description: 'Automated test mission',
        type: MissionTypes.SURVEY,
        status: MissionStatusCodes.PENDING,
        assignedDevices: [],
        coverageArea: {
            type: 'Polygon',
            coordinates: [[[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]]
        },
        startTime: new Date(),
        endTime: null,
        progress: 0,
        parameters: {
            altitude: 50,
            speed: 5,
            scanResolution: 2,
            treatmentType: '',
            treatmentDensity: 0
        }
    };

    return { ...defaultMission, ...overrides };
};

/**
 * Helper function to fill mission creation form
 * @param page - Playwright page object
 * @param missionData - Mission data to fill in form
 */
async function fillMissionForm(page: Page, missionData: IMission): Promise<void> {
    await page.fill('[data-testid="mission-name"]', missionData.name);
    await page.fill('[data-testid="mission-description"]', missionData.description);
    await page.selectOption('[data-testid="mission-type"]', missionData.type);
    
    // Fill coverage area coordinates
    await page.click('[data-testid="coverage-area-input"]');
    const coordinates = JSON.stringify(missionData.coverageArea.coordinates);
    await page.fill('[data-testid="coverage-area-input"]', coordinates);
    
    // Fill mission parameters based on type
    await page.fill('[data-testid="altitude"]', missionData.parameters.altitude.toString());
    await page.fill('[data-testid="speed"]', missionData.parameters.speed.toString());
    
    if (missionData.type === MissionTypes.SURVEY) {
        await page.fill('[data-testid="scan-resolution"]', 
            missionData.parameters.scanResolution.toString());
    }
    
    if (missionData.type === MissionTypes.TREATMENT) {
        await page.fill('[data-testid="treatment-type"]', missionData.parameters.treatmentType);
        await page.fill('[data-testid="treatment-density"]', 
            missionData.parameters.treatmentDensity.toString());
    }
}

test.describe('Mission Management', () => {
    let testMission: IMission;

    test.beforeEach(async ({ page }) => {
        testMission = createTestMission();
        await page.goto('/missions');
    });

    test('should display mission list', async ({ page }) => {
        await expect(page.locator('[data-testid="mission-list"]')).toBeVisible();
        await expect(page.locator('[data-testid="mission-list-item"]')).toHaveCount(0);
    });

    test('should create new mission', async ({ page }) => {
        await page.click('[data-testid="create-mission-button"]');
        await fillMissionForm(page, testMission);
        await page.click('[data-testid="submit-mission"]');

        // Verify success notification
        await expect(page.locator('[data-testid="success-notification"]')).toBeVisible();
        await expect(page.locator('[data-testid="success-notification"]'))
            .toContainText('Mission created successfully');

        // Verify mission appears in list
        await expect(page.locator(`[data-testid="mission-${testMission.id}"]`)).toBeVisible();
    });

    test('should validate required fields', async ({ page }) => {
        await page.click('[data-testid="create-mission-button"]');
        await page.click('[data-testid="submit-mission"]');

        // Verify validation messages
        await expect(page.locator('[data-testid="name-error"]')).toBeVisible();
        await expect(page.locator('[data-testid="type-error"]')).toBeVisible();
        await expect(page.locator('[data-testid="coverage-error"]')).toBeVisible();
    });

    test('should update mission status', async ({ page }) => {
        // Create test mission first
        await page.click('[data-testid="create-mission-button"]');
        await fillMissionForm(page, testMission);
        await page.click('[data-testid="submit-mission"]');

        // Update status
        await page.click(`[data-testid="mission-${testMission.id}-status"]`);
        await page.selectOption('[data-testid="status-select"]', MissionStatusCodes.IN_PROGRESS);
        await page.click('[data-testid="update-status"]');

        // Verify status update
        await expect(page.locator(`[data-testid="mission-${testMission.id}-status"]`))
            .toContainText('In Progress');
    });

    test('should handle mission deletion', async ({ page }) => {
        // Create test mission first
        await page.click('[data-testid="create-mission-button"]');
        await fillMissionForm(page, testMission);
        await page.click('[data-testid="submit-mission"]');

        // Delete mission
        await page.click(`[data-testid="mission-${testMission.id}-delete"]`);
        await page.click('[data-testid="confirm-delete"]');

        // Verify mission removed from list
        await expect(page.locator(`[data-testid="mission-${testMission.id}"]`)).not.toBeVisible();
    });

    test('should filter missions by type', async ({ page }) => {
        // Create missions of different types
        const surveyMission = createTestMission({ type: MissionTypes.SURVEY });
        const treatmentMission = createTestMission({ type: MissionTypes.TREATMENT });

        // Add test missions
        for (const mission of [surveyMission, treatmentMission]) {
            await page.click('[data-testid="create-mission-button"]');
            await fillMissionForm(page, mission);
            await page.click('[data-testid="submit-mission"]');
        }

        // Filter by survey type
        await page.selectOption('[data-testid="type-filter"]', MissionTypes.SURVEY);
        await expect(page.locator(`[data-testid="mission-${surveyMission.id}"]`)).toBeVisible();
        await expect(page.locator(`[data-testid="mission-${treatmentMission.id}"]`)).not.toBeVisible();
    });

    test('should handle network errors gracefully', async ({ page }) => {
        // Mock network error
        await page.route(API_ENDPOINTS.MISSIONS.CREATE, route => 
            route.fulfill({ status: 500 }));

        await page.click('[data-testid="create-mission-button"]');
        await fillMissionForm(page, testMission);
        await page.click('[data-testid="submit-mission"]');

        // Verify error handling
        await expect(page.locator('[data-testid="error-notification"]')).toBeVisible();
        await expect(page.locator('[data-testid="error-notification"]'))
            .toContainText('Failed to create mission');
    });

    test('should meet accessibility requirements', async ({ page }) => {
        // Check ARIA labels
        await expect(page.locator('[data-testid="create-mission-button"]'))
            .toHaveAttribute('aria-label', 'Create new mission');

        // Verify keyboard navigation
        await page.keyboard.press('Tab');
        await expect(page.locator('[data-testid="create-mission-button"]')).toBeFocused();
    });

    test('should handle mission details view', async ({ page }) => {
        // Create test mission
        await page.click('[data-testid="create-mission-button"]');
        await fillMissionForm(page, testMission);
        await page.click('[data-testid="submit-mission"]');

        // View mission details
        await page.click(`[data-testid="mission-${testMission.id}-details"]`);

        // Verify details display
        await expect(page.locator('[data-testid="mission-details-name"]'))
            .toContainText(testMission.name);
        await expect(page.locator('[data-testid="mission-details-type"]'))
            .toContainText(testMission.type);
        await expect(page.locator('[data-testid="mission-details-status"]'))
            .toContainText(testMission.status);
    });
});