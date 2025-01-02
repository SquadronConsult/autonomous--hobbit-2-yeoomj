/**
 * @fileoverview Defines standardized color schemes and gradients for data visualization 
 * components following Material Design 3.0 color system with WCAG 2.1 Level AA compliance.
 * @version 1.0.0
 */

/**
 * Material Design 3.0 compliant color arrays for different chart elements and states.
 * Each array provides 5 shades from darkest to lightest for flexible visualization options.
 */
export const CHART_COLORS: Record<string, string[]> = {
  // Primary blue palette - Used for main metrics and KPIs
  primary: [
    '#1976d2', // 700 - Main
    '#2196f3', // 500 - Light
    '#64b5f6', // 300 - Lighter
    '#90caf9', // 200 - Lightest
    '#bbdefb'  // 100 - Background
  ],

  // Secondary purple palette - Used for secondary metrics
  secondary: [
    '#7b1fa2', // 700 - Main
    '#9c27b0', // 500 - Light  
    '#ba68c8', // 300 - Lighter
    '#ce93d8', // 200 - Lightest
    '#e1bee7'  // 100 - Background
  ],

  // Success green palette - Used for positive indicators
  success: [
    '#388e3c', // 700 - Main
    '#4caf50', // 500 - Light
    '#81c784', // 300 - Lighter
    '#a5d6a7', // 200 - Lightest
    '#c8e6c9'  // 100 - Background
  ],

  // Warning orange palette - Used for warnings and alerts
  warning: [
    '#f57c00', // 700 - Main
    '#ff9800', // 500 - Light
    '#ffb74d', // 300 - Lighter
    '#ffcc80', // 200 - Lightest
    '#ffe0b2'  // 100 - Background
  ],

  // Error red palette - Used for errors and critical alerts
  error: [
    '#d32f2f', // 700 - Main
    '#f44336', // 500 - Light
    '#e57373', // 300 - Lighter
    '#ef9a9a', // 200 - Lightest
    '#ffcdd2'  // 100 - Background
  ]
};

/**
 * WCAG 2.1 compliant gradient color schemes for area charts and backgrounds.
 * Each gradient array contains two colors: start color (0.8 opacity) and end color (0.1 opacity).
 */
export const CHART_GRADIENTS: Record<string, string[]> = {
  // Used for performance and system metrics visualization
  performance: [
    'rgba(25, 118, 210, 0.8)',  // Primary blue with 80% opacity
    'rgba(25, 118, 210, 0.1)'   // Primary blue with 10% opacity
  ],

  // Used for battery and resource level indicators
  battery: [
    'rgba(56, 142, 60, 0.8)',   // Success green with 80% opacity
    'rgba(56, 142, 60, 0.1)'    // Success green with 10% opacity
  ],

  // Used for coverage and progress area charts
  coverage: [
    'rgba(123, 31, 162, 0.8)',  // Secondary purple with 80% opacity
    'rgba(123, 31, 162, 0.1)'   // Secondary purple with 10% opacity
  ]
};

/**
 * Standardized opacity values for different chart elements to ensure consistent visualization.
 * Values are optimized for WCAG 2.1 Level AA contrast requirements.
 */
export const CHART_OPACITY: Record<string, number> = {
  fill: 0.2,   // Background fill opacity for areas
  line: 0.8,   // Line opacity for borders and strokes
  point: 1.0   // Point opacity for data markers
};