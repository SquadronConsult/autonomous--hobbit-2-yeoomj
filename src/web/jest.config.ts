/**
 * @fileoverview Jest configuration for Agricultural Management System web frontend
 * Configures comprehensive testing environment for React components with real-time features
 * @version 1.0.0
 */

import type { Config } from '@jest/types';

/**
 * Creates and exports the Jest configuration object with comprehensive testing setup
 */
const config: Config.InitialOptions = {
  // Use jsdom for DOM testing environment
  testEnvironment: 'jsdom',

  // Global test setup configuration
  setupFilesAfterEnv: ['<rootDir>/tests/setup.ts'],

  // Module resolution configuration
  moduleNameMapper: {
    // Handle static asset imports
    '\\.(css|less|sass|scss)$': 'identity-obj-proxy',
    '\\.(gif|ttf|eot|svg|png|jpg|jpeg)$': '<rootDir>/tests/__mocks__/fileMock.ts',
    
    // Handle path aliases from tsconfig
    '^@/(.*)$': '<rootDir>/src/$1',
    '^@components/(.*)$': '<rootDir>/src/components/$1',
    '^@constants/(.*)$': '<rootDir>/src/constants/$1',
    '^@services/(.*)$': '<rootDir>/src/services/$1',
    '^@utils/(.*)$': '<rootDir>/src/utils/$1',
    '^@hooks/(.*)$': '<rootDir>/src/hooks/$1'
  },

  // File pattern matching
  testMatch: [
    '<rootDir>/src/**/__tests__/**/*.{ts,tsx}',
    '<rootDir>/src/**/*.{spec,test}.{ts,tsx}'
  ],

  // Transform configuration
  transform: {
    '^.+\\.tsx?$': ['ts-jest', {
      tsconfig: '<rootDir>/tsconfig.json',
      diagnostics: {
        warnOnly: true
      }
    }]
  },

  // Coverage configuration
  collectCoverage: true,
  collectCoverageFrom: [
    'src/**/*.{ts,tsx}',
    '!src/**/*.d.ts',
    '!src/vite-env.d.ts',
    '!src/main.tsx',
    '!src/**/*.stories.{ts,tsx}',
    '!src/**/__mocks__/**'
  ],
  coverageDirectory: 'coverage',
  coverageReporters: ['text', 'lcov', 'html'],
  coverageThreshold: {
    global: {
      statements: 80,
      branches: 75,
      functions: 80,
      lines: 80
    }
  },

  // Test execution configuration
  testTimeout: 10000,
  maxWorkers: '50%',
  maxConcurrency: 5,

  // Reporter configuration
  reporters: [
    'default',
    ['jest-junit', {
      outputDirectory: 'reports/junit',
      outputName: 'jest-junit.xml',
      classNameTemplate: '{classname}',
      titleTemplate: '{title}',
      ancestorSeparator: ' › ',
      usePathForSuiteName: true
    }]
  ],

  // Global configuration
  globals: {
    'ts-jest': {
      isolatedModules: true,
      diagnostics: {
        warnOnly: true
      }
    }
  },

  // Module file extensions
  moduleFileExtensions: ['ts', 'tsx', 'js', 'jsx', 'json', 'node'],

  // Test environment configuration
  testEnvironmentOptions: {
    url: 'http://localhost',
    customExportConditions: [''],
    testURL: 'http://localhost'
  },

  // Retry configuration for flaky tests
  retry: 2,

  // Clear mocks between tests
  clearMocks: true,
  resetMocks: false,
  restoreMocks: false,

  // Verbose output for debugging
  verbose: true,

  // Watch configuration
  watchPlugins: [
    'jest-watch-typeahead/filename',
    'jest-watch-typeahead/testname'
  ],

  // Error handling
  bail: 0,
  errorOnDeprecated: true
};

export default config;