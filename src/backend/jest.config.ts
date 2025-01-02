import type { Config } from '@jest/types'; // version 29.5.0

const jestConfig: Config.InitialOptions = {
  // Use ts-jest as the default preset for TypeScript testing
  preset: 'ts-jest',

  // Set Node.js as the testing environment
  testEnvironment: 'node',

  // Define root directories for tests and source files
  roots: [
    '<rootDir>/src',
    '<rootDir>/tests'
  ],

  // Pattern matching for test files
  testMatch: [
    '**/__tests__/**/*.+(ts|tsx)',
    '**/?(*.)+(spec|test).+(ts|tsx)'
  ],

  // TypeScript transformation configuration
  transform: {
    '^.+\\.(ts|tsx)$': 'ts-jest'
  },

  // Module path aliases for cleaner imports
  moduleNameMapper: {
    '@/(.*)': '<rootDir>/src/$1'
  },

  // Setup files to run before tests
  setupFilesAfterEnv: [
    '<rootDir>/tests/setup.ts'
  ],

  // Coverage configuration
  coverageDirectory: '<rootDir>/coverage',
  coverageReporters: [
    'text',
    'lcov',
    'json',
    'html'
  ],
  collectCoverageFrom: [
    'src/**/*.{ts,tsx}',
    '!src/**/*.d.ts',
    '!src/types/**/*'
  ],
  coverageThreshold: {
    global: {
      branches: 80,
      functions: 80,
      lines: 80,
      statements: 80
    }
  },

  // Paths to ignore during testing
  testPathIgnorePatterns: [
    '/node_modules/',
    '/dist/'
  ],

  // Supported file extensions
  moduleFileExtensions: [
    'ts',
    'tsx',
    'js',
    'jsx',
    'json'
  ],

  // TypeScript configuration
  globals: {
    'ts-jest': {
      tsconfig: '<rootDir>/tsconfig.json'
    }
  },

  // Enable verbose output for detailed test results
  verbose: true,

  // Set timeout for long-running tests (30 seconds)
  testTimeout: 30000,

  // Limit parallel test execution to 50% of available CPU cores
  maxWorkers: '50%'
};

export default jestConfig;