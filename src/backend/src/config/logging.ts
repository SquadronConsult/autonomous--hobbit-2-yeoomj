import winston from 'winston';  // v3.10.0
import DailyRotateFile from 'winston-daily-rotate-file';  // v4.7.1
import { ElasticsearchTransport } from 'winston-elasticsearch';  // v0.17.4
import { format } from 'winston';

// Type definitions
type LogLevel = 'error' | 'warn' | 'info' | 'debug';
type TransportType = 'console' | 'file' | 'elasticsearch';

interface SecurityConfig {
  piiFilter: boolean;
  sanitization: boolean;
  encryption: boolean;
  audit: boolean;
  maxSensitivityLevel: number;
}

interface PerformanceConfig {
  batchSize: number;
  flushInterval: number;
  compressionLevel: number;
  bufferSize: number;
}

interface SecurityOptions {
  encryption: boolean;
  tls: boolean;
  certificateValidation: boolean;
  authRequired: boolean;
}

interface PerformanceOptions {
  batchSize: number;
  flushIntervalMs: number;
  compressionEnabled: boolean;
  bufferLimit: number;
}

interface ValidationOptions {
  validateMessages: boolean;
  maxSize: number;
  allowedSeverities: string[];
}

interface MonitoringOptions {
  enableMetrics: boolean;
  alertThresholds: Record<string, number>;
  samplingRate: number;
}

interface TransportOptions {
  level: LogLevel;
  format: winston.Logform.Format;
  filename?: string;
  maxFiles?: string;
  maxSize?: string;
  node?: string;
  index?: string;
}

interface LoggingConfig {
  level: LogLevel;
  format: winston.Logform.Format;
  transports: winston.transport[];
  exitOnError: boolean;
  silent: boolean;
  handleExceptions: boolean;
  handleRejections: boolean;
  maxListeners: number;
  security: SecurityConfig;
  performance: PerformanceConfig;
}

// Global constants
const DEFAULT_LOG_LEVEL: LogLevel = 'info';
const LOG_FILE_PATH = 'logs/agricultural-system-%DATE%.log';
const ELASTICSEARCH_NODE = process.env.ELASTICSEARCH_NODE || 'http://localhost:9200';
const MAX_LOG_SIZE = process.env.MAX_LOG_SIZE || '20m';
const LOG_RETENTION = process.env.LOG_RETENTION || '14d';
const BATCH_SIZE = Number(process.env.BATCH_SIZE || 200);
const FLUSH_INTERVAL = Number(process.env.FLUSH_INTERVAL || 5000);

// Custom format for log sanitization and PII filtering
const secureFormat = format.combine(
  format.timestamp(),
  format.errors({ stack: true }),
  format.metadata(),
  format((info) => {
    // Remove sensitive data patterns (credit cards, passwords, etc.)
    if (info.message) {
      info.message = info.message.replace(/\b\d{4}[- ]?\d{4}[- ]?\d{4}[- ]?\d{4}\b/g, '[REDACTED]');
      info.message = info.message.replace(/password\s*:\s*\S+/gi, 'password: [REDACTED]');
    }
    return info;
  })(),
  format.json()
);

// Get appropriate log level based on environment
const getLogLevel = (environment: string, securityConfig: SecurityConfig): LogLevel => {
  if (!environment || typeof environment !== 'string') {
    throw new Error('Invalid environment parameter');
  }

  let baseLevel: LogLevel;
  switch (environment.toLowerCase()) {
    case 'production':
      baseLevel = 'info';
      break;
    case 'development':
      baseLevel = 'debug';
      break;
    case 'test':
      baseLevel = 'error';
      break;
    default:
      baseLevel = DEFAULT_LOG_LEVEL;
  }

  // Apply security policies
  if (securityConfig.maxSensitivityLevel < 3) {
    return baseLevel === 'debug' ? 'info' : baseLevel;
  }

  return baseLevel;
};

// Create transport configuration based on environment
const createTransportConfig = (
  environment: string,
  securityConfig: SecurityConfig,
  performanceConfig: PerformanceConfig
): winston.transport[] => {
  const transports: winston.transport[] = [];

  // Console Transport
  transports.push(new winston.transports.Console({
    level: getLogLevel(environment, securityConfig),
    format: secureFormat
  }));

  // File Rotation Transport
  const fileTransport = new DailyRotateFile({
    level: getLogLevel(environment, securityConfig),
    filename: LOG_FILE_PATH,
    datePattern: 'YYYY-MM-DD',
    maxSize: MAX_LOG_SIZE,
    maxFiles: LOG_RETENTION,
    format: secureFormat,
    zippedArchive: true,
    // Additional security options
    options: {
      flags: 'a',
      mode: 0o600 // Restrictive file permissions
    }
  });

  transports.push(fileTransport);

  // Elasticsearch Transport (for production only)
  if (environment === 'production') {
    const elasticsearchTransport = new ElasticsearchTransport({
      level: 'info',
      clientOpts: {
        node: ELASTICSEARCH_NODE,
        auth: {
          username: process.env.ELASTICSEARCH_USERNAME,
          password: process.env.ELASTICSEARCH_PASSWORD
        },
        ssl: {
          rejectUnauthorized: true,
          ca: process.env.ELASTICSEARCH_CA
        }
      },
      bufferLimit: performanceConfig.bufferSize,
      flushInterval: performanceConfig.flushInterval,
      bulk: {
        size: performanceConfig.batchSize
      }
    });

    transports.push(elasticsearchTransport);
  }

  return transports;
};

// Export the logging configuration
export const loggingConfig: LoggingConfig = {
  level: getLogLevel(process.env.NODE_ENV || 'development', {
    piiFilter: true,
    sanitization: true,
    encryption: true,
    audit: true,
    maxSensitivityLevel: 3
  }),
  format: secureFormat,
  transports: createTransportConfig(
    process.env.NODE_ENV || 'development',
    {
      piiFilter: true,
      sanitization: true,
      encryption: true,
      audit: true,
      maxSensitivityLevel: 3
    },
    {
      batchSize: BATCH_SIZE,
      flushInterval: FLUSH_INTERVAL,
      compressionLevel: 6,
      bufferSize: 1000
    }
  ),
  exitOnError: false,
  silent: false,
  handleExceptions: true,
  handleRejections: true,
  maxListeners: 30,
  security: {
    piiFilter: true,
    sanitization: true,
    encryption: true,
    audit: true,
    maxSensitivityLevel: 3
  },
  performance: {
    batchSize: BATCH_SIZE,
    flushInterval: FLUSH_INTERVAL,
    compressionLevel: 6,
    bufferSize: 1000
  }
};

export default loggingConfig;