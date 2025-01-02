import winston from 'winston';  // v3.10.0
import DailyRotateFile from 'winston-daily-rotate-file';  // v4.7.1
import { ElasticsearchTransport } from 'winston-elasticsearch';  // v0.17.4
import { loggingConfig } from '../config/logging';
import crypto from 'crypto';

// Interfaces for structured logging
interface SecurityContext {
  userId: string;
  accessLevel: string;
  ipAddress: string;
  encryptionLevel: string;
}

interface ResourceMetrics {
  cpuUsage?: number;
  memoryUsage?: number;
  ioOperations?: number;
}

interface PerformanceMetrics {
  startTime: number;
  duration: number;
  resourceUsage: ResourceMetrics;
}

interface PIIFlags {
  containsPersonalData: boolean;
  dataTypes: string[];
  encryptionRequired: boolean;
}

interface LogContext {
  correlationId: string;
  component: string;
  operation: string;
  securityContext?: SecurityContext;
  performanceMetrics?: PerformanceMetrics;
  piiFlags?: PIIFlags;
  details?: any;
}

// PII detection patterns
const PII_PATTERNS = {
  email: /[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}/g,
  phone: /\b\d{3}[-.]?\d{3}[-.]?\d{4}\b/g,
  ssn: /\b\d{3}-\d{2}-\d{4}\b/g,
  creditCard: /\b\d{4}[- ]?\d{4}[- ]?\d{4}[- ]?\d{4}\b/g
};

// Create enhanced Winston logger instance
const enhancedLogger = winston.createLogger({
  ...loggingConfig,
  format: winston.format.combine(
    winston.format.timestamp(),
    winston.format.errors({ stack: true }),
    winston.format.metadata(),
    winston.format.json()
  )
});

// Utility function to generate correlation ID
const generateCorrelationId = (): string => {
  return crypto.randomBytes(16).toString('hex');
};

// Utility function to detect and handle PII
const handlePII = (message: string, context: LogContext): { message: string; piiFlags: PIIFlags } => {
  const piiFlags: PIIFlags = {
    containsPersonalData: false,
    dataTypes: [],
    encryptionRequired: false
  };

  if (loggingConfig.security.piiFilter) {
    Object.entries(PII_PATTERNS).forEach(([type, pattern]) => {
      if (pattern.test(message)) {
        piiFlags.containsPersonalData = true;
        piiFlags.dataTypes.push(type);
        piiFlags.encryptionRequired = true;
        message = message.replace(pattern, '[REDACTED]');
      }
    });
  }

  return { message, piiFlags };
};

// Utility function to track performance metrics
const trackPerformance = (startTime: number): PerformanceMetrics => {
  const endTime = process.hrtime.bigint();
  return {
    startTime,
    duration: Number(endTime - BigInt(startTime)) / 1e6, // Convert to milliseconds
    resourceUsage: {
      memoryUsage: process.memoryUsage().heapUsed,
      cpuUsage: process.cpuUsage().user
    }
  };
};

// Enhanced logging function
const log = (level: string, message: string, context: LogContext): void => {
  const startTime = Number(process.hrtime.bigint());
  
  // Ensure correlation ID
  context.correlationId = context.correlationId || generateCorrelationId();

  // Handle PII in message
  const { message: sanitizedMessage, piiFlags } = handlePII(message, context);
  context.piiFlags = piiFlags;

  // Track performance if enabled
  if (loggingConfig.performance.batchSize > 0) {
    context.performanceMetrics = trackPerformance(startTime);
  }

  // Log with enhanced context
  enhancedLogger.log({
    level,
    message: sanitizedMessage,
    correlationId: context.correlationId,
    component: context.component,
    operation: context.operation,
    securityContext: context.securityContext,
    performanceMetrics: context.performanceMetrics,
    piiFlags: context.piiFlags,
    details: context.details,
    timestamp: new Date().toISOString()
  });
};

// Enhanced error logging function
const error = (error: Error, context: LogContext): void => {
  const errorDetails = {
    name: error.name,
    message: error.message,
    stack: loggingConfig.security.sanitization ? error.stack?.split('\n').slice(0, 3).join('\n') : error.stack
  };

  log('error', error.message, {
    ...context,
    details: errorDetails
  });
};

// Convenience methods for different log levels
const info = (message: string, context: LogContext): void => log('info', message, context);
const debug = (message: string, context: LogContext): void => log('debug', message, context);
const warn = (message: string, context: LogContext): void => log('warn', message, context);

// Export enhanced logger instance
export const logger = {
  log,
  error,
  info,
  debug,
  warn
};

export type { LogContext, SecurityContext, PerformanceMetrics, PIIFlags };