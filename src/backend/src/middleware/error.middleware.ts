import { Request, Response, NextFunction } from 'express';  // v4.18.2
import now from 'performance-now';  // v2.1.0
import { logger } from '../utils/logger';
import { ErrorCodes, ErrorMessages, ErrorCodeToHttpStatus } from '../constants/errorCodes';
import crypto from 'crypto';

// Enhanced error interface with security context and performance tracking
interface CustomError extends Error {
  code?: number;
  status?: number;
  details?: any;
  correlationId?: string;
  timestamp?: number;
  performanceMetrics?: {
    startTime: number;
    duration: number;
    resourceUsage: {
      cpuUsage?: number;
      memoryUsage?: number;
    };
  };
}

// Standardized error response structure
interface ErrorResponse {
  code: number;
  status: number;
  message: string;
  correlationId: string;
  timestamp: number;
  details?: any;
}

// PII patterns for sanitization
const PII_PATTERNS = {
  email: /[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}/g,
  phone: /\b\d{3}[-.]?\d{3}[-.]?\d{4}\b/g,
  ssn: /\b\d{3}-\d{2}-\d{4}\b/g,
  creditCard: /\b\d{4}[- ]?\d{4}[- ]?\d{4}[- ]?\d{4}\b/g,
  coordinates: /\b-?\d+\.\d+,\s*-?\d+\.\d+\b/g
};

/**
 * Sanitizes error details to remove sensitive information
 * @param details Error details to sanitize
 * @returns Sanitized error details
 */
const sanitizeErrorDetails = (details: any): any => {
  if (!details) return details;

  const sanitized = JSON.parse(JSON.stringify(details));
  
  // Recursively traverse object and sanitize strings
  const sanitizeObject = (obj: any) => {
    for (const key in obj) {
      if (typeof obj[key] === 'string') {
        // Apply PII pattern sanitization
        Object.values(PII_PATTERNS).forEach(pattern => {
          obj[key] = obj[key].replace(pattern, '[REDACTED]');
        });
        
        // Remove potential stack traces
        if (key.toLowerCase().includes('stack')) {
          obj[key] = '[REDACTED]';
        }
      } else if (typeof obj[key] === 'object' && obj[key] !== null) {
        sanitizeObject(obj[key]);
      }
    }
  };

  sanitizeObject(sanitized);
  return sanitized;
};

/**
 * Enhanced Express error handling middleware with security context and performance monitoring
 */
const errorHandler = (
  error: Error | CustomError,
  req: Request,
  res: Response,
  next: NextFunction
): Response => {
  const startTime = now();
  
  // Generate correlation ID if not exists
  const correlationId = (error as CustomError).correlationId || 
    req.headers['x-correlation-id'] as string || 
    crypto.randomBytes(16).toString('hex');

  // Create security context
  const securityContext = {
    userId: req.user?.id || 'anonymous',
    accessLevel: req.user?.role || 'public',
    ipAddress: req.ip,
    encryptionLevel: req.secure ? 'TLS' : 'none'
  };

  // Determine error code and status
  const errorCode = (error as CustomError).code || ErrorCodes.INTERNAL_SERVER_ERROR;
  const httpStatus = ErrorCodeToHttpStatus[errorCode] || 500;

  // Track performance metrics
  const performanceMetrics = {
    startTime,
    duration: now() - startTime,
    resourceUsage: {
      cpuUsage: process.cpuUsage().user,
      memoryUsage: process.memoryUsage().heapUsed
    }
  };

  // Sanitize error details
  const sanitizedDetails = sanitizeErrorDetails((error as CustomError).details);

  // Prepare error response
  const errorResponse: ErrorResponse = {
    code: errorCode,
    status: httpStatus,
    message: ErrorMessages[errorCode] || error.message,
    correlationId,
    timestamp: Date.now(),
    details: sanitizedDetails
  };

  // Log error with security context
  logger.error(error, {
    correlationId,
    component: 'ErrorMiddleware',
    operation: 'handleError',
    securityContext,
    performanceMetrics,
    details: {
      path: req.path,
      method: req.method,
      errorCode,
      errorMessage: error.message,
      stack: process.env.NODE_ENV === 'development' ? error.stack : undefined
    }
  });

  // Remove sensitive information from response in production
  if (process.env.NODE_ENV === 'production') {
    delete errorResponse.details;
    if (httpStatus === 500) {
      errorResponse.message = ErrorMessages[ErrorCodes.INTERNAL_SERVER_ERROR];
    }
  }

  return res.status(httpStatus).json(errorResponse);
};

export default errorHandler;