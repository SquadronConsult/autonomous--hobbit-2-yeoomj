import { Request, Response, NextFunction } from 'express'; // v4.18.2
import { v4 as uuidv4 } from 'uuid'; // v9.0.0
import { logger, LogContext, SecurityContext } from '../utils/logger';
import { metrics } from '../utils/metrics';
import { security } from '../utils/security';

// Extended Request interface with context
interface RequestWithContext extends Request {
  correlationId: string;
  startTime: number;
  securityContext: SecurityContext;
  customContext: Record<string, unknown>;
}

// Security context for request tracking
interface SecurityContext {
  sourceIp: string;
  userAgent: string;
  authLevel: string;
}

/**
 * Enhanced logging middleware for HTTP request/response tracking
 * Implements structured logging with correlation IDs, metrics, and PII protection
 */
const loggingMiddleware = (req: Request, res: Response, next: NextFunction): void => {
  try {
    const request = req as RequestWithContext;
    
    // Generate correlation ID and attach timing
    request.correlationId = req.headers['x-correlation-id'] as string || uuidv4();
    request.startTime = Date.now();

    // Create and attach security context
    request.securityContext = {
      sourceIp: req.ip,
      userAgent: req.headers['user-agent'] || 'unknown',
      authLevel: req.headers['authorization'] ? 'authenticated' : 'public'
    };

    // Log incoming request with PII protection
    logger.log('info', 'Incoming request', {
      correlationId: request.correlationId,
      component: 'http',
      operation: 'request',
      securityContext: request.securityContext,
      details: formatRequestLog(request)
    });

    // Track request metrics
    metrics.incrementRequestCount(req.method, req.path);

    // Override response end for logging
    const originalEnd = res.end;
    res.end = function(chunk?: any, encoding?: string | (() => void), cb?: () => void): Response {
      // Calculate request duration
      const duration = Date.now() - request.startTime;
      
      // Record request latency metrics
      metrics.recordLatency(req.method, req.path, duration);

      // Log response with performance metrics
      logger.log('info', 'Outgoing response', {
        correlationId: request.correlationId,
        component: 'http',
        operation: 'response',
        securityContext: request.securityContext,
        details: formatResponseLog(res, duration)
      });

      // Call original end
      return originalEnd.call(this, chunk, encoding as string, cb);
    };

    next();
  } catch (error) {
    // Log error with secure context
    logger.error(error as Error, {
      correlationId: (req as RequestWithContext).correlationId || uuidv4(),
      component: 'http',
      operation: 'middleware',
      securityContext: {
        sourceIp: req.ip,
        userAgent: req.headers['user-agent'] || 'unknown',
        authLevel: 'system'
      }
    });
    next(error);
  }
};

/**
 * Formats request details with PII protection for logging
 */
const formatRequestLog = (req: RequestWithContext): object => {
  // Extract and sanitize request details
  const sanitizedHeaders = security.sanitizeData(req.headers);
  const sanitizedQuery = security.sanitizeData(req.query);
  const sanitizedBody = req.body ? security.sanitizeData(req.body) : undefined;

  return {
    method: req.method,
    url: req.url,
    headers: sanitizedHeaders,
    query: sanitizedQuery,
    body: sanitizedBody,
    correlationId: req.correlationId,
    securityContext: req.securityContext,
    timestamp: new Date().toISOString()
  };
};

/**
 * Formats response details with performance metrics for logging
 */
const formatResponseLog = (res: Response, duration: number): object => {
  // Extract and sanitize response details
  const sanitizedHeaders = security.sanitizeData(res.getHeaders());

  return {
    statusCode: res.statusCode,
    headers: sanitizedHeaders,
    responseSize: parseInt(res.getHeader('content-length') as string) || 0,
    duration,
    timestamp: new Date().toISOString()
  };
};

export default loggingMiddleware;