import { Request, Response, NextFunction, RequestHandler, ErrorRequestHandler } from 'express';
import { Keycloak } from 'keycloak-connect';  // v21.1.1
import { RateLimiterMemory } from 'rate-limiter-flexible';  // v2.4.1
import { securityConfig } from '../config/security';
import { verifyToken } from '../utils/security';
import { ErrorCodes } from '../constants/errorCodes';
import { SecurityLogger } from '../utils/logger';

// Extend Express Request interface to include authenticated user information
interface AuthenticatedRequest extends Request {
  user?: {
    id: string;
    role: string;
    permissions: string[];
    tokenExp: number;
    refreshToken: string;
  };
  securityContext?: {
    ipAddress: string;
    userAgent: string;
    sessionId: string;
  };
}

// Constants
const ALLOWED_ROLES = ['administrator', 'operator', 'analyst', 'service'];
const ROLE_HIERARCHY = {
  administrator: ['operator', 'analyst', 'service'],
  operator: ['analyst'],
  analyst: [],
  service: []
};
const TOKEN_HEADER = 'Authorization';
const TOKEN_PREFIX = 'Bearer ';
const MAX_LOGIN_ATTEMPTS = 3;
const LOCKOUT_DURATION = 900; // 15 minutes in seconds

// Initialize Keycloak instance
const keycloak = new Keycloak({
  realm: securityConfig.auth.keycloak.realm,
  'auth-server-url': securityConfig.auth.keycloak.authServerUrl,
  resource: securityConfig.auth.keycloak.resource,
  credentials: securityConfig.auth.keycloak.credentials
});

// Initialize rate limiter
const rateLimiter = new RateLimiterMemory({
  points: securityConfig.validation.rateLimiting.max,
  duration: securityConfig.validation.rateLimiting.windowMs / 1000
});

// Authentication middleware
export const authenticateRequest = async (
  req: AuthenticatedRequest,
  res: Response,
  next: NextFunction
): Promise<void> => {
  try {
    // Rate limiting check
    const clientIp = req.ip;
    await rateLimiter.consume(clientIp);

    // Extract token from header
    const authHeader = req.headers[TOKEN_HEADER.toLowerCase()];
    if (!authHeader || typeof authHeader !== 'string') {
      throw new Error('Missing or invalid authorization header');
    }

    const token = authHeader.startsWith(TOKEN_PREFIX) 
      ? authHeader.slice(TOKEN_PREFIX.length) 
      : authHeader;

    // Verify token and decode payload
    const decodedToken = await verifyToken(token);

    // Attach user information to request
    req.user = {
      id: decodedToken.userId,
      role: decodedToken.role,
      permissions: decodedToken.permissions,
      tokenExp: decodedToken.exp,
      refreshToken: token
    };

    // Attach security context
    req.securityContext = {
      ipAddress: clientIp,
      userAgent: req.headers['user-agent'] || 'unknown',
      sessionId: req.sessionID || 'unknown'
    };

    // Log successful authentication
    SecurityLogger.logSecurityEvent('Authentication successful', {
      userId: req.user.id,
      role: req.user.role,
      ipAddress: clientIp
    });

    next();
  } catch (error) {
    // Handle rate limiting errors
    if (error.name === 'RateLimiterError') {
      SecurityLogger.logSecurityEvent('Rate limit exceeded', {
        ipAddress: req.ip,
        endpoint: req.path
      });
      res.status(429).json({
        code: ErrorCodes.AUTHENTICATION_ERROR,
        message: 'Too many requests, please try again later'
      });
      return;
    }

    // Handle authentication errors
    SecurityLogger.logAuthFailure(error as Error, {
      ipAddress: req.ip,
      attempt: req.path
    });

    res.status(401).json({
      code: ErrorCodes.AUTHENTICATION_ERROR,
      message: 'Authentication failed'
    });
  }
};

// Role-based authorization middleware factory
export const authorizeRole = (
  allowedRoles: string[],
  requiredPermissions: string[] = []
): RequestHandler => {
  return (req: AuthenticatedRequest, res: Response, next: NextFunction) => {
    try {
      const user = req.user;
      if (!user) {
        throw new Error('User not authenticated');
      }

      // Check role authorization
      const hasValidRole = allowedRoles.some(role => {
        if (user.role === role) return true;
        return ROLE_HIERARCHY[user.role]?.includes(role) || false;
      });

      if (!hasValidRole) {
        throw new Error('Insufficient role permissions');
      }

      // Check specific permissions if required
      if (requiredPermissions.length > 0) {
        const hasRequiredPermissions = requiredPermissions.every(
          permission => user.permissions.includes(permission)
        );
        if (!hasRequiredPermissions) {
          throw new Error('Missing required permissions');
        }
      }

      // Log successful authorization
      SecurityLogger.logSecurityEvent('Authorization successful', {
        userId: user.id,
        role: user.role,
        permissions: requiredPermissions
      });

      next();
    } catch (error) {
      SecurityLogger.logAuthFailure(error as Error, {
        userId: req.user?.id,
        role: req.user?.role,
        requiredRoles: allowedRoles,
        requiredPermissions
      });

      res.status(403).json({
        code: ErrorCodes.AUTHORIZATION_ERROR,
        message: 'Unauthorized access'
      });
    }
  };
};

// Enhanced Keycloak error handler
export const handleKeycloakFailure: ErrorRequestHandler = (
  error: Error,
  req: AuthenticatedRequest,
  res: Response,
  next: NextFunction
): void => {
  SecurityLogger.logAuthFailure(error, {
    component: 'keycloak',
    ipAddress: req.ip,
    endpoint: req.path
  });

  // Check for specific Keycloak error types
  if (error.name === 'TokenExpiredError') {
    res.status(401).json({
      code: ErrorCodes.AUTHENTICATION_ERROR,
      message: 'Token expired, please refresh'
    });
    return;
  }

  res.status(500).json({
    code: ErrorCodes.INTERNAL_SERVER_ERROR,
    message: 'Authentication service error'
  });
};