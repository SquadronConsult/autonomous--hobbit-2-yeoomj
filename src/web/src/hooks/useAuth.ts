import { useContext, useMemo } from 'react'; // v18.0.0
import { AuthContext } from '../contexts/AuthContext';
import { IUser, UserRole } from '../interfaces/IUser';

// Security constants
const RATE_LIMIT_WINDOW = 60 * 1000; // 1 minute
const MAX_SECURITY_EVENTS = 100; // Maximum security events per window
const SECURITY_LOG_RETENTION = 7 * 24 * 60 * 60 * 1000; // 7 days

// Security event types
type AuthEvent = {
  type: 'LOGIN' | 'LOGOUT' | 'TOKEN_REFRESH' | 'PERMISSION_CHECK' | 'ROLE_CHECK' | 'SESSION_VALIDATION';
  timestamp: number;
  success: boolean;
  details?: string;
};

interface UseAuthReturn {
  user: IUser | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (username: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  getAccessToken: () => Promise<string>;
  refreshSession: () => Promise<void>;
  hasRole: (requiredRole: UserRole) => boolean;
  hasPermission: (permission: string) => boolean;
  validateSession: () => Promise<void>;
  getSessionFingerprint: () => string;
  logSecurityEvent: (event: AuthEvent) => void;
}

/**
 * Enhanced custom hook for secure authentication management
 * Implements OAuth 2.0 + OpenID Connect with comprehensive security features
 * @returns {UseAuthReturn} Secure authentication state and methods
 * @throws {Error} If used outside AuthProvider context
 */
export const useAuth = (): UseAuthReturn => {
  const context = useContext(AuthContext);

  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }

  const { state, login, logout, getAccessToken, refreshSession, checkPermission, validateSession, getAuthMetrics } = context;

  // Security event logging with rate limiting
  const securityEvents: AuthEvent[] = [];
  let lastWindowStart = Date.now();

  const logSecurityEvent = (event: AuthEvent): void => {
    const now = Date.now();
    
    // Clear old events and reset window if needed
    if (now - lastWindowStart > RATE_LIMIT_WINDOW) {
      securityEvents.length = 0;
      lastWindowStart = now;
    }

    // Check rate limit
    if (securityEvents.length >= MAX_SECURITY_EVENTS) {
      console.error('Security event rate limit exceeded');
      return;
    }

    // Add event with cleanup of old events
    securityEvents.push({
      ...event,
      timestamp: now
    });

    // Cleanup old events beyond retention period
    const cutoff = now - SECURITY_LOG_RETENTION;
    while (securityEvents.length > 0 && securityEvents[0].timestamp < cutoff) {
      securityEvents.shift();
    }
  };

  /**
   * Enhanced role validation with security logging
   * @param {UserRole} requiredRole - Role to check against
   * @returns {boolean} Whether user has required role
   */
  const hasRole = useMemo(() => (requiredRole: UserRole): boolean => {
    const result = state.user?.role === requiredRole || 
                  state.user?.role === UserRole.ADMINISTRATOR;

    logSecurityEvent({
      type: 'ROLE_CHECK',
      timestamp: Date.now(),
      success: result,
      details: `Required role: ${requiredRole}`
    });

    return result;
  }, [state.user]);

  /**
   * Permission validation with security logging
   * @param {string} permission - Permission to check
   * @returns {boolean} Whether user has permission
   */
  const hasPermission = useMemo(() => (permission: string): boolean => {
    const result = state.user?.permissions.includes(permission) || 
                  state.user?.role === UserRole.ADMINISTRATOR;

    logSecurityEvent({
      type: 'PERMISSION_CHECK',
      timestamp: Date.now(),
      success: result,
      details: `Required permission: ${permission}`
    });

    return result;
  }, [state.user]);

  /**
   * Get current session fingerprint
   * @returns {string} Session fingerprint for validation
   */
  const getSessionFingerprint = (): string => {
    return state.sessionFingerprint || '';
  };

  return useMemo(() => ({
    user: state.user,
    isAuthenticated: state.isAuthenticated,
    isLoading: state.isLoading,
    login: async (username: string, password: string) => {
      try {
        await login(username, password);
        logSecurityEvent({
          type: 'LOGIN',
          timestamp: Date.now(),
          success: true
        });
      } catch (error) {
        logSecurityEvent({
          type: 'LOGIN',
          timestamp: Date.now(),
          success: false,
          details: error instanceof Error ? error.message : 'Unknown error'
        });
        throw error;
      }
    },
    logout: async () => {
      try {
        await logout();
        logSecurityEvent({
          type: 'LOGOUT',
          timestamp: Date.now(),
          success: true
        });
      } catch (error) {
        logSecurityEvent({
          type: 'LOGOUT',
          timestamp: Date.now(),
          success: false,
          details: error instanceof Error ? error.message : 'Unknown error'
        });
        throw error;
      }
    },
    getAccessToken,
    refreshSession: async () => {
      try {
        await refreshSession();
        logSecurityEvent({
          type: 'TOKEN_REFRESH',
          timestamp: Date.now(),
          success: true
        });
      } catch (error) {
        logSecurityEvent({
          type: 'TOKEN_REFRESH',
          timestamp: Date.now(),
          success: false,
          details: error instanceof Error ? error.message : 'Unknown error'
        });
        throw error;
      }
    },
    hasRole,
    hasPermission,
    validateSession: async () => {
      try {
        await validateSession();
        logSecurityEvent({
          type: 'SESSION_VALIDATION',
          timestamp: Date.now(),
          success: true
        });
      } catch (error) {
        logSecurityEvent({
          type: 'SESSION_VALIDATION',
          timestamp: Date.now(),
          success: false,
          details: error instanceof Error ? error.message : 'Unknown error'
        });
        throw error;
      }
    },
    getSessionFingerprint,
    logSecurityEvent
  }), [
    state.user,
    state.isAuthenticated,
    state.isLoading,
    state.sessionFingerprint,
    login,
    logout,
    getAccessToken,
    refreshSession,
    hasRole,
    hasPermission,
    validateSession
  ]);
};