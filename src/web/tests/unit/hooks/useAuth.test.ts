import { renderHook, act } from '@testing-library/react-hooks';
import { describe, it, expect, jest, beforeEach, afterEach } from '@jest/globals';
import { waitFor } from '@testing-library/react';
import React from 'react';
import { useAuth } from '../../src/hooks/useAuth';
import { AuthContext } from '../../src/contexts/AuthContext';
import { UserRole } from '../../src/interfaces/IUser';

// Security-related constants for testing
const TEST_TOKEN_EXPIRY = 3600 * 1000; // 1 hour
const TEST_SESSION_TIMEOUT = 1800 * 1000; // 30 minutes
const TEST_REFRESH_BUFFER = 300 * 1000; // 5 minutes

// Mock user with different role configurations
const mockAdminUser = {
  id: 'admin-123',
  username: 'admin',
  email: 'admin@system.com',
  role: UserRole.ADMINISTRATOR,
  permissions: ['*'],
  firstName: 'Admin',
  lastName: 'User',
  lastLogin: new Date(),
};

const mockOperatorUser = {
  id: 'operator-123',
  username: 'operator',
  email: 'operator@system.com',
  role: UserRole.OPERATOR,
  permissions: ['MISSION_CONTROL', 'FLEET_MANAGEMENT'],
  firstName: 'Operator',
  lastName: 'User',
  lastLogin: new Date(),
};

// Mock AuthContext value
const mockAuthContext = {
  state: {
    user: null,
    isAuthenticated: false,
    isLoading: false,
    accessToken: null,
    refreshToken: null,
    sessionExpiry: null,
    sessionFingerprint: null,
    failedAttempts: 0,
    isLocked: false,
  },
  login: jest.fn(),
  logout: jest.fn(),
  getAccessToken: jest.fn(),
  refreshSession: jest.fn(),
  checkPermission: jest.fn(),
  validateSession: jest.fn(),
  getAuthMetrics: jest.fn(),
};

// Test wrapper component
const wrapper = ({ children }: { children: React.ReactNode }) => (
  <AuthContext.Provider value={mockAuthContext}>
    {children}
  </AuthContext.Provider>
);

describe('useAuth Hook', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockAuthContext.state = {
      ...mockAuthContext.state,
      user: null,
      isAuthenticated: false,
    };
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  describe('OAuth Authentication Flow', () => {
    it('should handle successful login flow', async () => {
      const { result } = renderHook(() => useAuth(), { wrapper });
      
      mockAuthContext.login.mockResolvedValueOnce(undefined);
      mockAuthContext.state.user = mockOperatorUser;
      mockAuthContext.state.isAuthenticated = true;

      await act(async () => {
        await result.current.login('operator', 'password123');
      });

      expect(mockAuthContext.login).toHaveBeenCalledWith('operator', 'password123');
      expect(result.current.isAuthenticated).toBe(true);
      expect(result.current.user).toEqual(mockOperatorUser);
    });

    it('should handle failed login attempts', async () => {
      const { result } = renderHook(() => useAuth(), { wrapper });
      
      mockAuthContext.login.mockRejectedValueOnce(new Error('Invalid credentials'));

      await expect(
        act(async () => {
          await result.current.login('invalid', 'wrong');
        })
      ).rejects.toThrow('Invalid credentials');

      expect(mockAuthContext.login).toHaveBeenCalledWith('invalid', 'wrong');
      expect(result.current.isAuthenticated).toBe(false);
      expect(result.current.user).toBeNull();
    });

    it('should handle logout process', async () => {
      const { result } = renderHook(() => useAuth(), { wrapper });
      
      mockAuthContext.logout.mockResolvedValueOnce(undefined);

      await act(async () => {
        await result.current.logout();
      });

      expect(mockAuthContext.logout).toHaveBeenCalled();
      expect(result.current.isAuthenticated).toBe(false);
      expect(result.current.user).toBeNull();
    });
  });

  describe('Token Management', () => {
    it('should handle token refresh', async () => {
      const { result } = renderHook(() => useAuth(), { wrapper });
      
      mockAuthContext.refreshSession.mockResolvedValueOnce(undefined);
      mockAuthContext.state.user = mockOperatorUser;

      await act(async () => {
        await result.current.refreshSession();
      });

      expect(mockAuthContext.refreshSession).toHaveBeenCalled();
    });

    it('should validate session state', async () => {
      const { result } = renderHook(() => useAuth(), { wrapper });
      
      mockAuthContext.validateSession.mockResolvedValueOnce(undefined);
      mockAuthContext.state.user = mockOperatorUser;
      mockAuthContext.state.isAuthenticated = true;

      await act(async () => {
        await result.current.validateSession();
      });

      expect(mockAuthContext.validateSession).toHaveBeenCalled();
    });
  });

  describe('Role-Based Access Control', () => {
    it('should validate administrator privileges', () => {
      mockAuthContext.state.user = mockAdminUser;
      const { result } = renderHook(() => useAuth(), { wrapper });

      expect(result.current.hasRole(UserRole.ADMINISTRATOR)).toBe(true);
      expect(result.current.hasRole(UserRole.OPERATOR)).toBe(true);
      expect(result.current.hasPermission('ANY_PERMISSION')).toBe(true);
    });

    it('should validate operator permissions', () => {
      mockAuthContext.state.user = mockOperatorUser;
      const { result } = renderHook(() => useAuth(), { wrapper });

      expect(result.current.hasRole(UserRole.OPERATOR)).toBe(true);
      expect(result.current.hasRole(UserRole.ADMINISTRATOR)).toBe(false);
      expect(result.current.hasPermission('MISSION_CONTROL')).toBe(true);
      expect(result.current.hasPermission('SYSTEM_CONFIG')).toBe(false);
    });

    it('should handle unauthorized access attempts', () => {
      mockAuthContext.state.user = mockOperatorUser;
      const { result } = renderHook(() => useAuth(), { wrapper });

      expect(result.current.hasRole(UserRole.ANALYST)).toBe(false);
      expect(result.current.hasPermission('ADMIN_SETTINGS')).toBe(false);
    });
  });

  describe('Security Event Logging', () => {
    it('should log authentication events', async () => {
      const { result } = renderHook(() => useAuth(), { wrapper });
      
      await act(async () => {
        await result.current.login('operator', 'password123');
      });

      // Verify security event was logged
      expect(result.current.logSecurityEvent).toHaveBeenCalledWith(
        expect.objectContaining({
          type: 'LOGIN',
          success: true,
        })
      );
    });

    it('should log permission checks', () => {
      mockAuthContext.state.user = mockOperatorUser;
      const { result } = renderHook(() => useAuth(), { wrapper });

      act(() => {
        result.current.hasPermission('MISSION_CONTROL');
      });

      expect(result.current.logSecurityEvent).toHaveBeenCalledWith(
        expect.objectContaining({
          type: 'PERMISSION_CHECK',
          success: true,
          details: 'Required permission: MISSION_CONTROL'
        })
      );
    });

    it('should enforce rate limiting on security events', () => {
      const { result } = renderHook(() => useAuth(), { wrapper });

      // Generate events beyond rate limit
      for (let i = 0; i < 105; i++) {
        act(() => {
          result.current.logSecurityEvent({
            type: 'PERMISSION_CHECK',
            timestamp: Date.now(),
            success: true,
          });
        });
      }

      // Verify last few events were blocked
      expect(console.error).toHaveBeenCalledWith('Security event rate limit exceeded');
    });
  });
});