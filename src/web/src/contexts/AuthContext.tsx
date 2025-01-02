import React, { createContext, useCallback, useEffect, useMemo, useState } from 'react';
import { ReactKeycloak, useKeycloak } from '@react-keycloak/web';
import axios from 'axios'; // v1.4.0
import jwtDecode from 'jwt-decode'; // v3.1.2
import { IUser, UserRole } from '../interfaces/IUser';

// Security constants
const MAX_LOGIN_ATTEMPTS = 3;
const LOCKOUT_DURATION = 15 * 60 * 1000; // 15 minutes
const TOKEN_REFRESH_BUFFER = 60 * 1000; // 1 minute
const SESSION_TIMEOUT = 30 * 60 * 1000; // 30 minutes

interface AuthContextState {
  user: IUser | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  accessToken: string | null;
  refreshToken: string | null;
  sessionExpiry: number | null;
  sessionFingerprint: string | null;
  failedAttempts: number;
  isLocked: boolean;
}

interface AuthMetrics {
  lastActivity: number;
  loginAttempts: number;
  sessionDuration: number;
  tokenRefreshCount: number;
}

interface AuthContextValue {
  state: AuthContextState;
  login: (username: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  getAccessToken: () => Promise<string>;
  refreshSession: () => Promise<void>;
  checkPermission: (resource: string) => Promise<boolean>;
  validateSession: () => Promise<void>;
  getAuthMetrics: () => AuthMetrics;
}

const defaultState: AuthContextState = {
  user: null,
  isAuthenticated: false,
  isLoading: true,
  accessToken: null,
  refreshToken: null,
  sessionExpiry: null,
  sessionFingerprint: null,
  failedAttempts: 0,
  isLocked: false,
};

export const AuthContext = createContext<AuthContextValue>({
  state: defaultState,
  login: async () => {},
  logout: async () => {},
  getAccessToken: async () => '',
  refreshSession: async () => {},
  checkPermission: async () => false,
  validateSession: async () => {},
  getAuthMetrics: () => ({
    lastActivity: 0,
    loginAttempts: 0,
    sessionDuration: 0,
    tokenRefreshCount: 0,
  }),
});

const generateSessionFingerprint = (): string => {
  const userAgent = window.navigator.userAgent;
  const screenRes = `${window.screen.width}x${window.screen.height}`;
  const timestamp = Date.now();
  return btoa(`${userAgent}|${screenRes}|${timestamp}`);
};

const validateTokenClaims = (token: string): boolean => {
  try {
    const decoded = jwtDecode<any>(token);
    return (
      decoded.exp * 1000 > Date.now() &&
      decoded.aud === process.env.REACT_APP_KEYCLOAK_CLIENT_ID
    );
  } catch {
    return false;
  }
};

export const AuthProvider: React.FC<React.PropsWithChildren<{}>> = ({ children }) => {
  const { keycloak } = useKeycloak();
  const [state, setState] = useState<AuthContextState>(defaultState);
  const [metrics, setMetrics] = useState<AuthMetrics>({
    lastActivity: 0,
    loginAttempts: 0,
    sessionDuration: 0,
    tokenRefreshCount: 0,
  });

  // Configure axios interceptors for token management
  useEffect(() => {
    const interceptor = axios.interceptors.request.use(
      async (config) => {
        if (state.accessToken) {
          config.headers.Authorization = `Bearer ${state.accessToken}`;
        }
        return config;
      },
      (error) => Promise.reject(error)
    );

    return () => {
      axios.interceptors.request.eject(interceptor);
    };
  }, [state.accessToken]);

  // Session monitoring
  useEffect(() => {
    let sessionCheckInterval: NodeJS.Timeout;
    
    if (state.isAuthenticated) {
      sessionCheckInterval = setInterval(() => {
        validateSession();
      }, 30000); // Check every 30 seconds
    }

    return () => {
      if (sessionCheckInterval) {
        clearInterval(sessionCheckInterval);
      }
    };
  }, [state.isAuthenticated]);

  const login = useCallback(async (username: string, password: string): Promise<void> => {
    try {
      if (state.isLocked) {
        throw new Error('Account is temporarily locked. Please try again later.');
      }

      setState((prev) => ({ ...prev, isLoading: true }));

      const response = await keycloak.login({
        username,
        password,
        scope: 'openid profile email',
      });

      if (!response || !validateTokenClaims(response.access_token)) {
        throw new Error('Invalid authentication response');
      }

      const decodedToken = jwtDecode<any>(response.access_token);
      const user: IUser = {
        id: decodedToken.sub,
        username: decodedToken.preferred_username,
        email: decodedToken.email,
        role: decodedToken.realm_access.roles.includes('admin') 
          ? UserRole.ADMINISTRATOR 
          : UserRole.OPERATOR,
        permissions: decodedToken.realm_access.roles,
        firstName: decodedToken.given_name,
        lastName: decodedToken.family_name,
        lastLogin: new Date(),
      };

      const sessionFingerprint = generateSessionFingerprint();

      setState({
        user,
        isAuthenticated: true,
        isLoading: false,
        accessToken: response.access_token,
        refreshToken: response.refresh_token,
        sessionExpiry: Date.now() + SESSION_TIMEOUT,
        sessionFingerprint,
        failedAttempts: 0,
        isLocked: false,
      });

      setMetrics((prev) => ({
        ...prev,
        lastActivity: Date.now(),
        loginAttempts: prev.loginAttempts + 1,
      }));

    } catch (error) {
      setState((prev) => ({
        ...prev,
        isLoading: false,
        failedAttempts: prev.failedAttempts + 1,
        isLocked: prev.failedAttempts + 1 >= MAX_LOGIN_ATTEMPTS,
      }));

      if (state.failedAttempts + 1 >= MAX_LOGIN_ATTEMPTS) {
        setTimeout(() => {
          setState((prev) => ({
            ...prev,
            failedAttempts: 0,
            isLocked: false,
          }));
        }, LOCKOUT_DURATION);
      }

      throw error;
    }
  }, [keycloak, state.isLocked, state.failedAttempts]);

  const logout = useCallback(async (): Promise<void> => {
    try {
      await keycloak.logout();
      setState(defaultState);
      setMetrics((prev) => ({
        ...prev,
        lastActivity: Date.now(),
        sessionDuration: 0,
      }));
    } catch (error) {
      console.error('Logout failed:', error);
      throw error;
    }
  }, [keycloak]);

  const refreshSession = useCallback(async (): Promise<void> => {
    try {
      if (!state.refreshToken) {
        throw new Error('No refresh token available');
      }

      const response = await keycloak.updateToken(TOKEN_REFRESH_BUFFER);

      if (!response || !validateTokenClaims(response.access_token)) {
        throw new Error('Invalid token refresh response');
      }

      setState((prev) => ({
        ...prev,
        accessToken: response.access_token,
        refreshToken: response.refresh_token,
        sessionExpiry: Date.now() + SESSION_TIMEOUT,
      }));

      setMetrics((prev) => ({
        ...prev,
        lastActivity: Date.now(),
        tokenRefreshCount: prev.tokenRefreshCount + 1,
      }));
    } catch (error) {
      await logout();
      throw error;
    }
  }, [keycloak, state.refreshToken, logout]);

  const checkPermission = useCallback(async (resource: string): Promise<boolean> => {
    if (!state.user || !state.accessToken) {
      return false;
    }

    return state.user.permissions.includes(resource) ||
           state.user.role === UserRole.ADMINISTRATOR;
  }, [state.user, state.accessToken]);

  const validateSession = useCallback(async (): Promise<void> => {
    if (!state.isAuthenticated || !state.sessionExpiry) {
      return;
    }

    const currentFingerprint = generateSessionFingerprint();
    if (currentFingerprint !== state.sessionFingerprint) {
      await logout();
      throw new Error('Invalid session fingerprint');
    }

    if (Date.now() >= state.sessionExpiry) {
      await logout();
      throw new Error('Session expired');
    }

    if (state.accessToken && !validateTokenClaims(state.accessToken)) {
      await refreshSession();
    }
  }, [state.isAuthenticated, state.sessionExpiry, state.sessionFingerprint, 
      state.accessToken, logout, refreshSession]);

  const getAccessToken = useCallback(async (): Promise<string> => {
    await validateSession();
    if (!state.accessToken) {
      throw new Error('No access token available');
    }
    return state.accessToken;
  }, [state.accessToken, validateSession]);

  const getAuthMetrics = useCallback((): AuthMetrics => metrics, [metrics]);

  const contextValue = useMemo(
    () => ({
      state,
      login,
      logout,
      getAccessToken,
      refreshSession,
      checkPermission,
      validateSession,
      getAuthMetrics,
    }),
    [state, login, logout, getAccessToken, refreshSession, 
     checkPermission, validateSession, getAuthMetrics]
  );

  return (
    <AuthContext.Provider value={contextValue}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = React.useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};