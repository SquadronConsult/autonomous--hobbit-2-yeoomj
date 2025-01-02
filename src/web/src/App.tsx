/**
 * Root component of the Agricultural Management System web application
 * Provides global context providers, routing setup, and real-time communication
 * @version 1.0.0
 */

import React, { useEffect } from 'react';
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom';
import { ErrorBoundary } from 'react-error-boundary';

// Context Providers
import { AuthProvider, useAuth } from './contexts/AuthContext';
import { ThemeProvider } from './contexts/ThemeContext';
import { WebSocketProvider } from './contexts/WebSocketContext';

// Error Fallback Component
const ErrorFallback: React.FC<{ error: Error; resetErrorBoundary: () => void }> = ({
  error,
  resetErrorBoundary
}) => (
  <div role="alert" className="error-boundary">
    <h2>Something went wrong</h2>
    <pre>{error.message}</pre>
    <button onClick={resetErrorBoundary}>Try again</button>
  </div>
);

// Protected Route Wrapper
const ProtectedRoute: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const { state: { isAuthenticated, isLoading } } = useAuth();

  if (isLoading) {
    return <div>Loading...</div>;
  }

  return isAuthenticated ? <>{children}</> : <Navigate to="/login" replace />;
};

// WebSocket Configuration
const wsConfig = {
  reconnectLimit: 5,
  heartbeatInterval: 30000,
  messageTimeout: 5000,
  enableEncryption: true,
  rateLimit: {
    maxRequests: 100,
    timeWindow: 60000,
  },
};

/**
 * Root Application Component
 * Implements provider hierarchy and global error boundary
 */
const App: React.FC = () => {
  // Error boundary handler
  const handleError = (error: Error, info: { componentStack: string }) => {
    // Log error to monitoring service
    console.error('Application Error:', error, info);
  };

  // CSP Headers setup
  useEffect(() => {
    // Set Content Security Policy headers
    const meta = document.createElement('meta');
    meta.httpEquiv = 'Content-Security-Policy';
    meta.content = `
      default-src 'self';
      connect-src 'self' wss://*.agricultural-system.com;
      img-src 'self' data: https:;
      script-src 'self';
    `;
    document.head.appendChild(meta);

    return () => {
      document.head.removeChild(meta);
    };
  }, []);

  return (
    <ErrorBoundary
      FallbackComponent={ErrorFallback}
      onError={handleError}
      onReset={() => {
        // Reset application state on error recovery
        window.location.href = '/';
      }}
    >
      <BrowserRouter>
        <AuthProvider>
          <WebSocketProvider config={wsConfig}>
            <ThemeProvider>
              <Routes>
                {/* Public Routes */}
                <Route path="/login" element={<div>Login Page</div>} />
                <Route path="/register" element={<div>Register Page</div>} />
                
                {/* Protected Routes */}
                <Route
                  path="/"
                  element={
                    <ProtectedRoute>
                      <div>Dashboard</div>
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/missions/*"
                  element={
                    <ProtectedRoute>
                      <div>Mission Management</div>
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/analytics/*"
                  element={
                    <ProtectedRoute>
                      <div>Analytics Dashboard</div>
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/devices/*"
                  element={
                    <ProtectedRoute>
                      <div>Device Management</div>
                    </ProtectedRoute>
                  }
                />
                <Route
                  path="/treatments/*"
                  element={
                    <ProtectedRoute>
                      <div>Treatment Operations</div>
                    </ProtectedRoute>
                  }
                />

                {/* Fallback Route */}
                <Route path="*" element={<Navigate to="/" replace />} />
              </Routes>
            </ThemeProvider>
          </WebSocketProvider>
        </AuthProvider>
      </BrowserRouter>
    </ErrorBoundary>
  );
};

export default App;