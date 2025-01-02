/**
 * Entry point for the Agricultural Management System web application
 * Initializes React application with required providers and global configurations
 * @version 1.0.0
 */

import React from 'react'; // ^18.2.0
import ReactDOM from 'react-dom/client'; // ^18.2.0
import { StrictMode } from 'react'; // ^18.2.0

// Root application component
import App from './App';

// Global styles with Material Design 3.0
import './styles/global.css';

// Root element ID for React mounting
const ROOT_ELEMENT_ID = 'root';

// Environment configuration
const IS_DEVELOPMENT = process.env.NODE_ENV === 'development';

/**
 * Error tracking configuration for production environment
 * Uses Sentry for production error monitoring
 */
const ERROR_TRACKING_CONFIG = {
  dsn: process.env.SENTRY_DSN,
  environment: process.env.NODE_ENV,
  tracesSampleRate: 1.0,
  integrations: ['react']
};

/**
 * Initializes error tracking in production environment
 */
const initializeErrorTracking = (): void => {
  if (!IS_DEVELOPMENT && ERROR_TRACKING_CONFIG.dsn) {
    import('@sentry/react').then(Sentry => {
      Sentry.init(ERROR_TRACKING_CONFIG);
    });
  }
};

/**
 * Initializes performance monitoring
 */
const initializePerformanceMonitoring = (): void => {
  if (!IS_DEVELOPMENT) {
    // Initialize Web Vitals reporting
    import('web-vitals').then(({ getCLS, getFID, getFCP, getLCP, getTTFB }) => {
      getCLS(console.log);
      getFID(console.log);
      getFCP(console.log);
      getLCP(console.log);
      getTTFB(console.log);
    });
  }
};

/**
 * Configures development tools and debugging
 */
const configureDevelopmentTools = (): void => {
  if (IS_DEVELOPMENT) {
    // Enable React strict mode warnings
    console.log('Development mode enabled - React Strict Mode active');
    
    // Enable React DevTools
    if (window.__REACT_DEVTOOLS_GLOBAL_HOOK__) {
      console.log('React DevTools detected');
    }
  }
};

/**
 * Renders the React application with error boundaries and performance monitoring
 */
const renderApp = (): void => {
  const rootElement = document.getElementById(ROOT_ELEMENT_ID);

  if (!rootElement) {
    throw new Error(`Unable to find root element with id '${ROOT_ELEMENT_ID}'`);
  }

  // Initialize error tracking
  initializeErrorTracking();

  // Initialize performance monitoring
  initializePerformanceMonitoring();

  // Configure development tools
  configureDevelopmentTools();

  // Create React root with concurrent features
  const root = ReactDOM.createRoot(rootElement);

  // Render application with strict mode for development checks
  root.render(
    <StrictMode>
      <App />
    </StrictMode>
  );

  // Configure resource hints for performance
  if (!IS_DEVELOPMENT) {
    const linkPrefetch = document.createElement('link');
    linkPrefetch.rel = 'prefetch';
    linkPrefetch.href = '/assets/fonts/roboto-v30-latin-regular.woff2';
    document.head.appendChild(linkPrefetch);
  }
};

// Initialize application
renderApp();

// Enable hot module replacement in development
if (IS_DEVELOPMENT && module.hot) {
  module.hot.accept('./App', () => {
    renderApp();
  });
}

// Handle unhandled promise rejections
window.addEventListener('unhandledrejection', (event) => {
  console.error('Unhandled promise rejection:', event.reason);
  event.preventDefault();
});

// Export for testing purposes
export { renderApp };