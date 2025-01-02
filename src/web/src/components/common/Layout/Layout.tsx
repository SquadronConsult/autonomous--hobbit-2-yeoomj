import React, { useCallback, useEffect, useState, memo } from 'react';
import styled from '@emotion/styled';
import { useNavigate } from 'react-router-dom';
import { Sidebar } from '../Sidebar/Sidebar';
import Loading from '../Loading/Loading';
import { useAuth } from '../../../hooks/useAuth';

// Security constants
const SECURITY_HEADERS = {
  'Content-Security-Policy': "default-src 'self'",
  'X-Frame-Options': 'DENY',
  'X-Content-Type-Options': 'nosniff'
};

// Layout constants
const SIDEBAR_COLLAPSED_KEY = 'sidebar_collapsed';
const MOBILE_BREAKPOINT = 768;
const ANIMATION_DURATION = 300;

// Styled components with Material Design 3.0 specifications
const LayoutContainer = styled.div`
  display: flex;
  min-height: 100vh;
  background-color: var(--md-sys-color-background);
  transition: all ${ANIMATION_DURATION}ms var(--transition-timing-ease-in-out);
  position: relative;
  isolation: isolate;

  @media (prefers-reduced-motion: reduce) {
    transition: none;
  }
`;

const MainContent = styled.main<{ sidebarCollapsed: boolean }>`
  flex: 1;
  padding: var(--spacing-lg);
  margin-left: ${props => props.sidebarCollapsed ? '64px' : '240px'};
  transition: margin-left ${ANIMATION_DURATION}ms var(--transition-timing-ease-in-out);
  background-color: var(--md-sys-color-surface);
  min-height: 100vh;
  position: relative;

  @media (max-width: ${MOBILE_BREAKPOINT}px) {
    margin-left: 0;
    padding: var(--spacing-md);
  }

  &:focus-visible {
    outline: 2px solid var(--md-sys-color-primary);
    outline-offset: -2px;
  }
`;

const SkipLink = styled.a`
  position: absolute;
  top: -40px;
  left: 0;
  background: var(--md-sys-color-primary);
  color: var(--md-sys-color-on-primary);
  padding: var(--spacing-sm);
  z-index: var(--z-index-modal);
  text-decoration: none;
  border-radius: var(--border-radius-sm);
  
  &:focus {
    top: var(--spacing-sm);
    left: var(--spacing-sm);
  }
`;

interface LayoutProps {
  children: React.ReactNode;
  className?: string;
  securityContext?: {
    allowedRoutes?: string[];
    requiredPermissions?: string[];
  };
}

const Layout: React.FC<LayoutProps> = memo(({ 
  children, 
  className,
  securityContext 
}) => {
  const { isAuthenticated, validateSession } = useAuth();
  const navigate = useNavigate();
  const [isLoading, setIsLoading] = useState(true);
  const [sidebarCollapsed, setSidebarCollapsed] = useState(() => {
    return localStorage.getItem(SIDEBAR_COLLAPSED_KEY) === 'true';
  });

  // Security validation
  useEffect(() => {
    const validateAccess = async () => {
      try {
        setIsLoading(true);
        if (!isAuthenticated) {
          navigate('/login');
          return;
        }
        await validateSession();
      } catch (error) {
        console.error('Session validation failed:', error);
        navigate('/login');
      } finally {
        setIsLoading(false);
      }
    };

    validateAccess();
  }, [isAuthenticated, navigate, validateSession]);

  // Set security headers
  useEffect(() => {
    Object.entries(SECURITY_HEADERS).forEach(([key, value]) => {
      document.head.querySelector(`meta[http-equiv="${key}"]`)?.remove();
      const meta = document.createElement('meta');
      meta.httpEquiv = key;
      meta.content = value;
      document.head.appendChild(meta);
    });
  }, []);

  // Handle sidebar toggle with localStorage persistence
  const handleSidebarToggle = useCallback(() => {
    setSidebarCollapsed(prev => {
      const newState = !prev;
      localStorage.setItem(SIDEBAR_COLLAPSED_KEY, String(newState));
      return newState;
    });
  }, []);

  // Handle keyboard navigation
  const handleKeyDown = useCallback((event: KeyboardEvent) => {
    if (event.key === 'Escape') {
      const mainContent = document.querySelector('main');
      mainContent?.focus();
    }
  }, []);

  useEffect(() => {
    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [handleKeyDown]);

  if (isLoading) {
    return <Loading size="lg" label="Loading application..." />;
  }

  return (
    <LayoutContainer className={className}>
      <SkipLink href="#main-content">
        Skip to main content
      </SkipLink>

      <Sidebar
        isCollapsed={sidebarCollapsed}
        onToggle={handleSidebarToggle}
        securityContext={securityContext}
      />

      <MainContent
        id="main-content"
        sidebarCollapsed={sidebarCollapsed}
        tabIndex={-1}
        role="main"
        aria-label="Main content"
      >
        {children}
      </MainContent>
    </LayoutContainer>
  );
});

Layout.displayName = 'Layout';

export default Layout;