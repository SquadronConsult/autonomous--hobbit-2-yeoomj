import React from 'react';
import '../../../styles/variables.css';

interface LoadingProps {
  /**
   * Size variant of the loading spinner
   * sm: 24px, md: 40px, lg: 56px
   */
  size?: 'sm' | 'md' | 'lg';
  /**
   * Accessible label for screen readers
   */
  label?: string;
  /**
   * Optional CSS class name for custom styling
   */
  className?: string;
}

/**
 * A reusable loading spinner component that provides visual feedback during
 * asynchronous operations. Follows Material Design 3.0 specifications and
 * WCAG 2.1 Level AA accessibility guidelines.
 * 
 * @version 1.0.0
 */
const Loading: React.FC<LoadingProps> = ({
  size = 'md',
  label = 'Loading...',
  className = ''
}) => {
  // Size mapping based on design specifications
  const sizeMap = {
    sm: '24px',
    md: '40px',
    lg: '56px'
  };

  // Dynamic styles for the spinner container
  const containerStyle: React.CSSProperties = {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    position: 'relative',
    width: sizeMap[size],
    height: sizeMap[size]
  };

  // Dynamic styles for the spinner element
  const spinnerStyle: React.CSSProperties = {
    position: 'absolute',
    width: '100%',
    height: '100%',
    border: '4px solid var(--theme-background-paper, #f5f5f5)',
    borderTopColor: 'var(--theme-primary-main, #1976d2)',
    borderRadius: '50%',
    animation: 'loading-spin 1s linear infinite',
    willChange: 'transform',
    transform: 'translateZ(0)' // Hardware acceleration
  };

  // Styles for visually hidden label (screen reader only)
  const labelStyle: React.CSSProperties = {
    position: 'absolute',
    width: '1px',
    height: '1px',
    padding: '0',
    margin: '-1px',
    overflow: 'hidden',
    clip: 'rect(0, 0, 0, 0)',
    whiteSpace: 'nowrap',
    border: '0'
  };

  return (
    <div 
      className={`loading-container ${className}`}
      style={containerStyle}
      role="status"
      aria-live="polite"
    >
      {/* Spinner animation */}
      <div className="loading-spinner" style={spinnerStyle} />
      
      {/* Screen reader label */}
      <span style={labelStyle}>
        {label}
      </span>

      {/* CSS animation keyframes */}
      <style>
        {`
          @keyframes loading-spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
          }

          @media (prefers-reduced-motion: reduce) {
            .loading-spinner {
              animation-duration: 1.5s;
              animation-timing-function: linear;
            }
          }
        `}
      </style>
    </div>
  );
};

export default Loading;