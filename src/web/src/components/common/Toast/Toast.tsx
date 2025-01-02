import React from 'react'; // ^18.0.0
import styled from '@emotion/styled'; // ^11.11.0
import { motion, AnimatePresence } from 'framer-motion'; // ^10.0.0
import { useTheme } from '../../contexts/ThemeContext';

// Constants
const ANIMATION_DURATION = 300;
const DEFAULT_DURATION = 5000;
const TOAST_Z_INDEX = 9999;
const MINIMUM_CONTRAST_RATIO = 4.5;
const FOCUS_RETURN_DELAY = 50;

// Types and Interfaces
interface ToastProps {
  message: string | React.ReactNode;
  severity: 'success' | 'error' | 'warning' | 'info';
  duration?: number;
  onClose: () => void;
  position?: 'top-right' | 'top-left' | 'bottom-right' | 'bottom-left';
  autoHide?: boolean;
  pauseOnHover?: boolean;
  enableReducedMotion?: boolean;
  role?: 'alert' | 'status';
  toastId?: string;
}

// Styled Components
const ToastContainer = styled(motion.div)<{ position: string }>`
  position: fixed;
  ${({ position }) => {
    const [vertical, horizontal] = position.split('-');
    return `
      ${vertical}: 24px;
      ${horizontal}: 24px;
    `;
  }}
  z-index: ${TOAST_Z_INDEX};
  min-width: 300px;
  max-width: 600px;
`;

const ToastContent = styled.div<{ severity: string }>`
  display: flex;
  align-items: center;
  padding: 12px 16px;
  border-radius: 8px;
  background-color: ${({ theme }) => theme.background.paper};
  box-shadow: ${({ theme }) => theme.elevation[2]};
  color: ${({ theme }) => theme.text.primary};
  font-family: 'Roboto', sans-serif;
  font-size: 14px;
  line-height: 1.5;
`;

const IconWrapper = styled.div`
  margin-right: 12px;
  display: flex;
  align-items: center;
`;

const MessageWrapper = styled.div`
  flex: 1;
  margin-right: 12px;
`;

const CloseButton = styled.button`
  background: none;
  border: none;
  padding: 4px;
  cursor: pointer;
  color: inherit;
  opacity: 0.7;
  transition: opacity 0.2s;
  
  &:hover {
    opacity: 1;
  }
  
  &:focus-visible {
    outline: 2px solid currentColor;
    outline-offset: 2px;
    border-radius: 4px;
  }
`;

// Helper function to get appropriate color based on severity
const getToastColor = (severity: string, theme: any) => {
  const colors = {
    success: theme.status.success,
    error: theme.status.error,
    warning: theme.status.warning,
    info: theme.status.info
  };
  return colors[severity as keyof typeof colors];
};

// Toast Component
const Toast: React.FC<ToastProps> = ({
  message,
  severity,
  duration = DEFAULT_DURATION,
  onClose,
  position = 'top-right',
  autoHide = true,
  pauseOnHover = true,
  enableReducedMotion = false,
  role = 'alert',
  toastId
}) => {
  const { isDarkMode, prefersReducedMotion } = useTheme();
  const [isVisible, setIsVisible] = React.useState(true);
  const [isPaused, setIsPaused] = React.useState(false);
  const timerRef = React.useRef<NodeJS.Timeout>();
  const previousFocus = React.useRef<HTMLElement | null>(null);

  // Animation variants
  const variants = {
    initial: { opacity: 0, y: position.includes('top') ? -20 : 20 },
    animate: { opacity: 1, y: 0 },
    exit: { opacity: 0, y: position.includes('top') ? -20 : 20 }
  };

  // Handle close
  const handleClose = React.useCallback(() => {
    setIsVisible(false);
    if (timerRef.current) {
      clearTimeout(timerRef.current);
    }
    
    // Return focus after animation
    setTimeout(() => {
      if (previousFocus.current) {
        previousFocus.current.focus();
      }
      onClose();
    }, ANIMATION_DURATION + FOCUS_RETURN_DELAY);
  }, [onClose]);

  // Auto-hide timer
  React.useEffect(() => {
    if (autoHide && !isPaused && isVisible) {
      timerRef.current = setTimeout(handleClose, duration);
    }
    return () => {
      if (timerRef.current) {
        clearTimeout(timerRef.current);
      }
    };
  }, [autoHide, duration, handleClose, isPaused, isVisible]);

  // Store previous focus
  React.useEffect(() => {
    previousFocus.current = document.activeElement as HTMLElement;
    return () => {
      previousFocus.current = null;
    };
  }, []);

  // Handle pause on hover
  const handleMouseEnter = () => {
    if (pauseOnHover) {
      setIsPaused(true);
    }
  };

  const handleMouseLeave = () => {
    if (pauseOnHover) {
      setIsPaused(false);
    }
  };

  return (
    <AnimatePresence>
      {isVisible && (
        <ToastContainer
          position={position}
          variants={variants}
          initial="initial"
          animate="animate"
          exit="exit"
          transition={{
            duration: (prefersReducedMotion || enableReducedMotion) ? 0 : ANIMATION_DURATION / 1000
          }}
          onMouseEnter={handleMouseEnter}
          onMouseLeave={handleMouseLeave}
          role={role}
          aria-live={role === 'alert' ? 'assertive' : 'polite'}
          data-testid={toastId}
        >
          <ToastContent severity={severity}>
            <IconWrapper>
              {/* Severity Icon */}
              <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
                {severity === 'success' && (
                  <path d="M9 16.17L4.83 12l-1.42 1.41L9 19 21 7l-1.41-1.41L9 16.17z" />
                )}
                {severity === 'error' && (
                  <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-2h2v2zm0-4h-2V7h2v6z" />
                )}
                {severity === 'warning' && (
                  <path d="M1 21h22L12 2 1 21zm12-3h-2v-2h2v2zm0-4h-2v-4h2v4z" />
                )}
                {severity === 'info' && (
                  <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-6h2v6zm0-8h-2V7h2v2z" />
                )}
              </svg>
            </IconWrapper>
            <MessageWrapper>{message}</MessageWrapper>
            <CloseButton
              onClick={handleClose}
              aria-label="Close notification"
              tabIndex={0}
            >
              <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
                <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12 19 6.41z" />
              </svg>
            </CloseButton>
          </ToastContent>
        </ToastContainer>
      )}
    </AnimatePresence>
  );
};

export default Toast;