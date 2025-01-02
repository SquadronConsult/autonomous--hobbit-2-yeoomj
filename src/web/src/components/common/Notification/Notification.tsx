import React, { useEffect, useCallback, useRef } from 'react'; // ^18.0.0
import styled from '@emotion/styled'; // ^11.11.0
import { motion, AnimatePresence } from 'framer-motion'; // ^10.0.0
import { Icon } from '../Icons/Icons';
import { ThemeMode } from '../../../constants/theme';

// Types for notification positions and severity levels
type NotificationPosition = 'top-right' | 'top-left' | 'bottom-right' | 'bottom-left';
type NotificationSeverity = 'success' | 'warning' | 'error' | 'info';

// Interface for notification component props
interface NotificationProps {
  message: string;
  severity: NotificationSeverity;
  duration?: number;
  onClose?: () => void;
  position?: NotificationPosition;
  autoHideDuration?: number | null;
  priority?: number;
}

// Styled components
const NotificationWrapper = styled(motion.div)<{
  severity: NotificationSeverity;
  position: NotificationPosition;
  priority: number;
}>`
  display: flex;
  align-items: center;
  padding: 12px 16px;
  border-radius: 8px;
  margin-bottom: 8px;
  min-width: 300px;
  max-width: 500px;
  box-shadow: ${({ theme }) => theme.elevation[2]};
  background-color: ${({ severity, theme }) => getBackgroundColor(severity, theme)};
  color: ${({ severity, theme }) => getTextColor(severity, theme)};
  z-index: ${({ priority }) => 1000 + priority};
  position: fixed;
  ${({ position }) => getPositionStyles(position)};
  transform: translate3d(0, 0, 0);
  transition: all 0.2s ease-in-out;
`;

const IconContainer = styled.div`
  margin-right: 12px;
  display: flex;
  align-items: center;
`;

const MessageContainer = styled.div`
  flex: 1;
  margin-right: 12px;
  font-size: 14px;
  line-height: 1.5;
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

  &:focus {
    outline: 2px solid currentColor;
    border-radius: 4px;
  }
`;

// Helper functions
const getBackgroundColor = (severity: NotificationSeverity, theme: any) => {
  const alpha = theme.mode === ThemeMode.LIGHT ? '0.95' : '0.98';
  const baseColor = theme.status[severity];
  return `${baseColor}${alpha}`;
};

const getTextColor = (severity: NotificationSeverity, theme: any) => {
  return theme.mode === ThemeMode.LIGHT ? theme.text.primary : theme.text.primary;
};

const getPositionStyles = (position: NotificationPosition) => {
  const base = {
    'top-right': 'top: 24px; right: 24px;',
    'top-left': 'top: 24px; left: 24px;',
    'bottom-right': 'bottom: 24px; right: 24px;',
    'bottom-left': 'bottom: 24px; left: 24px;',
  };
  return base[position];
};

const getSeverityIcon = (severity: NotificationSeverity) => {
  const icons = {
    success: 'analytics',
    warning: 'drone',
    error: 'drone',
    info: 'analytics',
  };
  return icons[severity];
};

// Animation variants
const variants = {
  initial: (position: NotificationPosition) => ({
    opacity: 0,
    x: position.includes('right') ? 100 : -100,
    y: 0,
  }),
  animate: {
    opacity: 1,
    x: 0,
    y: 0,
    transition: {
      duration: 0.2,
      ease: 'easeOut',
    },
  },
  exit: (position: NotificationPosition) => ({
    opacity: 0,
    x: position.includes('right') ? 100 : -100,
    transition: {
      duration: 0.15,
      ease: 'easeIn',
    },
  }),
};

export const Notification: React.FC<NotificationProps> = ({
  message,
  severity,
  duration = 5000,
  onClose,
  position = 'top-right',
  autoHideDuration = 5000,
  priority = 0,
}) => {
  const timeoutRef = useRef<NodeJS.Timeout>();

  const handleClose = useCallback(() => {
    if (onClose) {
      onClose();
    }
  }, [onClose]);

  useEffect(() => {
    if (autoHideDuration !== null) {
      timeoutRef.current = setTimeout(() => {
        handleClose();
      }, autoHideDuration);
    }

    return () => {
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
    };
  }, [autoHideDuration, handleClose]);

  const handleKeyDown = (event: React.KeyboardEvent) => {
    if (event.key === 'Escape') {
      handleClose();
    }
  };

  return (
    <AnimatePresence mode="wait">
      <NotificationWrapper
        role="alert"
        aria-live="polite"
        aria-atomic="true"
        severity={severity}
        position={position}
        priority={priority}
        variants={variants}
        initial="initial"
        animate="animate"
        exit="exit"
        custom={position}
        onKeyDown={handleKeyDown}
        tabIndex={0}
      >
        <IconContainer>
          <Icon
            name={getSeverityIcon(severity)}
            size={20}
            color="currentColor"
            aria-hidden="true"
          />
        </IconContainer>
        <MessageContainer id="notification-message">
          {message}
        </MessageContainer>
        <CloseButton
          onClick={handleClose}
          aria-label="Close notification"
          tabIndex={0}
        >
          <Icon name="analytics" size={16} color="currentColor" aria-hidden="true" />
        </CloseButton>
      </NotificationWrapper>
    </AnimatePresence>
  );
};

export type { NotificationProps, NotificationPosition, NotificationSeverity };