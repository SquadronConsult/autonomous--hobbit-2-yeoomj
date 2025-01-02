import React, { useCallback, useEffect, useRef, memo } from 'react';
import styled from '@emotion/styled';
import debounce from 'lodash/debounce';
import Card from '../../common/Card/Card';
import { useNotification, NotificationType, Notification } from '../../../hooks/useNotification';

// Props interface for the AlertPanel component
interface AlertPanelProps {
  maxAlerts?: number;
  className?: string;
  autoDismissTimeout?: number;
  groupSimilar?: boolean;
}

// Styled components with Material Design 3.0 specifications
const AlertContainer = styled(Card)`
  display: flex;
  flex-direction: column;
  gap: 8px;
  max-height: 400px;
  overflow-y: auto;
  padding: 16px;
  scrollbar-width: thin;
  scrollbar-color: ${props => `${props.theme.text.disabled} transparent`};

  &::-webkit-scrollbar {
    width: 6px;
  }

  &::-webkit-scrollbar-track {
    background: transparent;
  }

  &::-webkit-scrollbar-thumb {
    background-color: ${props => props.theme.text.disabled};
    border-radius: 3px;
  }
`;

const AlertItem = styled.div<{ severity: NotificationType }>`
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 12px;
  border-radius: 4px;
  background-color: ${props => getAlertBackground(props.severity, props.theme)};
  color: ${props => props.theme.text.primary};
  transition: opacity 0.3s ease-in-out;
  position: relative;
  
  &:focus-visible {
    outline: 2px solid ${props => props.theme.primary.main};
    outline-offset: 2px;
  }
`;

const AlertContent = styled.div`
  flex: 1;
  margin-right: 16px;
`;

const AlertMessage = styled.div`
  font-size: 14px;
  line-height: 20px;
  font-weight: 500;
`;

const AlertTimestamp = styled.div`
  font-size: 12px;
  color: ${props => props.theme.text.secondary};
  margin-top: 4px;
`;

const AlertActions = styled.div`
  display: flex;
  gap: 8px;
  align-items: center;
`;

const ActionButton = styled.button`
  background: none;
  border: none;
  padding: 4px;
  cursor: pointer;
  color: ${props => props.theme.text.secondary};
  border-radius: 4px;
  transition: all 0.2s ease-in-out;

  &:hover {
    background-color: ${props => props.theme.background.elevated};
    color: ${props => props.theme.text.primary};
  }

  &:focus-visible {
    outline: 2px solid ${props => props.theme.primary.main};
    outline-offset: 2px;
  }
`;

// Helper function to get background color based on severity
const getAlertBackground = (severity: NotificationType, theme: any) => {
  const alpha = '14'; // 8% opacity for background
  switch (severity) {
    case NotificationType.ERROR:
      return `${theme.status.error}${alpha}`;
    case NotificationType.WARNING:
      return `${theme.status.warning}${alpha}`;
    case NotificationType.SUCCESS:
      return `${theme.status.success}${alpha}`;
    case NotificationType.INFO:
    default:
      return `${theme.status.info}${alpha}`;
  }
};

// Format relative time for accessibility
const formatRelativeTime = (timestamp: Date): string => {
  const diff = Date.now() - timestamp.getTime();
  const minutes = Math.floor(diff / 60000);
  
  if (minutes < 1) return 'just now';
  if (minutes === 1) return '1 minute ago';
  if (minutes < 60) return `${minutes} minutes ago`;
  
  const hours = Math.floor(minutes / 60);
  if (hours === 1) return '1 hour ago';
  if (hours < 24) return `${hours} hours ago`;
  
  return timestamp.toLocaleDateString();
};

const AlertPanel: React.FC<AlertPanelProps> = memo(({
  maxAlerts = 5,
  className,
  autoDismissTimeout = 5000,
  groupSimilar = true,
}) => {
  const { notifications, dismissNotification, groupNotifications } = useNotification();
  const containerRef = useRef<HTMLDivElement>(null);
  const lastFocusedAlertRef = useRef<string | null>(null);

  // Debounced scroll to bottom for new alerts
  const scrollToBottom = useCallback(
    debounce(() => {
      if (containerRef.current) {
        containerRef.current.scrollTop = containerRef.current.scrollHeight;
      }
    }, 100),
    []
  );

  // Handle alert dismissal with keyboard support
  const handleDismiss = useCallback((alertId: string, event?: React.KeyboardEvent) => {
    if (event && event.key !== 'Enter' && event.key !== ' ') return;
    
    const alerts = containerRef.current?.querySelectorAll('[role="alert"]');
    const currentIndex = Array.from(alerts || []).findIndex(
      alert => alert.getAttribute('data-alert-id') === alertId
    );

    dismissNotification(alertId);
    lastFocusedAlertRef.current = alertId;

    // Focus management after dismissal
    if (alerts && currentIndex !== -1) {
      const nextAlert = alerts[currentIndex + 1] || alerts[currentIndex - 1];
      if (nextAlert) {
        (nextAlert as HTMLElement).focus();
      }
    }
  }, [dismissNotification]);

  // Auto-dismiss non-critical alerts
  useEffect(() => {
    const timers = notifications
      .filter(n => n.type !== NotificationType.ERROR && !n.actions?.length)
      .map(n => setTimeout(() => dismissNotification(n.id), autoDismissTimeout));

    return () => timers.forEach(clearTimeout);
  }, [notifications, dismissNotification, autoDismissTimeout]);

  // Scroll to bottom when new alerts arrive
  useEffect(() => {
    scrollToBottom();
  }, [notifications.length, scrollToBottom]);

  // Process and filter notifications
  const displayedNotifications = groupSimilar
    ? groupNotifications('system_alerts').slice(0, maxAlerts)
    : notifications.slice(0, maxAlerts);

  return (
    <AlertContainer
      ref={containerRef}
      variant="elevated"
      className={className}
      aria-label="Notification panel"
      role="region"
    >
      {displayedNotifications.map((alert) => (
        <AlertItem
          key={alert.id}
          severity={alert.type}
          role="alert"
          aria-live="polite"
          data-alert-id={alert.id}
          tabIndex={0}
        >
          <AlertContent>
            <AlertMessage>{alert.message}</AlertMessage>
            <AlertTimestamp aria-label={`Alert time: ${formatRelativeTime(alert.timestamp)}`}>
              {formatRelativeTime(alert.timestamp)}
            </AlertTimestamp>
          </AlertContent>
          <AlertActions>
            {alert.actions?.map(action => (
              <ActionButton
                key={action.label}
                onClick={action.onClick}
                aria-label={action.ariaLabel}
              >
                {action.label}
              </ActionButton>
            ))}
            <ActionButton
              onClick={() => handleDismiss(alert.id)}
              onKeyDown={(e) => handleDismiss(alert.id, e)}
              aria-label={`Dismiss ${alert.ariaLabel || 'alert'}`}
            >
              ✕
            </ActionButton>
          </AlertActions>
        </AlertItem>
      ))}
    </AlertContainer>
  );
});

AlertPanel.displayName = 'AlertPanel';

export default AlertPanel;