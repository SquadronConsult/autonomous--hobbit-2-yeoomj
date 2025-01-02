/**
 * @fileoverview Custom React hook for managing notifications with Material Design 3.0 compliance and accessibility support
 * @version 1.0.0
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { useWebSocketContext, SystemAlert } from '../contexts/WebSocketContext';

// Notification types following Material Design 3.0 guidelines
export enum NotificationType {
  SUCCESS = 'success',
  ERROR = 'error',
  WARNING = 'warning',
  INFO = 'info'
}

// Action interface for interactive notifications
export interface NotificationAction {
  label: string;
  ariaLabel: string;
  onClick: () => void;
  color?: string;
  icon?: string;
}

// Core notification interface
export interface Notification {
  id: string;
  type: NotificationType;
  message: string;
  duration?: number;
  timestamp: Date;
  priority: number;
  actions?: NotificationAction[];
  ariaLabel: string;
  groupId?: string;
}

// Rate limiting configuration
const RATE_LIMIT = {
  maxNotifications: 5,
  timeWindow: 5000, // 5 seconds
};

// Default durations by notification type (in ms)
const DEFAULT_DURATIONS = {
  [NotificationType.SUCCESS]: 5000,
  [NotificationType.ERROR]: 0, // Requires manual dismissal
  [NotificationType.WARNING]: 7000,
  [NotificationType.INFO]: 5000,
};

/**
 * Custom hook for managing notifications with Material Design 3.0 compliance
 * and WCAG 2.1 Level AA accessibility support
 */
export const useNotification = () => {
  const [notifications, setNotifications] = useState<Notification[]>([]);
  const notificationQueue = useRef<Notification[]>([]);
  const lastNotificationTime = useRef<number>(0);
  const notificationCount = useRef<number>(0);
  const { subscribe } = useWebSocketContext<SystemAlert>();

  // Clear notification queue and reset rate limiting
  const resetRateLimit = useCallback(() => {
    notificationCount.current = 0;
    lastNotificationTime.current = Date.now();
  }, []);

  // Check rate limiting before showing notification
  const checkRateLimit = useCallback((): boolean => {
    const now = Date.now();
    if (now - lastNotificationTime.current > RATE_LIMIT.timeWindow) {
      resetRateLimit();
    }
    return notificationCount.current < RATE_LIMIT.maxNotifications;
  }, [resetRateLimit]);

  // Process notification queue
  const processQueue = useCallback(() => {
    if (notificationQueue.current.length === 0) return;
    
    if (checkRateLimit()) {
      const notification = notificationQueue.current.shift();
      if (notification) {
        setNotifications(prev => [...prev, notification].sort((a, b) => b.priority - a.priority));
        notificationCount.current++;
      }
    }
  }, [checkRateLimit]);

  // Show new notification with rate limiting
  const showNotification = useCallback((
    message: string,
    type: NotificationType = NotificationType.INFO,
    options: Partial<Omit<Notification, 'id' | 'timestamp' | 'message' | 'type'>> = {}
  ) => {
    const notification: Notification = {
      id: crypto.randomUUID(),
      message,
      type,
      timestamp: new Date(),
      priority: options.priority || 1,
      duration: options.duration || DEFAULT_DURATIONS[type],
      actions: options.actions || [],
      ariaLabel: options.ariaLabel || message,
      groupId: options.groupId,
    };

    notificationQueue.current.push(notification);
    processQueue();
  }, [processQueue]);

  // Dismiss notification with accessibility announcement
  const dismissNotification = useCallback((id: string) => {
    setNotifications(prev => {
      const notification = prev.find(n => n.id === id);
      if (notification) {
        // Announce dismissal for screen readers
        const dismissalMessage = `Notification dismissed: ${notification.message}`;
        const announcement = document.createElement('div');
        announcement.setAttribute('role', 'alert');
        announcement.setAttribute('aria-live', 'polite');
        announcement.textContent = dismissalMessage;
        document.body.appendChild(announcement);
        setTimeout(() => document.body.removeChild(announcement), 1000);
      }
      return prev.filter(n => n.id !== id);
    });
  }, []);

  // Clear all notifications by group or type
  const clearAllNotifications = useCallback((
    options?: { groupId?: string; type?: NotificationType }
  ) => {
    setNotifications(prev => {
      if (!options) return [];
      return prev.filter(n => 
        (options.groupId && n.groupId !== options.groupId) ||
        (options.type && n.type !== options.type)
      );
    });
    notificationQueue.current = [];
    resetRateLimit();
  }, [resetRateLimit]);

  // Group notifications by groupId
  const groupNotifications = useCallback((groupId: string) => {
    return notifications.filter(n => n.groupId === groupId);
  }, [notifications]);

  // Auto-dismiss notifications based on duration
  useEffect(() => {
    const timers = notifications
      .filter(n => n.duration && n.duration > 0)
      .map(n => setTimeout(() => dismissNotification(n.id), n.duration));

    return () => timers.forEach(timer => clearTimeout(timer));
  }, [notifications, dismissNotification]);

  // Subscribe to system alerts via WebSocket
  useEffect(() => {
    const unsubscribe = subscribe('system_alert', (alert: SystemAlert) => {
      showNotification(alert.message, alert.severity as NotificationType, {
        priority: alert.priority || 3,
        duration: alert.duration,
        ariaLabel: `System alert: ${alert.message}`,
        groupId: 'system_alerts'
      });
    });

    return () => unsubscribe();
  }, [subscribe, showNotification]);

  // Process queue periodically
  useEffect(() => {
    const interval = setInterval(processQueue, 1000);
    return () => clearInterval(interval);
  }, [processQueue]);

  return {
    notifications,
    showNotification,
    dismissNotification,
    clearAllNotifications,
    groupNotifications,
  };
};