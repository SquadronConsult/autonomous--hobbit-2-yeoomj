import React, { useCallback, useEffect, useState } from 'react';
import styled from '@emotion/styled';
import { useDebounce } from 'use-debounce';
import Layout from '../../components/common/Layout/Layout';
import SystemConfig from '../../components/settings/SystemConfig/SystemConfig';
import AlertConfig from '../../components/settings/AlertConfig/AlertConfig';
import { useAuth } from '../../hooks/useAuth';
import { ThemeMode } from '../../constants/theme';
import { UserRole } from '../../interfaces/IUser';

// Styled components with Material Design 3.0 specifications
const SettingsContainer = styled.div`
  display: flex;
  flex-direction: column;
  gap: 24px;
  max-width: 1200px;
  margin: 0 auto;
  padding: clamp(16px, 5vw, 24px);
  transition: all 0.3s ease;

  @media (max-width: 768px) {
    gap: 16px;
  }
`;

const SettingsSection = styled.section`
  display: flex;
  flex-direction: column;
  gap: 16px;
  padding: 24px;
  background-color: var(--md-sys-color-surface);
  border-radius: 16px;
  box-shadow: var(--md-sys-elevation-1);

  @media (prefers-reduced-motion) {
    transition: none;
  }
`;

const SectionTitle = styled.h2`
  font-size: var(--font-size-xl);
  font-weight: var(--font-weight-medium);
  color: var(--md-sys-color-on-surface);
  margin: 0;
`;

const ErrorMessage = styled.div`
  color: var(--md-sys-color-error);
  padding: 16px;
  border-radius: 8px;
  background-color: var(--md-sys-color-error-container);
  margin-bottom: 16px;
`;

// Initial system configuration
const defaultSystemConfig = {
  themeMode: ThemeMode.SYSTEM,
  dataRetentionDays: 90,
  maxMissions: 8,
  notificationEmail: '',
  alertThreshold: 80,
  performanceMode: 'balanced'
};

// Initial alert configuration
const defaultAlertConfig = {
  containerSecurity: {
    enabled: true,
    threshold: 1,
    priority: 'HIGH',
    notificationChannels: ['email', 'system']
  },
  networkTraffic: {
    enabled: true,
    requestsPerSecond: 100,
    priority: 'MEDIUM',
    aggregationWindow: 60
  },
  authentication: {
    enabled: true,
    failuresPerMinute: 3,
    lockoutThreshold: 5,
    notifyOnLockout: true
  },
  resourceUsage: {
    enabled: true,
    cpuThreshold: 80,
    memoryThreshold: 85,
    gpuThreshold: 90,
    storageThreshold: 85
  }
};

const Settings: React.FC = () => {
  const { user, isAuthenticated } = useAuth();
  const [error, setError] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [systemConfig, setSystemConfig] = useState(defaultSystemConfig);
  const [alertConfig, setAlertConfig] = useState(defaultAlertConfig);

  // Debounced save handlers to prevent excessive API calls
  const [debouncedSystemSave] = useDebounce(
    async (config) => {
      try {
        setIsLoading(true);
        // API call would go here
        await new Promise(resolve => setTimeout(resolve, 500));
        setSystemConfig(config);
        setError(null);
      } catch (err) {
        setError('Failed to save system configuration');
        console.error('System config save error:', err);
      } finally {
        setIsLoading(false);
      }
    },
    1000
  );

  const [debouncedAlertSave] = useDebounce(
    async (config) => {
      try {
        setIsLoading(true);
        // API call would go here
        await new Promise(resolve => setTimeout(resolve, 500));
        setAlertConfig(config);
        setError(null);
      } catch (err) {
        setError('Failed to save alert configuration');
        console.error('Alert config save error:', err);
      } finally {
        setIsLoading(false);
      }
    },
    1000
  );

  // Handle system configuration changes
  const handleSystemConfigSave = useCallback(async (values) => {
    if (!isAuthenticated || user?.role !== UserRole.ADMINISTRATOR) {
      setError('Unauthorized: Admin privileges required');
      return;
    }
    await debouncedSystemSave(values);
  }, [isAuthenticated, user, debouncedSystemSave]);

  // Handle alert configuration changes
  const handleAlertConfigSave = useCallback(async (values) => {
    if (!isAuthenticated || user?.role !== UserRole.ADMINISTRATOR) {
      setError('Unauthorized: Admin privileges required');
      return;
    }
    await debouncedAlertSave(values);
  }, [isAuthenticated, user, debouncedAlertSave]);

  // Handle configuration cancellation
  const handleCancel = useCallback(() => {
    setSystemConfig(defaultSystemConfig);
    setAlertConfig(defaultAlertConfig);
    setError(null);
  }, []);

  // Check authentication and permissions
  useEffect(() => {
    if (!isAuthenticated) {
      setError('Please log in to access settings');
    } else if (user?.role !== UserRole.ADMINISTRATOR) {
      setError('Insufficient permissions: Admin access required');
    } else {
      setError(null);
    }
  }, [isAuthenticated, user]);

  return (
    <Layout>
      <SettingsContainer role="main" aria-label="Settings Page">
        {error && (
          <ErrorMessage role="alert" aria-live="polite">
            {error}
          </ErrorMessage>
        )}

        <SettingsSection>
          <SectionTitle>System Configuration</SectionTitle>
          <SystemConfig
            onSave={handleSystemConfigSave}
            initialValues={systemConfig}
            onCancel={handleCancel}
          />
        </SettingsSection>

        <SettingsSection>
          <SectionTitle>Security & Alerts</SectionTitle>
          <AlertConfig
            userRole={user?.role || UserRole.OPERATOR}
            onSave={handleAlertConfigSave}
            initialConfig={alertConfig}
            isLoading={isLoading}
          />
        </SettingsSection>
      </SettingsContainer>
    </Layout>
  );
};

export default Settings;