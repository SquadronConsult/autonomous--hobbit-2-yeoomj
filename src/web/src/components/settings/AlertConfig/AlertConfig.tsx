import React, { useCallback, useEffect } from 'react';
import styled from '@emotion/styled';
import { useForm } from 'react-hook-form';
import Card from '../../common/Card/Card';
import Input from '../../common/Input/Input';
import { UserRole } from '../../../interfaces/IUser';
import { validateThreshold } from '../../../utils/validation';
import { theme } from '../../../constants/theme';

// Alert priority levels
export enum AlertPriority {
  HIGH = 'HIGH',
  MEDIUM = 'MEDIUM',
  LOW = 'LOW'
}

// Props interface for the AlertConfig component
interface AlertConfigProps {
  userRole: UserRole;
  onSave: (config: AlertConfigType) => Promise<void>;
  initialConfig: AlertConfigType;
  isLoading: boolean;
}

// Type definition for alert configuration
export interface AlertConfigType {
  containerSecurity: {
    enabled: boolean;
    threshold: number;
    priority: AlertPriority;
    notificationChannels: string[];
  };
  networkTraffic: {
    enabled: boolean;
    requestsPerSecond: number;
    priority: AlertPriority;
    aggregationWindow: number;
  };
  authentication: {
    enabled: boolean;
    failuresPerMinute: number;
    lockoutThreshold: number;
    notifyOnLockout: boolean;
  };
  resourceUsage: {
    enabled: boolean;
    cpuThreshold: number;
    memoryThreshold: number;
    gpuThreshold: number;
    storageThreshold: number;
  };
}

// Styled components
const ConfigContainer = styled.form`
  display: grid;
  grid-gap: 1.5rem;
  padding: 1.5rem;
  max-width: 800px;
  margin: 0 auto;
  background-color: ${theme.background.default};
  border-radius: 8px;
`;

const ConfigSection = styled(Card)`
  display: flex;
  flex-direction: column;
  gap: 1rem;
`;

const SectionHeader = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1rem;
`;

const SectionTitle = styled.h3`
  color: ${theme.text.primary};
  margin: 0;
  font-size: 1.1rem;
  font-weight: 500;
`;

const ToggleSwitch = styled.label`
  position: relative;
  display: inline-block;
  width: 48px;
  height: 24px;
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  opacity: ${props => props.disabled ? 0.5 : 1};
`;

const AlertConfig: React.FC<AlertConfigProps> = ({
  userRole,
  onSave,
  initialConfig,
  isLoading
}) => {
  const { register, handleSubmit, watch, setValue, formState: { errors } } = useForm<AlertConfigType>({
    defaultValues: initialConfig
  });

  // Check if user has admin privileges
  const isAdmin = userRole === UserRole.ADMINISTRATOR;

  // Handle form submission
  const handleSave = useCallback(async (formData: AlertConfigType) => {
    try {
      if (!isAdmin) {
        throw new Error('Unauthorized: Admin privileges required');
      }

      // Validate thresholds
      Object.entries(formData.resourceUsage).forEach(([key, value]) => {
        if (typeof value === 'number') {
          validateThreshold(value, 0, 100, `${key} threshold`);
        }
      });

      await onSave(formData);
    } catch (error) {
      console.error('Failed to save alert configuration:', error);
      // Error handling would be implemented here
    }
  }, [isAdmin, onSave]);

  // Reset section to defaults
  const handleReset = useCallback((section: keyof AlertConfigType) => {
    setValue(section, initialConfig[section]);
  }, [setValue, initialConfig]);

  return (
    <ConfigContainer
      onSubmit={handleSubmit(handleSave)}
      aria-label="Alert Configuration Form"
      role="form"
    >
      <ConfigSection variant="elevated">
        <SectionHeader>
          <SectionTitle>Container Security Alerts</SectionTitle>
          <ToggleSwitch disabled={!isAdmin}>
            <input
              type="checkbox"
              {...register('containerSecurity.enabled')}
              disabled={!isAdmin}
              aria-label="Enable container security alerts"
            />
          </ToggleSwitch>
        </SectionHeader>
        
        <Input
          name="containerSecurity.threshold"
          type="number"
          label="Critical Violation Threshold"
          disabled={!isAdmin || !watch('containerSecurity.enabled')}
          error={errors.containerSecurity?.threshold}
          required
          {...register('containerSecurity.threshold')}
        />
      </ConfigSection>

      <ConfigSection variant="elevated">
        <SectionHeader>
          <SectionTitle>Network Traffic Monitoring</SectionTitle>
          <ToggleSwitch disabled={!isAdmin}>
            <input
              type="checkbox"
              {...register('networkTraffic.enabled')}
              disabled={!isAdmin}
              aria-label="Enable network traffic monitoring"
            />
          </ToggleSwitch>
        </SectionHeader>

        <Input
          name="networkTraffic.requestsPerSecond"
          type="number"
          label="Maximum Requests per Second"
          disabled={!isAdmin || !watch('networkTraffic.enabled')}
          error={errors.networkTraffic?.requestsPerSecond}
          required
          {...register('networkTraffic.requestsPerSecond')}
        />
      </ConfigSection>

      <ConfigSection variant="elevated">
        <SectionHeader>
          <SectionTitle>Authentication Monitoring</SectionTitle>
          <ToggleSwitch disabled={!isAdmin}>
            <input
              type="checkbox"
              {...register('authentication.enabled')}
              disabled={!isAdmin}
              aria-label="Enable authentication monitoring"
            />
          </ToggleSwitch>
        </SectionHeader>

        <Input
          name="authentication.failuresPerMinute"
          type="number"
          label="Failed Attempts per Minute"
          disabled={!isAdmin || !watch('authentication.enabled')}
          error={errors.authentication?.failuresPerMinute}
          required
          {...register('authentication.failuresPerMinute')}
        />

        <Input
          name="authentication.lockoutThreshold"
          type="number"
          label="Account Lockout Threshold"
          disabled={!isAdmin || !watch('authentication.enabled')}
          error={errors.authentication?.lockoutThreshold}
          required
          {...register('authentication.lockoutThreshold')}
        />
      </ConfigSection>

      <ConfigSection variant="elevated">
        <SectionHeader>
          <SectionTitle>Resource Usage Alerts</SectionTitle>
          <ToggleSwitch disabled={!isAdmin}>
            <input
              type="checkbox"
              {...register('resourceUsage.enabled')}
              disabled={!isAdmin}
              aria-label="Enable resource usage alerts"
            />
          </ToggleSwitch>
        </SectionHeader>

        <Input
          name="resourceUsage.cpuThreshold"
          type="number"
          label="CPU Usage Threshold (%)"
          disabled={!isAdmin || !watch('resourceUsage.enabled')}
          error={errors.resourceUsage?.cpuThreshold}
          required
          {...register('resourceUsage.cpuThreshold')}
        />

        <Input
          name="resourceUsage.memoryThreshold"
          type="number"
          label="Memory Usage Threshold (%)"
          disabled={!isAdmin || !watch('resourceUsage.enabled')}
          error={errors.resourceUsage?.memoryThreshold}
          required
          {...register('resourceUsage.memoryThreshold')}
        />

        <Input
          name="resourceUsage.gpuThreshold"
          type="number"
          label="GPU Usage Threshold (%)"
          disabled={!isAdmin || !watch('resourceUsage.enabled')}
          error={errors.resourceUsage?.gpuThreshold}
          required
          {...register('resourceUsage.gpuThreshold')}
        />

        <Input
          name="resourceUsage.storageThreshold"
          type="number"
          label="Storage Usage Threshold (%)"
          disabled={!isAdmin || !watch('resourceUsage.enabled')}
          error={errors.resourceUsage?.storageThreshold}
          required
          {...register('resourceUsage.storageThreshold')}
        />
      </ConfigSection>

      <button
        type="submit"
        disabled={!isAdmin || isLoading}
        aria-busy={isLoading}
      >
        {isLoading ? 'Saving...' : 'Save Configuration'}
      </button>
    </ConfigContainer>
  );
};

export default AlertConfig;