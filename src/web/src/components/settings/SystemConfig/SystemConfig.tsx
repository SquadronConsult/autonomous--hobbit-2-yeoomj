import React, { useCallback, useEffect } from 'react';
import styled from '@emotion/styled';
import { useForm, Controller } from 'react-hook-form';
import debounce from 'lodash/debounce';

import Card from '../../common/Card/Card';
import Input from '../../common/Input/Input';
import { ThemeMode } from '../../../constants/theme';
import { validateEmail, validateRange } from '../../../utils/validation';

// Performance mode options for system optimization
export enum PerformanceMode {
  BALANCED = 'balanced',
  PERFORMANCE = 'performance',
  EFFICIENCY = 'efficiency'
}

// Interface for system configuration values
export interface SystemConfigValues {
  themeMode: ThemeMode;
  dataRetentionDays: number;
  maxMissions: number;
  notificationEmail: string;
  alertThreshold: number;
  performanceMode: PerformanceMode;
}

// Props interface for the SystemConfig component
export interface SystemConfigProps {
  onSave: (config: SystemConfigValues) => Promise<void>;
  initialValues: SystemConfigValues;
  onCancel: () => void;
}

// Styled components with Material Design 3.0 specifications
const ConfigContainer = styled.div`
  display: flex;
  flex-direction: column;
  gap: 24px;
  padding: 24px;
  max-width: 800px;
  margin: 0 auto;
  min-height: calc(100vh - 200px);
`;

const ConfigSection = styled(Card)`
  display: flex;
  flex-direction: column;
  gap: 16px;
  padding: 24px;
`;

const SectionTitle = styled.h2`
  font-size: 20px;
  font-weight: 500;
  color: ${props => props.theme.text.primary};
  margin: 0 0 16px 0;
`;

const ButtonGroup = styled.div`
  display: flex;
  justify-content: flex-end;
  gap: 16px;
  margin-top: 24px;
  padding: 16px;
  border-top: 1px solid ${props => props.theme.text.disabled};
`;

const Button = styled.button<{ variant?: 'primary' | 'secondary' }>`
  padding: 12px 24px;
  border-radius: 4px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s ease-in-out;
  
  ${props => props.variant === 'primary' ? `
    background-color: ${props.theme.primary.main};
    color: ${props.theme.primary.contrast};
    border: none;
    
    &:hover {
      background-color: ${props.theme.primary.dark};
    }
  ` : `
    background-color: transparent;
    color: ${props.theme.text.primary};
    border: 1px solid ${props.theme.text.disabled};
    
    &:hover {
      background-color: ${props.theme.background.elevated};
    }
  `}
`;

const Select = styled.select`
  width: 100%;
  padding: 12px 16px;
  border-radius: 4px;
  border: 1px solid ${props => props.theme.text.disabled};
  background-color: ${props => props.theme.background.paper};
  color: ${props => props.theme.text.primary};
  font-size: 16px;
  
  &:focus {
    outline: none;
    border-color: ${props => props.theme.primary.main};
    box-shadow: 0 0 0 2px ${props => `${props.theme.primary.main}33`};
  }
`;

export const SystemConfig: React.FC<SystemConfigProps> = ({
  onSave,
  initialValues,
  onCancel
}) => {
  const { control, handleSubmit, formState: { errors }, reset } = useForm<SystemConfigValues>({
    defaultValues: initialValues
  });

  // Reset form when initial values change
  useEffect(() => {
    reset(initialValues);
  }, [initialValues, reset]);

  // Debounced save handler
  const debouncedSave = useCallback(
    debounce(async (values: SystemConfigValues) => {
      try {
        await onSave(values);
      } catch (error) {
        console.error('Failed to save configuration:', error);
      }
    }, 500),
    [onSave]
  );

  // Form submission handler
  const onSubmit = useCallback((values: SystemConfigValues) => {
    debouncedSave(values);
  }, [debouncedSave]);

  return (
    <ConfigContainer>
      <form onSubmit={handleSubmit(onSubmit)} aria-label="System Configuration Form">
        <ConfigSection variant="elevated">
          <SectionTitle>Theme Settings</SectionTitle>
          <Controller
            name="themeMode"
            control={control}
            render={({ field }) => (
              <Select
                {...field}
                aria-label="Theme Mode"
              >
                <option value={ThemeMode.LIGHT}>Light</option>
                <option value={ThemeMode.DARK}>Dark</option>
                <option value={ThemeMode.SYSTEM}>System</option>
              </Select>
            )}
          />
        </ConfigSection>

        <ConfigSection variant="elevated">
          <SectionTitle>Data Management</SectionTitle>
          <Controller
            name="dataRetentionDays"
            control={control}
            rules={{
              validate: value => validateRange(value, 30, 365) || 'Must be between 30 and 365 days'
            }}
            render={({ field }) => (
              <Input
                {...field}
                type="number"
                label="Data Retention Period (days)"
                error={errors.dataRetentionDays}
                aria-label="Data Retention Period"
              />
            )}
          />
        </ConfigSection>

        <ConfigSection variant="elevated">
          <SectionTitle>Performance Settings</SectionTitle>
          <Controller
            name="maxMissions"
            control={control}
            rules={{
              validate: value => validateRange(value, 1, 24) || 'Must be between 1 and 24 missions'
            }}
            render={({ field }) => (
              <Input
                {...field}
                type="number"
                label="Maximum Concurrent Missions"
                error={errors.maxMissions}
                aria-label="Maximum Concurrent Missions"
              />
            )}
          />
          <Controller
            name="performanceMode"
            control={control}
            render={({ field }) => (
              <Select
                {...field}
                aria-label="Performance Mode"
              >
                <option value={PerformanceMode.BALANCED}>Balanced</option>
                <option value={PerformanceMode.PERFORMANCE}>Performance</option>
                <option value={PerformanceMode.EFFICIENCY}>Efficiency</option>
              </Select>
            )}
          />
        </ConfigSection>

        <ConfigSection variant="elevated">
          <SectionTitle>Notifications</SectionTitle>
          <Controller
            name="notificationEmail"
            control={control}
            rules={{
              validate: value => validateEmail(value) || 'Invalid email address'
            }}
            render={({ field }) => (
              <Input
                {...field}
                type="email"
                label="Notification Email"
                error={errors.notificationEmail}
                aria-label="Notification Email"
              />
            )}
          />
          <Controller
            name="alertThreshold"
            control={control}
            rules={{
              validate: value => validateRange(value, 0, 100) || 'Must be between 0 and 100'
            }}
            render={({ field }) => (
              <Input
                {...field}
                type="number"
                label="Alert Threshold (%)"
                error={errors.alertThreshold}
                aria-label="Alert Threshold"
              />
            )}
          />
        </ConfigSection>

        <ButtonGroup>
          <Button
            type="button"
            onClick={onCancel}
            aria-label="Cancel Configuration"
          >
            Cancel
          </Button>
          <Button
            type="submit"
            variant="primary"
            aria-label="Save Configuration"
          >
            Save Changes
          </Button>
        </ButtonGroup>
      </form>
    </ConfigContainer>
  );
};

export default SystemConfig;