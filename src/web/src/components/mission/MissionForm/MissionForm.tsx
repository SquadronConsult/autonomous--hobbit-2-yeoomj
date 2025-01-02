/**
 * @fileoverview Enhanced mission form component for agricultural operations
 * @version 1.0.0
 * 
 * Provides a comprehensive form interface for creating and editing agricultural missions
 * with support for real-time updates, offline operations, and enhanced validation.
 */

import React, { useState, useEffect, useMemo } from 'react';
import { useForm, FormProvider } from 'react-hook-form'; // v7.45.0
import * as yup from 'yup'; // v1.2.0
import { debounce } from 'lodash'; // v4.17.21
import { IMission, MissionParameters } from '../../../interfaces/IMission';
import { useWebSocket } from '../../../hooks/useWebSocket';
import { MissionTypes } from '../../../constants/missionTypes';

// Form validation schema
const validationSchema = yup.object().shape({
  name: yup.string()
    .required('Mission name is required')
    .min(3, 'Name must be at least 3 characters')
    .max(50, 'Name must not exceed 50 characters'),
  description: yup.string()
    .required('Description is required')
    .max(500, 'Description must not exceed 500 characters'),
  type: yup.string()
    .required('Mission type is required')
    .oneOf(Object.values(MissionTypes)),
  startTime: yup.date()
    .required('Start time is required')
    .min(new Date(), 'Start time must be in the future'),
  coverageArea: yup.object()
    .required('Coverage area is required')
    .test('valid-area', 'Invalid coverage area', (value) => {
      return value?.coordinates?.[0]?.length >= 3;
    }),
  parameters: yup.object().shape({
    altitude: yup.number()
      .required('Altitude is required')
      .min(10, 'Minimum altitude is 10m')
      .max(120, 'Maximum altitude is 120m'),
    speed: yup.number()
      .required('Speed is required')
      .min(1, 'Minimum speed is 1m/s')
      .max(15, 'Maximum speed is 15m/s'),
    scanResolution: yup.number()
      .when('type', {
        is: MissionTypes.SURVEY,
        then: yup.number().required('Scan resolution is required')
      }),
    treatmentDensity: yup.number()
      .when('type', {
        is: MissionTypes.TREATMENT,
        then: yup.number().required('Treatment density is required')
      })
  }),
  offlineEnabled: yup.boolean(),
  priority: yup.number()
    .min(1)
    .max(5)
    .default(3),
});

interface MissionFormProps {
  mission?: IMission | null;
  onSubmit: (mission: IMission) => Promise<void>;
  onCancel: () => void;
  onProgress?: (progress: number) => void;
  autoSave?: boolean;
  template?: IMission | null;
}

export const MissionForm: React.FC<MissionFormProps> = ({
  mission,
  onSubmit,
  onCancel,
  onProgress,
  autoSave = true,
  template
}) => {
  const [isSaving, setIsSaving] = useState(false);
  const [offlineMode, setOfflineMode] = useState(false);
  const [validationErrors, setValidationErrors] = useState<string[]>([]);

  const { isConnected, connectionStatus, subscribe } = useWebSocket({
    autoConnect: true,
    enableReconnect: true
  });

  // Initialize form with validation
  const methods = useForm({
    defaultValues: useMemo(() => ({
      ...mission || template || {
        name: '',
        description: '',
        type: MissionTypes.SURVEY,
        startTime: new Date(),
        coverageArea: null,
        parameters: {
          altitude: 30,
          speed: 5,
          scanResolution: 2,
          treatmentDensity: 0,
          treatmentType: ''
        },
        offlineEnabled: false,
        priority: 3
      }
    }), [mission, template]),
    mode: 'onChange',
    resolver: async (data) => {
      try {
        await validationSchema.validate(data, { abortEarly: false });
        return { values: data, errors: {} };
      } catch (errors) {
        return {
          values: {},
          errors: errors.inner.reduce(
            (acc, err) => ({
              ...acc,
              [err.path]: err.message
            }),
            {}
          )
        };
      }
    }
  });

  // Auto-save functionality
  const debouncedSave = useMemo(
    () => debounce(async (data: IMission) => {
      if (autoSave && !offlineMode) {
        try {
          await onSubmit(data);
        } catch (error) {
          console.error('Auto-save failed:', error);
        }
      }
    }, 2000),
    [autoSave, offlineMode, onSubmit]
  );

  // Subscribe to real-time updates
  useEffect(() => {
    if (mission?.id) {
      const unsubscribe = subscribe(`mission.${mission.id}`, (update) => {
        methods.reset({ ...methods.getValues(), ...update });
      });
      return () => unsubscribe();
    }
  }, [mission?.id, subscribe, methods]);

  // Handle form submission
  const handleSubmit = async (data: IMission) => {
    try {
      setIsSaving(true);
      await onSubmit(data);
      setValidationErrors([]);
    } catch (error) {
      setValidationErrors([error.message]);
    } finally {
      setIsSaving(false);
    }
  };

  return (
    <FormProvider {...methods}>
      <form 
        onSubmit={methods.handleSubmit(handleSubmit)}
        className="form-container"
        noValidate
      >
        {!isConnected && (
          <div className="offline-indicator">
            Offline Mode - Changes will be synchronized when connection is restored
          </div>
        )}

        <div className="form-section">
          <label htmlFor="name">Mission Name</label>
          <input
            id="name"
            type="text"
            {...methods.register('name')}
            aria-invalid={!!methods.formState.errors.name}
          />
          {methods.formState.errors.name && (
            <span className="error">{methods.formState.errors.name.message}</span>
          )}
        </div>

        <div className="form-section">
          <label htmlFor="type">Mission Type</label>
          <select
            id="type"
            {...methods.register('type')}
            aria-invalid={!!methods.formState.errors.type}
          >
            {Object.values(MissionTypes).map((type) => (
              <option key={type} value={type}>
                {type}
              </option>
            ))}
          </select>
        </div>

        <div className="form-section">
          <label htmlFor="startTime">Start Time</label>
          <input
            id="startTime"
            type="datetime-local"
            {...methods.register('startTime')}
            aria-invalid={!!methods.formState.errors.startTime}
          />
        </div>

        <div className="map-container">
          {/* Map component for coverage area selection */}
        </div>

        <div className="form-section">
          <label htmlFor="parameters.altitude">Altitude (m)</label>
          <input
            id="parameters.altitude"
            type="number"
            {...methods.register('parameters.altitude')}
            min={10}
            max={120}
          />
        </div>

        <div className="form-section">
          <label htmlFor="parameters.speed">Speed (m/s)</label>
          <input
            id="parameters.speed"
            type="number"
            {...methods.register('parameters.speed')}
            min={1}
            max={15}
          />
        </div>

        {methods.watch('type') === MissionTypes.SURVEY && (
          <div className="form-section">
            <label htmlFor="parameters.scanResolution">Scan Resolution (cm)</label>
            <input
              id="parameters.scanResolution"
              type="number"
              {...methods.register('parameters.scanResolution')}
            />
          </div>
        )}

        {methods.watch('type') === MissionTypes.TREATMENT && (
          <div className="form-section">
            <label htmlFor="parameters.treatmentDensity">Treatment Density (L/ha)</label>
            <input
              id="parameters.treatmentDensity"
              type="number"
              {...methods.register('parameters.treatmentDensity')}
            />
          </div>
        )}

        <div className="form-section">
          <label htmlFor="offlineEnabled">
            <input
              id="offlineEnabled"
              type="checkbox"
              {...methods.register('offlineEnabled')}
            />
            Enable Offline Operation
          </label>
        </div>

        <div className="form-actions">
          <button
            type="button"
            onClick={onCancel}
            disabled={isSaving}
          >
            Cancel
          </button>
          <button
            type="submit"
            disabled={isSaving || !methods.formState.isValid}
          >
            {isSaving ? 'Saving...' : 'Save Mission'}
          </button>
        </div>

        {validationErrors.length > 0 && (
          <div className="error-summary">
            {validationErrors.map((error, index) => (
              <p key={index} className="error">{error}</p>
            ))}
          </div>
        )}
      </form>
    </FormProvider>
  );
};

export default MissionForm;