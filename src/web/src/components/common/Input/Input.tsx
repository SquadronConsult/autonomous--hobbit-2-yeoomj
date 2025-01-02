import React, { useCallback, useRef, useState, useId } from 'react';
import styled from '@emotion/styled';
import { ValidationError } from '../../../utils/validation';
import { ThemeMode } from '../../../constants/theme';

// Input types supported by the component
type InputType = 'text' | 'number' | 'email' | 'password' | 'search' | 'tel' | 'url';

// Enhanced props interface with validation and accessibility support
export interface InputProps {
  name: string;
  type: InputType;
  value?: string;
  placeholder?: string;
  label?: string;
  error?: ValidationError;
  disabled?: boolean;
  required?: boolean;
  validationRules?: Record<string, any>;
  onChange: (value: string, context?: object) => void;
  onBlur?: () => void;
  'aria-describedby'?: string;
  'aria-label'?: string;
}

// Styled input container with Material Design 3.0 support
const InputContainer = styled.div<{ error?: boolean; disabled?: boolean }>`
  position: relative;
  width: 100%;
  margin-bottom: ${({ error }) => (error ? '24px' : '16px')};
  opacity: ${({ disabled }) => (disabled ? '0.5' : '1')};
`;

// Enhanced styled input with Material Design 3.0 specifications
const StyledInput = styled.input<{
  error?: boolean;
  hasLabel?: boolean;
  theme: any;
}>`
  width: 100%;
  padding: 12px 16px;
  font-size: 16px;
  line-height: 1.5;
  color: ${({ theme }) => theme.text.primary};
  background-color: ${({ theme }) => theme.background.paper};
  border: 1px solid ${({ theme, error }) =>
    error ? theme.status.error : theme.text.disabled};
  border-radius: 4px;
  outline: none;
  transition: all 0.2s ease-in-out;

  &:hover:not(:disabled) {
    border-color: ${({ theme, error }) =>
      error ? theme.status.error : theme.primary.main};
  }

  &:focus:not(:disabled) {
    border-color: ${({ theme, error }) =>
      error ? theme.status.error : theme.primary.main};
    box-shadow: 0 0 0 2px ${({ theme, error }) =>
      error ? `${theme.status.error}33` : `${theme.primary.main}33`};
  }

  &:disabled {
    cursor: not-allowed;
    background-color: ${({ theme }) => theme.background.elevated};
  }

  ${({ hasLabel }) => hasLabel && `
    margin-top: 8px;
  `}

  &::placeholder {
    color: ${({ theme }) => theme.text.disabled};
    opacity: 1;
  }
`;

// Styled label with enhanced accessibility
const StyledLabel = styled.label<{ required?: boolean; error?: boolean; theme: any }>`
  display: block;
  margin-bottom: 4px;
  font-size: 14px;
  font-weight: 500;
  color: ${({ theme, error }) =>
    error ? theme.status.error : theme.text.primary};

  ${({ required }) =>
    required &&
    `
    &::after {
      content: '*';
      margin-left: 4px;
      color: ${({ theme }) => theme.status.error};
    }
  `}
`;

// Error message with screen reader support
const ErrorMessage = styled.div`
  position: absolute;
  bottom: -20px;
  left: 0;
  font-size: 12px;
  color: ${({ theme }) => theme.status.error};
`;

// Debounce utility for validation
const useDebounce = (callback: Function, delay: number) => {
  const timeoutRef = useRef<NodeJS.Timeout>();

  return useCallback(
    (...args: any[]) => {
      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
      timeoutRef.current = setTimeout(() => callback(...args), delay);
    },
    [callback, delay]
  );
};

// Enhanced Input component with validation and accessibility features
export const Input: React.FC<InputProps> = React.memo(({
  name,
  type,
  value = '',
  placeholder,
  label,
  error,
  disabled = false,
  required = false,
  validationRules,
  onChange,
  onBlur,
  'aria-describedby': ariaDescribedBy,
  'aria-label': ariaLabel,
}) => {
  const [isFocused, setIsFocused] = useState(false);
  const inputId = useId();
  const errorId = error ? `${inputId}-error` : undefined;
  
  // Debounced validation handler
  const debouncedValidation = useDebounce((value: string) => {
    if (validationRules) {
      try {
        // Apply validation rules
        Object.entries(validationRules).forEach(([rule, config]) => {
          // Add your validation logic here
        });
      } catch (err) {
        if (err instanceof ValidationError) {
          // Handle validation error
        }
      }
    }
  }, 300);

  // Enhanced change handler with validation
  const handleChange = useCallback(
    (event: React.ChangeEvent<HTMLInputElement>) => {
      const newValue = event.target.value;
      onChange(newValue);
      debouncedValidation(newValue);
    },
    [onChange, debouncedValidation]
  );

  // Focus handlers for accessibility
  const handleFocus = () => setIsFocused(true);
  const handleBlur = () => {
    setIsFocused(false);
    onBlur?.();
  };

  return (
    <InputContainer error={!!error} disabled={disabled}>
      {label && (
        <StyledLabel
          htmlFor={inputId}
          required={required}
          error={!!error}
        >
          {label}
        </StyledLabel>
      )}
      <StyledInput
        id={inputId}
        name={name}
        type={type}
        value={value}
        onChange={handleChange}
        onFocus={handleFocus}
        onBlur={handleBlur}
        disabled={disabled}
        required={required}
        placeholder={placeholder}
        aria-invalid={!!error}
        aria-required={required}
        aria-describedby={error ? errorId : ariaDescribedBy}
        aria-label={!label ? ariaLabel : undefined}
        hasLabel={!!label}
      />
      {error && (
        <ErrorMessage
          id={errorId}
          role="alert"
        >
          {error.message}
        </ErrorMessage>
      )}
    </InputContainer>
  );
});

Input.displayName = 'Input';

export default Input;