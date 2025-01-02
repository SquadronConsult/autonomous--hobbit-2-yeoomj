import React, { forwardRef } from 'react';
import styled from '@emotion/styled';
import { lightTheme, darkTheme, ThemeMode } from '../../constants/theme';

// Version comments for dependencies
// @emotion/styled: ^11.11.0
// react: ^18.0.0

interface ButtonProps {
  children: React.ReactNode;
  variant?: 'contained' | 'outlined' | 'text';
  size?: 'small' | 'medium' | 'large';
  color?: 'primary' | 'secondary' | 'success' | 'error' | 'warning';
  disabled?: boolean;
  loading?: boolean;
  fullWidth?: boolean;
  startIcon?: React.ReactNode;
  endIcon?: React.ReactNode;
  onClick?: (event: React.MouseEvent<HTMLButtonElement>) => void;
  type?: 'button' | 'submit' | 'reset';
  ariaLabel?: string;
  ariaPressed?: boolean;
  elevation?: number;
  ripple?: boolean;
  customStyles?: React.CSSProperties;
}

// Utility function to get theme-aware colors
const getThemeColors = (color: ButtonProps['color'] = 'primary', theme = lightTheme) => {
  switch (color) {
    case 'secondary':
      return theme.secondary;
    case 'success':
      return { main: theme.status.success, contrast: '#FFFFFF' };
    case 'error':
      return { main: theme.status.error, contrast: '#FFFFFF' };
    case 'warning':
      return { main: theme.status.warning, contrast: '#FFFFFF' };
    default:
      return theme.primary;
  }
};

// Styled button base with comprehensive styling
const StyledButton = styled.button<ButtonProps>`
  /* Base styles */
  font-family: var(--font-family-primary, 'Roboto, sans-serif');
  font-weight: 500;
  border-radius: 4px;
  transition: all 200ms cubic-bezier(0.4, 0, 0.2, 1);
  cursor: ${props => props.disabled ? 'not-allowed' : 'pointer'};
  display: inline-flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  outline: none;
  position: relative;
  overflow: hidden;
  user-select: none;
  touch-action: manipulation;
  -webkit-tap-highlight-color: transparent;
  width: ${props => props.fullWidth ? '100%' : 'auto'};

  /* Size variants */
  ${props => {
    switch (props.size) {
      case 'small':
        return `
          padding: 6px 16px;
          font-size: 0.8125rem;
          min-height: 32px;
        `;
      case 'large':
        return `
          padding: 12px 24px;
          font-size: 0.9375rem;
          min-height: 44px;
        `;
      default: // medium
        return `
          padding: 8px 20px;
          font-size: 0.875rem;
          min-height: 36px;
        `;
    }
  }}

  /* Variant styles */
  ${props => {
    const colors = getThemeColors(props.color);
    switch (props.variant) {
      case 'outlined':
        return `
          background-color: transparent;
          border: 1px solid ${colors.main};
          color: ${colors.main};

          &:hover:not(:disabled) {
            background-color: ${colors.main}14;
          }
        `;
      case 'text':
        return `
          background-color: transparent;
          border: none;
          color: ${colors.main};

          &:hover:not(:disabled) {
            background-color: ${colors.main}14;
          }
        `;
      default: // contained
        return `
          background-color: ${colors.main};
          border: none;
          color: ${colors.contrast};
          box-shadow: ${props.elevation ? lightTheme.elevation[props.elevation as keyof typeof lightTheme.elevation] : 'none'};

          &:hover:not(:disabled) {
            background-color: ${colors.dark || colors.main};
            box-shadow: ${lightTheme.elevation[2]};
          }
        `;
    }
  }}

  /* Disabled state */
  ${props => props.disabled && `
    opacity: 0.6;
    pointer-events: none;
  `}

  /* Loading state */
  ${props => props.loading && `
    pointer-events: none;
    opacity: 0.8;
    &::after {
      content: '';
      position: absolute;
      width: 16px;
      height: 16px;
      border: 2px solid currentColor;
      border-radius: 50%;
      border-top-color: transparent;
      animation: button-spin 0.8s linear infinite;
    }
  `}

  /* Focus styles */
  &:focus-visible {
    outline: 2px solid ${props => getThemeColors(props.color).main};
    outline-offset: 2px;
  }

  /* Ripple effect */
  ${props => props.ripple && `
    &::before {
      content: '';
      position: absolute;
      top: 50%;
      left: 50%;
      width: 100%;
      height: 100%;
      background-color: currentColor;
      border-radius: 50%;
      transform: translate(-50%, -50%) scale(0);
      opacity: 0.3;
      transition: transform 0.3s ease-out;
    }

    &:active::before {
      transform: translate(-50%, -50%) scale(2);
      opacity: 0;
    }
  `}

  @keyframes button-spin {
    0% { transform: rotate(0deg); }
    100% { transform: rotate(360deg); }
  }
`;

export const Button = forwardRef<HTMLButtonElement, ButtonProps>(({
  children,
  variant = 'contained',
  size = 'medium',
  color = 'primary',
  disabled = false,
  loading = false,
  fullWidth = false,
  startIcon,
  endIcon,
  onClick,
  type = 'button',
  ariaLabel,
  ariaPressed,
  elevation = 1,
  ripple = true,
  customStyles,
  ...props
}, ref) => {
  return (
    <StyledButton
      ref={ref}
      variant={variant}
      size={size}
      color={color}
      disabled={disabled || loading}
      loading={loading}
      fullWidth={fullWidth}
      onClick={onClick}
      type={type}
      aria-label={ariaLabel}
      aria-pressed={ariaPressed}
      aria-disabled={disabled}
      elevation={elevation}
      ripple={ripple}
      style={customStyles}
      {...props}
    >
      {startIcon && !loading && <span className="button-start-icon">{startIcon}</span>}
      {loading ? <span className="button-loading-text">Loading...</span> : children}
      {endIcon && !loading && <span className="button-end-icon">{endIcon}</span>}
    </StyledButton>
  );
});

Button.displayName = 'Button';

export type { ButtonProps };