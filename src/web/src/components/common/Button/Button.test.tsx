import React from 'react';
import { render, fireEvent, screen, waitFor } from '@testing-library/react';
import { expect, describe, it, beforeEach, afterEach } from '@jest/globals';
import { axe, toHaveNoViolations } from 'jest-axe';
import { Button, ButtonProps } from './Button';
import { ThemeProvider } from '@emotion/react';
import { lightTheme, darkTheme } from '../../constants/theme';

// Version comments for test dependencies
// @testing-library/react: ^13.4.0
// @jest/globals: ^29.5.0
// jest-axe: ^4.7.0

expect.extend(toHaveNoViolations);

const createTestProps = (overrides: Partial<ButtonProps> = {}): ButtonProps => ({
  children: 'Test Button',
  onClick: jest.fn(),
  ...overrides
});

const renderButton = (props: Partial<ButtonProps> = {}) => {
  const mergedProps = createTestProps(props);
  return render(
    <ThemeProvider theme={lightTheme}>
      <Button {...mergedProps} />
    </ThemeProvider>
  );
};

describe('Button Component', () => {
  let mockOnClick: jest.Mock;

  beforeEach(() => {
    mockOnClick = jest.fn();
  });

  afterEach(() => {
    jest.clearAllMocks();
  });

  describe('Rendering', () => {
    it('renders without crashing', () => {
      const { container } = renderButton();
      expect(container).toBeTruthy();
    });

    it('renders children correctly', () => {
      renderButton({ children: 'Click Me' });
      expect(screen.getByText('Click Me')).toBeInTheDocument();
    });

    it('applies default props correctly', () => {
      const { container } = renderButton();
      const button = container.firstChild as HTMLElement;
      expect(button).toHaveAttribute('type', 'button');
      expect(button).not.toHaveAttribute('disabled');
      expect(button).toHaveStyle({ width: 'auto' });
    });
  });

  describe('Variants', () => {
    it('renders contained variant with correct styles', () => {
      const { container } = renderButton({ variant: 'contained' });
      const button = container.firstChild as HTMLElement;
      expect(button).toHaveStyle({
        backgroundColor: lightTheme.primary.main,
        color: lightTheme.primary.contrast
      });
    });

    it('renders outlined variant with correct styles', () => {
      const { container } = renderButton({ variant: 'outlined' });
      const button = container.firstChild as HTMLElement;
      expect(button).toHaveStyle({
        backgroundColor: 'transparent',
        border: `1px solid ${lightTheme.primary.main}`
      });
    });

    it('renders text variant with correct styles', () => {
      const { container } = renderButton({ variant: 'text' });
      const button = container.firstChild as HTMLElement;
      expect(button).toHaveStyle({
        backgroundColor: 'transparent',
        border: 'none'
      });
    });
  });

  describe('Sizes', () => {
    it('applies small size styles correctly', () => {
      const { container } = renderButton({ size: 'small' });
      const button = container.firstChild as HTMLElement;
      expect(button).toHaveStyle({
        padding: '6px 16px',
        fontSize: '0.8125rem',
        minHeight: '32px'
      });
    });

    it('applies medium size styles correctly', () => {
      const { container } = renderButton({ size: 'medium' });
      const button = container.firstChild as HTMLElement;
      expect(button).toHaveStyle({
        padding: '8px 20px',
        fontSize: '0.875rem',
        minHeight: '36px'
      });
    });

    it('applies large size styles correctly', () => {
      const { container } = renderButton({ size: 'large' });
      const button = container.firstChild as HTMLElement;
      expect(button).toHaveStyle({
        padding: '12px 24px',
        fontSize: '0.9375rem',
        minHeight: '44px'
      });
    });
  });

  describe('States', () => {
    it('handles disabled state correctly', () => {
      const { container } = renderButton({ disabled: true });
      const button = container.firstChild as HTMLElement;
      expect(button).toBeDisabled();
      expect(button).toHaveStyle({ opacity: '0.6' });
      fireEvent.click(button);
      expect(mockOnClick).not.toHaveBeenCalled();
    });

    it('handles loading state correctly', () => {
      const { container, getByText } = renderButton({ loading: true });
      const button = container.firstChild as HTMLElement;
      expect(getByText('Loading...')).toBeInTheDocument();
      expect(button).toHaveStyle({ opacity: '0.8' });
      fireEvent.click(button);
      expect(mockOnClick).not.toHaveBeenCalled();
    });

    it('handles fullWidth prop correctly', () => {
      const { container } = renderButton({ fullWidth: true });
      const button = container.firstChild as HTMLElement;
      expect(button).toHaveStyle({ width: '100%' });
    });
  });

  describe('Interactions', () => {
    it('calls onClick handler when clicked', () => {
      const { container } = renderButton({ onClick: mockOnClick });
      const button = container.firstChild as HTMLElement;
      fireEvent.click(button);
      expect(mockOnClick).toHaveBeenCalledTimes(1);
    });

    it('handles keyboard interaction correctly', () => {
      const { container } = renderButton({ onClick: mockOnClick });
      const button = container.firstChild as HTMLElement;
      fireEvent.keyDown(button, { key: 'Enter' });
      expect(mockOnClick).toHaveBeenCalledTimes(1);
      fireEvent.keyDown(button, { key: 'Space' });
      expect(mockOnClick).toHaveBeenCalledTimes(2);
    });
  });

  describe('Icons', () => {
    it('renders start icon correctly', () => {
      const startIcon = <span data-testid="start-icon">→</span>;
      renderButton({ startIcon });
      expect(screen.getByTestId('start-icon')).toBeInTheDocument();
    });

    it('renders end icon correctly', () => {
      const endIcon = <span data-testid="end-icon">←</span>;
      renderButton({ endIcon });
      expect(screen.getByTestId('end-icon')).toBeInTheDocument();
    });

    it('hides icons during loading state', () => {
      const startIcon = <span data-testid="start-icon">→</span>;
      const endIcon = <span data-testid="end-icon">←</span>;
      renderButton({ startIcon, endIcon, loading: true });
      expect(screen.queryByTestId('start-icon')).not.toBeInTheDocument();
      expect(screen.queryByTestId('end-icon')).not.toBeInTheDocument();
    });
  });

  describe('Accessibility', () => {
    it('meets WCAG 2.1 accessibility guidelines', async () => {
      const { container } = renderButton({
        ariaLabel: 'Accessible Button',
        ariaPressed: false
      });
      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    it('applies correct ARIA attributes', () => {
      const { container } = renderButton({
        disabled: true,
        ariaLabel: 'Test Button',
        ariaPressed: true
      });
      const button = container.firstChild as HTMLElement;
      expect(button).toHaveAttribute('aria-label', 'Test Button');
      expect(button).toHaveAttribute('aria-pressed', 'true');
      expect(button).toHaveAttribute('aria-disabled', 'true');
    });

    it('maintains focus visibility', async () => {
      const { container } = renderButton();
      const button = container.firstChild as HTMLElement;
      button.focus();
      await waitFor(() => {
        expect(document.activeElement).toBe(button);
      });
    });
  });

  describe('Theme Integration', () => {
    it('applies theme colors correctly', () => {
      const { container, rerender } = render(
        <ThemeProvider theme={lightTheme}>
          <Button color="primary">Test</Button>
        </ThemeProvider>
      );
      expect(container.firstChild).toHaveStyle({
        backgroundColor: lightTheme.primary.main
      });

      rerender(
        <ThemeProvider theme={darkTheme}>
          <Button color="primary">Test</Button>
        </ThemeProvider>
      );
      expect(container.firstChild).toHaveStyle({
        backgroundColor: darkTheme.primary.main
      });
    });
  });
});