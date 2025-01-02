import React from 'react';
import { render, screen, fireEvent, within, waitFor } from '@testing-library/react';
import { describe, it, expect, jest } from '@jest/globals';
import { axe, toHaveNoViolations } from '@axe-core/react';
import { ThemeProvider } from '@emotion/react';
import { Button, ButtonProps } from '../../../src/components/common/Button/Button';
import { lightTheme, darkTheme, ThemeMode } from '../../../src/constants/theme';

// @testing-library/react: ^14.0.0
// @jest/globals: ^29.5.0
// @axe-core/react: ^4.7.3
// @emotion/react: ^11.11.0

expect.extend(toHaveNoViolations);

const renderWithTheme = (props: Partial<ButtonProps>, mode: ThemeMode = ThemeMode.LIGHT) => {
  const theme = mode === ThemeMode.LIGHT ? lightTheme : darkTheme;
  return render(
    <ThemeProvider theme={theme}>
      <Button {...props} />
    </ThemeProvider>
  );
};

describe('Button Component', () => {
  // Mock handlers
  const mockClick = jest.fn();
  const mockSubmit = jest.fn();
  const MockIcon = () => <span data-testid="mock-icon">icon</span>;

  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('Rendering', () => {
    it('renders all variants correctly', () => {
      const variants: Array<ButtonProps['variant']> = ['contained', 'outlined', 'text'];
      variants.forEach(variant => {
        const { container } = renderWithTheme({ variant, children: 'Test Button' });
        expect(container.firstChild).toHaveAttribute('variant', variant);
      });
    });

    it('renders all sizes correctly', () => {
      const sizes: Array<ButtonProps['size']> = ['small', 'medium', 'large'];
      sizes.forEach(size => {
        const { container } = renderWithTheme({ size, children: 'Test Button' });
        expect(container.firstChild).toHaveAttribute('size', size);
      });
    });

    it('renders all color variants correctly', () => {
      const colors: Array<ButtonProps['color']> = ['primary', 'secondary', 'success', 'error', 'warning'];
      colors.forEach(color => {
        const { container } = renderWithTheme({ color, children: 'Test Button' });
        expect(container.firstChild).toHaveAttribute('color', color);
      });
    });

    it('renders with icons correctly', () => {
      const { getByTestId } = renderWithTheme({
        children: 'Test Button',
        startIcon: <MockIcon />,
        endIcon: <MockIcon />
      });
      
      expect(getByTestId('mock-icon')).toBeInTheDocument();
      expect(document.querySelector('.button-start-icon')).toBeInTheDocument();
      expect(document.querySelector('.button-end-icon')).toBeInTheDocument();
    });

    it('renders full width correctly', () => {
      const { container } = renderWithTheme({ fullWidth: true, children: 'Test Button' });
      expect(container.firstChild).toHaveStyle({ width: '100%' });
    });
  });

  describe('Behavior', () => {
    it('handles click events correctly', async () => {
      renderWithTheme({ onClick: mockClick, children: 'Test Button' });
      const button = screen.getByRole('button');
      
      fireEvent.click(button);
      expect(mockClick).toHaveBeenCalledTimes(1);
      
      // Test click event properties
      fireEvent.click(button);
      const lastCall = mockClick.mock.lastCall[0];
      expect(lastCall).toHaveProperty('currentTarget');
      expect(lastCall).toHaveProperty('preventDefault');
    });

    it('handles form submission correctly', async () => {
      render(
        <form onSubmit={mockSubmit}>
          <Button type="submit">Submit</Button>
        </form>
      );
      
      fireEvent.submit(screen.getByRole('button'));
      expect(mockSubmit).toHaveBeenCalledTimes(1);
    });

    it('handles loading state correctly', () => {
      const { rerender, getByText, queryByText } = renderWithTheme({
        loading: true,
        children: 'Test Button'
      });

      expect(getByText('Loading...')).toBeInTheDocument();
      expect(queryByText('Test Button')).not.toBeInTheDocument();

      rerender(
        <ThemeProvider theme={lightTheme}>
          <Button loading={false}>Test Button</Button>
        </ThemeProvider>
      );

      expect(queryByText('Loading...')).not.toBeInTheDocument();
      expect(getByText('Test Button')).toBeInTheDocument();
    });

    it('handles disabled state correctly', () => {
      const { container } = renderWithTheme({
        disabled: true,
        onClick: mockClick,
        children: 'Test Button'
      });

      const button = container.firstChild as HTMLButtonElement;
      expect(button).toBeDisabled();
      expect(button).toHaveAttribute('aria-disabled', 'true');

      fireEvent.click(button);
      expect(mockClick).not.toHaveBeenCalled();
    });
  });

  describe('Accessibility', () => {
    it('meets WCAG 2.1 Level AA standards', async () => {
      const { container } = renderWithTheme({ children: 'Test Button' });
      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    it('has correct ARIA attributes', () => {
      const { container } = renderWithTheme({
        ariaLabel: 'Test Button',
        ariaPressed: true,
        children: 'Test Button'
      });

      expect(container.firstChild).toHaveAttribute('aria-label', 'Test Button');
      expect(container.firstChild).toHaveAttribute('aria-pressed', 'true');
    });

    it('supports keyboard navigation', () => {
      renderWithTheme({ children: 'Test Button' });
      const button = screen.getByRole('button');

      button.focus();
      expect(document.activeElement).toBe(button);

      fireEvent.keyDown(button, { key: 'Enter' });
      fireEvent.keyDown(button, { key: ' ' });
      expect(button).toHaveAttribute('type', 'button');
    });

    it('maintains sufficient color contrast', async () => {
      const { container } = renderWithTheme({
        variant: 'contained',
        color: 'primary',
        children: 'Test Button'
      });

      const results = await axe(container, {
        rules: {
          'color-contrast': { enabled: true }
        }
      });
      expect(results).toHaveNoViolations();
    });
  });

  describe('Theme Integration', () => {
    it('renders correctly in light theme', () => {
      const { container } = renderWithTheme({ children: 'Test Button' }, ThemeMode.LIGHT);
      expect(container.firstChild).toHaveStyle({
        backgroundColor: lightTheme.primary.main
      });
    });

    it('renders correctly in dark theme', () => {
      const { container } = renderWithTheme({ children: 'Test Button' }, ThemeMode.DARK);
      expect(container.firstChild).toHaveStyle({
        backgroundColor: darkTheme.primary.main
      });
    });
  });

  describe('Animation and Visual Effects', () => {
    it('applies ripple effect correctly', async () => {
      const { container } = renderWithTheme({
        ripple: true,
        children: 'Test Button'
      });

      const button = container.firstChild as HTMLElement;
      fireEvent.mouseDown(button);
      
      await waitFor(() => {
        const rippleElement = button.querySelector('::before');
        expect(rippleElement).toBeTruthy();
      });
    });

    it('shows loading spinner animation', () => {
      const { container } = renderWithTheme({
        loading: true,
        children: 'Test Button'
      });

      const button = container.firstChild as HTMLElement;
      const styles = window.getComputedStyle(button);
      expect(styles.getPropertyValue('animation')).toBeTruthy();
    });
  });
});