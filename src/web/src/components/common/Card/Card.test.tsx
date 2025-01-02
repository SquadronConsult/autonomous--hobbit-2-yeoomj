import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { axe, toHaveNoViolations } from 'jest-axe';
import { ThemeProvider } from '@emotion/react';
import { lightTheme, darkTheme } from '../../constants/theme';
import Card from './Card';

// Extend Jest matchers
expect.extend(toHaveNoViolations);

describe('Card', () => {
  // Test setup helper to render with theme
  const renderWithTheme = (ui: React.ReactElement, theme = lightTheme) => {
    return render(
      <ThemeProvider theme={theme}>
        {ui}
      </ThemeProvider>
    );
  };

  describe('rendering', () => {
    test('renders with default props', () => {
      renderWithTheme(<Card>Content</Card>);
      expect(screen.getByText('Content')).toBeInTheDocument();
    });

    test('applies flat variant styles correctly', () => {
      const { container } = renderWithTheme(<Card variant="flat">Content</Card>);
      expect(container.firstChild).toHaveStyle({
        borderRadius: '8px',
        padding: '16px'
      });
    });

    test('applies elevated variant styles correctly', () => {
      const { container } = renderWithTheme(<Card variant="elevated">Content</Card>);
      expect(container.firstChild).toHaveStyle({
        borderRadius: '8px',
        padding: '16px'
      });
      // Check for box-shadow presence
      expect(container.firstChild).toHaveStyle({
        boxShadow: expect.any(String)
      });
    });

    test('applies custom className', () => {
      const { container } = renderWithTheme(
        <Card className="custom-class">Content</Card>
      );
      expect(container.firstChild).toHaveClass('custom-class');
    });
  });

  describe('theming', () => {
    test('applies light theme styles correctly', () => {
      const { container } = renderWithTheme(<Card>Content</Card>, lightTheme);
      expect(container.firstChild).toHaveStyle({
        backgroundColor: lightTheme.background.paper
      });
    });

    test('applies dark theme styles correctly', () => {
      const { container } = renderWithTheme(<Card>Content</Card>, darkTheme);
      expect(container.firstChild).toHaveStyle({
        backgroundColor: darkTheme.background.paper
      });
    });

    test('handles theme transitions', () => {
      const { container, rerender } = renderWithTheme(<Card>Content</Card>, lightTheme);
      expect(container.firstChild).toHaveStyle({
        transition: 'all 0.3s cubic-bezier(0.4, 0, 0.2, 1)'
      });
      
      // Rerender with dark theme
      rerender(
        <ThemeProvider theme={darkTheme}>
          <Card>Content</Card>
        </ThemeProvider>
      );
      expect(container.firstChild).toHaveStyle({
        backgroundColor: darkTheme.background.paper
      });
    });
  });

  describe('interactions', () => {
    test('handles click events', async () => {
      const handleClick = jest.fn();
      renderWithTheme(
        <Card onClick={handleClick}>Clickable</Card>
      );
      
      await userEvent.click(screen.getByText('Clickable'));
      expect(handleClick).toHaveBeenCalledTimes(1);
    });

    test('handles keyboard interactions', async () => {
      const handleClick = jest.fn();
      renderWithTheme(
        <Card onClick={handleClick}>Interactive</Card>
      );
      
      const card = screen.getByText('Interactive');
      fireEvent.keyDown(card, { key: 'Enter' });
      expect(handleClick).toHaveBeenCalledTimes(1);

      fireEvent.keyDown(card, { key: ' ' });
      expect(handleClick).toHaveBeenCalledTimes(2);
    });

    test('shows correct hover states for elevated variant', async () => {
      const { container } = renderWithTheme(
        <Card variant="elevated" onClick={() => {}}>Content</Card>
      );
      
      // Simulate hover
      fireEvent.mouseOver(container.firstChild as Element);
      expect(container.firstChild).toHaveStyle({
        transform: 'translateY(-2px)'
      });

      // Simulate mouse leave
      fireEvent.mouseLeave(container.firstChild as Element);
      expect(container.firstChild).toHaveStyle({
        transform: 'translateY(0)'
      });
    });
  });

  describe('accessibility', () => {
    test('meets WCAG accessibility guidelines', async () => {
      const { container } = renderWithTheme(
        <Card ariaLabel="Test card">Accessible content</Card>
      );
      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    test('has correct ARIA attributes when interactive', () => {
      renderWithTheme(
        <Card onClick={() => {}} ariaLabel="Interactive card">
          Content
        </Card>
      );
      
      const card = screen.getByRole('button');
      expect(card).toHaveAttribute('aria-label', 'Interactive card');
      expect(card).toHaveAttribute('tabIndex', '0');
    });

    test('supports keyboard focus management', () => {
      renderWithTheme(
        <Card onClick={() => {}}>Focusable</Card>
      );
      
      const card = screen.getByRole('button');
      card.focus();
      expect(card).toHaveFocus();
    });
  });

  describe('responsive', () => {
    beforeEach(() => {
      // Mock window.matchMedia for responsive tests
      Object.defineProperty(window, 'matchMedia', {
        writable: true,
        value: jest.fn().mockImplementation(query => ({
          matches: false,
          media: query,
          onchange: null,
          addListener: jest.fn(),
          removeListener: jest.fn(),
          addEventListener: jest.fn(),
          removeEventListener: jest.fn(),
          dispatchEvent: jest.fn(),
        })),
      });
    });

    test('adapts padding for mobile viewport', () => {
      // Mock mobile viewport
      window.matchMedia = jest.fn().mockImplementation(query => ({
        matches: query === '(max-width: 768px)',
        media: query,
        onchange: null,
        addListener: jest.fn(),
        removeListener: jest.fn(),
        addEventListener: jest.fn(),
        removeEventListener: jest.fn(),
        dispatchEvent: jest.fn(),
      }));

      const { container } = renderWithTheme(<Card>Mobile content</Card>);
      expect(container.firstChild).toHaveStyle({
        padding: '12px'
      });
    });

    test('maintains touch target size on mobile', () => {
      const { container } = renderWithTheme(
        <Card onClick={() => {}}>Touch target</Card>
      );
      
      const card = container.firstChild as HTMLElement;
      const styles = window.getComputedStyle(card);
      const height = parseFloat(styles.height);
      
      // Minimum touch target size should be 44px per WCAG 2.1
      expect(height).toBeGreaterThanOrEqual(44);
    });
  });
});