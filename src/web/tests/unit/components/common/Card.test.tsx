import React from 'react';
import { render, screen, fireEvent, within } from '@testing-library/react';
import { ThemeProvider } from '@emotion/react';
import { axe, toHaveNoViolations } from 'jest-axe';
import Card from '../../../src/components/common/Card/Card';
import { lightTheme } from '../../../src/constants/theme';

// Add jest-axe matchers
expect.extend(toHaveNoViolations);

// Helper function to render components with theme context
const renderWithTheme = (component: React.ReactNode, theme = lightTheme) => {
  return render(
    <ThemeProvider theme={theme}>
      {component}
    </ThemeProvider>
  );
};

describe('Card', () => {
  // Mock handlers
  const mockOnClick = jest.fn();
  const mockKeyPress = jest.fn();

  // Reset mocks before each test
  beforeEach(() => {
    mockOnClick.mockReset();
    mockKeyPress.mockReset();
  });

  describe('Rendering', () => {
    it('renders children content correctly', () => {
      const testContent = 'Test Card Content';
      renderWithTheme(<Card>{testContent}</Card>);
      
      expect(screen.getByText(testContent)).toBeInTheDocument();
    });

    it('applies default flat variant styles', () => {
      const { container } = renderWithTheme(<Card>Content</Card>);
      const card = container.firstChild as HTMLElement;
      
      expect(card).toHaveStyle({
        borderRadius: '8px',
        padding: '16px',
        backgroundColor: lightTheme.background.paper,
        border: `1px solid ${lightTheme.text.disabled}`
      });
    });

    it('renders with custom className', () => {
      const customClass = 'custom-card';
      renderWithTheme(<Card className={customClass}>Content</Card>);
      
      expect(screen.getByText('Content').parentElement).toHaveClass(customClass);
    });
  });

  describe('Variants', () => {
    it('applies elevated variant styles correctly', () => {
      const { container } = renderWithTheme(
        <Card variant="elevated">Content</Card>
      );
      const card = container.firstChild as HTMLElement;
      
      expect(card).toHaveStyle({
        boxShadow: lightTheme.elevation[1]
      });
      expect(card).not.toHaveStyle({
        border: `1px solid ${lightTheme.text.disabled}`
      });
    });

    it('transitions elevation on hover for elevated variant', async () => {
      const { container } = renderWithTheme(
        <Card variant="elevated" onClick={mockOnClick}>Content</Card>
      );
      const card = container.firstChild as HTMLElement;
      
      fireEvent.mouseOver(card);
      
      expect(card).toHaveStyle({
        boxShadow: lightTheme.elevation[2],
        transform: 'translateY(-2px)'
      });
    });
  });

  describe('Interaction', () => {
    it('handles click events when interactive', () => {
      renderWithTheme(
        <Card onClick={mockOnClick}>Interactive Card</Card>
      );
      
      fireEvent.click(screen.getByText('Interactive Card'));
      expect(mockOnClick).toHaveBeenCalledTimes(1);
    });

    it('supports keyboard navigation with Enter key', () => {
      renderWithTheme(
        <Card onClick={mockOnClick}>Interactive Card</Card>
      );
      
      const card = screen.getByText('Interactive Card').parentElement;
      fireEvent.keyDown(card!, { key: 'Enter' });
      
      expect(mockOnClick).toHaveBeenCalledTimes(1);
    });

    it('supports keyboard navigation with Space key', () => {
      renderWithTheme(
        <Card onClick={mockOnClick}>Interactive Card</Card>
      );
      
      const card = screen.getByText('Interactive Card').parentElement;
      fireEvent.keyDown(card!, { key: ' ' });
      
      expect(mockOnClick).toHaveBeenCalledTimes(1);
    });
  });

  describe('Accessibility', () => {
    it('meets WCAG accessibility standards', async () => {
      const { container } = renderWithTheme(
        <Card ariaLabel="Test card">Accessible Content</Card>
      );
      
      const results = await axe(container);
      expect(results).toHaveNoViolations();
    });

    it('applies correct ARIA attributes for interactive cards', () => {
      renderWithTheme(
        <Card onClick={mockOnClick} ariaLabel="Interactive card">
          Content
        </Card>
      );
      
      const card = screen.getByRole('button');
      expect(card).toHaveAttribute('aria-label', 'Interactive card');
      expect(card).toHaveAttribute('tabIndex', '0');
    });

    it('does not apply button role to non-interactive cards', () => {
      renderWithTheme(
        <Card>Non-interactive content</Card>
      );
      
      expect(screen.queryByRole('button')).not.toBeInTheDocument();
    });

    it('shows focus indicator on keyboard focus', () => {
      const { container } = renderWithTheme(
        <Card onClick={mockOnClick}>Interactive Card</Card>
      );
      
      const card = container.firstChild as HTMLElement;
      card.focus();
      
      expect(card).toHaveStyle({
        outline: `2px solid ${lightTheme.primary.main}`,
        outlineOffset: '2px'
      });
    });
  });

  describe('Theming', () => {
    it('applies theme background colors correctly', () => {
      const { container } = renderWithTheme(
        <Card>Themed Content</Card>
      );
      
      expect(container.firstChild).toHaveStyle({
        backgroundColor: lightTheme.background.paper
      });
    });

    it('applies theme-based elevation shadows', () => {
      const { container } = renderWithTheme(
        <Card variant="elevated">Elevated Content</Card>
      );
      
      expect(container.firstChild).toHaveStyle({
        boxShadow: lightTheme.elevation[1]
      });
    });

    it('supports responsive padding on mobile viewport', () => {
      // Mock mobile viewport
      window.matchMedia = jest.fn().mockImplementation(query => ({
        matches: query === '(max-width: 768px)',
        media: query,
        onchange: null,
        addListener: jest.fn(),
        removeListener: jest.fn(),
      }));

      const { container } = renderWithTheme(
        <Card>Mobile Content</Card>
      );
      
      expect(container.firstChild).toHaveStyle({
        padding: '12px'
      });
    });
  });
});