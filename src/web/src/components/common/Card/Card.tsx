import React, { useCallback, KeyboardEvent } from 'react';
import styled from '@emotion/styled';
import { lightTheme, darkTheme } from '../../constants/theme';

// Props interface for the Card component
interface CardProps {
  children: React.ReactNode;
  variant?: 'flat' | 'elevated';
  className?: string;
  onClick?: (event: React.MouseEvent<HTMLDivElement>) => void;
  ariaLabel?: string;
}

// Styled component for the card container
const CardContainer = styled.div<{ variant: 'flat' | 'elevated'; isInteractive: boolean }>`
  /* Base styles */
  border-radius: 8px;
  padding: 16px;
  position: relative;
  width: 100%;
  max-width: 100%;
  box-sizing: border-box;
  background-color: ${props => props.theme.background.paper};
  cursor: ${props => props.isInteractive ? 'pointer' : 'default'};
  
  /* Transition for smooth animations */
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  
  /* Variant-specific styles */
  ${props => props.variant === 'flat' ? `
    border: 1px solid ${props.theme.text.disabled};
  ` : `
    box-shadow: ${props.theme.elevation[1]};
  `}
  
  /* Interactive states for elevated variant */
  ${props => props.variant === 'elevated' && props.isInteractive && `
    &:hover {
      box-shadow: ${props.theme.elevation[2]};
      transform: translateY(-2px);
    }
    
    &:active {
      box-shadow: ${props.theme.elevation[1]};
      transform: translateY(0);
    }
  `}
  
  /* Focus styles for accessibility */
  &:focus-visible {
    outline: 2px solid ${props => props.theme.primary.main};
    outline-offset: 2px;
  }
  
  /* Media query for responsive behavior */
  @media (max-width: 768px) {
    padding: 12px;
  }
  
  /* High contrast mode support */
  @media (forced-colors: active) {
    border: 1px solid CanvasText;
  }
`;

/**
 * A Material Design 3.0 compliant Card component that provides a flexible
 * container for content with support for different elevation levels and
 * interactive states.
 */
const Card: React.FC<CardProps> = ({
  children,
  variant = 'flat',
  className,
  onClick,
  ariaLabel,
}) => {
  // Handle keyboard interactions for accessibility
  const handleKeyDown = useCallback((event: KeyboardEvent<HTMLDivElement>) => {
    if (onClick && (event.key === 'Enter' || event.key === ' ')) {
      event.preventDefault();
      onClick(event as unknown as React.MouseEvent<HTMLDivElement>);
    }
  }, [onClick]);

  // Determine if the card is interactive
  const isInteractive = Boolean(onClick);

  // Compute ARIA attributes based on interactive state
  const ariaAttributes = {
    role: isInteractive ? 'button' : undefined,
    tabIndex: isInteractive ? 0 : undefined,
    'aria-label': ariaLabel,
  };

  return (
    <CardContainer
      variant={variant}
      isInteractive={isInteractive}
      className={className}
      onClick={onClick}
      onKeyDown={handleKeyDown}
      {...ariaAttributes}
    >
      {children}
    </CardContainer>
  );
};

// Default export with display name for dev tools
Card.displayName = 'Card';

export default Card;

// Type export for consumers
export type { CardProps };