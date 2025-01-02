import React, { memo } from 'react'; // ^18.0.0
import styled from '@emotion/styled'; // ^11.11.0
import { ReactComponent as AnalyticsIcon } from '../../../assets/icons/analytics.svg';
import { ReactComponent as DroneIcon } from '../../../assets/icons/drone.svg';
import { ThemeMode } from '../../../constants/theme';

// Icon name mapping type
type IconName = 'analytics' | 'drone';

// Props interface for the Icon component
export interface IconProps {
  name: IconName;
  size?: string | number;
  color?: string;
  className?: string;
  title?: string;
  ariaLabel?: string;
}

// Props interface for the styled wrapper component
interface IconWrapperProps {
  size?: string | number;
  color?: string;
  className?: string;
}

// Map of icon names to their components
const iconMap: Record<IconName, React.FC<React.SVGProps<SVGSVGElement>>> = {
  analytics: AnalyticsIcon,
  drone: DroneIcon,
};

// Styled wrapper component for consistent icon styling
const IconWrapper = styled.div<IconWrapperProps>`
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: ${props => (typeof props.size === 'number' ? `${props.size}px` : props.size || '24px')};
  height: ${props => (typeof props.size === 'number' ? `${props.size}px` : props.size || '24px')};
  color: ${props => props.color || 'currentColor'};
  transition: color 0.2s ease-in-out;
  transform: translateZ(0); // GPU acceleration
  will-change: color;
  line-height: 0;

  svg {
    width: 100%;
    height: 100%;
  }

  /* Ensure proper color inheritance for light/dark themes */
  [data-theme="${ThemeMode.LIGHT}"] & {
    color: ${props => props.color || 'inherit'};
  }

  [data-theme="${ThemeMode.DARK}"] & {
    color: ${props => props.color || 'inherit'};
  }
`;

// Main Icon component with memoization for performance
export const Icon: React.FC<IconProps> = memo(({
  name,
  size = 24,
  color,
  className,
  title,
  ariaLabel,
}) => {
  const IconComponent = iconMap[name];

  if (!IconComponent) {
    console.warn(`Icon "${name}" not found in icon map`);
    return null;
  }

  return (
    <IconWrapper
      size={size}
      color={color}
      className={className}
      role="img"
      aria-hidden={!ariaLabel}
      aria-label={ariaLabel}
    >
      <IconComponent
        width="100%"
        height="100%"
        aria-hidden="true"
        role="presentation"
        title={title}
      />
    </IconWrapper>
  );
});

// Display name for debugging
Icon.displayName = 'Icon';

// Default props
Icon.defaultProps = {
  size: 24,
  color: undefined,
  className: undefined,
  title: undefined,
  ariaLabel: undefined,
};

// Type export for component props
export type { IconName };