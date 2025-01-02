import React, { useCallback, useMemo } from 'react';
import styled from '@emotion/styled';
import { NavLink } from 'react-router-dom';
import { navigationItems } from '../../../constants/navigationItems';
import { useAuth } from '../../../hooks/useAuth';
import { Icon } from '../Icons/Icons';

// Styled components with Material Design 3.0 specifications
const SidebarContainer = styled.nav<{ isCollapsed: boolean }>`
  width: ${props => props.isCollapsed ? '64px' : '240px'};
  height: 100vh;
  background-color: var(--md-sys-color-surface);
  border-right: 1px solid var(--md-sys-color-outline);
  transition: width 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  position: fixed;
  left: 0;
  top: 0;
  z-index: var(--md-sys-z-index-drawer);
  overflow-x: hidden;
  overflow-y: auto;
  will-change: width;
  transform: translateZ(0);

  @media (max-width: 768px) {
    transform: ${props => props.isCollapsed ? 'translateX(-100%)' : 'translateX(0)'};
  }
`;

const NavList = styled.ul`
  list-style: none;
  padding: 16px 0;
  margin: 0;
  width: 100%;
`;

const NavItem = styled.li`
  margin: 4px 8px;
`;

const StyledNavLink = styled(NavLink)`
  display: flex;
  align-items: center;
  padding: 12px 16px;
  border-radius: 16px;
  text-decoration: none;
  color: var(--md-sys-color-on-surface);
  transition: background-color 0.2s ease;
  user-select: none;
  -webkit-tap-highlight-color: transparent;

  &:hover {
    background-color: var(--md-sys-color-surface-variant);
  }

  &.active {
    background-color: var(--md-sys-color-secondary-container);
    color: var(--md-sys-color-on-secondary-container);
  }

  &:focus-visible {
    outline: 2px solid var(--md-sys-color-primary);
    outline-offset: -2px;
  }
`;

const IconWrapper = styled.span<{ isCollapsed: boolean }>`
  display: flex;
  align-items: center;
  justify-content: center;
  width: 24px;
  height: 24px;
  margin-right: ${props => props.isCollapsed ? '0' : '16px'};
`;

const Label = styled.span<{ isCollapsed: boolean }>`
  font-family: var(--md-sys-typescale-label-large-font);
  font-size: var(--md-sys-typescale-label-large-size);
  line-height: var(--md-sys-typescale-label-large-line-height);
  font-weight: var(--md-sys-typescale-label-large-weight);
  opacity: ${props => props.isCollapsed ? 0 : 1};
  transition: opacity 0.2s ease;
  white-space: nowrap;
`;

interface SidebarProps {
  isCollapsed: boolean;
  onToggle: () => void;
  className?: string;
}

export const Sidebar: React.FC<SidebarProps> = React.memo(({ 
  isCollapsed, 
  onToggle, 
  className 
}) => {
  const { user, hasRole } = useAuth();

  const filteredNavItems = useMemo(() => {
    if (!user) return [];
    return navigationItems
      .filter(item => item.allowedRoles.some(role => hasRole(role)))
      .sort((a, b) => a.order - b.order);
  }, [user, hasRole]);

  const handleKeyDown = useCallback((event: React.KeyboardEvent) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      onToggle();
    }
  }, [onToggle]);

  if (!user) return null;

  return (
    <SidebarContainer 
      isCollapsed={isCollapsed}
      className={className}
      role="navigation"
      aria-label="Main navigation"
      aria-expanded={!isCollapsed}
      aria-orientation="vertical"
    >
      <NavList>
        {filteredNavItems.map(item => (
          <NavItem key={item.id}>
            <StyledNavLink
              to={item.path}
              aria-label={item.ariaLabel}
              role="menuitem"
            >
              <IconWrapper isCollapsed={isCollapsed}>
                <Icon 
                  name={item.icon as any}
                  size={24}
                  aria-hidden="true"
                />
              </IconWrapper>
              <Label isCollapsed={isCollapsed}>
                {item.label}
              </Label>
            </StyledNavLink>
          </NavItem>
        ))}
      </NavList>
    </SidebarContainer>
  );
});

Sidebar.displayName = 'Sidebar';