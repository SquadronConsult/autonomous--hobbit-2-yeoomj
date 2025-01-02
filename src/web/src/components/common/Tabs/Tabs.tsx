import React, { useState, useRef, useCallback, useEffect } from 'react';
import styled from '@emotion/styled';
import { lightTheme, darkTheme } from '../../../constants/theme';
import { Button } from '../Button/Button';

// @emotion/styled: ^11.11.0
// react: ^18.0.0

export interface TabItem {
  id: string;
  label: string;
  content: React.ReactNode;
  disabled?: boolean;
  icon?: React.ReactNode;
}

export interface TabsProps {
  tabs: TabItem[];
  defaultActiveTab?: string;
  orientation?: 'horizontal' | 'vertical';
  variant?: 'standard' | 'contained' | 'fullWidth';
  onChange?: (tabId: string) => void;
  className?: string;
}

const TabsContainer = styled.div<{ orientation?: 'horizontal' | 'vertical' }>`
  display: flex;
  flex-direction: ${props => props.orientation === 'vertical' ? 'column' : 'row'};
  width: 100%;
  position: relative;
  box-sizing: border-box;

  &[data-orientation='vertical'] {
    min-height: 200px;
    gap: 0;
  }

  &[data-orientation='horizontal'] {
    border-bottom: 1px solid ${lightTheme.primary.light}25;
    gap: var(--spacing-sm, 8px);
  }

  @media (prefers-color-scheme: dark) {
    &[data-orientation='horizontal'] {
      border-bottom-color: ${darkTheme.primary.light}25;
    }
  }
`;

const TabList = styled.div<{ orientation?: 'horizontal' | 'vertical' }>`
  display: flex;
  flex-direction: ${props => props.orientation === 'vertical' ? 'column' : 'row'};
  ${props => props.orientation === 'vertical' ? 'width: 200px;' : ''}
  position: relative;
`;

const TabButton = styled(Button)<{ isActive: boolean; orientation?: string }>`
  border-radius: 0;
  background: transparent;
  color: ${props => props.isActive ? lightTheme.primary.main : lightTheme.text.secondary};
  padding: 12px 16px;
  min-width: ${props => props.orientation === 'vertical' ? '100%' : '120px'};
  justify-content: ${props => props.orientation === 'vertical' ? 'flex-start' : 'center'};
  position: relative;
  transition: all 0.2s ease-in-out;

  &::after {
    content: '';
    position: absolute;
    ${props => props.orientation === 'vertical' ? 'right: 0; top: 0; width: 2px; height: 100%;' : 'bottom: -1px; left: 0; width: 100%; height: 2px;'}
    background: ${props => props.isActive ? lightTheme.primary.main : 'transparent'};
    transition: all 0.2s ease-in-out;
  }

  &:hover:not(:disabled) {
    background: ${lightTheme.primary.light}10;
    color: ${lightTheme.primary.main};
  }

  &:focus-visible {
    outline: 2px solid ${lightTheme.primary.main};
    outline-offset: -2px;
  }

  @media (prefers-color-scheme: dark) {
    color: ${props => props.isActive ? darkTheme.primary.main : darkTheme.text.secondary};

    &::after {
      background: ${props => props.isActive ? darkTheme.primary.main : 'transparent'};
    }

    &:hover:not(:disabled) {
      background: ${darkTheme.primary.light}10;
      color: ${darkTheme.primary.main};
    }

    &:focus-visible {
      outline-color: ${darkTheme.primary.main};
    }
  }
`;

const TabContent = styled.div<{ active: boolean }>`
  display: ${props => props.active ? 'block' : 'none'};
  padding: 16px;
  width: 100%;
  animation: ${props => props.active ? 'fadeIn 0.2s ease-in-out' : 'none'};

  @keyframes fadeIn {
    from { opacity: 0; }
    to { opacity: 1; }
  }
`;

export const Tabs: React.FC<TabsProps> = ({
  tabs,
  defaultActiveTab,
  orientation = 'horizontal',
  variant = 'standard',
  onChange,
  className
}) => {
  const [activeTab, setActiveTab] = useState(defaultActiveTab || tabs[0]?.id);
  const tabListRef = useRef<HTMLDivElement>(null);
  const [focusedTab, setFocusedTab] = useState<string | null>(null);

  const handleTabChange = useCallback((tabId: string) => {
    const tab = tabs.find(t => t.id === tabId);
    if (tab?.disabled) return;

    setActiveTab(tabId);
    onChange?.(tabId);
    setFocusedTab(tabId);
  }, [tabs, onChange]);

  const handleKeyNavigation = useCallback((event: React.KeyboardEvent) => {
    const currentIndex = tabs.findIndex(tab => tab.id === focusedTab);
    let nextIndex: number;

    switch (event.key) {
      case 'ArrowRight':
      case 'ArrowDown':
        event.preventDefault();
        nextIndex = currentIndex + 1;
        if (nextIndex >= tabs.length) nextIndex = 0;
        while (tabs[nextIndex]?.disabled && nextIndex !== currentIndex) {
          nextIndex = nextIndex + 1 >= tabs.length ? 0 : nextIndex + 1;
        }
        setFocusedTab(tabs[nextIndex]?.id);
        break;

      case 'ArrowLeft':
      case 'ArrowUp':
        event.preventDefault();
        nextIndex = currentIndex - 1;
        if (nextIndex < 0) nextIndex = tabs.length - 1;
        while (tabs[nextIndex]?.disabled && nextIndex !== currentIndex) {
          nextIndex = nextIndex - 1 < 0 ? tabs.length - 1 : nextIndex - 1;
        }
        setFocusedTab(tabs[nextIndex]?.id);
        break;

      case 'Home':
        event.preventDefault();
        nextIndex = 0;
        while (tabs[nextIndex]?.disabled && nextIndex < tabs.length) {
          nextIndex++;
        }
        setFocusedTab(tabs[nextIndex]?.id);
        break;

      case 'End':
        event.preventDefault();
        nextIndex = tabs.length - 1;
        while (tabs[nextIndex]?.disabled && nextIndex > 0) {
          nextIndex--;
        }
        setFocusedTab(tabs[nextIndex]?.id);
        break;

      case 'Enter':
      case ' ':
        event.preventDefault();
        if (focusedTab) handleTabChange(focusedTab);
        break;
    }
  }, [tabs, focusedTab, handleTabChange]);

  useEffect(() => {
    if (focusedTab) {
      const focusedElement = tabListRef.current?.querySelector(`[data-tab-id="${focusedTab}"]`) as HTMLElement;
      focusedElement?.focus();
    }
  }, [focusedTab]);

  return (
    <TabsContainer
      orientation={orientation}
      className={className}
      data-orientation={orientation}
      role="tablist"
      aria-orientation={orientation}
    >
      <TabList
        ref={tabListRef}
        orientation={orientation}
        onKeyDown={handleKeyNavigation}
      >
        {tabs.map((tab) => (
          <TabButton
            key={tab.id}
            data-tab-id={tab.id}
            isActive={activeTab === tab.id}
            orientation={orientation}
            onClick={() => handleTabChange(tab.id)}
            disabled={tab.disabled}
            variant="text"
            aria-selected={activeTab === tab.id}
            aria-controls={`tab-panel-${tab.id}`}
            aria-disabled={tab.disabled}
            role="tab"
            tabIndex={focusedTab === tab.id || (!focusedTab && activeTab === tab.id) ? 0 : -1}
          >
            {tab.icon && <span className="tab-icon">{tab.icon}</span>}
            {tab.label}
          </TabButton>
        ))}
      </TabList>

      {tabs.map((tab) => (
        <TabContent
          key={tab.id}
          id={`tab-panel-${tab.id}`}
          active={activeTab === tab.id}
          role="tabpanel"
          aria-labelledby={`tab-${tab.id}`}
          tabIndex={0}
        >
          {activeTab === tab.id && tab.content}
        </TabContent>
      ))}
    </TabsContainer>
  );
};