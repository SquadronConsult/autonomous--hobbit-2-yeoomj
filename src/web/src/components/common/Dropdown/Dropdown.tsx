import React, { useCallback, useEffect, useRef, useState, memo } from 'react';
import styled from '@emotion/styled';
import { Icon } from '../Icons/Icons';
import { lightTheme } from '../../../constants/theme';

// Types and Interfaces
interface DropdownOption {
  value: string | number;
  label: string;
  icon?: string;
  disabled?: boolean;
}

export interface DropdownProps {
  options: DropdownOption[];
  value?: string | number;
  defaultValue?: string | number;
  onChange: (value: string | number) => void;
  placeholder?: string;
  disabled?: boolean;
  error?: boolean;
  className?: string;
  dir?: 'ltr' | 'rtl';
  virtualized?: boolean;
}

// Styled Components
const DropdownContainer = styled.div<{ dir?: 'ltr' | 'rtl' }>`
  position: relative;
  width: 100%;
  direction: ${props => props.dir || 'ltr'};
`;

const DropdownButton = styled.button<{ error?: boolean; isOpen: boolean }>`
  width: 100%;
  min-height: 48px;
  padding: 12px 16px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  background: ${({ theme }) => theme.background.paper};
  border: 1px solid ${({ theme, error }) => error ? theme.status.error : theme.text.disabled};
  border-radius: 4px;
  cursor: pointer;
  transition: all 150ms cubic-bezier(0.4, 0, 0.2, 1);
  box-shadow: ${({ isOpen }) => isOpen ? lightTheme.elevation[2] : 'none'};

  &:hover:not(:disabled) {
    border-color: ${({ theme, error }) => error ? theme.status.error : theme.text.primary};
    box-shadow: ${lightTheme.elevation[1]};
  }

  &:disabled {
    cursor: not-allowed;
    opacity: 0.6;
    background: ${({ theme }) => theme.background.default};
  }

  &:focus-visible {
    outline: 2px solid ${({ theme }) => theme.primary.main};
    outline-offset: 2px;
  }
`;

const DropdownList = styled.ul<{ isOpen: boolean }>`
  position: absolute;
  top: calc(100% + 4px);
  left: 0;
  right: 0;
  max-height: 300px;
  overflow-y: auto;
  background: ${({ theme }) => theme.background.paper};
  border-radius: 4px;
  box-shadow: ${lightTheme.elevation[3]};
  z-index: 1000;
  list-style: none;
  padding: 8px 0;
  margin: 0;
  opacity: ${({ isOpen }) => isOpen ? 1 : 0};
  visibility: ${({ isOpen }) => isOpen ? 'visible' : 'hidden'};
  transform: ${({ isOpen }) => isOpen ? 'translateY(0)' : 'translateY(-8px)'};
  transition: all 150ms cubic-bezier(0.4, 0, 0.2, 1);

  &:focus {
    outline: none;
  }
`;

const DropdownOption = styled.li<{ isSelected: boolean; disabled?: boolean }>`
  padding: 12px 16px;
  display: flex;
  align-items: center;
  gap: 8px;
  cursor: ${({ disabled }) => disabled ? 'not-allowed' : 'pointer'};
  background: ${({ isSelected, theme }) => isSelected ? theme.background.elevated : 'transparent'};
  color: ${({ disabled, theme }) => disabled ? theme.text.disabled : theme.text.primary};
  opacity: ${({ disabled }) => disabled ? 0.6 : 1};

  &:hover:not([disabled]) {
    background: ${({ theme }) => theme.background.elevated};
  }

  &:focus-visible {
    outline: 2px solid ${({ theme }) => theme.primary.main};
    outline-offset: -2px;
  }
`;

// Main Component
export const Dropdown = memo(({
  options,
  value,
  defaultValue,
  onChange,
  placeholder = 'Select an option',
  disabled = false,
  error = false,
  className,
  dir = 'ltr',
  virtualized = false,
}: DropdownProps) => {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedValue, setSelectedValue] = useState<string | number | undefined>(value || defaultValue);
  const [activeIndex, setActiveIndex] = useState(-1);
  
  const containerRef = useRef<HTMLDivElement>(null);
  const buttonRef = useRef<HTMLButtonElement>(null);
  const listRef = useRef<HTMLUListElement>(null);
  const optionRefs = useRef<(HTMLLIElement | null)[]>([]);

  const handleClickOutside = useCallback((event: MouseEvent) => {
    if (containerRef.current && !containerRef.current.contains(event.target as Node)) {
      setIsOpen(false);
      setActiveIndex(-1);
    }
  }, []);

  useEffect(() => {
    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [isOpen, handleClickOutside]);

  useEffect(() => {
    if (value !== undefined) {
      setSelectedValue(value);
    }
  }, [value]);

  const handleClick = useCallback((event: React.MouseEvent | React.TouchEvent) => {
    event.preventDefault();
    if (!disabled) {
      setIsOpen(prev => !prev);
    }
  }, [disabled]);

  const handleKeyDown = useCallback((event: React.KeyboardEvent) => {
    if (disabled) return;

    switch (event.key) {
      case 'Enter':
      case ' ':
        event.preventDefault();
        setIsOpen(prev => !prev);
        break;
      case 'Escape':
        setIsOpen(false);
        buttonRef.current?.focus();
        break;
      case 'ArrowDown':
        event.preventDefault();
        if (isOpen) {
          setActiveIndex(prev => (prev + 1) % options.length);
          optionRefs.current[activeIndex + 1]?.focus();
        } else {
          setIsOpen(true);
        }
        break;
      case 'ArrowUp':
        event.preventDefault();
        if (isOpen) {
          setActiveIndex(prev => (prev - 1 + options.length) % options.length);
          optionRefs.current[activeIndex - 1]?.focus();
        }
        break;
      case 'Home':
        if (isOpen) {
          event.preventDefault();
          setActiveIndex(0);
          optionRefs.current[0]?.focus();
        }
        break;
      case 'End':
        if (isOpen) {
          event.preventDefault();
          setActiveIndex(options.length - 1);
          optionRefs.current[options.length - 1]?.focus();
        }
        break;
    }
  }, [disabled, isOpen, options.length, activeIndex]);

  const handleOptionSelect = useCallback((optionValue: string | number) => {
    const option = options.find(opt => opt.value === optionValue);
    if (option?.disabled) return;

    setSelectedValue(optionValue);
    onChange(optionValue);
    setIsOpen(false);
    buttonRef.current?.focus();
  }, [options, onChange]);

  const selectedOption = options.find(option => option.value === selectedValue);
  const dropdownId = useRef(`dropdown-${Math.random().toString(36).substr(2, 9)}`);
  const listboxId = useRef(`listbox-${dropdownId.current}`);

  return (
    <DropdownContainer ref={containerRef} className={className} dir={dir}>
      <DropdownButton
        ref={buttonRef}
        type="button"
        disabled={disabled}
        error={error}
        isOpen={isOpen}
        onClick={handleClick}
        onKeyDown={handleKeyDown}
        aria-haspopup="listbox"
        aria-expanded={isOpen}
        aria-controls={listboxId.current}
        aria-labelledby={dropdownId.current}
      >
        <span id={dropdownId.current}>
          {selectedOption ? selectedOption.label : placeholder}
        </span>
        <Icon
          name="analytics"
          size={24}
          style={{
            transform: `rotate(${isOpen ? '180deg' : '0deg'}) scaleX(${dir === 'rtl' ? -1 : 1})`,
            transition: 'transform 150ms cubic-bezier(0.4, 0, 0.2, 1)',
          }}
        />
      </DropdownButton>

      <DropdownList
        ref={listRef}
        role="listbox"
        id={listboxId.current}
        isOpen={isOpen}
        aria-activedescendant={activeIndex >= 0 ? `option-${options[activeIndex].value}` : undefined}
        tabIndex={-1}
      >
        {options.map((option, index) => (
          <DropdownOption
            key={option.value}
            ref={el => optionRefs.current[index] = el}
            role="option"
            id={`option-${option.value}`}
            aria-selected={option.value === selectedValue}
            isSelected={option.value === selectedValue}
            disabled={option.disabled}
            onClick={() => handleOptionSelect(option.value)}
            onKeyDown={e => {
              if (e.key === 'Enter' || e.key === ' ') {
                e.preventDefault();
                handleOptionSelect(option.value);
              }
            }}
            tabIndex={isOpen ? 0 : -1}
          >
            {option.icon && <Icon name={option.icon as any} size={20} />}
            {option.label}
          </DropdownOption>
        ))}
      </DropdownList>
    </DropdownContainer>
  );
});

Dropdown.displayName = 'Dropdown';