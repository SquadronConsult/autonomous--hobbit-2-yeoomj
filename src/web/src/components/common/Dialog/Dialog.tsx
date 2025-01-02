import React, { useCallback, useEffect } from 'react';
import styled from '@emotion/styled';
import FocusTrap from 'focus-trap-react';
import { lightTheme, darkTheme } from '../../../constants/theme';
import { Button } from '../Button/Button';

// @emotion/styled: ^11.11.0
// react: ^18.0.0
// focus-trap-react: ^10.0.0

export interface DialogProps {
  isOpen: boolean;
  onClose: () => void;
  title: string;
  children: React.ReactNode;
  actions?: React.ReactNode;
  maxWidth?: 'sm' | 'md' | 'lg' | 'xl';
  fullWidth?: boolean;
  disableBackdropClick?: boolean;
  disableEscapeKeyDown?: boolean;
  ariaLabelledBy?: string;
  ariaDescribedBy?: string;
}

const getMaxWidth = (maxWidth?: 'sm' | 'md' | 'lg' | 'xl'): string => {
  switch (maxWidth) {
    case 'sm': return '400px';
    case 'md': return '600px';
    case 'lg': return '800px';
    case 'xl': return '1200px';
    default: return '600px';
  }
};

const DialogBackdrop = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1300;
  animation: fadeIn 0.2s ease-in-out;

  @keyframes fadeIn {
    from { opacity: 0; }
    to { opacity: 1; }
  }
`;

const DialogContainer = styled.div<{ maxWidth?: 'sm' | 'md' | 'lg' | 'xl', fullWidth?: boolean }>`
  background-color: ${({ theme }) => theme.background.paper};
  border-radius: 8px;
  box-shadow: ${({ theme }) => theme.elevation[3]};
  max-width: ${({ maxWidth }) => getMaxWidth(maxWidth)};
  width: ${({ fullWidth }) => fullWidth ? '90%' : 'auto'};
  margin: 24px;
  display: flex;
  flex-direction: column;
  max-height: 90vh;
  overflow: hidden;
  animation: slideIn 0.2s ease-out;
  position: relative;

  @keyframes slideIn {
    from {
      opacity: 0;
      transform: translateY(-20px);
    }
    to {
      opacity: 1;
      transform: translateY(0);
    }
  }
`;

const DialogHeader = styled.div`
  padding: 24px 24px 16px;
  border-bottom: 1px solid ${({ theme }) => theme.background.default};
  display: flex;
  align-items: center;
  justify-content: space-between;
  min-height: 64px;

  h2 {
    margin: 0;
    font-size: 1.25rem;
    font-weight: 500;
    color: ${({ theme }) => theme.text.primary};
  }
`;

const DialogContent = styled.div`
  padding: 24px;
  overflow-y: auto;
  flex: 1;
  overscroll-behavior: contain;
  -webkit-overflow-scrolling: touch;
  color: ${({ theme }) => theme.text.primary};
`;

const DialogActions = styled.div`
  padding: 16px 24px;
  display: flex;
  align-items: center;
  justify-content: flex-end;
  gap: 8px;
  border-top: 1px solid ${({ theme }) => theme.background.default};
  min-height: 52px;
`;

export const Dialog: React.FC<DialogProps> = ({
  isOpen,
  onClose,
  title,
  children,
  actions,
  maxWidth = 'md',
  fullWidth = false,
  disableBackdropClick = false,
  disableEscapeKeyDown = false,
  ariaLabelledBy,
  ariaDescribedBy,
}) => {
  const handleBackdropClick = useCallback((event: React.MouseEvent) => {
    if (event.target === event.currentTarget && !disableBackdropClick) {
      onClose();
    }
  }, [disableBackdropClick, onClose]);

  const handleKeyDown = useCallback((event: KeyboardEvent) => {
    if (event.key === 'Escape' && !disableEscapeKeyDown) {
      event.stopPropagation();
      onClose();
    }
  }, [disableEscapeKeyDown, onClose]);

  useEffect(() => {
    if (isOpen) {
      document.addEventListener('keydown', handleKeyDown);
      document.body.style.overflow = 'hidden';
      
      return () => {
        document.removeEventListener('keydown', handleKeyDown);
        document.body.style.overflow = '';
      };
    }
  }, [isOpen, handleKeyDown]);

  if (!isOpen) return null;

  return (
    <DialogBackdrop 
      onClick={handleBackdropClick}
      role="presentation"
      aria-hidden="true"
    >
      <FocusTrap
        focusTrapOptions={{
          initialFocus: false,
          returnFocusOnDeactivate: true,
          allowOutsideClick: true,
        }}
      >
        <DialogContainer
          role="dialog"
          aria-modal="true"
          aria-labelledby={ariaLabelledBy || 'dialog-title'}
          aria-describedby={ariaDescribedBy}
          maxWidth={maxWidth}
          fullWidth={fullWidth}
        >
          <DialogHeader>
            <h2 id={ariaLabelledBy || 'dialog-title'}>{title}</h2>
            <Button
              variant="text"
              size="small"
              onClick={onClose}
              ariaLabel="Close dialog"
              color="secondary"
            >
              ✕
            </Button>
          </DialogHeader>
          <DialogContent>{children}</DialogContent>
          {actions && <DialogActions>{actions}</DialogActions>}
        </DialogContainer>
      </FocusTrap>
    </DialogBackdrop>
  );
};

export default Dialog;