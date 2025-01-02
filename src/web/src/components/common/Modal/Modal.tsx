import React from 'react'; // ^18.0.0
import styled from '@emotion/styled'; // ^11.11.0
import FocusTrap from 'focus-trap-react'; // ^10.0.0
import { Portal } from '@reach/portal'; // ^0.18.0
import { useTheme } from '../../../contexts/ThemeContext';

// Modal size configuration
const maxWidthMap = {
  sm: '400px',
  md: '600px',
  lg: '800px',
  xl: '1200px',
};

// Props interface with comprehensive configuration options
export interface ModalProps {
  isOpen: boolean;
  onClose: () => void;
  children: React.ReactNode;
  maxWidth?: keyof typeof maxWidthMap;
  fullWidth?: boolean;
  disableBackdropClick?: boolean;
  disableEscapeKeyDown?: boolean;
  ariaLabelledBy?: string;
  ariaDescribedBy?: string;
  className?: string;
  transitionDuration?: number;
  keepMounted?: boolean;
}

// Styled backdrop with theme-aware styling
const ModalBackdrop = styled.div<{ transitionDuration: number }>`
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: ${({ theme }) =>
    theme.isDarkMode ? 'rgba(0, 0, 0, 0.7)' : 'rgba(0, 0, 0, 0.5)'};
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1300;
  opacity: 0;
  visibility: hidden;
  transition: opacity ${props => props.transitionDuration}ms ease-in-out,
              visibility ${props => props.transitionDuration}ms ease-in-out;

  &[data-open="true"] {
    opacity: 1;
    visibility: visible;
  }
`;

// Styled modal container with responsive sizing
const ModalContainer = styled.div<{
  maxWidth: keyof typeof maxWidthMap;
  fullWidth: boolean;
  transitionDuration: number;
}>`
  background-color: ${({ theme }) => theme.background.paper};
  border-radius: 8px;
  box-shadow: ${({ theme }) => theme.elevation[3]};
  max-width: ${({ maxWidth }) => maxWidthMap[maxWidth]};
  width: ${({ fullWidth }) => (fullWidth ? '90%' : 'auto')};
  margin: 16px;
  display: flex;
  flex-direction: column;
  max-height: 90vh;
  overflow: hidden;
  position: relative;
  transform: scale(0.95);
  opacity: 0;
  transition: transform ${props => props.transitionDuration}ms ease-in-out,
              opacity ${props => props.transitionDuration}ms ease-in-out;

  &[data-open="true"] {
    transform: scale(1);
    opacity: 1;
  }
`;

// Styled content wrapper with overflow handling
const ModalContent = styled.div`
  padding: 24px;
  overflow-y: auto;
  flex: 1;
  -webkit-overflow-scrolling: touch;

  &:focus {
    outline: none;
  }
`;

export const Modal: React.FC<ModalProps> = ({
  isOpen,
  onClose,
  children,
  maxWidth = 'sm',
  fullWidth = false,
  disableBackdropClick = false,
  disableEscapeKeyDown = false,
  ariaLabelledBy,
  ariaDescribedBy,
  className,
  transitionDuration = 225,
  keepMounted = false,
}) => {
  const { isDarkMode, theme } = useTheme();
  const modalRef = React.useRef<HTMLDivElement>(null);

  // Handle backdrop clicks
  const handleBackdropClick = (event: React.MouseEvent<HTMLDivElement>) => {
    if (event.target !== event.currentTarget) return;
    if (disableBackdropClick) return;
    
    event.stopPropagation();
    onClose();
  };

  // Handle keyboard events
  const handleKeyDown = (event: React.KeyboardEvent<HTMLDivElement>) => {
    if (event.key !== 'Escape') return;
    if (disableEscapeKeyDown) return;

    event.stopPropagation();
    onClose();
  };

  // Cleanup function for focus trap
  const cleanup = React.useCallback(() => {
    document.body.style.removeProperty('overflow');
  }, []);

  // Setup and cleanup effects
  React.useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    }
    return cleanup;
  }, [isOpen, cleanup]);

  // Don't render anything if modal is not open and keepMounted is false
  if (!isOpen && !keepMounted) {
    return null;
  }

  return (
    <Portal>
      <FocusTrap
        active={isOpen}
        focusTrapOptions={{
          initialFocus: false,
          allowOutsideClick: true,
          returnFocusOnDeactivate: true,
        }}
      >
        <ModalBackdrop
          data-open={isOpen}
          onClick={handleBackdropClick}
          onKeyDown={handleKeyDown}
          transitionDuration={transitionDuration}
          role="presentation"
        >
          <ModalContainer
            ref={modalRef}
            className={className}
            maxWidth={maxWidth}
            fullWidth={fullWidth}
            transitionDuration={transitionDuration}
            data-open={isOpen}
            role="dialog"
            aria-modal="true"
            aria-labelledby={ariaLabelledBy}
            aria-describedby={ariaDescribedBy}
          >
            <ModalContent
              tabIndex={-1}
              role="document"
            >
              {children}
            </ModalContent>
          </ModalContainer>
        </ModalBackdrop>
      </FocusTrap>
    </Portal>
  );
};

export default Modal;