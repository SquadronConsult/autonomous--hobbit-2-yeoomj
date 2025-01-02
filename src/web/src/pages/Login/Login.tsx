import React, { useState, useCallback, useEffect } from 'react';
import styled from '@emotion/styled';
import { useNavigate } from 'react-router-dom';
import { useAuth } from '../../hooks/useAuth';
import { Button } from '../../components/common/Button/Button';
import { Input } from '../../components/common/Input/Input';
import { ValidationError } from '../../utils/validation';

// Security constants
const MAX_LOGIN_ATTEMPTS = 3;
const LOCKOUT_DURATION = 15 * 60 * 1000; // 15 minutes
const MIN_PASSWORD_LENGTH = 8;

// Interface for login form data with enhanced security tracking
interface LoginFormData {
  username: string;
  password: string;
  securityToken: string;
  deviceFingerprint: string;
  lastAttempt: Date | null;
}

// Interface for login error handling with detailed context
interface LoginError {
  code: number;
  message: string;
  timestamp: Date;
  attempts: number;
}

// Styled components with WCAG 2.1 Level AA compliance
const LoginContainer = styled.div`
  display: flex;
  justify-content: center;
  align-items: center;
  min-height: 100vh;
  background-color: ${({ theme }) => theme.background.default};
  padding: var(--spacing-md);

  @media (prefers-reduced-motion: no-preference) {
    transition: background-color 0.3s;
  }
`;

const LoginForm = styled.form`
  width: 100%;
  max-width: 400px;
  padding: var(--spacing-lg);
  background-color: ${({ theme }) => theme.background.paper};
  border-radius: var(--border-radius-lg);
  box-shadow: ${({ theme }) => theme.elevation[2]};

  @media (prefers-reduced-motion: no-preference) {
    transition: transform 0.3s;
  }

  @media (max-width: 768px) {
    padding: var(--spacing-md);
  }
`;

const FormTitle = styled.h1`
  color: ${({ theme }) => theme.text.primary};
  font-size: 24px;
  margin-bottom: var(--spacing-lg);
  text-align: center;
`;

const ErrorMessage = styled.div`
  color: ${({ theme }) => theme.status.error};
  font-size: 14px;
  margin-top: var(--spacing-sm);
  padding: var(--spacing-sm);
  border-radius: var(--border-radius-sm);
  background-color: ${({ theme }) => `${theme.status.error}10`};
`;

const Login: React.FC = () => {
  const navigate = useNavigate();
  const { login, isLoading, isAuthenticated } = useAuth();
  
  const [formData, setFormData] = useState<LoginFormData>({
    username: '',
    password: '',
    securityToken: '',
    deviceFingerprint: '',
    lastAttempt: null
  });
  
  const [error, setError] = useState<LoginError | null>(null);
  const [attempts, setAttempts] = useState<number>(0);
  const [isLocked, setIsLocked] = useState<boolean>(false);

  // Generate device fingerprint for security tracking
  useEffect(() => {
    const generateFingerprint = (): string => {
      const userAgent = window.navigator.userAgent;
      const screenRes = `${window.screen.width}x${window.screen.height}`;
      const timestamp = Date.now();
      return btoa(`${userAgent}|${screenRes}|${timestamp}`);
    };

    setFormData(prev => ({
      ...prev,
      deviceFingerprint: generateFingerprint()
    }));
  }, []);

  // Check for account lockout
  useEffect(() => {
    if (isLocked && error?.timestamp) {
      const timeElapsed = Date.now() - error.timestamp.getTime();
      if (timeElapsed >= LOCKOUT_DURATION) {
        setIsLocked(false);
        setAttempts(0);
      }
    }
  }, [isLocked, error]);

  // Redirect if already authenticated
  useEffect(() => {
    if (isAuthenticated) {
      navigate('/dashboard');
    }
  }, [isAuthenticated, navigate]);

  // Enhanced input validation
  const validateInput = useCallback((name: string, value: string): string | null => {
    switch (name) {
      case 'username':
        if (!value.trim()) return 'Username is required';
        if (value.length < 3) return 'Username must be at least 3 characters';
        return null;
      case 'password':
        if (!value) return 'Password is required';
        if (value.length < MIN_PASSWORD_LENGTH) {
          return `Password must be at least ${MIN_PASSWORD_LENGTH} characters`;
        }
        return null;
      default:
        return null;
    }
  }, []);

  // Handle input changes with validation
  const handleInputChange = useCallback((
    event: React.ChangeEvent<HTMLInputElement>
  ) => {
    const { name, value } = event.target;
    const validationError = validateInput(name, value);

    setFormData(prev => ({
      ...prev,
      [name]: value
    }));

    if (validationError) {
      setError({
        code: 400,
        message: validationError,
        timestamp: new Date(),
        attempts
      });
    } else {
      setError(null);
    }
  }, [attempts, validateInput]);

  // Enhanced form submission with security measures
  const handleSubmit = async (event: React.FormEvent) => {
    event.preventDefault();

    if (isLocked) {
      setError({
        code: 429,
        message: `Account locked. Please try again in ${Math.ceil(LOCKOUT_DURATION / 60000)} minutes`,
        timestamp: new Date(),
        attempts
      });
      return;
    }

    try {
      await login(formData.username, formData.password);
      navigate('/dashboard');
    } catch (err) {
      const newAttempts = attempts + 1;
      setAttempts(newAttempts);

      if (newAttempts >= MAX_LOGIN_ATTEMPTS) {
        setIsLocked(true);
        setError({
          code: 429,
          message: `Too many failed attempts. Account locked for ${LOCKOUT_DURATION / 60000} minutes`,
          timestamp: new Date(),
          attempts: newAttempts
        });
      } else {
        setError({
          code: 401,
          message: 'Invalid username or password',
          timestamp: new Date(),
          attempts: newAttempts
        });
      }
    }
  };

  return (
    <LoginContainer>
      <LoginForm onSubmit={handleSubmit} noValidate>
        <FormTitle>Agricultural Management System</FormTitle>
        
        <Input
          name="username"
          type="text"
          label="Username"
          value={formData.username}
          onChange={value => handleInputChange({ 
            target: { name: 'username', value } 
          } as React.ChangeEvent<HTMLInputElement>)}
          disabled={isLocked || isLoading}
          required
          aria-label="Username"
        />

        <Input
          name="password"
          type="password"
          label="Password"
          value={formData.password}
          onChange={value => handleInputChange({ 
            target: { name: 'password', value } 
          } as React.ChangeEvent<HTMLInputElement>)}
          disabled={isLocked || isLoading}
          required
          aria-label="Password"
        />

        {error && (
          <ErrorMessage role="alert" aria-live="polite">
            {error.message}
          </ErrorMessage>
        )}

        <Button
          type="submit"
          variant="contained"
          color="primary"
          fullWidth
          loading={isLoading}
          disabled={isLocked || isLoading}
          aria-label="Sign in"
        >
          Sign In
        </Button>
      </LoginForm>
    </LoginContainer>
  );
};

export default Login;