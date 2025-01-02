import React, { useCallback, useEffect, useState } from 'react';
import styled from '@emotion/styled';
import { useAuth } from '../../hooks/useAuth';
import Card from '../../common/Card/Card';
import { IUser, UserRole } from '../../interfaces/IUser';

// Version comments for dependencies
// @emotion/styled: ^11.11.0
// react: ^18.0.0

interface UserProfileProps {
  className?: string;
  testId?: string;
  ariaLabel?: string;
}

const ProfileContainer = styled.div`
  display: flex;
  flex-direction: column;
  gap: 16px;
  padding: 24px;
  position: relative;
  transition: all 0.3s ease;
  
  @media (prefers-reduced-motion) {
    transition: none;
  }
`;

const ProfileSection = styled.div`
  display: grid;
  grid-template-columns: 120px 1fr;
  gap: 16px;
  align-items: center;
  
  @media (max-width: 768px) {
    grid-template-columns: 1fr;
    gap: 8px;
  }
`;

const Label = styled.span`
  color: ${props => props.theme.text.secondary};
  font-size: 0.875rem;
  font-weight: 500;
`;

const Value = styled.span`
  color: ${props => props.theme.text.primary};
  font-size: 1rem;
`;

const RoleBadge = styled.div<{ role: UserRole }>`
  display: inline-flex;
  align-items: center;
  padding: 4px 12px;
  border-radius: 16px;
  font-size: 0.875rem;
  font-weight: 500;
  
  ${({ role, theme }) => {
    switch (role) {
      case UserRole.ADMINISTRATOR:
        return `
          background-color: ${theme.status.error};
          color: ${theme.text.primary};
        `;
      case UserRole.OPERATOR:
        return `
          background-color: ${theme.status.success};
          color: ${theme.text.primary};
        `;
      case UserRole.ANALYST:
        return `
          background-color: ${theme.status.info};
          color: ${theme.text.primary};
        `;
      default:
        return `
          background-color: ${theme.text.disabled};
          color: ${theme.text.primary};
        `;
    }
  }}
`;

const LastLoginText = styled.span`
  color: ${props => props.theme.text.secondary};
  font-size: 0.75rem;
  font-style: italic;
`;

const PermissionsList = styled.ul`
  list-style: none;
  padding: 0;
  margin: 0;
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
`;

const PermissionItem = styled.li`
  background-color: ${props => props.theme.background.elevated};
  padding: 4px 8px;
  border-radius: 4px;
  font-size: 0.75rem;
  color: ${props => props.theme.text.secondary};
`;

const UserProfile: React.FC<UserProfileProps> = ({
  className,
  testId = 'user-profile',
  ariaLabel = 'User Profile',
}) => {
  const { user, isLoading, refreshToken } = useAuth();
  const [lastRefresh, setLastRefresh] = useState<Date>(new Date());

  useEffect(() => {
    const refreshInterval = setInterval(() => {
      refreshToken().then(() => setLastRefresh(new Date()));
    }, 5 * 60 * 1000); // Refresh every 5 minutes

    return () => clearInterval(refreshInterval);
  }, [refreshToken]);

  const formatDate = useCallback((date: Date): string => {
    return new Intl.DateTimeFormat('en-US', {
      dateStyle: 'medium',
      timeStyle: 'short'
    }).format(date);
  }, []);

  if (isLoading) {
    return (
      <Card>
        <ProfileContainer>
          <div role="status" aria-label="Loading user profile">
            Loading...
          </div>
        </ProfileContainer>
      </Card>
    );
  }

  if (!user) {
    return (
      <Card>
        <ProfileContainer>
          <div role="alert" aria-label="User profile error">
            Unable to load user profile
          </div>
        </ProfileContainer>
      </Card>
    );
  }

  return (
    <Card
      className={className}
      data-testid={testId}
      variant="elevated"
      ariaLabel={ariaLabel}
    >
      <ProfileContainer>
        <ProfileSection>
          <Label>Name</Label>
          <Value>{`${user.firstName} ${user.lastName}`}</Value>
        </ProfileSection>

        <ProfileSection>
          <Label>Username</Label>
          <Value>{user.username}</Value>
        </ProfileSection>

        <ProfileSection>
          <Label>Email</Label>
          <Value>{user.email}</Value>
        </ProfileSection>

        <ProfileSection>
          <Label>Role</Label>
          <RoleBadge role={user.role} aria-label={`User role: ${user.role}`}>
            {user.role}
          </RoleBadge>
        </ProfileSection>

        <ProfileSection>
          <Label>Permissions</Label>
          <PermissionsList aria-label="User permissions">
            {user.permissions.map((permission, index) => (
              <PermissionItem key={index} role="listitem">
                {permission}
              </PermissionItem>
            ))}
          </PermissionsList>
        </ProfileSection>

        <ProfileSection>
          <Label>Last Login</Label>
          <LastLoginText>
            {formatDate(user.lastLogin)}
          </LastLoginText>
        </ProfileSection>

        <div aria-live="polite" aria-atomic="true" className="sr-only">
          Profile last updated: {formatDate(lastRefresh)}
        </div>
      </ProfileContainer>
    </Card>
  );
};

UserProfile.displayName = 'UserProfile';

export default UserProfile;
export type { UserProfileProps };