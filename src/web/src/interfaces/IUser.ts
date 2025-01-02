/**
 * Enumeration of available user roles in the system
 * Used for role-based access control (RBAC) implementation
 * @version 1.0.0
 */
export enum UserRole {
    /**
     * Full system access with all privileges
     */
    ADMINISTRATOR = 'ADMINISTRATOR',
    
    /**
     * Access to operation control and mission management
     */
    OPERATOR = 'OPERATOR',
    
    /**
     * Read-only access to analytics and reports
     */
    ANALYST = 'ANALYST',
    
    /**
     * Limited API access for automated services
     */
    SERVICE_ACCOUNT = 'SERVICE_ACCOUNT'
}

/**
 * Core user interface definition for authentication and authorization
 * Implements OAuth 2.0 + OpenID Connect via Keycloak integration
 * @version 1.0.0
 */
export interface IUser {
    /**
     * Unique identifier for the user
     */
    id: string;

    /**
     * User's login username
     */
    username: string;

    /**
     * User's email address for notifications and recovery
     */
    email: string;

    /**
     * User's assigned role for access control
     */
    role: UserRole;

    /**
     * Array of specific permissions granted to the user
     */
    permissions: string[];

    /**
     * User's first name
     */
    firstName: string;

    /**
     * User's last name
     */
    lastName: string;

    /**
     * Timestamp of user's last successful login
     */
    lastLogin: Date;
}