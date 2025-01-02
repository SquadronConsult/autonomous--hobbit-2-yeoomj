import { UserRole } from '../interfaces/IUser';

/**
 * Interface defining the structure of navigation items
 * @version 1.0.0
 */
export interface NavigationItem {
    /** Unique identifier for the navigation item */
    id: string;
    /** Display label for the navigation item */
    label: string;
    /** Route path for the navigation item */
    path: string;
    /** Icon name from the Icons component */
    icon: string;
    /** Display order in the navigation */
    order: number;
    /** Roles that have access to this navigation item */
    allowedRoles: UserRole[];
    /** Accessibility label for screen readers */
    ariaLabel: string;
    /** Additional metadata for security and analytics tracking */
    metadata: {
        requiresAuth: boolean;
        analyticsId: string;
        lazyLoad: boolean;
    };
}

/**
 * Navigation configuration for the agricultural management system
 * Implements role-based access control and defines the main navigation structure
 * @version 1.0.0
 */
export const navigationItems: NavigationItem[] = [
    {
        id: 'dashboard',
        label: 'Dashboard',
        path: '/dashboard',
        icon: 'dashboard',
        order: 1,
        allowedRoles: [
            UserRole.ADMINISTRATOR,
            UserRole.OPERATOR,
            UserRole.ANALYST
        ],
        ariaLabel: 'Main dashboard view',
        metadata: {
            requiresAuth: true,
            analyticsId: 'nav_dashboard',
            lazyLoad: true
        }
    },
    {
        id: 'fleet',
        label: 'Fleet Status',
        path: '/fleet',
        icon: 'drone',
        order: 2,
        allowedRoles: [
            UserRole.ADMINISTRATOR,
            UserRole.OPERATOR
        ],
        ariaLabel: 'Fleet status and management',
        metadata: {
            requiresAuth: true,
            analyticsId: 'nav_fleet',
            lazyLoad: true
        }
    },
    {
        id: 'analytics',
        label: 'Analytics',
        path: '/analytics',
        icon: 'analytics',
        order: 3,
        allowedRoles: [
            UserRole.ADMINISTRATOR,
            UserRole.ANALYST
        ],
        ariaLabel: 'Analytics and reporting',
        metadata: {
            requiresAuth: true,
            analyticsId: 'nav_analytics',
            lazyLoad: true
        }
    },
    {
        id: 'missions',
        label: 'Missions',
        path: '/missions',
        icon: 'mission',
        order: 4,
        allowedRoles: [
            UserRole.ADMINISTRATOR,
            UserRole.OPERATOR
        ],
        ariaLabel: 'Mission planning and execution',
        metadata: {
            requiresAuth: true,
            analyticsId: 'nav_missions',
            lazyLoad: true
        }
    },
    {
        id: 'settings',
        label: 'Settings',
        path: '/settings',
        icon: 'settings',
        order: 5,
        allowedRoles: [
            UserRole.ADMINISTRATOR
        ],
        ariaLabel: 'System settings and configuration',
        metadata: {
            requiresAuth: true,
            analyticsId: 'nav_settings',
            lazyLoad: true
        }
    }
];