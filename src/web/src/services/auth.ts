import Keycloak from 'keycloak-js'; // v21.0.1
import CryptoJS from 'crypto-js'; // v4.1.1
import { IUser, UserRole } from '../interfaces/IUser';

/**
 * Constants for authentication and security configuration
 */
const TOKEN_REFRESH_INTERVAL = 300000; // 5 minutes
const TOKEN_EXPIRY_BUFFER = 600000; // 10 minutes
const RATE_LIMIT_ATTEMPTS = 5;
const RATE_LIMIT_WINDOW = 300000; // 5 minutes
const SESSION_FINGERPRINT_SALT = process.env.SESSION_FINGERPRINT_SALT || 'default-salt-value';

/**
 * Interface for rate limiting tracking
 */
interface RateLimitTracker {
    attempts: number;
    firstAttempt: number;
}

/**
 * Enhanced Authentication Service with comprehensive security features
 */
class AuthService {
    private keycloak: Keycloak;
    private rateLimitMap: Map<string, RateLimitTracker>;
    private securityMetrics: {
        failedAttempts: number;
        activeUsers: number;
        lastSecurityEvent: Date;
    };

    constructor() {
        this.keycloak = new Keycloak({
            url: process.env.KEYCLOAK_URL,
            realm: process.env.KEYCLOAK_REALM,
            clientId: process.env.KEYCLOAK_CLIENT_ID
        });
        this.rateLimitMap = new Map();
        this.securityMetrics = {
            failedAttempts: 0,
            activeUsers: 0,
            lastSecurityEvent: new Date()
        };
    }

    /**
     * Initializes the authentication service with enhanced security features
     */
    public async initializeAuth(): Promise<void> {
        try {
            const pkceChallenge = this.generatePKCEChallenge();
            
            await this.keycloak.init({
                onLoad: 'check-sso',
                pkceMethod: 'S256',
                checkLoginIframe: false,
                enableLogging: process.env.NODE_ENV === 'development'
            });

            this.setupTokenRefresh();
            this.initializeSecurityMonitoring();
            
            console.info('Authentication service initialized successfully');
        } catch (error) {
            console.error('Failed to initialize authentication service:', error);
            throw new Error('Authentication initialization failed');
        }
    }

    /**
     * Secure user login with rate limiting and session fingerprinting
     */
    public async login(username: string, password: string): Promise<IUser> {
        try {
            this.checkRateLimit(username);
            
            const sessionFingerprint = this.generateSessionFingerprint(username);
            
            await this.keycloak.login({
                loginHint: username,
                scope: 'openid profile email',
            });

            if (!this.keycloak.authenticated) {
                throw new Error('Authentication failed');
            }

            const userProfile = await this.keycloak.loadUserProfile();
            const userRoles = await this.keycloak.loadUserInfo();

            const user: IUser = {
                id: userProfile.id!,
                username: userProfile.username!,
                email: userProfile.email!,
                role: this.mapKeycloakRole(userRoles),
                permissions: await this.loadUserPermissions(),
                firstName: userProfile.firstName!,
                lastName: userProfile.lastName!,
                lastLogin: new Date()
            };

            this.encryptAndStoreTokens();
            this.logSecurityEvent('login', username);
            this.securityMetrics.activeUsers++;

            return user;
        } catch (error) {
            this.handleLoginFailure(username);
            throw error;
        }
    }

    /**
     * Secure logout with session cleanup
     */
    public async logout(): Promise<void> {
        try {
            await this.keycloak.logout();
            this.clearSecureStorage();
            this.securityMetrics.activeUsers--;
            this.logSecurityEvent('logout');
        } catch (error) {
            console.error('Logout failed:', error);
            throw new Error('Logout failed');
        }
    }

    /**
     * Validates current session security context
     */
    public async validateSession(): Promise<boolean> {
        try {
            if (!this.keycloak.authenticated) {
                return false;
            }

            const tokenExpiry = this.keycloak.tokenParsed?.exp! * 1000;
            const currentTime = new Date().getTime();

            if (tokenExpiry - currentTime < TOKEN_EXPIRY_BUFFER) {
                await this.keycloak.updateToken(30);
            }

            return true;
        } catch (error) {
            console.error('Session validation failed:', error);
            return false;
        }
    }

    /**
     * Retrieves current security metrics
     */
    public getSecurityMetrics() {
        return { ...this.securityMetrics };
    }

    /**
     * Private helper methods
     */
    private generatePKCEChallenge(): { verifier: string; challenge: string } {
        const verifier = CryptoJS.lib.WordArray.random(32).toString();
        const challenge = CryptoJS.SHA256(verifier).toString(CryptoJS.enc.Base64url);
        return { verifier, challenge };
    }

    private generateSessionFingerprint(username: string): string {
        const browserInfo = navigator.userAgent;
        const timestamp = new Date().getTime();
        return CryptoJS.SHA256(`${username}${browserInfo}${timestamp}${SESSION_FINGERPRINT_SALT}`).toString();
    }

    private checkRateLimit(username: string): void {
        const now = Date.now();
        const userAttempts = this.rateLimitMap.get(username);

        if (userAttempts) {
            if (now - userAttempts.firstAttempt > RATE_LIMIT_WINDOW) {
                this.rateLimitMap.set(username, { attempts: 1, firstAttempt: now });
            } else if (userAttempts.attempts >= RATE_LIMIT_ATTEMPTS) {
                throw new Error('Rate limit exceeded');
            } else {
                userAttempts.attempts++;
            }
        } else {
            this.rateLimitMap.set(username, { attempts: 1, firstAttempt: now });
        }
    }

    private setupTokenRefresh(): void {
        setInterval(async () => {
            try {
                await this.keycloak.updateToken(30);
                this.encryptAndStoreTokens();
            } catch (error) {
                console.error('Token refresh failed:', error);
                this.logSecurityEvent('token_refresh_failure');
            }
        }, TOKEN_REFRESH_INTERVAL);
    }

    private encryptAndStoreTokens(): void {
        const encryptionKey = CryptoJS.lib.WordArray.random(32);
        const encryptedToken = CryptoJS.AES.encrypt(
            this.keycloak.token!,
            encryptionKey.toString()
        ).toString();

        sessionStorage.setItem('auth_token', encryptedToken);
        sessionStorage.setItem('key', encryptionKey.toString());
    }

    private clearSecureStorage(): void {
        sessionStorage.removeItem('auth_token');
        sessionStorage.removeItem('key');
    }

    private async loadUserPermissions(): Promise<string[]> {
        const roles = await this.keycloak.loadUserProfile();
        return (roles as any).permissions || [];
    }

    private mapKeycloakRole(userInfo: any): UserRole {
        const roleMapping: { [key: string]: UserRole } = {
            'realm-admin': UserRole.ADMINISTRATOR,
            'operator': UserRole.OPERATOR,
            'analyst': UserRole.ANALYST,
            'service-account': UserRole.SERVICE_ACCOUNT
        };

        const keycloakRole = userInfo.roles?.[0] || 'analyst';
        return roleMapping[keycloakRole] || UserRole.ANALYST;
    }

    private handleLoginFailure(username: string): void {
        this.securityMetrics.failedAttempts++;
        this.logSecurityEvent('login_failure', username);
    }

    private logSecurityEvent(eventType: string, username?: string): void {
        this.securityMetrics.lastSecurityEvent = new Date();
        console.log(`Security Event: ${eventType}`, {
            timestamp: new Date(),
            username,
            eventType,
            clientIP: this.getClientIP()
        });
    }

    private getClientIP(): string {
        // Implementation would depend on your server setup
        return 'IP_ADDRESS';
    }
}

export const authService = new AuthService();