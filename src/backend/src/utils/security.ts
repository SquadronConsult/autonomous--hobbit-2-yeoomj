import { securityConfig } from '../config/security';
import { logger } from '../utils/logger';
import jsonwebtoken from 'jsonwebtoken';  // v9.0.0
import crypto from 'crypto';  // built-in
import bcrypt from 'bcrypt';  // v5.1.0

// Constants for security operations
const SALT_ROUNDS = 12;
const TOKEN_EXPIRY = '24h';
const AUTH_TAG_LENGTH = 16;
const IV_LENGTH = 12;
const MAX_PASSWORD_LENGTH = 72;
const MIN_PASSWORD_LENGTH = 12;
const KEY_ROTATION_INTERVAL = 7776000; // 90 days in seconds
const MAX_FAILED_ATTEMPTS = 5;
const LOCKOUT_DURATION = 900; // 15 minutes in seconds

// Interfaces
interface TokenPayload {
    userId: string;
    role: string;
    permissions: string[];
    exp: number;
    iat: number;
    aud: string;
    jti: string;
    deviceId: string;
}

interface EncryptionResult {
    encrypted: Buffer;
    iv: Buffer;
    tag: Buffer;
    algorithm: string;
    keyId: string;
    timestamp: number;
}

// Token verification with enhanced security checks
export async function verifyToken(token: string): Promise<TokenPayload> {
    try {
        // Initial token format validation
        if (!token || typeof token !== 'string') {
            throw new Error('Invalid token format');
        }

        // Verify and decode token
        const decoded = jsonwebtoken.verify(token, securityConfig.auth.jwt.secret, {
            algorithms: [securityConfig.auth.jwt.algorithm],
            audience: securityConfig.auth.jwt.audience,
            issuer: securityConfig.auth.jwt.issuer,
            clockTolerance: securityConfig.auth.jwt.clockTolerance,
            complete: true
        }) as TokenPayload;

        // Additional security validations
        if (!decoded.userId || !decoded.role || !decoded.permissions) {
            throw new Error('Invalid token payload structure');
        }

        // Validate role against allowed roles
        if (!securityConfig.auth.allowedRoles.includes(decoded.role)) {
            throw new Error('Invalid role specified in token');
        }

        // Log successful verification
        logger.info('Token verified successfully', {
            component: 'security',
            operation: 'verifyToken',
            securityContext: {
                userId: decoded.userId,
                accessLevel: decoded.role,
                ipAddress: 'system',
                encryptionLevel: 'high'
            }
        });

        return decoded;

    } catch (error) {
        // Enhanced error handling with secure logging
        logger.error(error as Error, {
            component: 'security',
            operation: 'verifyToken',
            details: {
                errorType: error instanceof jsonwebtoken.JsonWebTokenError ? 'TokenError' : 'ValidationError'
            }
        });

        throw new Error('Token verification failed');
    }
}

// Data encryption using AES-256-GCM with enhanced security
export async function encrypt(data: Buffer | string): Promise<EncryptionResult> {
    try {
        // Input validation and conversion
        const buffer = Buffer.isBuffer(data) ? data : Buffer.from(data);
        
        // Generate cryptographically secure IV
        const iv = crypto.randomBytes(IV_LENGTH);
        
        // Create cipher with configured algorithm
        const cipher = crypto.createCipheriv(
            securityConfig.encryption.algorithm,
            Buffer.from(process.env.ENCRYPTION_KEY || '', 'hex'),
            iv,
            { authTagLength: AUTH_TAG_LENGTH }
        );

        // Perform encryption
        const encrypted = Buffer.concat([
            cipher.update(buffer),
            cipher.final()
        ]);

        // Get authentication tag
        const tag = cipher.getAuthTag();

        // Generate key identifier for key rotation
        const keyId = crypto.createHash('sha256')
            .update(process.env.ENCRYPTION_KEY || '')
            .digest('hex')
            .substring(0, 8);

        const result: EncryptionResult = {
            encrypted,
            iv,
            tag,
            algorithm: securityConfig.encryption.algorithm,
            keyId,
            timestamp: Date.now()
        };

        // Log encryption operation
        logger.info('Data encrypted successfully', {
            component: 'security',
            operation: 'encrypt',
            securityContext: {
                accessLevel: 'system',
                ipAddress: 'system',
                encryptionLevel: 'high'
            },
            details: {
                algorithm: result.algorithm,
                keyId: result.keyId
            }
        });

        return result;

    } catch (error) {
        // Secure error handling
        logger.error(error as Error, {
            component: 'security',
            operation: 'encrypt',
            details: {
                errorType: 'EncryptionError'
            }
        });

        throw new Error('Encryption operation failed');
    }
}

// Password hashing with bcrypt
export async function hashPassword(password: string): Promise<string> {
    try {
        // Validate password requirements
        if (!password || 
            password.length < MIN_PASSWORD_LENGTH || 
            password.length > MAX_PASSWORD_LENGTH) {
            throw new Error('Invalid password length');
        }

        // Generate salt and hash password
        const salt = await bcrypt.genSalt(SALT_ROUNDS);
        const hash = await bcrypt.hash(password, salt);

        logger.info('Password hashed successfully', {
            component: 'security',
            operation: 'hashPassword',
            securityContext: {
                accessLevel: 'system',
                ipAddress: 'system',
                encryptionLevel: 'high'
            }
        });

        return hash;

    } catch (error) {
        logger.error(error as Error, {
            component: 'security',
            operation: 'hashPassword',
            details: {
                errorType: 'HashingError'
            }
        });

        throw new Error('Password hashing failed');
    }
}

// Secure random token generation
export function generateSecureToken(length: number = 32): string {
    try {
        return crypto.randomBytes(length).toString('hex');
    } catch (error) {
        logger.error(error as Error, {
            component: 'security',
            operation: 'generateSecureToken',
            details: {
                errorType: 'TokenGenerationError'
            }
        });

        throw new Error('Secure token generation failed');
    }
}