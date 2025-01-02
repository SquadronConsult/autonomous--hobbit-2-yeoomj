import { logger } from '../utils/logger';  // v3.10.0
import dotenv from 'dotenv';  // v16.0.3
import crypto from 'crypto';  // Node.js built-in
import fs from 'fs';  // Node.js built-in

// Load environment variables
dotenv.config();

// Interfaces for security configuration
interface KeycloakConfig {
  realm: string;
  authServerUrl: string;
  resource: string;
  credentials: {
    secret: string;
  };
  confidentialPort: number;
  sslRequired: string;
  verifyTokenAudience: boolean;
  enablePkce: boolean;
  tokenValidation: {
    clockTolerance: number;
    validateNbf: boolean;
  };
}

interface JwtConfig {
  secret: string;
  expiresIn: string;
  algorithm: string;
  issuer: string;
  audience: string[];
  clockTolerance: number;
  maxAge: string;
  notBefore: string;
}

interface EncryptionConfig {
  algorithm: string;
  keySize: number;
  ivLength: number;
  saltLength: number;
  keyDerivation: {
    iterations: number;
    digest: string;
  };
  tagLength: number;
}

interface TlsConfig {
  version: string;
  ciphers: string[];
  cert: string;
  key: string;
  ca: string;
  minVersion: string;
  maxVersion: string;
  secureOptions: number;
  sessionTimeout: number;
}

// Security constants
const DEFAULT_JWT_EXPIRY = '1h';
const DEFAULT_ENCRYPTION_ALGORITHM = 'aes-256-gcm';
const DEFAULT_TLS_VERSION = 'TLSv1.3';
const ALLOWED_ROLES = ['administrator', 'operator', 'analyst', 'service'];
const MIN_KEY_SIZE = 256;
const MIN_PASSWORD_LENGTH = 16;
const MAX_LOGIN_ATTEMPTS = 3;
const TOKEN_REFRESH_WINDOW = '15m';

// Secure cipher suites for TLS 1.3
const SECURE_CIPHERS = [
  'TLS_AES_256_GCM_SHA384',
  'TLS_CHACHA20_POLY1305_SHA256',
  'TLS_AES_128_GCM_SHA256'
];

// Load and validate Keycloak configuration
const loadKeycloakConfig = (): KeycloakConfig => {
  try {
    const config: KeycloakConfig = {
      realm: process.env.KEYCLOAK_REALM || '',
      authServerUrl: process.env.KEYCLOAK_AUTH_SERVER_URL || '',
      resource: process.env.KEYCLOAK_RESOURCE || '',
      credentials: {
        secret: process.env.KEYCLOAK_SECRET || ''
      },
      confidentialPort: parseInt(process.env.KEYCLOAK_CONFIDENTIAL_PORT || '8443'),
      sslRequired: 'external',
      verifyTokenAudience: true,
      enablePkce: true,
      tokenValidation: {
        clockTolerance: 10,
        validateNbf: true
      }
    };

    // Validate required fields
    if (!config.realm || !config.authServerUrl || !config.credentials.secret) {
      throw new Error('Missing required Keycloak configuration');
    }

    logger.info('Keycloak configuration loaded successfully', {
      component: 'security',
      operation: 'loadKeycloakConfig'
    });

    return config;
  } catch (error) {
    logger.error(error as Error, {
      component: 'security',
      operation: 'loadKeycloakConfig'
    });
    throw error;
  }
};

// Load and validate JWT configuration
const loadJwtConfig = (): JwtConfig => {
  try {
    if (!process.env.JWT_SECRET || Buffer.from(process.env.JWT_SECRET).length < 32) {
      throw new Error('JWT secret must be at least 256 bits');
    }

    const config: JwtConfig = {
      secret: process.env.JWT_SECRET,
      expiresIn: process.env.JWT_EXPIRES_IN || DEFAULT_JWT_EXPIRY,
      algorithm: 'HS256',
      issuer: process.env.JWT_ISSUER || 'agricultural-management-system',
      audience: process.env.JWT_AUDIENCE ? JSON.parse(process.env.JWT_AUDIENCE) : ['ams-api'],
      clockTolerance: 30,
      maxAge: '1d',
      notBefore: '0s'
    };

    logger.info('JWT configuration loaded successfully', {
      component: 'security',
      operation: 'loadJwtConfig'
    });

    return config;
  } catch (error) {
    logger.error(error as Error, {
      component: 'security',
      operation: 'loadJwtConfig'
    });
    throw error;
  }
};

// Load and validate encryption configuration
const loadEncryptionConfig = (): EncryptionConfig => {
  try {
    const config: EncryptionConfig = {
      algorithm: DEFAULT_ENCRYPTION_ALGORITHM,
      keySize: MIN_KEY_SIZE,
      ivLength: 16,
      saltLength: 32,
      keyDerivation: {
        iterations: 100000,
        digest: 'sha512'
      },
      tagLength: 16
    };

    if (!process.env.ENCRYPTION_KEY) {
      throw new Error('Encryption key is required');
    }

    logger.info('Encryption configuration loaded successfully', {
      component: 'security',
      operation: 'loadEncryptionConfig'
    });

    return config;
  } catch (error) {
    logger.error(error as Error, {
      component: 'security',
      operation: 'loadEncryptionConfig'
    });
    throw error;
  }
};

// Load and validate TLS configuration
const loadTlsConfig = (): TlsConfig => {
  try {
    const config: TlsConfig = {
      version: DEFAULT_TLS_VERSION,
      ciphers: SECURE_CIPHERS,
      cert: fs.readFileSync(process.env.TLS_CERT_PATH || '').toString(),
      key: fs.readFileSync(process.env.TLS_KEY_PATH || '').toString(),
      ca: process.env.TLS_CA_PATH ? fs.readFileSync(process.env.TLS_CA_PATH).toString() : '',
      minVersion: DEFAULT_TLS_VERSION,
      maxVersion: DEFAULT_TLS_VERSION,
      secureOptions: crypto.constants.SSL_OP_NO_TLSv1 | 
                    crypto.constants.SSL_OP_NO_TLSv1_1 |
                    crypto.constants.SSL_OP_NO_RENEGOTIATION,
      sessionTimeout: 3600
    };

    if (!config.cert || !config.key) {
      throw new Error('TLS certificate and key are required');
    }

    logger.info('TLS configuration loaded successfully', {
      component: 'security',
      operation: 'loadTlsConfig'
    });

    return config;
  } catch (error) {
    logger.error(error as Error, {
      component: 'security',
      operation: 'loadTlsConfig'
    });
    throw error;
  }
};

// Export security configuration
export const securityConfig = {
  auth: {
    keycloak: loadKeycloakConfig(),
    jwt: loadJwtConfig(),
    allowedRoles: ALLOWED_ROLES,
    maxLoginAttempts: MAX_LOGIN_ATTEMPTS,
    tokenRefreshWindow: TOKEN_REFRESH_WINDOW,
    minPasswordLength: MIN_PASSWORD_LENGTH
  },
  encryption: loadEncryptionConfig(),
  tls: loadTlsConfig(),
  validation: {
    sanitizeInput: true,
    validateContentType: true,
    maxRequestSize: '10mb',
    rateLimiting: {
      windowMs: 15 * 60 * 1000,
      max: 100
    }
  }
};

export type {
  KeycloakConfig,
  JwtConfig,
  EncryptionConfig,
  TlsConfig
};