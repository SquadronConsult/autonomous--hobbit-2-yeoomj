import { Client, BucketItemCopy } from 'minio'; // v7.1.1
import { logger } from '../utils/logger';

// Environment variables with secure defaults
const MINIO_ENDPOINT = process.env.MINIO_ENDPOINT || 'minio';
const MINIO_PORT = parseInt(process.env.MINIO_PORT || '9000');
const MINIO_USE_SSL = process.env.MINIO_USE_SSL === 'true';
const MINIO_ACCESS_KEY = process.env.MINIO_ACCESS_KEY;
const MINIO_SECRET_KEY = process.env.MINIO_SECRET_KEY;
const MINIO_REGION = process.env.MINIO_REGION || 'us-east-1';
const MAX_RETRY_ATTEMPTS = 3;
const RETRY_DELAY_MS = 1000;

// Type definitions for enhanced configuration
interface MinioConfig {
    endPoint: string;
    port: number;
    useSSL: boolean;
    accessKey: string;
    secretKey: string;
    region: string;
    maxRetryAttempts: number;
    retryDelayMs: number;
}

interface EncryptionConfig {
    algorithm: string;
    serverSide: boolean;
}

interface RetentionPolicy {
    mode: 'COMPLIANCE' | 'GOVERNANCE';
    days: number;
}

interface ReplicationConfig {
    role?: string;
    destination?: string;
}

interface BucketConfig {
    name: string;
    policy: 'private' | 'public-read' | 'public-read-write';
    versioning: boolean;
    encryption: EncryptionConfig;
    retention: RetentionPolicy;
    replication?: ReplicationConfig;
}

// Constants for storage configuration
const ENCRYPTION_ALGORITHM = 'AES-256-GCM';
const DEFAULT_REGION = 'us-east-1';

const REQUIRED_BUCKETS: BucketConfig[] = [
    {
        name: 'models',
        policy: 'private',
        versioning: true,
        encryption: {
            algorithm: ENCRYPTION_ALGORITHM,
            serverSide: true
        },
        retention: {
            mode: 'COMPLIANCE',
            days: 90
        }
    },
    {
        name: 'video-data',
        policy: 'private',
        versioning: false,
        encryption: {
            algorithm: ENCRYPTION_ALGORITHM,
            serverSide: true
        },
        retention: {
            mode: 'GOVERNANCE',
            days: 30
        }
    },
    {
        name: 'telemetry',
        policy: 'private',
        versioning: true,
        encryption: {
            algorithm: ENCRYPTION_ALGORITHM,
            serverSide: true
        },
        retention: {
            mode: 'COMPLIANCE',
            days: 365
        }
    }
];

// Initialize MinIO client with retry logic
async function initializeMinioClient(config: MinioConfig): Promise<Client> {
    if (!config.accessKey || !config.secretKey) {
        throw new Error('MinIO access credentials are required');
    }

    const client = new Client({
        endPoint: config.endPoint,
        port: config.port,
        useSSL: config.useSSL,
        accessKey: config.accessKey,
        secretKey: config.secretKey,
        region: config.region
    });

    let attempts = 0;
    while (attempts < config.maxRetryAttempts) {
        try {
            // Verify connection and credentials
            await client.listBuckets();
            logger.info('MinIO client initialized successfully', {
                component: 'storage',
                operation: 'initialization',
                details: {
                    endpoint: config.endPoint,
                    port: config.port,
                    useSSL: config.useSSL,
                    region: config.region
                }
            });
            return client;
        } catch (error) {
            attempts++;
            if (attempts === config.maxRetryAttempts) {
                logger.error(error as Error, {
                    component: 'storage',
                    operation: 'initialization',
                    details: { attempts }
                });
                throw new Error('Failed to initialize MinIO client after maximum retry attempts');
            }
            logger.warn(`Retrying MinIO connection (${attempts}/${config.maxRetryAttempts})`, {
                component: 'storage',
                operation: 'initialization'
            });
            await new Promise(resolve => setTimeout(resolve, config.retryDelayMs));
        }
    }
    throw new Error('MinIO client initialization failed');
}

// Create and configure bucket with security policies
async function createBucketIfNotExists(
    minioClient: Client,
    bucketConfig: BucketConfig
): Promise<void> {
    try {
        const bucketExists = await minioClient.bucketExists(bucketConfig.name);
        
        if (!bucketExists) {
            await minioClient.makeBucket(bucketConfig.name, MINIO_REGION);
            
            // Configure bucket encryption
            if (bucketConfig.encryption.serverSide) {
                await minioClient.setBucketEncryption(bucketConfig.name, {
                    Rule: [{
                        ApplyServerSideEncryptionByDefault: {
                            SSEAlgorithm: bucketConfig.encryption.algorithm
                        }
                    }]
                });
            }

            // Set bucket policy
            const policy = JSON.stringify({
                Version: '2012-10-17',
                Statement: [{
                    Effect: 'Allow',
                    Principal: { AWS: ['*'] },
                    Action: bucketConfig.policy === 'private' ? 
                        ['s3:GetObject'] : 
                        ['s3:GetObject', 's3:PutObject'],
                    Resource: [`arn:aws:s3:::${bucketConfig.name}/*`]
                }]
            });
            await minioClient.setBucketPolicy(bucketConfig.name, policy);

            // Enable versioning if required
            if (bucketConfig.versioning) {
                await minioClient.setBucketVersioning(bucketConfig.name, {
                    Status: 'Enabled'
                });
            }

            logger.info(`Bucket ${bucketConfig.name} created and configured`, {
                component: 'storage',
                operation: 'bucket_creation',
                details: {
                    name: bucketConfig.name,
                    policy: bucketConfig.policy,
                    versioning: bucketConfig.versioning,
                    encryption: bucketConfig.encryption.algorithm
                }
            });
        }
    } catch (error) {
        logger.error(error as Error, {
            component: 'storage',
            operation: 'bucket_creation',
            details: { bucketName: bucketConfig.name }
        });
        throw error;
    }
}

// Export MinIO configuration
export const minioConfig: MinioConfig = {
    endPoint: MINIO_ENDPOINT,
    port: MINIO_PORT,
    useSSL: MINIO_USE_SSL,
    accessKey: MINIO_ACCESS_KEY!,
    secretKey: MINIO_SECRET_KEY!,
    region: MINIO_REGION,
    maxRetryAttempts: MAX_RETRY_ATTEMPTS,
    retryDelayMs: RETRY_DELAY_MS
};

// Initialize and export MinIO client instance
export const minioClient = await initializeMinioClient(minioConfig);

// Initialize required buckets
for (const bucketConfig of REQUIRED_BUCKETS) {
    await createBucketIfNotExists(minioClient, bucketConfig);
}

export { BucketConfig, MinioConfig };