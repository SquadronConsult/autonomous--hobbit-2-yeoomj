import dotenv from 'dotenv'; // v16.0.0
import path from 'path'; // v1.7.0
import { logger } from '../utils/logger';
import { taoConfig } from './tao';

// Load environment variables
dotenv.config();

// Global constants for DeepStream configuration
export const DEEPSTREAM_VERSION = '6.2';
export const MAX_BATCH_SIZE = 8;
export const MAX_STREAMS = 16;
export const DEFAULT_INFERENCE_PRECISION = 'FP16';
export const DEFAULT_GPU_ID = 0;
export const PERFORMANCE_MONITORING_INTERVAL = 1000;
export const MAX_LATENCY_THRESHOLD = 100;

// Interface definitions for DeepStream configuration
interface IPipelineConfig {
  streamingWidth: number;
  streamingHeight: number;
  frameRate: number;
  scalingWidth: number;
  scalingHeight: number;
  latencyTarget: number;
}

interface IInferenceConfig {
  precision: string;
  modelEngine: string;
  labelPath: string;
  networkMode: string;
  customLibPath: string;
  optimizationLevel: string;
}

interface IOutputConfig {
  rtspEnabled: boolean;
  rtspPort: number;
  fileOutput: boolean;
  outputPath: string;
  overlayEnabled: boolean;
}

interface ISecurityConfig {
  tlsEnabled: boolean;
  certificatePath: string;
  keyPath: string;
  authToken: string;
}

interface IMonitoringConfig {
  performanceMetricsEnabled: boolean;
  metricsInterval: number;
  latencyThreshold: number;
  gpuUtilizationThreshold: number;
}

interface IDeepStreamConfig {
  version: string;
  gpuId: number;
  maxBatchSize: number;
  maxStreams: number;
  pipelineConfig: IPipelineConfig;
  inferenceConfig: IInferenceConfig;
  outputConfig: IOutputConfig;
  securityConfig: ISecurityConfig;
  monitoringConfig: IMonitoringConfig;
}

// Validate DeepStream configuration parameters
const validateConfig = (config: IDeepStreamConfig): boolean => {
  try {
    // Version validation
    if (config.version !== DEEPSTREAM_VERSION) {
      throw new Error(`Unsupported DeepStream version: ${config.version}`);
    }

    // Resource limits validation
    if (config.maxBatchSize > MAX_BATCH_SIZE || config.maxStreams > MAX_STREAMS) {
      throw new Error('Exceeded maximum resource limits');
    }

    // Pipeline configuration validation
    if (config.pipelineConfig.latencyTarget > MAX_LATENCY_THRESHOLD) {
      throw new Error('Latency target exceeds threshold');
    }

    // Security configuration validation
    if (config.securityConfig.tlsEnabled) {
      if (!path.isAbsolute(config.securityConfig.certificatePath) ||
          !path.isAbsolute(config.securityConfig.keyPath)) {
        throw new Error('Invalid security certificate paths');
      }
    }

    return true;
  } catch (error) {
    logger.error(error as Error, {
      component: 'DeepStream',
      operation: 'config_validation',
      details: { config }
    });
    return false;
  }
};

// Initialize DeepStream with security and monitoring
const initializeDeepStream = async (config: IDeepStreamConfig): Promise<boolean> => {
  try {
    // Validate configuration
    if (!validateConfig(config)) {
      throw new Error('DeepStream configuration validation failed');
    }

    logger.info('Initializing DeepStream configuration', {
      component: 'DeepStream',
      operation: 'initialization',
      details: {
        version: config.version,
        gpuId: config.gpuId,
        maxStreams: config.maxStreams
      }
    });

    // Initialize security context if enabled
    if (config.securityConfig.tlsEnabled) {
      logger.debug('Initializing TLS security', {
        component: 'DeepStream',
        operation: 'security_initialization'
      });
    }

    // Initialize performance monitoring
    if (config.monitoringConfig.performanceMetricsEnabled) {
      logger.debug('Initializing performance monitoring', {
        component: 'DeepStream',
        operation: 'monitoring_initialization',
        details: {
          interval: config.monitoringConfig.metricsInterval
        }
      });
    }

    return true;
  } catch (error) {
    logger.error(error as Error, {
      component: 'DeepStream',
      operation: 'initialization'
    });
    return false;
  }
};

// DeepStream configuration instance
export const deepstreamConfig: IDeepStreamConfig = {
  version: DEEPSTREAM_VERSION,
  gpuId: DEFAULT_GPU_ID,
  maxBatchSize: MAX_BATCH_SIZE,
  maxStreams: MAX_STREAMS,
  pipelineConfig: {
    streamingWidth: 1920,
    streamingHeight: 1080,
    frameRate: 30,
    scalingWidth: 960,
    scalingHeight: 540,
    latencyTarget: MAX_LATENCY_THRESHOLD
  },
  inferenceConfig: {
    precision: DEFAULT_INFERENCE_PRECISION,
    modelEngine: `${taoConfig.modelPaths.pestDetection}/model.engine`,
    labelPath: `${taoConfig.modelPaths.pestDetection}/labels.txt`,
    networkMode: 'primary',
    customLibPath: '/opt/nvidia/deepstream/lib/libnvdsinfer_custom.so',
    optimizationLevel: 'max_throughput'
  },
  outputConfig: {
    rtspEnabled: true,
    rtspPort: 8554,
    fileOutput: true,
    outputPath: '/var/lib/deepstream/output',
    overlayEnabled: true
  },
  securityConfig: {
    tlsEnabled: true,
    certificatePath: '/etc/deepstream/certs/server.crt',
    keyPath: '/etc/deepstream/certs/server.key',
    authToken: process.env.DEEPSTREAM_AUTH_TOKEN || ''
  },
  monitoringConfig: {
    performanceMetricsEnabled: true,
    metricsInterval: PERFORMANCE_MONITORING_INTERVAL,
    latencyThreshold: MAX_LATENCY_THRESHOLD,
    gpuUtilizationThreshold: 85
  }
};

// Export configuration and initialization functions
export { 
  IDeepStreamConfig,
  IPipelineConfig,
  IInferenceConfig,
  IOutputConfig,
  ISecurityConfig,
  IMonitoringConfig,
  initializeDeepStream,
  validateConfig
};