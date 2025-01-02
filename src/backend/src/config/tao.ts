import dotenv from 'dotenv'; // v16.0.0
import path from 'path'; // v1.7.0
import { logger } from '../utils/logger';
import { minioConfig } from './minio';

// Load environment variables
dotenv.config();

// Global constants
const TAO_VERSION = '4.0';
const DEFAULT_PRECISION = 'FP16';
const DEFAULT_GPU_ID = 0;
const MODEL_STORE_PATH = '/var/lib/tao/models';

// Interfaces for TAO configuration
interface IModelPaths {
  pestDetection: string;
  cropAnalysis: string;
  baseModels: string;
}

interface ITrainingConfig {
  batchSize: number;
  epochs: number;
  learningRate: number;
  momentum: number;
  weightDecay: number;
  scheduler: {
    type: string;
    warmupEpochs: number;
  };
}

interface IInferenceConfig {
  precision: string;
  batchSize: number;
  workspace: number;
  minConfidence: number;
  dynamicBatching: boolean;
}

interface IOptimizationConfig {
  pruning: {
    enabled: boolean;
    targetSparsity: number;
    schedule: string;
  };
  quantization: {
    enabled: boolean;
    calibrationBatches: number;
    precision: string;
  };
}

interface ISecurityConfig {
  modelEncryption: boolean;
  accessControl: string;
  auditLogging: boolean;
}

interface IMonitoringConfig {
  metrics: boolean;
  resourceTracking: boolean;
  alertThresholds: {
    gpuMemory: number;
    accuracy: number;
    latency: number;
  };
}

interface ITAOConfig {
  version: string;
  gpuId: number;
  modelPaths: IModelPaths;
  trainingConfig: ITrainingConfig;
  inferenceConfig: IInferenceConfig;
  optimizationConfig: IOptimizationConfig;
  securityConfig: ISecurityConfig;
  monitoringConfig: IMonitoringConfig;
}

// Validate TAO configuration
const validateConfig = (config: ITAOConfig): boolean => {
  try {
    // Version validation
    if (config.version !== TAO_VERSION) {
      throw new Error(`Unsupported TAO version: ${config.version}`);
    }

    // GPU validation
    if (config.gpuId < 0) {
      throw new Error('Invalid GPU ID');
    }

    // Path validation
    Object.values(config.modelPaths).forEach(modelPath => {
      if (!path.isAbsolute(modelPath)) {
        throw new Error(`Invalid model path: ${modelPath}`);
      }
    });

    // Training config validation
    if (config.trainingConfig.batchSize <= 0 || 
        config.trainingConfig.epochs <= 0 || 
        config.trainingConfig.learningRate <= 0) {
      throw new Error('Invalid training parameters');
    }

    // Inference config validation
    if (config.inferenceConfig.minConfidence < 0 || 
        config.inferenceConfig.minConfidence > 1) {
      throw new Error('Invalid confidence threshold');
    }

    return true;
  } catch (error) {
    logger.error(error as Error, {
      component: 'TAO',
      operation: 'config_validation',
      details: { config }
    });
    return false;
  }
};

// Initialize TAO configuration
const initializeTAO = async (config: ITAOConfig): Promise<boolean> => {
  try {
    // Validate configuration
    if (!validateConfig(config)) {
      throw new Error('TAO configuration validation failed');
    }

    logger.info('Initializing TAO configuration', {
      component: 'TAO',
      operation: 'initialization',
      details: {
        version: config.version,
        gpuId: config.gpuId
      }
    });

    return true;
  } catch (error) {
    logger.error(error as Error, {
      component: 'TAO',
      operation: 'initialization'
    });
    return false;
  }
};

// TAO configuration instance
export const taoConfig: ITAOConfig = {
  version: TAO_VERSION,
  gpuId: DEFAULT_GPU_ID,
  modelPaths: {
    pestDetection: `${MODEL_STORE_PATH}/pest_detection`,
    cropAnalysis: `${MODEL_STORE_PATH}/crop_analysis`,
    baseModels: `${MODEL_STORE_PATH}/base_models`
  },
  trainingConfig: {
    batchSize: 32,
    epochs: 100,
    learningRate: 0.001,
    momentum: 0.9,
    weightDecay: 0.0005,
    scheduler: {
      type: 'cosine',
      warmupEpochs: 5
    }
  },
  inferenceConfig: {
    precision: DEFAULT_PRECISION,
    batchSize: 8,
    workspace: 1024,
    minConfidence: 0.85,
    dynamicBatching: true
  },
  optimizationConfig: {
    pruning: {
      enabled: true,
      targetSparsity: 0.5,
      schedule: 'polynomial'
    },
    quantization: {
      enabled: true,
      calibrationBatches: 10,
      precision: 'INT8'
    }
  },
  securityConfig: {
    modelEncryption: true,
    accessControl: 'RBAC',
    auditLogging: true
  },
  monitoringConfig: {
    metrics: true,
    resourceTracking: true,
    alertThresholds: {
      gpuMemory: 0.9,
      accuracy: 0.95,
      latency: 100
    }
  }
};

// Export configuration and initialization functions
export { ITAOConfig, IOptimizationConfig, ISecurityConfig, initializeTAO, validateConfig };