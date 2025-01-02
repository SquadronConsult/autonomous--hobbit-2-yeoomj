/**
 * @fileoverview ROS 2 JAZZY configuration for agricultural robotics system
 * Provides comprehensive settings for robot fleet management, navigation control,
 * and telemetry collection with optimized real-time performance parameters
 * @version 1.0.0
 */

import { RobotType, RobotStatus } from '../interfaces/IDevice';
import * as rclnodejs from 'rclnodejs'; // v0.21.3
import { QoSProfile } from 'rclnodejs'; // v0.21.3

/**
 * Interface for ROS 2 node configuration
 */
interface ROSNodeConfig {
    namespace: string;
    name: string;
    parameters: Record<string, any>;
    dependencies?: string[];
    healthCheck: {
        enabled: boolean;
        interval: number;
        timeout: number;
    };
}

/**
 * Interface for ROS 2 topic configuration
 */
interface ROSTopicConfig {
    name: string;
    messageType: string;
    qosProfile: string;
    monitoring: {
        rateLimit: number;
        diagnostics: boolean;
        priority?: string;
    };
}

// Global ROS 2 configuration parameters
export const ROS_DOMAIN_ID = 42;
export const ROS_NAMESPACE = '/agricultural_robotics';

/**
 * Default QoS profile for general communication
 * Provides reliable delivery with moderate latency requirements
 */
const DEFAULT_QOS_PROFILE: QoSProfile = {
    reliability: 'RELIABLE',
    durability: 'VOLATILE',
    history: 'KEEP_LAST',
    depth: 10,
    deadline: 100,
    liveliness: 'AUTOMATIC',
    livelinessLeaseDuration: 1000
};

/**
 * Real-time QoS profile for high-frequency telemetry data
 * Optimized for low-latency with best-effort reliability
 */
const REALTIME_QOS_PROFILE: QoSProfile = {
    reliability: 'BEST_EFFORT',
    durability: 'VOLATILE',
    history: 'KEEP_LAST',
    depth: 1,
    deadline: 50,
    liveliness: 'AUTOMATIC',
    livelinessLeaseDuration: 500
};

/**
 * Node configurations for core system components
 */
const NODE_CONFIGS: Record<string, ROSNodeConfig> = {
    fleetController: {
        name: 'fleet_controller',
        namespace: '/agricultural_robotics',
        parameters: {
            maxDrones: 12,
            updateRate: 20,
            missionTimeout: 3600,
            heartbeatInterval: 50,
            networkLatencyThreshold: 50,
            failureDetectionTimeout: 200
        },
        healthCheck: {
            enabled: true,
            interval: 1000,
            timeout: 500
        }
    },
    navigationController: {
        name: 'navigation_controller',
        namespace: '/agricultural_robotics',
        parameters: {
            pathPlanningRate: 10,
            obstacleThreshold: 2.0,
            safetyMargin: 1.5,
            emergencyStopDistance: 3.0,
            maxVelocity: 5.0
        },
        healthCheck: {
            enabled: true,
            interval: 1000,
            timeout: 500
        }
    },
    telemetryCollector: {
        name: 'telemetry_collector',
        namespace: '/agricultural_robotics',
        parameters: {
            samplingRate: 20,
            bufferSize: 1000,
            batchSize: 100,
            compressionEnabled: true,
            retentionPeriod: 86400
        },
        healthCheck: {
            enabled: true,
            interval: 1000,
            timeout: 500
        }
    }
};

/**
 * Topic configurations for system communication
 */
const TOPIC_CONFIGS: Record<string, ROSTopicConfig> = {
    deviceStatus: {
        name: '/device_status',
        messageType: 'DeviceStatus',
        qosProfile: 'DEFAULT_QOS_PROFILE',
        monitoring: {
            rateLimit: 100,
            diagnostics: true
        }
    },
    missionCommand: {
        name: '/mission_command',
        messageType: 'MissionCommand',
        qosProfile: 'DEFAULT_QOS_PROFILE',
        monitoring: {
            rateLimit: 50,
            diagnostics: true
        }
    },
    telemetryData: {
        name: '/telemetry_data',
        messageType: 'TelemetryData',
        qosProfile: 'REALTIME_QOS_PROFILE',
        monitoring: {
            rateLimit: 200,
            diagnostics: true
        }
    },
    fleetStatus: {
        name: '/fleet_status',
        messageType: 'FleetStatus',
        qosProfile: 'DEFAULT_QOS_PROFILE',
        monitoring: {
            rateLimit: 10,
            diagnostics: true
        }
    },
    emergencyControl: {
        name: '/emergency_control',
        messageType: 'EmergencyCommand',
        qosProfile: 'DEFAULT_QOS_PROFILE',
        monitoring: {
            rateLimit: 100,
            diagnostics: true,
            priority: 'HIGH'
        }
    }
};

/**
 * Parameter configurations for ROS 2 nodes
 */
const PARAMETER_CONFIG = {
    fleetManagement: {
        maxDevices: 24,
        updateInterval: 50,
        timeoutThreshold: 1000
    },
    navigation: {
        planningFrequency: 10,
        safetyDistance: 2.0,
        maxAcceleration: 2.0
    },
    telemetry: {
        bufferSize: 1000,
        flushInterval: 100,
        compressionLevel: 'high'
    }
};

/**
 * QoS profile configurations for different communication patterns
 */
const QOS_PROFILES = {
    default: DEFAULT_QOS_PROFILE,
    realtime: REALTIME_QOS_PROFILE,
    reliable: {
        ...DEFAULT_QOS_PROFILE,
        reliability: 'RELIABLE',
        depth: 20
    },
    highThroughput: {
        ...REALTIME_QOS_PROFILE,
        depth: 5,
        deadline: 100
    }
};

/**
 * Exported ROS 2 configuration object
 */
export const rosConfig = {
    nodeConfig: NODE_CONFIGS,
    topicConfig: TOPIC_CONFIGS,
    parameterConfig: PARAMETER_CONFIG,
    qosProfiles: QOS_PROFILES
};