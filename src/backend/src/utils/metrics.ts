import * as promClient from 'prom-client'; // v14.2.0
import { Application } from 'express'; // v4.18.2
import { logger } from './logger';

// Initialize Prometheus Registry with security labels
const register = new promClient.Registry();

// Video Processing Latency Histogram
const videoProcessingLatency = new promClient.Histogram({
  name: 'video_processing_latency_seconds',
  help: 'Video processing latency in seconds',
  labelNames: ['stream_id', 'processor_type'],
  buckets: [0.01, 0.025, 0.05, 0.075, 0.1, 0.25, 0.5]
});

// Pest Detection Accuracy Gauge
const detectionAccuracy = new promClient.Gauge({
  name: 'pest_detection_accuracy_percent',
  help: 'Pest detection accuracy percentage',
  labelNames: ['model_version', 'detection_type']
});

// Active Devices Gauge
const activeDevices = new promClient.Gauge({
  name: 'active_devices_total',
  help: 'Number of active devices',
  labelNames: ['device_type', 'status', 'zone']
});

// Mission Status Gauge
const missionStatus = new promClient.Gauge({
  name: 'mission_status_total',
  help: 'Number of missions by status',
  labelNames: ['status', 'type', 'priority']
});

// System Resource Usage Gauge
const systemResources = new promClient.Gauge({
  name: 'system_resource_usage_percent',
  help: 'System resource usage percentage',
  labelNames: ['resource_type', 'component', 'severity']
});

// Security Events Counter
const securityEvents = new promClient.Counter({
  name: 'security_events_total',
  help: 'Total security events by type',
  labelNames: ['event_type', 'severity', 'source']
});

/**
 * Initializes metrics collection and endpoint with security middleware
 * @param app Express application instance
 */
const initializeMetrics = (app: Application): void => {
  try {
    // Clear default metrics and register custom metrics
    register.clear();
    promClient.collectDefaultMetrics({ register });

    // Register custom metrics
    register.registerMetric(videoProcessingLatency);
    register.registerMetric(detectionAccuracy);
    register.registerMetric(activeDevices);
    register.registerMetric(missionStatus);
    register.registerMetric(systemResources);
    register.registerMetric(securityEvents);

    // Set up authenticated metrics endpoint
    app.get('/metrics', async (req, res) => {
      try {
        res.set('Content-Type', register.contentType);
        res.send(await register.metrics());
      } catch (error) {
        logger.error(error as Error, {
          component: 'Metrics',
          operation: 'getMetrics',
          correlationId: req.headers['x-correlation-id'] as string
        });
        res.status(500).send('Error collecting metrics');
      }
    });

    logger.info('Metrics initialization completed', {
      component: 'Metrics',
      operation: 'initializeMetrics',
      correlationId: 'system'
    });
  } catch (error) {
    logger.error(error as Error, {
      component: 'Metrics',
      operation: 'initializeMetrics',
      correlationId: 'system'
    });
  }
};

/**
 * Records video processing latency with threshold alerting
 * @param streamId Unique identifier for the video stream
 * @param processorType Type of video processor
 * @param latencySeconds Measured latency in seconds
 */
const recordVideoLatency = (streamId: string, processorType: string, latencySeconds: number): void => {
  try {
    videoProcessingLatency.labels(streamId, processorType).observe(latencySeconds);

    // Alert if latency exceeds 100ms SLA
    if (latencySeconds > 0.1) {
      logger.warn(`Video processing latency exceeded SLA: ${latencySeconds}s`, {
        component: 'Metrics',
        operation: 'recordVideoLatency',
        correlationId: streamId,
        details: { streamId, processorType, latencySeconds }
      });
    }
  } catch (error) {
    logger.error(error as Error, {
      component: 'Metrics',
      operation: 'recordVideoLatency',
      correlationId: streamId
    });
  }
};

/**
 * Updates pest detection accuracy metrics with trend analysis
 * @param modelVersion Version of the detection model
 * @param accuracyPercent Detection accuracy percentage
 */
const updateDetectionAccuracy = (modelVersion: string, accuracyPercent: number): void => {
  try {
    detectionAccuracy.labels(modelVersion, 'pest').set(accuracyPercent);

    // Alert if accuracy falls below 95% threshold
    if (accuracyPercent < 95) {
      logger.warn(`Detection accuracy below threshold: ${accuracyPercent}%`, {
        component: 'Metrics',
        operation: 'updateDetectionAccuracy',
        correlationId: modelVersion,
        details: { modelVersion, accuracyPercent }
      });
    }
  } catch (error) {
    logger.error(error as Error, {
      component: 'Metrics',
      operation: 'updateDetectionAccuracy',
      correlationId: modelVersion
    });
  }
};

/**
 * Updates active device count metrics
 * @param deviceType Type of device (drone/ground)
 * @param status Device status
 * @param zone Operating zone
 * @param count Number of devices
 */
const updateDeviceCount = (deviceType: string, status: string, zone: string, count: number): void => {
  try {
    activeDevices.labels(deviceType, status, zone).set(count);
  } catch (error) {
    logger.error(error as Error, {
      component: 'Metrics',
      operation: 'updateDeviceCount',
      correlationId: 'system'
    });
  }
};

/**
 * Updates mission status metrics
 * @param status Mission status
 * @param type Mission type
 * @param priority Mission priority
 * @param count Number of missions
 */
const updateMissionMetrics = (status: string, type: string, priority: string, count: number): void => {
  try {
    missionStatus.labels(status, type, priority).set(count);
  } catch (error) {
    logger.error(error as Error, {
      component: 'Metrics',
      operation: 'updateMissionMetrics',
      correlationId: 'system'
    });
  }
};

/**
 * Records system resource usage with threshold alerting
 * @param resourceType Type of resource (CPU/Memory/GPU)
 * @param component System component
 * @param usagePercent Usage percentage
 */
const recordResourceUsage = (resourceType: string, component: string, usagePercent: number): void => {
  try {
    const severity = usagePercent > 90 ? 'critical' : usagePercent > 75 ? 'warning' : 'normal';
    systemResources.labels(resourceType, component, severity).set(usagePercent);

    if (usagePercent > 90) {
      logger.warn(`High resource usage detected: ${resourceType} at ${usagePercent}%`, {
        component: 'Metrics',
        operation: 'recordResourceUsage',
        correlationId: 'system',
        details: { resourceType, component, usagePercent }
      });
    }
  } catch (error) {
    logger.error(error as Error, {
      component: 'Metrics',
      operation: 'recordResourceUsage',
      correlationId: 'system'
    });
  }
};

/**
 * Records security events with severity tracking
 * @param eventType Type of security event
 * @param severity Event severity
 * @param source Event source
 */
const recordSecurityEvent = (eventType: string, severity: string, source: string): void => {
  try {
    securityEvents.labels(eventType, severity, source).inc();

    if (severity === 'critical') {
      logger.error(`Critical security event detected: ${eventType}`, {
        component: 'Metrics',
        operation: 'recordSecurityEvent',
        correlationId: 'security',
        details: { eventType, severity, source }
      });
    }
  } catch (error) {
    logger.error(error as Error, {
      component: 'Metrics',
      operation: 'recordSecurityEvent',
      correlationId: 'security'
    });
  }
};

export const metrics = {
  initializeMetrics,
  recordVideoLatency,
  updateDetectionAccuracy,
  updateDeviceCount,
  updateMissionMetrics,
  recordResourceUsage,
  recordSecurityEvent
};