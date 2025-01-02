/**
 * @fileoverview Integration tests for ROS 2 nodes in agricultural management system
 * @version 1.0.0
 */

import * as rclnodejs from 'rclnodejs'; // v0.21.1
import { Subject, firstValueFrom, timeout } from 'rxjs'; // v7.8.0
import { 
    RobotType, 
    RobotStatus, 
    RobotCapability 
} from '../../src/constants/robotTypes';
import { 
    TelemetryType, 
    TelemetryUnit 
} from '../../src/constants/telemetryTypes';
import { MissionStatus } from '../../src/constants/missionStatus';
import { IDevice, IDeviceLocation } from '../../src/interfaces/IDevice';
import { ITelemetry } from '../../src/interfaces/ITelemetry';
import { IMission } from '../../src/interfaces/IMission';

// Test configuration constants
const TEST_TIMEOUT = 30000; // 30 seconds
const PERFORMANCE_THRESHOLD = 100; // 100ms latency threshold
const NODE_DISCOVERY_TIMEOUT = 5000; // 5 seconds
const QOS_PROFILE = {
    reliability: rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    durability: rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    history: rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    depth: 10
};

/**
 * Performance monitoring utility class
 */
class PerformanceMonitor {
    private measurements: number[] = [];

    recordLatency(startTime: number): void {
        const latency = Date.now() - startTime;
        this.measurements.push(latency);
    }

    getAverageLatency(): number {
        return this.measurements.reduce((a, b) => a + b, 0) / this.measurements.length;
    }

    reset(): void {
        this.measurements = [];
    }
}

/**
 * Sets up the ROS 2 test environment
 */
async function setupTestEnvironment(): Promise<void> {
    if (!rclnodejs.isInitialized()) {
        await rclnodejs.init({
            contextDomainId: 42, // Isolated test domain
            enableTypedArray: true,
            enableRosTime: true
        });
    }
}

/**
 * Cleans up the ROS 2 test environment
 */
async function cleanupTestEnvironment(): Promise<void> {
    await rclnodejs.shutdown();
}

/**
 * Main ROS 2 integration test suite
 */
export class ROSIntegrationTestSuite {
    private testNode: rclnodejs.Node;
    private deviceStatusSubject: Subject<IDevice>;
    private telemetrySubject: Subject<ITelemetry>;
    private perfMonitor: PerformanceMonitor;

    constructor() {
        this.testNode = new rclnodejs.Node('integration_test_node');
        this.deviceStatusSubject = new Subject<IDevice>();
        this.telemetrySubject = new Subject<ITelemetry>();
        this.perfMonitor = new PerformanceMonitor();
    }

    /**
     * Tests fleet controller integration with multiple robots
     */
    async testFleetControllerIntegration(): Promise<void> {
        const fleetPublisher = this.testNode.createPublisher(
            'std_msgs/msg/String',
            '/fleet_commands',
            QOS_PROFILE
        );

        const statusSubscriber = this.testNode.createSubscription(
            'std_msgs/msg/String',
            '/fleet_status',
            (msg: any) => {
                const startTime = parseInt(msg.data.split(':')[1]);
                this.perfMonitor.recordLatency(startTime);
            },
            QOS_PROFILE
        );

        // Test multi-robot coordination
        const testMission: IMission = {
            id: 'test_mission_1',
            name: 'Integration Test Mission',
            status: MissionStatus.QUEUED,
            type: 'surveillance',
            assignedDevices: [
                { deviceId: 'drone_1', type: RobotType.AERIAL_DRONE },
                { deviceId: 'drone_2', type: RobotType.AERIAL_DRONE },
                { deviceId: 'ground_1', type: RobotType.GROUND_ROBOT }
            ],
            coverageArea: {
                type: 'Polygon',
                coordinates: [[0, 0], [0, 100], [100, 100], [100, 0], [0, 0]],
                properties: {}
            }
        } as IMission;

        // Publish mission commands
        await new Promise<void>((resolve) => {
            const startTime = Date.now();
            fleetPublisher.publish({ data: `MISSION:${JSON.stringify(testMission)}:${startTime}` });
            
            setTimeout(() => {
                const avgLatency = this.perfMonitor.getAverageLatency();
                expect(avgLatency).toBeLessThan(PERFORMANCE_THRESHOLD);
                resolve();
            }, 5000);
        });
    }

    /**
     * Tests navigation controller with dynamic path planning
     */
    async testNavigationControllerIntegration(): Promise<void> {
        const navPublisher = this.testNode.createPublisher(
            'geometry_msgs/msg/PoseStamped',
            '/navigation_goals',
            QOS_PROFILE
        );

        const pathSubscriber = this.testNode.createSubscription(
            'nav_msgs/msg/Path',
            '/planned_path',
            (msg: any) => {
                const startTime = parseInt(msg.header.frame_id);
                this.perfMonitor.recordLatency(startTime);
            },
            QOS_PROFILE
        );

        // Test navigation commands
        await new Promise<void>((resolve) => {
            const startTime = Date.now();
            const navigationGoal = {
                header: {
                    frame_id: startTime.toString()
                },
                pose: {
                    position: { x: 10.0, y: 20.0, z: 0.0 },
                    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
                }
            };

            navPublisher.publish(navigationGoal);

            setTimeout(() => {
                const avgLatency = this.perfMonitor.getAverageLatency();
                expect(avgLatency).toBeLessThan(PERFORMANCE_THRESHOLD);
                resolve();
            }, 5000);
        });
    }

    /**
     * Tests telemetry collection with performance validation
     */
    async testTelemetryCollectorIntegration(): Promise<void> {
        const telemetryPublisher = this.testNode.createPublisher(
            'std_msgs/msg/String',
            '/device_telemetry',
            QOS_PROFILE
        );

        const telemetrySubscriber = this.testNode.createSubscription(
            'std_msgs/msg/String',
            '/telemetry_processed',
            (msg: any) => {
                const startTime = parseInt(msg.data.split(':')[1]);
                this.perfMonitor.recordLatency(startTime);
            },
            QOS_PROFILE
        );

        // Test high-frequency telemetry
        await new Promise<void>((resolve) => {
            const interval = setInterval(() => {
                const startTime = Date.now();
                const telemetry: ITelemetry = {
                    id: `telemetry_${startTime}`,
                    deviceId: 'drone_1',
                    timestamp: new Date(),
                    type: TelemetryType.LOCATION,
                    value: { lat: 45.0, lon: -122.0, alt: 100.0 },
                    unit: TelemetryUnit.METERS,
                    metadata: {}
                };

                telemetryPublisher.publish({ 
                    data: `TELEMETRY:${JSON.stringify(telemetry)}:${startTime}` 
                });
            }, 100);

            setTimeout(() => {
                clearInterval(interval);
                const avgLatency = this.perfMonitor.getAverageLatency();
                expect(avgLatency).toBeLessThan(PERFORMANCE_THRESHOLD);
                resolve();
            }, 5000);
        });
    }
}

// Jest test suite configuration
describe('ROS 2 Integration Tests', () => {
    let testSuite: ROSIntegrationTestSuite;

    beforeAll(async () => {
        await setupTestEnvironment();
        testSuite = new ROSIntegrationTestSuite();
    });

    afterAll(async () => {
        await cleanupTestEnvironment();
    });

    beforeEach(() => {
        jest.setTimeout(TEST_TIMEOUT);
    });

    it('should coordinate multiple robots in fleet operations', async () => {
        await testSuite.testFleetControllerIntegration();
    });

    it('should handle dynamic navigation planning', async () => {
        await testSuite.testNavigationControllerIntegration();
    });

    it('should process telemetry data within latency requirements', async () => {
        await testSuite.testTelemetryCollectorIntegration();
    });
});