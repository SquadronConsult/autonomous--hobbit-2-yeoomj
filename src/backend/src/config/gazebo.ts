/**
 * @fileoverview Gazebo simulation environment configuration module
 * Provides comprehensive configuration for physics properties, world parameters,
 * and robot models for agricultural simulations with validation and environment variable support
 * @version 1.0.0
 */

import { z } from 'zod'; // v3.21.4
import * as dotenv from 'dotenv'; // v16.0.3
import { IDevice } from '../interfaces/IDevice';
import { validateGeolocation } from '../utils/validation';

// Load environment variables
dotenv.config();

/**
 * Physics engine configuration schema
 */
const PhysicsConfigSchema = z.object({
    engine: z.enum(['ode', 'bullet', 'simbody']).default('ode'),
    max_step_size: z.number().min(0.0001).max(0.01).default(0.001),
    real_time_factor: z.number().min(0.1).max(10.0).default(1.0),
    max_contacts: z.number().min(1).max(100).default(20),
    gravity: z.object({
        x: z.number().default(0.0),
        y: z.number().default(0.0),
        z: z.number().default(-9.81)
    }),
    constraints: z.object({
        cfm: z.number().min(0).default(0.0),
        erp: z.number().min(0).max(1).default(0.2),
        contact_max_correcting_vel: z.number().min(0).default(100.0),
        contact_surface_layer: z.number().min(0).default(0.001)
    })
});

/**
 * World configuration schema for agricultural environment
 */
const WorldConfigSchema = z.object({
    name: z.string().default('agricultural_simulation'),
    sdf_version: z.string().default('1.7'),
    scene: z.object({
        ambient_light: z.object({
            r: z.number().min(0).max(1).default(0.4),
            g: z.number().min(0).max(1).default(0.4),
            b: z.number().min(0).max(1).default(0.4),
            a: z.number().min(0).max(1).default(1.0)
        }),
        shadows: z.boolean().default(true),
        grid: z.boolean().default(false)
    }),
    environment: z.object({
        terrain_size: z.object({
            x: z.number().min(100).max(10000).default(1000),
            y: z.number().min(100).max(10000).default(1000)
        }),
        resolution: z.number().min(0.1).max(10.0).default(1.0),
        wind: z.object({
            linear_velocity: z.object({
                x: z.number().default(0.0),
                y: z.number().default(0.0),
                z: z.number().default(0.0)
            })
        })
    })
});

/**
 * Robot configuration schema for agricultural fleet
 */
const RobotConfigSchema = z.object({
    spawn_points: z.object({
        drones: z.array(z.object({
            x: z.number(),
            y: z.number(),
            z: z.number().min(0),
            yaw: z.number().min(-Math.PI).max(Math.PI)
        })),
        ground_robots: z.array(z.object({
            x: z.number(),
            y: z.number(),
            z: z.number().default(0),
            yaw: z.number().min(-Math.PI).max(Math.PI)
        }))
    }),
    model_paths: z.object({
        drone: z.string(),
        ground_robot: z.string()
    }),
    limits: z.object({
        max_velocity: z.number().min(0).max(20.0).default(10.0),
        max_acceleration: z.number().min(0).max(5.0).default(2.0),
        max_angular_velocity: z.number().min(0).max(3.14).default(1.57)
    })
});

/**
 * Resource configuration schema for simulation scaling
 */
const ResourceConfigSchema = z.object({
    cpu_limit: z.number().min(1).max(32).default(4),
    memory_limit: z.string().regex(/^\d+[KMGT]i$/).default('8Gi'),
    gpu_enabled: z.boolean().default(true),
    scaling_factor: z.number().min(0.1).max(2.0).default(1.0)
});

/**
 * Complete Gazebo configuration schema
 */
const GazeboConfigSchema = z.object({
    physics: PhysicsConfigSchema,
    world: WorldConfigSchema,
    robots: RobotConfigSchema,
    resources: ResourceConfigSchema
});

/**
 * Default Gazebo configuration with agricultural-specific settings
 */
export const gazeboConfig = {
    physics: {
        engine: process.env.GAZEBO_PHYSICS_ENGINE || 'ode',
        max_step_size: parseFloat(process.env.GAZEBO_MAX_STEP_SIZE || '0.001'),
        real_time_factor: parseFloat(process.env.GAZEBO_REAL_TIME_FACTOR || '1.0'),
        max_contacts: 20,
        gravity: { x: 0.0, y: 0.0, z: -9.81 },
        constraints: {
            cfm: 0.0,
            erp: 0.2,
            contact_max_correcting_vel: 100.0,
            contact_surface_layer: 0.001
        }
    },
    world: {
        name: 'agricultural_simulation',
        sdf_version: '1.7',
        scene: {
            ambient_light: { r: 0.4, g: 0.4, b: 0.4, a: 1.0 },
            shadows: true,
            grid: false
        },
        environment: {
            terrain_size: { x: 1000, y: 1000 },
            resolution: 1.0,
            wind: {
                linear_velocity: { x: 0.0, y: 0.0, z: 0.0 }
            }
        }
    },
    robots: {
        spawn_points: {
            drones: [{ x: 0.0, y: 0.0, z: 2.0, yaw: 0.0 }],
            ground_robots: [{ x: 5.0, y: 5.0, z: 0.0, yaw: 0.0 }]
        },
        model_paths: {
            drone: process.env.GAZEBO_MODEL_PATH ? `${process.env.GAZEBO_MODEL_PATH}/drone.sdf` : '../gazebo/models/drone.sdf',
            ground_robot: process.env.GAZEBO_MODEL_PATH ? `${process.env.GAZEBO_MODEL_PATH}/groundRobot.sdf` : '../gazebo/models/groundRobot.sdf'
        },
        limits: {
            max_velocity: 10.0,
            max_acceleration: 2.0,
            max_angular_velocity: 1.57
        }
    },
    resources: {
        cpu_limit: parseInt(process.env.GAZEBO_CPU_LIMIT || '4'),
        memory_limit: process.env.GAZEBO_MEMORY_LIMIT || '8Gi',
        gpu_enabled: process.env.GAZEBO_GPU_ENABLED !== 'false',
        scaling_factor: parseFloat(process.env.GAZEBO_SCALING_FACTOR || '1.0')
    }
};

/**
 * Validates Gazebo configuration settings
 * @param config Gazebo configuration object to validate
 * @returns Validation result with detailed error messages
 */
export function validateGazeboConfig(config: typeof gazeboConfig): z.SafeParseReturnType<typeof gazeboConfig, typeof gazeboConfig> {
    return GazeboConfigSchema.safeParse(config);
}

/**
 * Retrieves and merges Gazebo configuration with environment overrides
 * @returns Complete Gazebo configuration object with resolved overrides
 */
export function getGazeboConfig(): typeof gazeboConfig {
    const config = { ...gazeboConfig };

    // Apply environment variable overrides
    if (process.env.GAZEBO_MASTER_URI) {
        process.env.GAZEBO_MASTER_URI = process.env.GAZEBO_MASTER_URI;
    }

    if (process.env.GAZEBO_MODEL_PATH) {
        config.robots.model_paths.drone = `${process.env.GAZEBO_MODEL_PATH}/drone.sdf`;
        config.robots.model_paths.ground_robot = `${process.env.GAZEBO_MODEL_PATH}/groundRobot.sdf`;
    }

    // Validate final configuration
    const validationResult = validateGazeboConfig(config);
    if (!validationResult.success) {
        throw new Error(`Invalid Gazebo configuration: ${JSON.stringify(validationResult.error.errors)}`);
    }

    return config;
}