/**
 * @fileoverview MongoDB schema and model for agricultural missions with enhanced validation
 * @version 1.0.0
 */

import { Schema, model, Document } from 'mongoose'; // v6.9.1
import { IMission } from '../interfaces/IMission';
import { MissionStatus } from '../constants/missionStatus';
import { RobotType } from '../constants/robotTypes';

/**
 * GeoJSON schema for mission coverage area with ROS 2 compatibility
 */
const GeoJSONSchema = new Schema({
    type: {
        type: String,
        enum: ['Polygon'],
        required: true
    },
    coordinates: {
        type: [[[Number]]],
        required: true,
        validate: {
            validator: function(coords: number[][][]) {
                return coords.length > 0 && coords[0].length >= 3;
            },
            message: 'Coverage area must be a valid polygon with at least 3 points'
        }
    },
    properties: {
        type: Map,
        of: Schema.Types.Mixed,
        default: {}
    }
}, { _id: false });

/**
 * Mission parameters schema with validation
 */
const ParametersSchema = new Schema({
    altitude: {
        type: Number,
        min: 0,
        max: 120, // Maximum legal drone altitude in meters
    },
    speed: {
        type: Number,
        min: 0,
        max: 20, // Maximum speed in meters per second
    },
    overlap: {
        type: Number,
        min: 0,
        max: 100,
    },
    resolution: {
        type: Number,
        min: 0,
    },
    treatmentType: String,
    applicationRate: {
        type: Number,
        min: 0,
    },
    customParams: {
        type: Map,
        of: Schema.Types.Mixed,
    }
}, { _id: false });

/**
 * Mission metadata schema with audit trail
 */
const MetadataSchema = new Schema({
    createdAt: {
        type: Date,
        default: Date.now,
        required: true
    },
    updatedAt: {
        type: Date,
        default: Date.now,
        required: true
    },
    createdBy: {
        type: String,
        required: true
    },
    version: {
        type: Number,
        default: 1,
        min: 1
    },
    tags: [String],
    priority: {
        type: Number,
        default: 1,
        min: 1,
        max: 10
    },
    dependencies: [String],
    offline: {
        type: Boolean,
        default: false
    }
}, { _id: false });

/**
 * Enhanced MongoDB schema for agricultural missions
 */
const MissionSchema = new Schema<IMission>({
    name: {
        type: String,
        required: true,
        trim: true,
        maxlength: 100,
        index: true
    },
    description: {
        type: String,
        maxlength: 1000
    },
    status: {
        type: String,
        enum: Object.values(MissionStatus),
        default: MissionStatus.CREATED,
        required: true,
        index: true
    },
    type: {
        type: String,
        required: true,
        enum: ['surveillance', 'treatment', 'monitoring', 'intervention'],
        index: true
    },
    assignedDevices: [{
        deviceId: {
            type: String,
            required: true
        },
        type: {
            type: String,
            enum: Object.values(RobotType),
            required: true
        }
    }],
    coverageArea: {
        type: GeoJSONSchema,
        required: true,
        index: '2dsphere'
    },
    startTime: {
        type: Date,
        required: true,
        validate: {
            validator: function(this: IMission, startTime: Date) {
                return startTime >= new Date();
            },
            message: 'Start time must be in the future'
        }
    },
    endTime: {
        type: Date,
        validate: {
            validator: function(this: IMission, endTime: Date) {
                return !endTime || endTime > this.startTime;
            },
            message: 'End time must be after start time'
        }
    },
    progress: {
        type: Number,
        default: 0,
        min: 0,
        max: 100
    },
    parameters: {
        type: ParametersSchema,
        required: true
    },
    metadata: {
        type: MetadataSchema,
        required: true
    }
}, {
    timestamps: true,
    versionKey: 'version'
});

// Indexes for optimized queries
MissionSchema.index({ 'metadata.createdAt': 1 });
MissionSchema.index({ 'metadata.priority': -1 });
MissionSchema.index({ status: 1, startTime: 1 });
MissionSchema.index({ 'assignedDevices.deviceId': 1 });

// Pre-save validation hook
MissionSchema.pre('save', async function(next) {
    // Update metadata timestamps
    this.metadata.updatedAt = new Date();
    
    // Validate status transitions
    if (this.isModified('status')) {
        const validTransitions: Record<MissionStatus, MissionStatus[]> = {
            [MissionStatus.CREATED]: [MissionStatus.QUEUED, MissionStatus.CANCELLED],
            [MissionStatus.QUEUED]: [MissionStatus.IN_PROGRESS, MissionStatus.CANCELLED],
            [MissionStatus.IN_PROGRESS]: [MissionStatus.PAUSED, MissionStatus.COMPLETED, MissionStatus.FAILED],
            [MissionStatus.PAUSED]: [MissionStatus.IN_PROGRESS, MissionStatus.CANCELLED],
            [MissionStatus.COMPLETED]: [],
            [MissionStatus.FAILED]: [],
            [MissionStatus.CANCELLED]: []
        };

        const oldStatus = this.status;
        const newStatus = this.get('status');
        
        if (!validTransitions[oldStatus]?.includes(newStatus)) {
            throw new Error(`Invalid status transition from ${oldStatus} to ${newStatus}`);
        }
    }

    next();
});

// Static methods
MissionSchema.statics.findByStatus = async function(
    status: MissionStatus,
    options: { page?: number; limit?: number; sort?: string }
): Promise<{ missions: IMission[]; total: number }> {
    const page = options.page || 1;
    const limit = options.limit || 10;
    const sort = options.sort || '-metadata.priority';

    const [missions, total] = await Promise.all([
        this.find({ status })
            .sort(sort)
            .skip((page - 1) * limit)
            .limit(limit)
            .exec(),
        this.countDocuments({ status })
    ]);

    return { missions, total };
};

MissionSchema.statics.findByDevice = async function(
    deviceId: string,
    type: RobotType
): Promise<IMission[]> {
    return this.find({
        'assignedDevices': {
            $elemMatch: {
                deviceId,
                type
            }
        },
        status: {
            $in: [MissionStatus.QUEUED, MissionStatus.IN_PROGRESS, MissionStatus.PAUSED]
        }
    }).sort('startTime').exec();
};

MissionSchema.statics.updateProgress = async function(
    missionId: string,
    progress: number,
    metadata?: Record<string, unknown>
): Promise<IMission> {
    const mission = await this.findById(missionId);
    if (!mission) {
        throw new Error('Mission not found');
    }

    if (mission.status !== MissionStatus.IN_PROGRESS) {
        throw new Error('Can only update progress for missions in progress');
    }

    mission.progress = progress;
    if (metadata) {
        mission.metadata = { ...mission.metadata, ...metadata };
    }

    if (progress >= 100) {
        mission.status = MissionStatus.COMPLETED;
        mission.endTime = new Date();
    }

    return mission.save();
};

// Export the model
export const Mission = model<IMission>('Mission', MissionSchema);