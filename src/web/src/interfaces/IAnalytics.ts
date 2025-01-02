// @turf/helpers v6.5.0 - GeoJSON Point type for location-based analytics
import { Point } from '@turf/helpers';

/**
 * Core interface for analytics data structure in the web frontend
 * Represents real-time video analytics, pest detection results and system metrics
 */
export interface IAnalytics {
    /** Unique identifier for the analytics record */
    id: string;
    
    /** Associated mission identifier */
    missionId: string;
    
    /** Device that generated the analytics */
    deviceId: string;
    
    /** Timestamp when analytics were generated */
    timestamp: Date;
    
    /** Geographic location where analytics were captured */
    location: Point;
    
    /** Type of analytics data */
    type: AnalyticsType;
    
    /** Confidence score of the analytics result (0-1) */
    confidence: number;
    
    /** Array of individual detections from video analytics */
    detections: IDetection[];
    
    /** Additional metadata key-value pairs */
    metadata: Record<string, any>;
}

/**
 * Interface for individual detection results from video analytics
 * Used for pest detection and crop analysis results
 */
export interface IDetection {
    /** Type of detected object/condition */
    type: string;
    
    /** Detection confidence score (0-1) */
    confidence: number;
    
    /** Bounding box coordinates in video frame */
    boundingBox: IBoundingBox;
}

/**
 * Interface for detection bounding box coordinates in video frames
 * Represents normalized coordinates (0-1) relative to frame dimensions
 */
export interface IBoundingBox {
    /** Normalized X coordinate of top-left corner */
    x: number;
    
    /** Normalized Y coordinate of top-left corner */
    y: number;
    
    /** Normalized width of bounding box */
    width: number;
    
    /** Normalized height of bounding box */
    height: number;
}

/**
 * Enum for different types of analytics supported by the system
 * Used to categorize and filter analytics data
 */
export enum AnalyticsType {
    /** Real-time pest detection analytics */
    PEST_DETECTION = 'PEST_DETECTION',
    
    /** Crop health and growth analysis */
    CROP_ANALYSIS = 'CROP_ANALYSIS',
    
    /** Treatment application coverage tracking */
    TREATMENT_COVERAGE = 'TREATMENT_COVERAGE'
}

/**
 * Interface for analytics query filters used in API requests
 * Enables filtering analytics data by various criteria
 */
export interface IAnalyticsFilters {
    /** Start date for analytics query range */
    startDate: Date;
    
    /** End date for analytics query range */
    endDate: Date;
    
    /** Filter by analytics type */
    type: AnalyticsType;
    
    /** Filter by specific device */
    deviceId: string;
}

/**
 * Minimum confidence threshold for analytics results
 * Based on 95% pest detection accuracy requirement
 */
export const MIN_CONFIDENCE_THRESHOLD = 0.95;