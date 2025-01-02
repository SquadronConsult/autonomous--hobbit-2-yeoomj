// @turf/helpers v6.5.0 - GeoJSON Point type for location-based analytics
import { Point } from '@turf/helpers';

/**
 * Minimum confidence threshold for analytics detection (95% as per requirements)
 */
export const MIN_CONFIDENCE_THRESHOLD = 0.95;

/**
 * Enum defining different types of agricultural analytics
 */
export enum AnalyticsType {
    PEST_DETECTION = 'PEST_DETECTION',
    CROP_ANALYSIS = 'CROP_ANALYSIS',
    TREATMENT_COVERAGE = 'TREATMENT_COVERAGE'
}

/**
 * Interface defining bounding box coordinates for object detection
 */
export interface IBoundingBox {
    /** X coordinate of top-left corner */
    x: number;
    /** Y coordinate of top-left corner */
    y: number;
    /** Width of bounding box */
    width: number;
    /** Height of bounding box */
    height: number;
}

/**
 * Interface for individual detection results from video analytics
 */
export interface IDetection {
    /** Type of detected object (e.g., specific pest species) */
    type: string;
    /** Confidence score of the detection (0-1) */
    confidence: number;
    /** Bounding box coordinates of the detection */
    boundingBox: IBoundingBox;
}

/**
 * Core interface for agricultural analytics data
 * Implements real-time video analytics requirements for crop monitoring and pest detection
 */
export interface IAnalytics {
    /** Unique identifier for the analytics record */
    id: string;
    
    /** Reference to the associated mission */
    missionId: string;
    
    /** Identifier of the device that generated the analytics */
    deviceId: string;
    
    /** Timestamp when the analytics were generated */
    timestamp: Date;
    
    /** GeoJSON Point representing the location where analytics were captured */
    location: Point;
    
    /** Type of analytics being performed */
    type: AnalyticsType;
    
    /** Overall confidence score for the analytics (0-1) */
    confidence: number;
    
    /** Array of individual detections from video analytics */
    detections: IDetection[];
    
    /** Additional metadata for extensibility */
    metadata: Record<string, any>;
}