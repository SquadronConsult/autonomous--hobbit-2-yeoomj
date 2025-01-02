/**
 * @fileoverview Mission type definitions for the agricultural management system
 * @version 1.0.0
 * 
 * Defines the available mission types for autonomous agricultural operations
 * and their corresponding human-readable labels for UI display.
 * 
 * @see Technical Specification Section 1.2 - Core Features
 * @see Technical Specification Section 2.2.1 - ROS 2 Framework
 */

/**
 * Enumeration of supported mission types for agricultural operations.
 * Used for type-safe mission categorization across the application.
 */
export enum MissionTypes {
    /** Aerial surveillance and mapping operations */
    SURVEY = 'SURVEY',
    
    /** Pest control and treatment application missions */
    TREATMENT = 'TREATMENT',
    
    /** Continuous crop health monitoring operations */
    MONITORING = 'MONITORING'
}

/**
 * Type utility for accessing mission type keys in a type-safe manner.
 * Useful for iterating over mission types or type checking.
 */
export type MissionTypeKeys = keyof typeof MissionTypes;

/**
 * Human-readable labels for mission types used in UI components.
 * Maps each mission type to its display-friendly description.
 * 
 * @readonly Prevents accidental modification of labels
 */
export const MissionTypeLabels: Readonly<Record<MissionTypes, string>> = {
    [MissionTypes.SURVEY]: 'Area Survey',
    [MissionTypes.TREATMENT]: 'Pest Treatment',
    [MissionTypes.MONITORING]: 'Crop Monitoring'
} as const;