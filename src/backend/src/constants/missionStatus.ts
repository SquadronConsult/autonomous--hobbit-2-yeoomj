/**
 * @fileoverview Mission status enumeration for agricultural management system
 * @version 1.0.0
 * 
 * Defines the possible states of autonomous agricultural missions throughout their lifecycle.
 * Used for tracking mission execution status across the distributed system including
 * drone fleets, ground robots, and monitoring systems.
 */

/**
 * Enumeration of all possible mission states in the agricultural management system.
 * Provides type-safe status tracking for real-time mission execution and monitoring.
 * 
 * @enum {string}
 * @readonly
 */
export enum MissionStatus {
    /**
     * Initial state when mission is first created in system but not yet scheduled
     * for execution. Entry point for all new missions.
     */
    CREATED = 'CREATED',

    /**
     * Mission is scheduled and awaiting resource allocation or execution slot.
     * Indicates mission is validated and ready for execution.
     */
    QUEUED = 'QUEUED',

    /**
     * Mission is currently being executed by drone/robot fleet.
     * Active state indicating ongoing mission operations.
     */
    IN_PROGRESS = 'IN_PROGRESS',

    /**
     * Mission execution temporarily halted but can resume from current state.
     * Allows for temporary suspension while maintaining mission context.
     */
    PAUSED = 'PAUSED',

    /**
     * Mission has successfully completed all objectives.
     * Terminal state indicating successful mission completion.
     */
    COMPLETED = 'COMPLETED',

    /**
     * Mission encountered unrecoverable error during execution.
     * Terminal state indicating mission failure.
     */
    FAILED = 'FAILED',

    /**
     * Mission manually terminated before completion.
     * Terminal state indicating intentional mission cancellation.
     */
    CANCELLED = 'CANCELLED'
}