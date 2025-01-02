import numeral from 'numeral'; // v2.0.6
import { ITelemetry } from '../interfaces/ITelemetry';
import { IAnalytics } from '../interfaces/IAnalytics';

// Default number of decimal places for general number formatting
const DEFAULT_DECIMALS = 2;

// Number of decimal places for geographic coordinate formatting
const COORDINATE_DECIMALS = 6;

/**
 * Formats a number with specified decimal places and optional units
 * @param value - Number to format
 * @param decimals - Number of decimal places (defaults to DEFAULT_DECIMALS)
 * @param unit - Optional unit to append to the formatted number
 * @returns Formatted number string with optional unit
 */
export const formatNumber = (
    value: number,
    decimals: number = DEFAULT_DECIMALS,
    unit?: string
): string => {
    if (!Number.isFinite(value)) {
        return 'N/A';
    }

    const formatString = `0,0.${'0'.repeat(decimals)}`;
    const formattedNumber = numeral(value).format(formatString);
    
    return unit ? `${formattedNumber} ${unit}` : formattedNumber;
};

/**
 * Formats a decimal number as a percentage with specified decimal places
 * @param value - Decimal value between 0 and 1 to format as percentage
 * @param decimals - Number of decimal places (defaults to DEFAULT_DECIMALS)
 * @returns Formatted percentage string
 */
export const formatPercentage = (
    value: number,
    decimals: number = DEFAULT_DECIMALS
): string => {
    if (!Number.isFinite(value) || value < 0 || value > 1) {
        return 'N/A';
    }

    const percentValue = value * 100;
    return `${formatNumber(percentValue, decimals)}%`;
};

/**
 * Formats telemetry values based on their type and unit
 * @param telemetry - Telemetry object containing value and unit
 * @returns Formatted telemetry string with appropriate unit
 */
export const formatTelemetryValue = (telemetry: ITelemetry): string => {
    if (!telemetry || telemetry.value === undefined) {
        return 'N/A';
    }

    const { value, unit } = telemetry;

    // Handle different value types
    if (typeof value === 'number') {
        return formatNumber(value, DEFAULT_DECIMALS, unit);
    } else if (typeof value === 'boolean') {
        return value ? 'Yes' : 'No';
    } else if (typeof value === 'object') {
        return JSON.stringify(value, null, 2);
    } else if (typeof value === 'string') {
        return unit ? `${value} ${unit}` : value;
    }

    return 'N/A';
};

/**
 * Formats detection confidence scores as percentages
 * @param confidence - Confidence value between 0 and 1
 * @returns Formatted confidence percentage string
 */
export const formatConfidence = (confidence: number): string => {
    if (!Number.isFinite(confidence) || confidence < 0 || confidence > 1) {
        return 'Invalid';
    }

    return formatPercentage(confidence, 1);
};

/**
 * Formats geographic coordinates with directional indicators
 * @param latitude - Latitude value between -90 and 90
 * @param longitude - Longitude value between -180 and 180
 * @returns Formatted coordinate string
 */
export const formatCoordinates = (
    latitude: number,
    longitude: number
): string => {
    if (
        !Number.isFinite(latitude) ||
        !Number.isFinite(longitude) ||
        latitude < -90 ||
        latitude > 90 ||
        longitude < -180 ||
        longitude > 180
    ) {
        return 'Invalid Coordinates';
    }

    const latDirection = latitude >= 0 ? 'N' : 'S';
    const lonDirection = longitude >= 0 ? 'E' : 'W';

    const formattedLat = formatNumber(Math.abs(latitude), COORDINATE_DECIMALS);
    const formattedLon = formatNumber(Math.abs(longitude), COORDINATE_DECIMALS);

    return `${formattedLat}°${latDirection} ${formattedLon}°${lonDirection}`;
};