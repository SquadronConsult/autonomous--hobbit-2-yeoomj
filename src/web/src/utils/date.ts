import { format, parseISO, addDays, differenceInMinutes, isValid } from 'date-fns'; // ^2.30.0
import { zonedTimeToUtc } from 'date-fns-tz'; // ^2.0.0

// Constants for date/time formatting
const DATE_FORMAT = 'yyyy-MM-dd';
const TIME_FORMAT = 'HH:mm:ss';
const DATETIME_FORMAT = 'yyyy-MM-dd HH:mm:ss';
const RELATIVE_TIME_THRESHOLD_MINUTES = 60;
const DEFAULT_TIMEZONE = 'UTC';
const DATE_LOCALE = 'en-US';

/**
 * Memoization decorator for performance optimization
 */
function memoize(target: any, propertyKey: string, descriptor: PropertyDescriptor) {
  const originalMethod = descriptor.value;
  const cache = new Map();

  descriptor.value = function(...args: any[]) {
    const key = JSON.stringify(args);
    if (cache.has(key)) {
      return cache.get(key);
    }
    const result = originalMethod.apply(this, args);
    cache.set(key, result);
    return result;
  };
  return descriptor;
}

/**
 * Formats a date to YYYY-MM-DD format with timezone handling
 * @param date Date object or ISO string
 * @param timezone Optional timezone (defaults to UTC)
 * @returns Formatted date string
 * @throws Error if date is invalid
 */
@memoize
export function formatDate(date: Date | string, timezone: string = DEFAULT_TIMEZONE): string {
  try {
    const dateObj = typeof date === 'string' ? parseISO(date) : date;
    if (!isValid(dateObj)) {
      throw new Error('Invalid date input');
    }
    const zonedDate = zonedTimeToUtc(dateObj, timezone);
    return format(zonedDate, DATE_FORMAT, { locale: DATE_LOCALE });
  } catch (error) {
    throw new Error(`Date formatting error: ${error.message}`);
  }
}

/**
 * Formats time to 24-hour format with timezone support
 * @param date Date object or ISO string
 * @param timezone Optional timezone (defaults to UTC)
 * @returns Formatted time string
 * @throws Error if time is invalid
 */
@memoize
export function formatTime(date: Date | string, timezone: string = DEFAULT_TIMEZONE): string {
  try {
    const dateObj = typeof date === 'string' ? parseISO(date) : date;
    if (!isValid(dateObj)) {
      throw new Error('Invalid time input');
    }
    const zonedDate = zonedTimeToUtc(dateObj, timezone);
    return format(zonedDate, TIME_FORMAT, { locale: DATE_LOCALE });
  } catch (error) {
    throw new Error(`Time formatting error: ${error.message}`);
  }
}

/**
 * Formats date and time with timezone awareness
 * @param date Date object or ISO string
 * @param timezone Optional timezone (defaults to UTC)
 * @returns Formatted date-time string
 * @throws Error if date-time is invalid
 */
@memoize
export function formatDateTime(date: Date | string, timezone: string = DEFAULT_TIMEZONE): string {
  try {
    const dateObj = typeof date === 'string' ? parseISO(date) : date;
    if (!isValid(dateObj)) {
      throw new Error('Invalid datetime input');
    }
    const zonedDate = zonedTimeToUtc(dateObj, timezone);
    return format(zonedDate, DATETIME_FORMAT, { locale: DATE_LOCALE });
  } catch (error) {
    throw new Error(`DateTime formatting error: ${error.message}`);
  }
}

/**
 * Returns human-readable relative time string
 * @param date Date object or ISO string to compare with current time
 * @returns Localized relative time string
 * @throws Error if date is invalid
 */
@memoize
export function getRelativeTime(date: Date | string): string {
  try {
    const dateObj = typeof date === 'string' ? parseISO(date) : date;
    if (!isValid(dateObj)) {
      throw new Error('Invalid date input for relative time');
    }

    const diffMinutes = differenceInMinutes(new Date(), dateObj);
    const absMinutes = Math.abs(diffMinutes);

    if (absMinutes < RELATIVE_TIME_THRESHOLD_MINUTES) {
      return `${absMinutes} minute${absMinutes !== 1 ? 's' : ''} ${diffMinutes >= 0 ? 'ago' : 'from now'}`;
    }
    
    const hours = Math.floor(absMinutes / 60);
    return `${hours} hour${hours !== 1 ? 's' : ''} ${diffMinutes >= 0 ? 'ago' : 'from now'}`;
  } catch (error) {
    throw new Error(`Relative time calculation error: ${error.message}`);
  }
}

/**
 * Calculates mission duration with timezone awareness
 * @param startTime Mission start time
 * @param endTime Mission end time
 * @param timezone Optional timezone (defaults to UTC)
 * @returns Formatted duration string
 * @throws Error if date range is invalid
 */
@memoize
export function getMissionDuration(
  startTime: Date | string,
  endTime: Date | string,
  timezone: string = DEFAULT_TIMEZONE
): string {
  try {
    const startDate = typeof startTime === 'string' ? parseISO(startTime) : startTime;
    const endDate = typeof endTime === 'string' ? parseISO(endTime) : endTime;

    if (!isValid(startDate) || !isValid(endDate)) {
      throw new Error('Invalid date range');
    }

    const zonedStartDate = zonedTimeToUtc(startDate, timezone);
    const zonedEndDate = zonedTimeToUtc(endDate, timezone);

    const durationMinutes = differenceInMinutes(zonedEndDate, zonedStartDate);
    if (durationMinutes < 0) {
      throw new Error('End time cannot be before start time');
    }

    const hours = Math.floor(durationMinutes / 60);
    const minutes = durationMinutes % 60;

    return `${hours}h ${minutes}m`;
  } catch (error) {
    throw new Error(`Mission duration calculation error: ${error.message}`);
  }
}

/**
 * Generates date range for analytics with timezone support
 * @param days Number of days to include in range
 * @param timezone Optional timezone (defaults to UTC)
 * @returns Object containing start and end dates
 * @throws Error if days parameter is invalid
 */
@memoize
export function getDateRange(days: number, timezone: string = DEFAULT_TIMEZONE): { startDate: Date; endDate: Date } {
  try {
    if (!Number.isInteger(days) || days <= 0) {
      throw new Error('Days must be a positive integer');
    }

    const endDate = zonedTimeToUtc(new Date(), timezone);
    const startDate = addDays(endDate, -days + 1);

    return {
      startDate,
      endDate
    };
  } catch (error) {
    throw new Error(`Date range generation error: ${error.message}`);
  }
}