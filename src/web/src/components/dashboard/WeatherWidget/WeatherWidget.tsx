import React, { useCallback, useEffect, useMemo, useState } from 'react';
import styled from '@emotion/styled';
import { Card } from '../../common/Card/Card';
import { ApiService } from '../../../services/api';

// Version comments for third-party dependencies
// @emotion/styled@11.11.0
// react@18.0.0

/**
 * Interface for weather data structure with comprehensive type safety
 */
interface WeatherData {
  temperature: number;
  humidity: number;
  windSpeed: number;
  windDirection: number;
  precipitation: number;
  lastUpdated: Date;
}

/**
 * Props interface for WeatherWidget component with validation
 */
interface WeatherWidgetProps {
  location: {
    lat: number;
    lon: number;
  };
  refreshInterval?: number;
}

/**
 * Styled container for weather information with responsive layout
 */
const WeatherContainer = styled.div`
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
  gap: 16px;
  padding: 16px;
  margin: 8px;
  border-radius: 8px;
  background-color: ${props => props.theme.background.paper};
  transition: all 0.3s ease;

  @media (max-width: 768px) {
    grid-template-columns: 1fr;
    gap: 12px;
    padding: 12px;
  }
`;

/**
 * Styled component for individual weather metrics with animations
 */
const WeatherMetric = styled.div`
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 14px;
  padding: 8px;
  border-radius: 4px;
  background-color: ${props => props.theme.background.elevated};
  transition: transform 0.2s ease;

  &:hover {
    transform: scale(1.02);
  }

  @media (prefers-reduced-motion: reduce) {
    transition: none;
    &:hover {
      transform: none;
    }
  }
`;

/**
 * Styled loading skeleton for weather metrics
 */
const LoadingSkeleton = styled.div`
  height: 36px;
  background: linear-gradient(
    90deg,
    ${props => props.theme.background.elevated} 25%,
    ${props => props.theme.background.paper} 50%,
    ${props => props.theme.background.elevated} 75%
  );
  background-size: 200% 100%;
  animation: loading 1.5s infinite;
  border-radius: 4px;

  @keyframes loading {
    0% { background-position: 200% 0; }
    100% { background-position: -200% 0; }
  }

  @media (prefers-reduced-motion: reduce) {
    animation: none;
    background: ${props => props.theme.background.elevated};
  }
`;

/**
 * Error message container with accessible styling
 */
const ErrorMessage = styled.div`
  color: ${props => props.theme.status.error};
  padding: 8px;
  border-radius: 4px;
  border: 1px solid currentColor;
  margin: 8px 0;
  font-size: 14px;
`;

/**
 * Custom hook for fetching and managing weather data with caching
 */
const useWeatherData = (
  location: { lat: number; lon: number },
  refreshInterval: number
) => {
  const [data, setData] = useState<WeatherData | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);

  const fetchWeatherData = useCallback(async () => {
    try {
      const response = await ApiService.request<WeatherData>({
        url: '/api/v1/weather',
        method: 'GET',
        params: {
          lat: location.lat,
          lon: location.lon,
        },
      });

      setData(response);
      setError(null);
    } catch (err) {
      setError(err as Error);
    } finally {
      setLoading(false);
    }
  }, [location.lat, location.lon]);

  useEffect(() => {
    fetchWeatherData();
    const interval = setInterval(fetchWeatherData, refreshInterval);

    return () => clearInterval(interval);
  }, [fetchWeatherData, refreshInterval]);

  return { data, loading, error };
};

/**
 * Weather widget component for displaying real-time weather information
 * with comprehensive error handling and accessibility features
 */
const WeatherWidget: React.FC<WeatherWidgetProps> = ({
  location,
  refreshInterval = 300000, // 5 minutes default refresh
}) => {
  // Validate location coordinates
  if (
    location.lat < -90 || location.lat > 90 ||
    location.lon < -180 || location.lon > 180
  ) {
    throw new Error('Invalid location coordinates provided to WeatherWidget');
  }

  const { data, loading, error } = useWeatherData(location, refreshInterval);

  // Format weather data for display
  const formattedData = useMemo(() => {
    if (!data) return null;

    return {
      temperature: `${data.temperature.toFixed(1)}°C`,
      humidity: `${data.humidity}%`,
      windSpeed: `${data.windSpeed.toFixed(1)} m/s`,
      windDirection: `${data.windDirection}°`,
      precipitation: `${data.precipitation}%`,
      lastUpdated: new Date(data.lastUpdated).toLocaleTimeString(),
    };
  }, [data]);

  return (
    <Card
      variant="elevated"
      aria-label="Weather Information"
      aria-live="polite"
      role="region"
    >
      <WeatherContainer>
        {error && (
          <ErrorMessage role="alert" aria-live="assertive">
            Unable to fetch weather data. Please try again later.
          </ErrorMessage>
        )}

        {loading ? (
          <>
            <LoadingSkeleton aria-busy="true" aria-label="Loading weather data" />
            <LoadingSkeleton aria-busy="true" aria-label="Loading weather data" />
            <LoadingSkeleton aria-busy="true" aria-label="Loading weather data" />
          </>
        ) : formattedData && (
          <>
            <WeatherMetric aria-label="Temperature">
              <span role="img" aria-hidden="true">🌡️</span>
              {formattedData.temperature}
            </WeatherMetric>
            <WeatherMetric aria-label="Humidity">
              <span role="img" aria-hidden="true">💧</span>
              {formattedData.humidity}
            </WeatherMetric>
            <WeatherMetric aria-label="Wind Speed and Direction">
              <span role="img" aria-hidden="true">🌬️</span>
              {formattedData.windSpeed} @ {formattedData.windDirection}
            </WeatherMetric>
            <WeatherMetric aria-label="Precipitation Chance">
              <span role="img" aria-hidden="true">🌧️</span>
              {formattedData.precipitation}
            </WeatherMetric>
            <WeatherMetric aria-label="Last Updated">
              <span role="img" aria-hidden="true">🕒</span>
              {formattedData.lastUpdated}
            </WeatherMetric>
          </>
        )}
      </WeatherContainer>
    </Card>
  );
};

// Set display name for debugging
WeatherWidget.displayName = 'WeatherWidget';

export default WeatherWidget;
export type { WeatherWidgetProps, WeatherData };