/// <reference types="vite/client" />

/**
 * Interface defining typed environment variables for the agricultural management system
 * @interface ImportMetaEnv
 */
interface ImportMetaEnv {
  /** Base URL for the agricultural management system API */
  readonly VITE_API_URL: string;

  /** WebSocket endpoint for real-time telemetry and updates */
  readonly VITE_WEBSOCKET_URL: string;

  /** MapBox API key for geospatial visualization */
  readonly VITE_MAP_API_KEY: string;

  /** Authentication domain for Keycloak integration */
  readonly VITE_AUTH_DOMAIN: string;

  /** Endpoint for system telemetry data collection */
  readonly VITE_TELEMETRY_ENDPOINT: string;
}

/**
 * Interface augmenting Vite's ImportMeta with agricultural system environment types
 * @interface ImportMeta
 */
interface ImportMeta {
  /** Environment variables for the agricultural management system */
  readonly env: ImportMetaEnv;
}