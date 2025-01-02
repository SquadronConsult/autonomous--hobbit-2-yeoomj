import React, { memo, useCallback, useEffect, useMemo, useRef, useState } from 'react';
import Map from '../../common/Map/Map';
import { IMission } from '../../../interfaces/IMission';
import { IDevice } from '../../../interfaces/IDevice';
import { useWebSocket } from '../../../hooks/useWebSocket';
import * as turf from '@turf/area'; // @turf/area@6.5.0
import Supercluster from 'supercluster'; // supercluster@7.1.5

interface CoverageMapProps {
  mission: IMission | null;
  devices: IDevice[];
  isEditable: boolean;
  onAreaChange: (area: GeoJSON.Polygon) => void;
  className?: string;
  maxDevices?: number;
  updateInterval?: number;
  clusterThreshold?: number;
}

/**
 * High-performance WebGL-accelerated map component for agricultural mission coverage
 * Supports real-time device tracking, interactive area selection, and mission visualization
 * 
 * @version 1.0.0
 */
const CoverageMap: React.FC<CoverageMapProps> = memo(({
  mission,
  devices,
  isEditable = false,
  onAreaChange,
  className = '',
  maxDevices = 24, // Based on fleet coordination requirement
  updateInterval = 100, // <100ms latency requirement
  clusterThreshold = 8 // Clustering threshold for device markers
}) => {
  // State and refs
  const [mapCenter, setMapCenter] = useState({ lat: 0, lng: 0 });
  const [mapZoom, setMapZoom] = useState(12);
  const deviceClusterRef = useRef<Supercluster>();
  const lastUpdateRef = useRef<number>(Date.now());

  // WebSocket connection for real-time updates
  const { subscribe, connectionStatus } = useWebSocket({
    autoConnect: true,
    enableReconnect: true,
    reconnectInterval: 1000,
    maxReconnectAttempts: 5
  });

  // Initialize device clustering
  useEffect(() => {
    deviceClusterRef.current = new Supercluster({
      radius: 40,
      maxZoom: 16,
      minPoints: clusterThreshold
    });
  }, [clusterThreshold]);

  // Calculate optimal map center based on coverage area
  useEffect(() => {
    if (mission?.coverageArea) {
      const center = turf.center(mission.coverageArea);
      setMapCenter({
        lat: center.geometry.coordinates[1],
        lng: center.geometry.coordinates[0]
      });
    }
  }, [mission?.coverageArea]);

  // Handle coverage area selection with validation
  const handleAreaSelect = useCallback((area: GeoJSON.Polygon) => {
    // Validate area size (support for 1000+ hectare operations)
    const areaSize = turf.area(area);
    const hectares = areaSize / 10000;

    if (hectares > 1000) {
      // Performance optimization for large areas using Web Workers
      const worker = new Worker('/workers/area-calculator.js');
      worker.postMessage({ area });
      worker.onmessage = (e) => {
        onAreaChange(e.data.validatedArea);
      };
    } else {
      onAreaChange(area);
    }
  }, [onAreaChange]);

  // Optimize device position updates
  const updateDevicePositions = useCallback((updatedDevices: IDevice[]) => {
    const now = Date.now();
    if (now - lastUpdateRef.current < updateInterval) return;

    // Enforce device limit
    const activeDevices = updatedDevices.slice(0, maxDevices);

    // Prepare GeoJSON features for clustering
    const points = activeDevices.map(device => ({
      type: 'Feature',
      geometry: {
        type: 'Point',
        coordinates: [device.location.longitude, device.location.latitude]
      },
      properties: {
        deviceId: device.id,
        status: device.status,
        batteryLevel: device.batteryLevel
      }
    }));

    // Update cluster data
    if (deviceClusterRef.current) {
      deviceClusterRef.current.load(points);
    }

    lastUpdateRef.current = now;
  }, [maxDevices, updateInterval]);

  // Subscribe to real-time device updates
  useEffect(() => {
    const unsubscribe = subscribe('device-position', (data: IDevice[]) => {
      updateDevicePositions(data);
    });

    return () => unsubscribe();
  }, [subscribe, updateDevicePositions]);

  // Memoize map props for performance
  const mapProps = useMemo(() => ({
    center: mapCenter,
    zoom: mapZoom,
    devices,
    missions: mission ? [mission] : [],
    isEditable,
    onAreaSelect: handleAreaSelect,
    enableClustering: true,
    className
  }), [mapCenter, mapZoom, devices, mission, isEditable, handleAreaSelect, className]);

  return (
    <div className="coverage-map" style={styles['coverage-map']}>
      <Map {...mapProps} />
      
      {/* Area information overlay */}
      {mission && (
        <div className="area-info" style={styles['area-info']}>
          <div>Coverage: {(turf.area(mission.coverageArea) / 10000).toFixed(2)} ha</div>
          <div>Devices: {devices.length}/{maxDevices}</div>
          <div>Update Latency: {connectionStatus.latency}ms</div>
        </div>
      )}
    </div>
  );
});

// Component styles
const styles: { [key: string]: React.CSSProperties } = {
  'coverage-map': {
    width: '100%',
    height: '100%',
    minHeight: '400px',
    position: 'relative',
    overflow: 'hidden'
  },
  'area-info': {
    position: 'absolute',
    bottom: '20px',
    left: '20px',
    background: 'var(--theme-background-paper)',
    padding: '10px',
    borderRadius: '4px',
    boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
    zIndex: 1000,
    backdropFilter: 'blur(8px)'
  },
  'device-cluster': {
    background: 'var(--theme-primary)',
    borderRadius: '50%',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    color: 'white',
    fontWeight: 'bold'
  }
};

// Display name for debugging
CoverageMap.displayName = 'CoverageMap';

export default CoverageMap;