import React, { useEffect, useRef, useState, useCallback } from 'react';
import mapboxgl from 'mapbox-gl'; // @mapbox/mapbox-gl-js@2.15.0
import * as turf from '@turf/turf'; // @turf/turf@6.5.0
import Supercluster from 'supercluster'; // supercluster@8.0.0

import { IDevice, DeviceType, DeviceStatus } from '../../../interfaces/IDevice';
import { IMission, MissionStatusCodes } from '../../../interfaces/IMission';
import Loading from '../Loading/Loading';

// Mapbox access token should be injected via environment variables
mapboxgl.accessToken = process.env.REACT_APP_MAPBOX_TOKEN || '';

interface MapProps {
  devices: IDevice[];
  missions: IMission[];
  center: { lat: number; lng: number };
  zoom: number;
  onDeviceClick?: (device: IDevice) => void;
  onAreaSelect?: (area: GeoJSON.Polygon) => void;
  isEditable?: boolean;
  className?: string;
  enableClustering?: boolean;
  offlineSupport?: boolean;
  accessibilityMode?: boolean;
}

/**
 * Interactive map component for agricultural monitoring and mission planning
 * Supports real-time device tracking, mission visualization, and area selection
 * 
 * @version 1.0.0
 */
const Map: React.FC<MapProps> = ({
  devices,
  missions,
  center,
  zoom = 12,
  onDeviceClick,
  onAreaSelect,
  isEditable = false,
  className = '',
  enableClustering = true,
  offlineSupport = false,
  accessibilityMode = false,
}) => {
  const mapContainer = useRef<HTMLDivElement>(null);
  const map = useRef<mapboxgl.Map | null>(null);
  const markers = useRef<{ [key: string]: mapboxgl.Marker }>({});
  const draw = useRef<mapboxgl.DrawControl | null>(null);
  const supercluster = useRef<Supercluster>();
  const [loading, setLoading] = useState(true);

  // Initialize map instance
  useEffect(() => {
    if (!mapContainer.current) return;

    const initMap = async () => {
      try {
        // Configure offline support if enabled
        if (offlineSupport) {
          await mapboxgl.setOfflineStorage({
            maxSize: 50 * 1024 * 1024, // 50MB cache
          });
        }

        map.current = new mapboxgl.Map({
          container: mapContainer.current,
          style: 'mapbox://styles/mapbox/satellite-v9',
          center: [center.lng, center.lat],
          zoom,
          minZoom: 5,
          maxZoom: 20,
          attributionControl: true,
          preserveDrawingBuffer: true, // Enable screenshot capability
          trackResize: true,
          keyboard: accessibilityMode,
        });

        // Add navigation controls
        map.current.addControl(new mapboxgl.NavigationControl(), 'top-right');
        map.current.addControl(new mapboxgl.ScaleControl(), 'bottom-right');

        // Initialize drawing tools if editable
        if (isEditable) {
          draw.current = new mapboxgl.DrawControl({
            displayControlsDefault: false,
            controls: {
              polygon: true,
              trash: true,
            },
          });
          map.current.addControl(draw.current);
        }

        // Initialize clustering
        if (enableClustering) {
          supercluster.current = new Supercluster({
            radius: 40,
            maxZoom: 16,
          });
        }

        map.current.on('load', () => setLoading(false));
      } catch (error) {
        console.error('Map initialization failed:', error);
        setLoading(false);
      }
    };

    initMap();

    return () => {
      if (map.current) {
        map.current.remove();
        map.current = null;
      }
    };
  }, [center, zoom, isEditable, enableClustering, offlineSupport, accessibilityMode]);

  // Update device markers with debouncing
  const updateDeviceMarkers = useCallback(() => {
    if (!map.current) return;

    // Clear existing markers
    Object.values(markers.current).forEach(marker => marker.remove());
    markers.current = {};

    const points = devices.map(device => ({
      type: 'Feature',
      geometry: {
        type: 'Point',
        coordinates: [device.location.longitude, device.location.latitude],
      },
      properties: { deviceId: device.id },
    }));

    if (enableClustering && supercluster.current) {
      supercluster.current.load(points);
      const bounds = map.current.getBounds();
      const zoom = map.current.getZoom();
      const clusters = supercluster.current.getClusters(
        [bounds.getWest(), bounds.getSouth(), bounds.getEast(), bounds.getNorth()],
        Math.floor(zoom)
      );

      clusters.forEach(cluster => {
        const [longitude, latitude] = cluster.geometry.coordinates;
        const el = document.createElement('div');
        
        if (cluster.properties.cluster) {
          el.className = 'device-cluster';
          el.innerHTML = `<span>${cluster.properties.point_count}</span>`;
        } else {
          const device = devices.find(d => d.id === cluster.properties.deviceId);
          if (!device) return;

          el.className = `device-marker ${device.type.toLowerCase()}`;
          el.setAttribute('data-status', device.status);
          el.setAttribute('data-battery', device.batteryLevel.toString());
        }

        const marker = new mapboxgl.Marker(el)
          .setLngLat([longitude, latitude])
          .addTo(map.current!);

        if (!cluster.properties.cluster && onDeviceClick) {
          marker.getElement().addEventListener('click', () => {
            const device = devices.find(d => d.id === cluster.properties.deviceId);
            if (device) onDeviceClick(device);
          });
        }

        markers.current[cluster.properties.deviceId || `cluster-${cluster.id}`] = marker;
      });
    } else {
      devices.forEach(device => {
        const el = document.createElement('div');
        el.className = `device-marker ${device.type.toLowerCase()}`;
        el.setAttribute('data-status', device.status);
        el.setAttribute('data-battery', device.batteryLevel.toString());

        const marker = new mapboxgl.Marker(el)
          .setLngLat([device.location.longitude, device.location.latitude])
          .addTo(map.current!);

        if (onDeviceClick) {
          marker.getElement().addEventListener('click', () => onDeviceClick(device));
        }

        markers.current[device.id] = marker;
      });
    }
  }, [devices, enableClustering, onDeviceClick]);

  // Update mission coverage areas
  const updateMissionAreas = useCallback(() => {
    if (!map.current) return;

    missions.forEach(mission => {
      const sourceId = `mission-${mission.id}`;
      const layerId = `mission-layer-${mission.id}`;

      if (!map.current!.getSource(sourceId)) {
        map.current!.addSource(sourceId, {
          type: 'geojson',
          data: {
            type: 'Feature',
            geometry: mission.coverageArea,
            properties: {
              missionId: mission.id,
              status: mission.status,
              progress: mission.progress,
            },
          },
        });

        map.current!.addLayer({
          id: layerId,
          type: 'fill',
          source: sourceId,
          paint: {
            'fill-color': getMissionColor(mission.status),
            'fill-opacity': 0.3,
            'fill-outline-color': '#000000',
          },
        });

        if (isEditable) {
          map.current!.on('click', layerId, (e) => {
            if (onAreaSelect && e.features?.[0]) {
              onAreaSelect(e.features[0].geometry as GeoJSON.Polygon);
            }
          });
        }
      }
    });
  }, [missions, isEditable, onAreaSelect]);

  // Update markers and areas when data changes
  useEffect(() => {
    if (!loading && map.current) {
      updateDeviceMarkers();
      updateMissionAreas();
    }
  }, [loading, devices, missions, updateDeviceMarkers, updateMissionAreas]);

  return (
    <div 
      ref={mapContainer}
      className={`map-container ${className}`}
      style={styles['map-container']}
      role="application"
      aria-label="Agricultural monitoring map"
    >
      {loading && (
        <div style={styles['loading-overlay']}>
          <Loading size="lg" label="Loading map..." />
        </div>
      )}
    </div>
  );
};

// Utility function to determine mission color based on status
const getMissionColor = (status: MissionStatusCodes): string => {
  const colors = {
    [MissionStatusCodes.IN_PROGRESS]: '#2196F3',
    [MissionStatusCodes.COMPLETED]: '#4CAF50',
    [MissionStatusCodes.FAILED]: '#F44336',
    [MissionStatusCodes.PAUSED]: '#FFC107',
    [MissionStatusCodes.QUEUED]: '#9E9E9E',
  };
  return colors[status] || '#000000';
};

// Component styles
const styles: { [key: string]: React.CSSProperties } = {
  'map-container': {
    width: '100%',
    height: '100%',
    minHeight: '400px',
    position: 'relative',
    outline: 'none',
  },
  'loading-overlay': {
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    background: 'rgba(255, 255, 255, 0.8)',
    zIndex: 1000,
  },
};

export default Map;