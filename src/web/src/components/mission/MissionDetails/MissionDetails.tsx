import React, { useCallback, useMemo, useState } from 'react';
import { format } from 'date-fns'; // date-fns@2.30.0
import { VirtualList } from 'react-window'; // react-window@1.8.9
import { debounce } from 'lodash'; // lodash@4.17.21

import { IMission } from '../../../interfaces/IMission';
import { useMission } from '../../../hooks/useMission';
import Map from '../../common/Map/Map';
import Loading from '../../common/Loading/Loading';

interface MissionDetailsProps {
  /**
   * Optional CSS class name for custom styling
   */
  className?: string;
}

/**
 * Displays detailed information about an agricultural mission including status,
 * coverage area, assigned devices, and mission parameters. Provides real-time
 * updates and interactive controls for mission management.
 *
 * @version 1.0.0
 */
const MissionDetails: React.FC<MissionDetailsProps> = ({ className = '' }) => {
  const {
    selectedMission,
    updateMission,
    deleteMission,
    isOnline,
    pendingActions
  } = useMission();

  const [isEditing, setIsEditing] = useState(false);
  const [isSaving, setIsSaving] = useState(false);
  const [showConfirmDelete, setShowConfirmDelete] = useState(false);

  // Format mission duration with timezone support
  const formatMissionDuration = useCallback((startTime: Date, endTime: Date | null): string => {
    if (!endTime) {
      const duration = Date.now() - startTime.getTime();
      const hours = Math.floor(duration / (1000 * 60 * 60));
      const minutes = Math.floor((duration % (1000 * 60 * 60)) / (1000 * 60));
      return `${hours}h ${minutes}m (Ongoing)`;
    }
    const duration = endTime.getTime() - startTime.getTime();
    const hours = Math.floor(duration / (1000 * 60 * 60));
    const minutes = Math.floor((duration % (1000 * 60 * 60)) / (1000 * 60));
    return `${hours}h ${minutes}m`;
  }, []);

  // Handle mission status changes with optimistic updates
  const handleStatusChange = useCallback(async (newStatus: MissionStatusCodes) => {
    if (!selectedMission) return;

    try {
      setIsSaving(true);
      await updateMission(selectedMission.id, { status: newStatus });
    } catch (error) {
      console.error('Failed to update mission status:', error);
    } finally {
      setIsSaving(false);
    }
  }, [selectedMission, updateMission]);

  // Debounced handler for mission parameter updates
  const handleParameterChange = useMemo(
    () => debounce(async (parameter: string, value: any) => {
      if (!selectedMission) return;

      try {
        setIsSaving(true);
        await updateMission(selectedMission.id, {
          parameters: {
            ...selectedMission.parameters,
            [parameter]: value
          }
        });
      } catch (error) {
        console.error('Failed to update mission parameter:', error);
      } finally {
        setIsSaving(false);
      }
    }, 500),
    [selectedMission, updateMission]
  );

  // Handle mission deletion with confirmation
  const handleDelete = useCallback(async () => {
    if (!selectedMission) return;

    try {
      setIsSaving(true);
      await deleteMission(selectedMission.id);
      setShowConfirmDelete(false);
    } catch (error) {
      console.error('Failed to delete mission:', error);
    } finally {
      setIsSaving(false);
    }
  }, [selectedMission, deleteMission]);

  if (!selectedMission) {
    return (
      <div className={`mission-details-empty ${className}`} style={styles.emptyState}>
        <p>No mission selected</p>
      </div>
    );
  }

  return (
    <div className={`mission-details ${className}`} style={styles.container}>
      {/* Header Section */}
      <header style={styles.header}>
        <div style={styles.headerContent}>
          <h2 style={styles.title}>{selectedMission.name}</h2>
          <div style={styles.statusBadge} data-status={selectedMission.status}>
            {selectedMission.status}
          </div>
        </div>
        <div style={styles.actions}>
          {!isOnline && <span style={styles.offlineIndicator}>Offline Mode</span>}
          <button
            onClick={() => setIsEditing(!isEditing)}
            disabled={isSaving}
            style={styles.button}
          >
            {isEditing ? 'View' : 'Edit'}
          </button>
          <button
            onClick={() => setShowConfirmDelete(true)}
            disabled={isSaving}
            style={{ ...styles.button, ...styles.deleteButton }}
          >
            Delete
          </button>
        </div>
      </header>

      {/* Main Content */}
      <div style={styles.content}>
        {/* Map View */}
        <div style={styles.mapContainer}>
          <Map
            devices={[]}
            missions={[selectedMission]}
            center={{
              lat: selectedMission.coverageArea.coordinates[0][0][1],
              lng: selectedMission.coverageArea.coordinates[0][0][0]
            }}
            zoom={14}
            isEditable={isEditing}
          />
        </div>

        {/* Mission Details */}
        <div style={styles.details}>
          <section style={styles.section}>
            <h3 style={styles.sectionTitle}>Mission Information</h3>
            <dl style={styles.definitionList}>
              <dt>Type</dt>
              <dd>{selectedMission.type}</dd>
              
              <dt>Duration</dt>
              <dd>{formatMissionDuration(selectedMission.startTime, selectedMission.endTime)}</dd>
              
              <dt>Progress</dt>
              <dd>
                <div style={styles.progressBar}>
                  <div
                    style={{
                      ...styles.progressFill,
                      width: `${selectedMission.progress}%`
                    }}
                  />
                  <span>{selectedMission.progress}%</span>
                </div>
              </dd>
            </dl>
          </section>

          {/* Assigned Devices */}
          <section style={styles.section}>
            <h3 style={styles.sectionTitle}>Assigned Devices</h3>
            <VirtualList
              height={200}
              width="100%"
              itemCount={selectedMission.assignedDevices.length}
              itemSize={48}
            >
              {({ index, style }) => (
                <div style={{ ...styles.deviceItem, ...style }}>
                  {selectedMission.assignedDevices[index]}
                </div>
              )}
            </VirtualList>
          </section>

          {/* Mission Parameters */}
          <section style={styles.section}>
            <h3 style={styles.sectionTitle}>Parameters</h3>
            <dl style={styles.definitionList}>
              {Object.entries(selectedMission.parameters).map(([key, value]) => (
                <React.Fragment key={key}>
                  <dt>{key}</dt>
                  <dd>
                    {isEditing ? (
                      <input
                        type="number"
                        value={value}
                        onChange={(e) => handleParameterChange(key, parseFloat(e.target.value))}
                        style={styles.input}
                      />
                    ) : (
                      value
                    )}
                  </dd>
                </React.Fragment>
              ))}
            </dl>
          </section>
        </div>
      </div>

      {/* Loading Overlay */}
      {isSaving && (
        <div style={styles.loadingOverlay}>
          <Loading size="lg" label="Saving changes..." />
        </div>
      )}

      {/* Delete Confirmation Dialog */}
      {showConfirmDelete && (
        <div style={styles.modal}>
          <div style={styles.modalContent}>
            <h3>Confirm Delete</h3>
            <p>Are you sure you want to delete this mission?</p>
            <div style={styles.modalActions}>
              <button
                onClick={() => setShowConfirmDelete(false)}
                style={styles.button}
              >
                Cancel
              </button>
              <button
                onClick={handleDelete}
                style={{ ...styles.button, ...styles.deleteButton }}
              >
                Delete
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

// Component styles
const styles: { [key: string]: React.CSSProperties } = {
  container: {
    display: 'flex',
    flexDirection: 'column',
    height: '100%',
    backgroundColor: 'var(--theme-background-paper, #ffffff)',
    borderRadius: 'var(--border-radius-lg)',
    boxShadow: 'var(--shadow-md)',
  },
  header: {
    padding: 'var(--spacing-md)',
    borderBottom: '1px solid var(--theme-divider)',
  },
  headerContent: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
  },
  title: {
    margin: 0,
    fontSize: 'var(--font-size-xl)',
    fontWeight: 'var(--font-weight-bold)',
  },
  statusBadge: {
    padding: 'var(--spacing-2xs) var(--spacing-xs)',
    borderRadius: 'var(--border-radius-full)',
    fontSize: 'var(--font-size-sm)',
    fontWeight: 'var(--font-weight-medium)',
  },
  actions: {
    display: 'flex',
    gap: 'var(--spacing-xs)',
    marginTop: 'var(--spacing-sm)',
  },
  content: {
    display: 'flex',
    flex: 1,
    gap: 'var(--spacing-md)',
    padding: 'var(--spacing-md)',
  },
  mapContainer: {
    flex: '1 1 60%',
    minHeight: '400px',
    borderRadius: 'var(--border-radius-md)',
    overflow: 'hidden',
  },
  details: {
    flex: '1 1 40%',
    display: 'flex',
    flexDirection: 'column',
    gap: 'var(--spacing-md)',
  },
  section: {
    padding: 'var(--spacing-md)',
    backgroundColor: 'var(--theme-background-default)',
    borderRadius: 'var(--border-radius-md)',
  },
  sectionTitle: {
    margin: '0 0 var(--spacing-sm)',
    fontSize: 'var(--font-size-lg)',
    fontWeight: 'var(--font-weight-medium)',
  },
  definitionList: {
    display: 'grid',
    gridTemplateColumns: 'auto 1fr',
    gap: 'var(--spacing-xs) var(--spacing-md)',
    margin: 0,
  },
  progressBar: {
    position: 'relative',
    height: '8px',
    backgroundColor: 'var(--theme-background-paper)',
    borderRadius: 'var(--border-radius-full)',
  },
  progressFill: {
    position: 'absolute',
    height: '100%',
    backgroundColor: 'var(--theme-primary-main)',
    borderRadius: 'var(--border-radius-full)',
    transition: 'width var(--transition-duration-normal) var(--transition-timing-ease-in-out)',
  },
  deviceItem: {
    padding: 'var(--spacing-xs)',
    borderBottom: '1px solid var(--theme-divider)',
  },
  button: {
    padding: 'var(--spacing-xs) var(--spacing-md)',
    borderRadius: 'var(--border-radius-sm)',
    border: 'none',
    backgroundColor: 'var(--theme-primary-main)',
    color: 'var(--theme-primary-contrast)',
    cursor: 'pointer',
    transition: 'background-color var(--transition-duration-fast) var(--transition-timing-ease)',
  },
  deleteButton: {
    backgroundColor: 'var(--theme-error-main)',
  },
  input: {
    width: '100%',
    padding: 'var(--spacing-xs)',
    borderRadius: 'var(--border-radius-sm)',
    border: '1px solid var(--theme-divider)',
  },
  loadingOverlay: {
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: 'rgba(255, 255, 255, 0.8)',
    zIndex: 'var(--z-index-modal)',
  },
  modal: {
    position: 'fixed',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: 'rgba(0, 0, 0, 0.5)',
    zIndex: 'var(--z-index-modal)',
  },
  modalContent: {
    padding: 'var(--spacing-lg)',
    backgroundColor: 'var(--theme-background-paper)',
    borderRadius: 'var(--border-radius-lg)',
    maxWidth: '400px',
    width: '100%',
  },
  modalActions: {
    display: 'flex',
    justifyContent: 'flex-end',
    gap: 'var(--spacing-sm)',
    marginTop: 'var(--spacing-lg)',
  },
  emptyState: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    height: '100%',
    fontSize: 'var(--font-size-lg)',
    color: 'var(--theme-text-secondary)',
  },
  offlineIndicator: {
    padding: 'var(--spacing-2xs) var(--spacing-xs)',
    backgroundColor: 'var(--theme-warning-light)',
    color: 'var(--theme-warning-main)',
    borderRadius: 'var(--border-radius-sm)',
    fontSize: 'var(--font-size-sm)',
  },
};

export default MissionDetails;