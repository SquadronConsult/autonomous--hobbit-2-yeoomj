import React, { useState, useCallback, useEffect, useMemo } from 'react';
import styled from '@emotion/styled';
import classNames from 'classnames';
import { Table, TableProps, Column } from '../Table/Table';
import { Loading } from '../Loading/Loading';
import { Button } from '../Button/Button';

// @emotion/styled: ^11.11.0
// react: ^18.0.0
// classnames: ^2.3.2

export interface DataGridColumn extends Column {
  field: string;
  headerName: string;
  width?: number;
  minWidth?: number;
  maxWidth?: number;
  resizable?: boolean;
  filterable?: boolean;
  sortable?: boolean;
  align?: 'left' | 'center' | 'right';
  dataType?: 'text' | 'number' | 'date' | 'location' | 'status' | 'sensor';
  renderCell?: (value: any, row: any) => React.ReactNode;
  renderFilter?: (column: DataGridColumn) => React.ReactNode;
  offlineValidation?: (value: any) => boolean;
  formatForExport?: (value: any) => string;
  visibilityMode?: 'always' | 'desktop' | 'mobile';
  highContrast?: boolean;
}

export interface DataGridProps extends Omit<TableProps, 'columns'> {
  columns: DataGridColumn[];
  data: any[];
  loading?: boolean;
  pageSize?: number;
  page?: number;
  totalRows?: number;
  selectable?: boolean;
  offlineMode?: boolean;
  syncStatus?: 'synced' | 'pending' | 'error';
  lastSyncTime?: Date;
  environmentalMode?: 'indoor' | 'outdoor';
  onSelectionChange?: (selectedRows: any[]) => void;
  onFilterChange?: (filters: Record<string, any>) => void;
  onColumnResize?: (field: string, width: number) => void;
  onDataSync?: () => Promise<void>;
  onOfflineChange?: (isOffline: boolean) => void;
  className?: string;
  stickyHeader?: boolean;
  maxHeight?: string;
}

const GridContainer = styled.div<{ environmentalMode?: 'indoor' | 'outdoor' }>`
  width: 100%;
  background: var(--theme-background-paper);
  border-radius: var(--border-radius-md);
  box-shadow: var(--shadow-md);
  overflow: hidden;
  touch-action: manipulation;

  ${props => props.environmentalMode === 'outdoor' && `
    background: var(--theme-background-elevated);
    box-shadow: var(--shadow-lg);
    .data-grid-cell {
      font-weight: var(--font-weight-medium);
      color: var(--theme-text-primary);
    }
  `}

  @media (max-width: 768px) {
    border-radius: 0;
  }
`;

const ToolbarContainer = styled.div`
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: var(--spacing-md);
  border-bottom: 1px solid var(--theme-background-elevated);
  min-height: 64px;
  background: var(--theme-background-paper);

  @media (max-width: 768px) {
    flex-direction: column;
    gap: var(--spacing-sm);
    padding: var(--spacing-sm);
  }
`;

const FilterContainer = styled.div`
  display: flex;
  gap: var(--spacing-sm);
  flex-wrap: wrap;
  align-items: center;
`;

const SyncIndicator = styled.div<{ status: 'synced' | 'pending' | 'error' }>`
  display: flex;
  align-items: center;
  gap: var(--spacing-xs);
  font-size: var(--font-size-sm);
  color: ${props => {
    switch (props.status) {
      case 'synced': return 'var(--theme-status-success)';
      case 'pending': return 'var(--theme-status-warning)';
      case 'error': return 'var(--theme-status-error)';
      default: return 'var(--theme-text-secondary)';
    }
  }};
`;

export const DataGrid: React.FC<DataGridProps> = ({
  columns,
  data,
  loading = false,
  pageSize = 10,
  page = 1,
  totalRows = 0,
  selectable = false,
  offlineMode = false,
  syncStatus = 'synced',
  lastSyncTime,
  environmentalMode = 'indoor',
  onSelectionChange,
  onFilterChange,
  onColumnResize,
  onDataSync,
  onOfflineChange,
  className,
  stickyHeader = true,
  maxHeight,
  ...tableProps
}) => {
  const [columnWidths, setColumnWidths] = useState<Record<string, number>>({});
  const [filters, setFilters] = useState<Record<string, any>>({});
  const [selectedRows, setSelectedRows] = useState<Set<string>>(new Set());

  const handleColumnResize = useCallback((field: string, width: number) => {
    setColumnWidths(prev => ({ ...prev, [field]: width }));
    onColumnResize?.(field, width);
  }, [onColumnResize]);

  const handleFilterChange = useCallback((field: string, value: any) => {
    setFilters(prev => {
      const newFilters = { ...prev, [field]: value };
      onFilterChange?.(newFilters);
      return newFilters;
    });
  }, [onFilterChange]);

  const handleSelectionChange = useCallback((rows: any[]) => {
    const newSelection = new Set(rows.map(row => row.id));
    setSelectedRows(newSelection);
    onSelectionChange?.(rows);
  }, [onSelectionChange]);

  const enhancedColumns = useMemo(() => {
    return columns.map(column => ({
      ...column,
      width: columnWidths[column.field] || column.width,
      renderCell: (value: any, row: any) => {
        if (column.renderCell) {
          return column.renderCell(value, row);
        }
        
        switch (column.dataType) {
          case 'location':
            return `${value.latitude.toFixed(6)}, ${value.longitude.toFixed(6)}`;
          case 'status':
            return (
              <span style={{ 
                color: value === 'active' ? 'var(--theme-status-success)' : 'var(--theme-status-error)',
                fontWeight: column.highContrast ? 'var(--font-weight-bold)' : 'inherit'
              }}>
                {value}
              </span>
            );
          case 'sensor':
            return `${value.reading}${value.unit} (±${value.accuracy}${value.unit})`;
          default:
            return value;
        }
      }
    }));
  }, [columns, columnWidths]);

  useEffect(() => {
    const handleOnline = () => onOfflineChange?.(false);
    const handleOffline = () => onOfflineChange?.(true);

    window.addEventListener('online', handleOnline);
    window.addEventListener('offline', handleOffline);

    return () => {
      window.removeEventListener('online', handleOnline);
      window.removeEventListener('offline', handleOffline);
    };
  }, [onOfflineChange]);

  if (loading) {
    return (
      <GridContainer environmentalMode={environmentalMode}>
        <Loading size="lg" label="Loading data grid..." />
      </GridContainer>
    );
  }

  return (
    <GridContainer 
      environmentalMode={environmentalMode}
      className={classNames('data-grid', className)}
    >
      <ToolbarContainer>
        <FilterContainer>
          {columns
            .filter(column => column.filterable)
            .map(column => (
              <div key={column.field}>
                {column.renderFilter ? (
                  column.renderFilter(column)
                ) : (
                  <input
                    type="text"
                    placeholder={`Filter ${column.headerName}`}
                    onChange={e => handleFilterChange(column.field, e.target.value)}
                    value={filters[column.field] || ''}
                  />
                )}
              </div>
            ))}
        </FilterContainer>
        
        <SyncIndicator status={syncStatus}>
          {syncStatus === 'synced' && '✓ Synced'}
          {syncStatus === 'pending' && '↻ Syncing...'}
          {syncStatus === 'error' && '⚠ Sync Error'}
          {lastSyncTime && (
            <span>
              Last updated: {new Date(lastSyncTime).toLocaleTimeString()}
            </span>
          )}
          {onDataSync && (
            <Button
              variant="outlined"
              size="small"
              onClick={onDataSync}
              disabled={syncStatus === 'pending'}
              ariaLabel="Sync data"
            >
              Sync
            </Button>
          )}
        </SyncIndicator>
      </ToolbarContainer>

      <Table
        {...tableProps}
        columns={enhancedColumns}
        data={data}
        selectable={selectable}
        onSelectionChange={handleSelectionChange}
        stickyHeader={stickyHeader}
        style={{ maxHeight }}
        className="data-grid-table"
      />
    </GridContainer>
  );
};