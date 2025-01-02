import React, { useState, useEffect, useCallback, useMemo } from 'react';
import styled from '@emotion/styled';
import { Loading } from '../Loading/Loading';
import { Button } from '../Button/Button';

// Version comments for dependencies
// @emotion/styled: ^11.11.0
// react: ^18.0.0

// Interfaces
interface TableProps {
  data: Array<Record<string, any>>;
  columns: Array<ColumnDefinition>;
  isLoading?: boolean;
  isOffline?: boolean;
  sortable?: boolean;
  selectable?: boolean;
  pagination?: boolean;
  pageSize?: number;
  onSort?: (column: string, direction: 'asc' | 'desc') => void;
  onSelect?: (selectedItems: Array<Record<string, any>>) => void;
  onPageChange?: (page: number) => void;
  onDataUpdate?: (updatedData: Array<Record<string, any>>) => void;
  refreshInterval?: number;
  lastUpdateTime?: Date;
}

interface ColumnDefinition {
  id: string;
  label: string;
  accessor: string;
  sortable?: boolean;
  width?: string;
  priority?: number;
  dataType?: 'text' | 'number' | 'date' | 'status' | 'coordinates' | 'sensor';
  formatOptions?: Record<string, any>;
  render?: (value: any, row: Record<string, any>) => React.ReactNode;
}

// Styled Components
const TableContainer = styled.div`
  width: 100%;
  overflow-x: auto;
  background: var(--theme-background-paper);
  border-radius: var(--border-radius-md);
  box-shadow: var(--shadow-sm);
  position: relative;

  &.offline {
    opacity: 0.9;
    &::after {
      content: 'Offline Mode';
      position: absolute;
      top: var(--spacing-xs);
      right: var(--spacing-xs);
      background: var(--theme-status-warning);
      color: white;
      padding: var(--spacing-2xs) var(--spacing-xs);
      border-radius: var(--border-radius-sm);
      font-size: var(--font-size-xs);
    }
  }
`;

const StyledTable = styled.table`
  width: 100%;
  border-collapse: separate;
  border-spacing: 0;
  font-family: var(--font-family-primary);
`;

const TableHeader = styled.thead`
  background: var(--theme-background-elevated);
  position: sticky;
  top: 0;
  z-index: var(--z-index-sticky);
`;

const TableHeaderCell = styled.th<{ sortable?: boolean; width?: string }>`
  padding: var(--spacing-sm);
  text-align: left;
  font-weight: var(--font-weight-medium);
  color: var(--theme-text-secondary);
  border-bottom: 1px solid var(--theme-background-elevated);
  white-space: nowrap;
  width: ${props => props.width || 'auto'};
  cursor: ${props => props.sortable ? 'pointer' : 'default'};

  &:hover {
    ${props => props.sortable && `
      background: var(--theme-background-elevated);
    `}
  }

  @media (max-width: 768px) {
    &[data-priority="low"] {
      display: none;
    }
  }
`;

const TableBody = styled.tbody`
  background: var(--theme-background-paper);
`;

const TableRow = styled.tr<{ selected?: boolean }>`
  &:hover {
    background: var(--theme-background-elevated);
  }

  ${props => props.selected && `
    background: var(--theme-primary-main)14;
  `}
`;

const TableCell = styled.td`
  padding: var(--spacing-sm);
  border-bottom: 1px solid var(--theme-background-elevated);
  color: var(--theme-text-primary);

  @media (max-width: 768px) {
    &[data-priority="low"] {
      display: none;
    }
  }
`;

const PaginationContainer = styled.div`
  display: flex;
  justify-content: flex-end;
  align-items: center;
  padding: var(--spacing-sm);
  gap: var(--spacing-xs);
`;

// Main Component
export const Table: React.FC<TableProps> = ({
  data,
  columns,
  isLoading = false,
  isOffline = false,
  sortable = true,
  selectable = false,
  pagination = true,
  pageSize = 10,
  onSort,
  onSelect,
  onPageChange,
  onDataUpdate,
  refreshInterval,
  lastUpdateTime
}) => {
  // State management
  const [currentPage, setCurrentPage] = useState(1);
  const [sortConfig, setSortConfig] = useState<{ column: string; direction: 'asc' | 'desc' } | null>(null);
  const [selectedRows, setSelectedRows] = useState<Set<string>>(new Set());
  const [displayData, setDisplayData] = useState<Array<Record<string, any>>>([]);

  // Memoized calculations
  const totalPages = useMemo(() => Math.ceil(data.length / pageSize), [data.length, pageSize]);
  const paginatedData = useMemo(() => {
    const start = (currentPage - 1) * pageSize;
    return displayData.slice(start, start + pageSize);
  }, [displayData, currentPage, pageSize]);

  // Sort handler
  const handleSort = useCallback((columnId: string) => {
    const column = columns.find(col => col.id === columnId);
    if (!column?.sortable) return;

    const direction = sortConfig?.column === columnId && sortConfig.direction === 'asc' ? 'desc' : 'asc';
    setSortConfig({ column: columnId, direction });
    onSort?.(columnId, direction);
  }, [columns, sortConfig, onSort]);

  // Selection handler
  const handleRowSelect = useCallback((rowId: string) => {
    setSelectedRows(prev => {
      const newSelection = new Set(prev);
      if (newSelection.has(rowId)) {
        newSelection.delete(rowId);
      } else {
        newSelection.add(rowId);
      }
      onSelect?.(Array.from(newSelection).map(id => data.find(row => row.id === id)).filter(Boolean));
      return newSelection;
    });
  }, [data, onSelect]);

  // Format cell value based on data type
  const formatCellValue = useCallback((value: any, dataType?: string, formatOptions?: Record<string, any>) => {
    if (!value) return '-';
    
    switch (dataType) {
      case 'date':
        return new Date(value).toLocaleString(undefined, formatOptions);
      case 'coordinates':
        return `${value.lat.toFixed(6)}, ${value.lng.toFixed(6)}`;
      case 'sensor':
        return `${value.reading}${value.unit} (±${value.accuracy}${value.unit})`;
      case 'status':
        return (
          <span style={{ 
            color: value === 'active' ? 'var(--theme-status-success)' : 'var(--theme-status-error)'
          }}>
            {value}
          </span>
        );
      default:
        return value.toString();
    }
  }, []);

  // Effect for auto-refresh
  useEffect(() => {
    if (!refreshInterval) return;
    
    const interval = setInterval(() => {
      onDataUpdate?.(data);
    }, refreshInterval);

    return () => clearInterval(interval);
  }, [refreshInterval, data, onDataUpdate]);

  // Render loading state
  if (isLoading) {
    return (
      <TableContainer>
        <Loading size="lg" label="Loading table data..." />
      </TableContainer>
    );
  }

  return (
    <TableContainer className={isOffline ? 'offline' : ''}>
      <StyledTable role="table" aria-label="Data table">
        <TableHeader>
          <tr>
            {selectable && (
              <TableHeaderCell width="40px">
                <input
                  type="checkbox"
                  onChange={() => {/* Implement select all */}}
                  aria-label="Select all rows"
                />
              </TableHeaderCell>
            )}
            {columns.map(column => (
              <TableHeaderCell
                key={column.id}
                sortable={sortable && column.sortable}
                width={column.width}
                onClick={() => sortable && column.sortable && handleSort(column.id)}
                aria-sort={sortConfig?.column === column.id ? sortConfig.direction : undefined}
                data-priority={column.priority || 'high'}
              >
                {column.label}
                {sortable && column.sortable && (
                  <span aria-hidden="true">
                    {sortConfig?.column === column.id ? (
                      sortConfig.direction === 'asc' ? ' ↑' : ' ↓'
                    ) : ' ↕'}
                  </span>
                )}
              </TableHeaderCell>
            ))}
          </tr>
        </TableHeader>
        <TableBody>
          {paginatedData.map((row, index) => (
            <TableRow
              key={row.id || index}
              selected={selectedRows.has(row.id)}
              onClick={() => selectable && handleRowSelect(row.id)}
            >
              {selectable && (
                <TableCell>
                  <input
                    type="checkbox"
                    checked={selectedRows.has(row.id)}
                    onChange={() => handleRowSelect(row.id)}
                    aria-label={`Select row ${index + 1}`}
                  />
                </TableCell>
              )}
              {columns.map(column => (
                <TableCell
                  key={`${row.id}-${column.id}`}
                  data-priority={column.priority || 'high'}
                >
                  {column.render ? 
                    column.render(row[column.accessor], row) :
                    formatCellValue(row[column.accessor], column.dataType, column.formatOptions)
                  }
                </TableCell>
              ))}
            </TableRow>
          ))}
        </TableBody>
      </StyledTable>

      {pagination && totalPages > 1 && (
        <PaginationContainer>
          <Button
            variant="outlined"
            size="small"
            disabled={currentPage === 1}
            onClick={() => {
              setCurrentPage(prev => prev - 1);
              onPageChange?.(currentPage - 1);
            }}
            ariaLabel="Previous page"
          >
            Previous
          </Button>
          <span>
            Page {currentPage} of {totalPages}
          </span>
          <Button
            variant="outlined"
            size="small"
            disabled={currentPage === totalPages}
            onClick={() => {
              setCurrentPage(prev => prev + 1);
              onPageChange?.(currentPage + 1);
            }}
            ariaLabel="Next page"
          >
            Next
          </Button>
        </PaginationContainer>
      )}
    </TableContainer>
  );
};

export type { TableProps, ColumnDefinition };