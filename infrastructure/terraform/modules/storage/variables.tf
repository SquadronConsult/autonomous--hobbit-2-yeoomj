# Storage class configuration for persistent volumes
variable "storage_class" {
  description = "Storage class type for persistent volumes (e.g. local-path, nfs)"
  type        = string
  default     = "local-path"

  validation {
    condition     = contains(["local-path", "nfs", "longhorn"], var.storage_class)
    error_message = "Storage class must be one of: local-path, nfs, or longhorn."
  }
}

# TimescaleDB storage configuration
variable "timescaledb_storage_size" {
  description = "Storage capacity for TimescaleDB persistent volume"
  type        = string
  default     = "500Gi"

  validation {
    condition     = can(regex("^[0-9]+[MGT]i$", var.timescaledb_storage_size))
    error_message = "TimescaleDB storage size must be specified in Mi, Gi, or Ti units."
  }
}

# MinIO object storage configuration
variable "minio_storage_size" {
  description = "Storage capacity for MinIO object storage"
  type        = string
  default     = "1000Gi"

  validation {
    condition     = can(regex("^[0-9]+[MGT]i$", var.minio_storage_size))
    error_message = "MinIO storage size must be specified in Mi, Gi, or Ti units."
  }
}

# Redis storage configuration
variable "redis_storage_size" {
  description = "Storage capacity for Redis persistent volume"
  type        = string
  default     = "20Gi"

  validation {
    condition     = can(regex("^[0-9]+[MGT]i$", var.redis_storage_size))
    error_message = "Redis storage size must be specified in Mi, Gi, or Ti units."
  }
}

# Data retention configuration
variable "backup_retention_days" {
  description = "Number of days to retain backup data"
  type        = number
  default     = 90

  validation {
    condition     = var.backup_retention_days >= 30 && var.backup_retention_days <= 365
    error_message = "Backup retention period must be between 30 and 365 days."
  }
}

# Storage encryption configuration
variable "encryption_enabled" {
  description = "Enable encryption for persistent volumes"
  type        = bool
  default     = true
}

# Time-series data partitioning configuration
variable "timescaledb_chunk_interval_days" {
  description = "Number of days per TimescaleDB chunk interval"
  type        = number
  default     = 1

  validation {
    condition     = var.timescaledb_chunk_interval_days >= 1 && var.timescaledb_chunk_interval_days <= 30
    error_message = "TimescaleDB chunk interval must be between 1 and 30 days."
  }
}

# High availability configuration
variable "replication_enabled" {
  description = "Enable replication for database storage"
  type        = bool
  default     = true
}

variable "replica_count" {
  description = "Number of storage replicas when replication is enabled"
  type        = number
  default     = 2

  validation {
    condition     = var.replica_count >= 1 && var.replica_count <= 5
    error_message = "Replica count must be between 1 and 5."
  }
}

# Performance configuration
variable "storage_iops" {
  description = "IOPS limit for storage volumes"
  type        = number
  default     = 3000

  validation {
    condition     = var.storage_iops >= 1000 && var.storage_iops <= 10000
    error_message = "IOPS must be between 1000 and 10000."
  }
}

# Monitoring configuration
variable "metrics_retention_days" {
  description = "Number of days to retain storage metrics"
  type        = number
  default     = 15

  validation {
    condition     = var.metrics_retention_days >= 7 && var.metrics_retention_days <= 90
    error_message = "Metrics retention period must be between 7 and 90 days."
  }
}

# Backup configuration
variable "backup_schedule" {
  description = "Cron schedule for storage backups"
  type        = string
  default     = "0 0 * * *"

  validation {
    condition     = can(regex("^[0-9*/-]+ [0-9*/-]+ [0-9*/-]+ [0-9*/-]+ [0-9*/-]+$", var.backup_schedule))
    error_message = "Backup schedule must be a valid cron expression."
  }
}