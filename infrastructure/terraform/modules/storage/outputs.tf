# Output configuration for storage module resources
# Terraform version requirements
terraform {
  required_version = ">= 1.5.0"
}

# TimescaleDB storage outputs
output "timescaledb_storage_class" {
  description = "Storage class name for TimescaleDB persistent volumes"
  value       = kubernetes_storage_class.timescaledb.metadata[0].name
}

output "timescaledb_pvc_name" {
  description = "PVC name for TimescaleDB data storage"
  value       = kubernetes_persistent_volume_claim.timescaledb.metadata[0].name
}

# MinIO storage outputs
output "minio_storage_class" {
  description = "Storage class name for MinIO object storage"
  value       = kubernetes_storage_class.minio.metadata[0].name
}

output "minio_pvc_name" {
  description = "PVC name for MinIO data storage"
  value       = kubernetes_persistent_volume_claim.minio.metadata[0].name
}

# Redis storage outputs
output "redis_storage_class" {
  description = "Storage class name for Redis persistent volumes"
  value       = kubernetes_storage_class.redis.metadata[0].name
}

output "redis_pvc_name" {
  description = "PVC name for Redis data storage"
  value       = kubernetes_persistent_volume_claim.redis.metadata[0].name
}

# Storage encryption status
output "storage_encryption_enabled" {
  description = "Indicates whether storage encryption is enabled"
  value       = var.encryption_enabled
}

# Storage configuration details
output "storage_details" {
  description = "Comprehensive storage configuration details for all components"
  value = {
    timescaledb = {
      storage_class = kubernetes_storage_class.timescaledb.metadata[0].name
      pvc_name     = kubernetes_persistent_volume_claim.timescaledb.metadata[0].name
      size         = var.timescaledb_storage_size
      encrypted    = var.encryption_enabled
    }
    minio = {
      storage_class = kubernetes_storage_class.minio.metadata[0].name
      pvc_name     = kubernetes_persistent_volume_claim.minio.metadata[0].name
      size         = var.minio_storage_size
      encrypted    = var.encryption_enabled
    }
    redis = {
      storage_class = kubernetes_storage_class.redis.metadata[0].name
      pvc_name     = kubernetes_persistent_volume_claim.redis.metadata[0].name
      size         = var.redis_storage_size
      encrypted    = var.encryption_enabled
    }
  }
  sensitive = false
}

# Storage replication status
output "storage_replication_config" {
  description = "Storage replication configuration details"
  value = {
    enabled       = var.replication_enabled
    replica_count = var.replica_count
  }
}

# Backup configuration
output "backup_config" {
  description = "Storage backup configuration details"
  value = {
    schedule         = var.backup_schedule
    retention_days   = var.backup_retention_days
  }
}