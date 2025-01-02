# Terraform configuration for storage resources
terraform {
  required_version = ">= 1.5.0"
  required_providers {
    kubernetes = {
      source  = "hashicorp/kubernetes"
      version = "~> 2.23.0"
    }
  }
}

# Local variables for storage configuration
locals {
  storage_labels = {
    "app.kubernetes.io/component" = "storage"
    "app.kubernetes.io/part-of"   = "aams"
  }

  monitoring_annotations = var.monitoring_enabled ? {
    "prometheus.io/scrape" = "true"
    "prometheus.io/port"   = "9090"
  } : {}

  encryption_config = var.encryption_enabled ? {
    "encryption.kubernetes.io/enabled" = "true"
    "encryption.kubernetes.io/cipher"  = "AES-256-GCM"
  } : {}
}

# TimescaleDB Storage Class
resource "kubernetes_storage_class" "timescaledb" {
  metadata {
    name = "timescaledb-storage"
    labels = merge(local.storage_labels, {
      "storage.kubernetes.io/type" = "timescaledb"
    })
  }

  storage_provisioner = var.storage_class
  reclaim_policy     = "Retain"
  volume_binding_mode = "WaitForFirstConsumer"

  parameters = {
    "type"                 = "pd-ssd"
    "fsType"              = "ext4"
    "iopsPerGB"           = var.storage_iops
    "replication-factor"  = var.replication_enabled ? var.replica_count : 1
  }

  allow_volume_expansion = true

  metadata {
    annotations = merge(
      local.monitoring_annotations,
      local.encryption_config,
      {
        "backup.kubernetes.io/schedule"         = var.backup_schedule
        "backup.kubernetes.io/retention-period" = "${var.backup_retention_days}d"
        "timescaledb.kubernetes.io/chunk-interval" = "${var.timescaledb_chunk_interval_days}d"
      }
    )
  }
}

# TimescaleDB Persistent Volume Claim
resource "kubernetes_persistent_volume_claim" "timescaledb" {
  metadata {
    name = "timescaledb-data"
    labels = merge(local.storage_labels, {
      "app.kubernetes.io/name" = "timescaledb"
    })
    annotations = merge(
      local.monitoring_annotations,
      local.encryption_config
    )
  }

  spec {
    storage_class_name = kubernetes_storage_class.timescaledb.metadata[0].name
    access_modes       = ["ReadWriteOnce"]
    resources {
      requests = {
        storage = var.timescaledb_storage_size
      }
    }
  }
}

# MinIO Storage Class
resource "kubernetes_storage_class" "minio" {
  metadata {
    name = "minio-storage"
    labels = merge(local.storage_labels, {
      "storage.kubernetes.io/type" = "minio"
    })
  }

  storage_provisioner = var.storage_class
  reclaim_policy     = "Retain"
  volume_binding_mode = "WaitForFirstConsumer"

  parameters = {
    "type"                = "pd-standard"
    "fsType"             = "ext4"
    "replication-factor" = var.replication_enabled ? var.replica_count : 1
  }

  allow_volume_expansion = true

  metadata {
    annotations = merge(
      local.monitoring_annotations,
      local.encryption_config,
      {
        "backup.kubernetes.io/schedule"         = var.backup_schedule
        "backup.kubernetes.io/retention-period" = "${var.backup_retention_days}d"
      }
    )
  }
}

# MinIO Persistent Volume Claim
resource "kubernetes_persistent_volume_claim" "minio" {
  metadata {
    name = "minio-data"
    labels = merge(local.storage_labels, {
      "app.kubernetes.io/name" = "minio"
    })
    annotations = merge(
      local.monitoring_annotations,
      local.encryption_config
    )
  }

  spec {
    storage_class_name = kubernetes_storage_class.minio.metadata[0].name
    access_modes       = ["ReadWriteOnce"]
    resources {
      requests = {
        storage = var.minio_storage_size
      }
    }
  }
}

# Redis Storage Class
resource "kubernetes_storage_class" "redis" {
  metadata {
    name = "redis-storage"
    labels = merge(local.storage_labels, {
      "storage.kubernetes.io/type" = "redis"
    })
  }

  storage_provisioner = var.storage_class
  reclaim_policy     = "Retain"
  volume_binding_mode = "WaitForFirstConsumer"

  parameters = {
    "type"                = "pd-ssd"
    "fsType"             = "ext4"
    "iopsPerGB"          = var.storage_iops
    "replication-factor" = var.replication_enabled ? var.replica_count : 1
  }

  allow_volume_expansion = true

  metadata {
    annotations = merge(
      local.monitoring_annotations,
      local.encryption_config,
      {
        "backup.kubernetes.io/schedule"         = var.backup_schedule
        "backup.kubernetes.io/retention-period" = "${var.backup_retention_days}d"
      }
    )
  }
}

# Redis Persistent Volume Claim
resource "kubernetes_persistent_volume_claim" "redis" {
  metadata {
    name = "redis-data"
    labels = merge(local.storage_labels, {
      "app.kubernetes.io/name" = "redis"
    })
    annotations = merge(
      local.monitoring_annotations,
      local.encryption_config
    )
  }

  spec {
    storage_class_name = kubernetes_storage_class.redis.metadata[0].name
    access_modes       = ["ReadWriteOnce"]
    resources {
      requests = {
        storage = var.redis_storage_size
      }
    }
  }
}

# Output values for use in other modules
output "timescaledb_storage" {
  value = {
    storage_class_name = kubernetes_storage_class.timescaledb.metadata[0].name
    pvc_name          = kubernetes_persistent_volume_claim.timescaledb.metadata[0].name
    storage_size      = var.timescaledb_storage_size
  }
  description = "TimescaleDB storage configuration details"
}

output "minio_storage" {
  value = {
    storage_class_name = kubernetes_storage_class.minio.metadata[0].name
    pvc_name          = kubernetes_persistent_volume_claim.minio.metadata[0].name
    storage_size      = var.minio_storage_size
  }
  description = "MinIO storage configuration details"
}

output "redis_storage" {
  value = {
    storage_class_name = kubernetes_storage_class.redis.metadata[0].name
    pvc_name          = kubernetes_persistent_volume_claim.redis.metadata[0].name
    storage_size      = var.redis_storage_size
  }
  description = "Redis storage configuration details"
}