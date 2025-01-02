# Network infrastructure outputs
output "network_configuration" {
  description = "Comprehensive network infrastructure configuration"
  value = {
    vpc_id              = module.network.vpc_id
    vpc_cidr_block      = module.network.vpc_cidr_block
    public_subnets      = module.network.public_subnet_ids
    private_subnets     = module.network.private_subnet_ids
    edge_subnets        = module.network.edge_subnet_ids
    field_subnets       = module.network.field_network_subnet_ids
    security_groups     = module.network.security_group_ids
    network_acls        = module.network.network_acl_ids
    nat_gateway_ips     = module.network.nat_gateway_ips
    security_zones      = module.network.security_zone_mappings
    flow_logs_enabled   = module.network.flow_logs_enabled
    flow_logs_group     = module.network.flow_logs_group_name
    network_metadata    = module.network.network_metadata
  }
  sensitive = false
}

# Kubernetes cluster outputs
output "kubernetes_cluster" {
  description = "Kubernetes cluster configuration and access details"
  value = {
    endpoint            = module.kubernetes.cluster_endpoint
    kubeconfig         = module.kubernetes.kubeconfig
    scaling_parameters = {
      gpu_nodes = {
        count           = module.kubernetes.node_pools.node_count
        gpu_enabled     = module.kubernetes.node_pools.gpu_enabled
        taints          = module.kubernetes.node_pools.node_taints
        labels          = module.kubernetes.node_pools.node_labels
      }
    }
    monitoring_configs = {
      prometheus_enabled = true
      grafana_enabled   = true
      retention_days    = 15
      storage_class     = "local-storage"
    }
  }
  sensitive = true
}

# Storage infrastructure outputs
output "storage_configuration" {
  description = "Storage infrastructure configuration details"
  value = {
    timescaledb = {
      storage_class     = module.storage.timescaledb_storage.storage_class_name
      pvc_name         = module.storage.timescaledb_storage.pvc_name
      capacity         = module.storage.timescaledb_storage.storage_size
      chunk_interval   = "${var.timescaledb_chunk_interval_days}d"
      backup_schedule  = var.backup_schedule
      retention_days   = var.backup_retention_days
    }
    minio = {
      storage_class     = module.storage.minio_storage.storage_class_name
      pvc_name         = module.storage.minio_storage.pvc_name
      capacity         = module.storage.minio_storage.storage_size
      backup_schedule  = var.backup_schedule
      retention_days   = var.backup_retention_days
    }
    redis = {
      storage_class     = module.storage.redis_storage.storage_class_name
      pvc_name         = module.storage.redis_storage.pvc_name
      capacity         = module.storage.redis_storage.storage_size
      backup_schedule  = var.backup_schedule
      retention_days   = var.backup_retention_days
    }
    backup_policies = {
      enabled          = true
      schedule         = var.backup_schedule
      retention_days   = var.backup_retention_days
      encryption       = var.encryption_enabled
    }
    performance_metrics = {
      iops_limit       = var.storage_iops
      replication = {
        enabled        = var.replication_enabled
        replica_count  = var.replica_count
      }
      monitoring = {
        enabled        = true
        retention_days = var.metrics_retention_days
      }
    }
  }
  sensitive = false
}

# System metadata outputs
output "system_metadata" {
  description = "System-wide metadata and configuration information"
  value = {
    environment         = "production"
    project_name       = "agricultural-management-system"
    terraform_version  = ">= 1.5.0"
    deployment_region  = data.aws_region.current.name
    deployment_time    = timestamp()
    tags = {
      Environment     = "production"
      Project        = "agricultural-management-system"
      Terraform      = "true"
      ManagedBy      = "terraform"
    }
  }
  sensitive = false
}

# Security configuration outputs
output "security_configuration" {
  description = "Security-related configuration and policies"
  value = {
    encryption = {
      storage_encryption = var.encryption_enabled
      tls_enabled       = true
      data_at_rest      = true
    }
    network_security = {
      flow_logs_enabled = module.network.flow_logs_enabled
      security_zones    = keys(module.network.security_zone_mappings)
      network_policies  = true
    }
    monitoring = {
      prometheus_enabled = true
      metrics_retention = var.metrics_retention_days
      log_aggregation   = true
    }
  }
  sensitive = false
}