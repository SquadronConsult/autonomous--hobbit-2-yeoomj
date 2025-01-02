# Development environment identifier
environment = "dev"

# Kubernetes version compatible with NVIDIA DeepStream and ROS 2
kubernetes_version = "1.26.0"

# Node pool configuration for development Jetson Orin AGX
node_pool_config = {
  primary = {
    instance_type = "agx-orin-32gb"
    min_size     = 1
    max_size     = 1
    disk_size    = 500
    gpu_enabled  = true
    labels = {
      "nvidia.com/gpu"    = "true"
      "environment"       = "development"
      "node-type"        = "jetson-agx"
      "deepstream-ready" = "true"
      "ros2-enabled"     = "true"
    }
    taints = [
      {
        key    = "nvidia.com/gpu"
        value  = "present"
        effect = "NoSchedule"
      }
    ]
  }
}

# Development storage configuration with reduced capacities
storage_config = {
  timescaledb = {
    storage_size          = 100
    replica_count         = 1
    backup_retention_days = 7
  }
  minio = {
    storage_size = 200
    replica_count = 1
    bucket_configuration = {
      models     = "models-dev"
      telemetry  = "telemetry-dev"
      analytics  = "analytics-dev"
      backups    = "backups-dev"
    }
  }
  redis = {
    memory_size   = 4
    replica_count = 1
  }
}

# Enable debug mode for development
debug_mode = true

# Enable monitoring for development environment
monitoring_enabled = true

# NVIDIA container runtime configuration
container_runtime = "nvidia-container-runtime"

# Development environment log retention
log_retention_days = 7

# Resource tagging for development environment
tags = {
  "Project"     = "AgricultureManagementSystem"
  "Environment" = "development"
  "Managed-by"  = "Terraform"
  "Purpose"     = "EdgeComputing"
  "Component"   = "JetsonDeployment"
}

# Network configuration for development
network_config = {
  vpc_cidr = "10.0.0.0/16"
  subnet_configuration = {
    primary = {
      cidr_block = "10.0.1.0/24"
      zone       = "primary"
      public     = false
    }
  }
  enable_vpn = true
  vpn_configuration = {
    bandwidth = "100Mbps"
    protocol  = "OpenVPN"
  }
}

# Security configuration for development
security_config = {
  enable_encryption        = true
  kms_key_rotation        = 30
  network_policies_enabled = true
  ssl_certificate = {
    provider = "letsencrypt"
    email    = "dev-admin@agricultural-system.local"
  }
}

# Backup configuration for development
backup_config = {
  enabled        = true
  retention_days = 7
  schedule       = "0 2 * * *"
  storage_class  = "standard"
}