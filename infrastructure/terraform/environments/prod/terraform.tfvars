# Production environment specification
environment = "prod"

# Geographic region for edge deployment
region = "us-west-1"

# Kubernetes version for container orchestration
kubernetes_version = "1.26.0"

# Node pool configuration for Jetson Orin clusters
node_pool_config = {
  compute = {
    instance_type = "agx-orin-64gb"
    min_size      = 3
    max_size      = 5
    desired_size  = 3
    disk_size     = 500
    gpu_enabled   = true
    labels = {
      "node.kubernetes.io/purpose" = "compute"
      "nvidia.com/gpu"            = "true"
    }
    taints = [
      {
        key    = "nvidia.com/gpu"
        value  = "true"
        effect = "NoSchedule"
      }
    ]
  }
  storage = {
    instance_type = "agx-orin-32gb"
    min_size      = 2
    max_size      = 4
    desired_size  = 2
    disk_size     = 1000
    gpu_enabled   = false
    labels = {
      "node.kubernetes.io/purpose" = "storage"
    }
    taints = []
  }
}

# Storage configuration for production environment
storage_config = {
  timescaledb = {
    storage_size         = "500Gi"
    replica_count        = 2
    backup_retention_days = 90
  }
  minio = {
    storage_size = "1Ti"
    replica_count = 4
    bucket_configuration = {
      versioning_enabled = "true"
      lifecycle_rules_enabled = "true"
      replication_enabled = "true"
    }
  }
  redis = {
    memory_size    = "100Gi"
    replica_count  = 3
  }
}

# Enable production monitoring stack
monitoring_enabled = true

# Enable high availability for production
high_availability = true

# NVIDIA container runtime configuration
container_runtime = "nvidia-container-runtime"

# Network configuration for production
network_config = {
  vpc_cidr = "10.0.0.0/16"
  subnet_configuration = {
    compute = {
      cidr_block = "10.0.1.0/24"
      zone       = "us-west-1a"
      public     = false
    }
    storage = {
      cidr_block = "10.0.2.0/24"
      zone       = "us-west-1b"
      public     = false
    }
    management = {
      cidr_block = "10.0.3.0/24"
      zone       = "us-west-1c"
      public     = true
    }
  }
  enable_vpn = true
  vpn_configuration = {
    bandwidth = "1Gbps"
    protocol  = "OpenVPN"
  }
}

# Security configuration for production
security_config = {
  enable_encryption = true
  kms_key_rotation = 90
  ssl_certificate = {
    provider = "letsencrypt"
    email    = "admin@agricultural-system.com"
  }
  network_policies_enabled = true
}

# Backup configuration for production
backup_config = {
  enabled        = true
  retention_days = 90
  schedule       = "0 1 * * *"
  storage_class  = "premium"
}

# Resource tagging for production environment
tags = {
  Environment = "production"
  Project     = "agricultural-management-system"
  ManagedBy   = "terraform"
  CostCenter  = "agriculture-ops"
  Compliance  = "iso27001"
}