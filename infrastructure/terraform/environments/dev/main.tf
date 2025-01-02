# Development environment configuration for Agricultural Management System
terraform {
  required_version = ">= 1.5.0"
  required_providers {
    kubernetes = {
      source  = "hashicorp/kubernetes"
      version = "~> 2.23.0"  # Latest stable version for Kubernetes provider
    }
    helm = {
      source  = "hashicorp/helm"
      version = "~> 2.11.0"  # Latest stable version for Helm provider
    }
  }
}

# Local variables for development environment
locals {
  environment = "dev"
  debug_mode  = true
  
  common_tags = {
    Environment = local.environment
    Project     = "agricultural-management-system"
    ManagedBy   = "terraform"
    Debug       = local.debug_mode
  }

  # Development-specific storage configurations
  storage_config = {
    timescaledb = {
      storage_size = "100Gi"  # Reduced size for development
      replica_count = 1       # Single replica for dev
      backup_retention_days = 7
    }
    minio = {
      storage_size = "200Gi"
      replica_count = 1
      bucket_configuration = {
        models = "tao-models"
        video  = "deepstream-data"
        logs   = "debug-logs"
      }
    }
    redis = {
      memory_size = "4Gi"
      replica_count = 1
    }
  }
}

# Network module configuration for development
module "network" {
  source = "../../modules/network"

  vpc_cidr = "10.0.0.0/16"
  subnet_configuration = [
    {
      name               = "dev-edge-1"
      cidr_block         = "10.0.1.0/24"
      availability_zone  = "us-west-2a"
      zone_type         = "edge"
      is_public         = false
      route_table_tags  = { Purpose = "edge-compute" }
    },
    {
      name               = "dev-field-1"
      cidr_block         = "10.0.2.0/24"
      availability_zone  = "us-west-2a"
      zone_type         = "field"
      is_public         = false
      route_table_tags  = { Purpose = "robot-network" }
    }
  ]

  enable_nat_gateway = true
  
  network_security_groups = {
    edge_compute = {
      name = "dev-edge-compute-sg"
      description = "Development security group for Jetson Orin nodes"
      ingress_rules = [
        {
          description = "Allow HTTPS from VPC"
          from_port   = 443
          to_port     = 443
          protocol    = "tcp"
          cidr_blocks = ["10.0.0.0/16"]
        },
        {
          description = "Allow debugging ports"
          from_port   = 8000
          to_port     = 8999
          protocol    = "tcp"
          cidr_blocks = ["10.0.0.0/16"]
        }
      ]
    }
  }

  flow_logs_config = {
    enabled = true
    log_destination = "cloudwatch"
    traffic_type = "ALL"
    retention_days = 7
    log_format = "$${version} $${account-id} $${interface-id} $${srcaddr} $${dstaddr} $${srcport} $${dstport} $${protocol} $${packets} $${bytes} $${start} $${end} $${action} $${log-status}"
  }

  tags = local.common_tags
}

# Kubernetes module configuration for development
module "kubernetes" {
  source = "../../modules/kubernetes"
  depends_on = [module.network]

  cluster_name = "dev-agricultural-edge"
  kubernetes_version = "1.26.0"
  
  node_pools = {
    compute = {
      name = "dev-jetson-pool"
      instance_type = "nvidia-jetson-orin"
      min_size = 1
      max_size = 2
      labels = {
        "nvidia.com/gpu" = "true"
        "environment" = "development"
      }
      taints = [{
        key = "nvidia.com/gpu"
        value = "true"
        effect = "NoSchedule"
      }]
    }
  }

  container_runtime = "nvidia-container-runtime"
  nvidia_driver_version = "535.104.05"

  # Development-specific resource quotas
  resource_quotas = {
    gpu_resources = {
      hard = {
        "requests.nvidia.com/gpu" = "1"
        "limits.nvidia.com/gpu" = "1"
      }
    }
  }

  # Enable debug endpoints for development
  monitoring_config = {
    metrics_retention = "7d"
    debug_endpoints = true
    verbose_logging = true
  }
}

# Storage module configuration for development
module "storage" {
  source = "../../modules/storage"
  depends_on = [module.kubernetes]

  storage_class = "local-path"
  
  timescaledb_storage_size = local.storage_config.timescaledb.storage_size
  minio_storage_size = local.storage_config.minio.storage_size
  redis_storage_size = local.storage_config.redis.memory_size
  
  backup_retention_days = 7
  encryption_enabled = true
  
  timescaledb_chunk_interval_days = 1
  replication_enabled = false
  replica_count = 1
  
  storage_iops = 3000
  metrics_retention_days = 7
  backup_schedule = "0 0 * * *"
}

# Development environment outputs
output "dev_network_config" {
  description = "Development network configuration"
  value = {
    vpc_id = module.network.vpc_id
    subnet_ids = {
      edge = module.network.edge_subnet_ids
      field = module.network.field_network_subnet_ids
    }
    security_groups = module.network.security_group_ids
  }
}

output "dev_kubernetes_config" {
  description = "Development Kubernetes configuration"
  value = {
    cluster_endpoint = module.kubernetes.kubernetes_cluster.endpoint
    gpu_node_pool = module.kubernetes.node_pools
  }
  sensitive = true
}

output "dev_storage_config" {
  description = "Development storage configuration"
  value = {
    timescaledb = module.storage.timescaledb_storage
    minio = module.storage.minio_storage
    redis = module.storage.redis_storage
  }
}