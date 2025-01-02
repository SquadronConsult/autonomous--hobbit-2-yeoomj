# Core Terraform configuration
terraform {
  required_version = ">= 1.5.0"
  required_providers {
    kubernetes = {
      source  = "hashicorp/kubernetes"
      version = "~> 2.23.0"
    }
    helm = {
      source  = "hashicorp/helm"
      version = "~> 2.11.0"
    }
    local = {
      source  = "hashicorp/local"
      version = "~> 2.4.0"
    }
  }
}

# Network module for VPC and security zones
module "network" {
  source = "./modules/network"

  vpc_cidr = var.network_config.vpc_cidr
  subnet_configuration = var.network_config.subnet_configuration
  enable_nat_gateway = true
  
  network_security_groups = {
    edge_compute = {
      name = "edge-compute-sg"
      description = "Security group for Jetson Orin edge compute nodes"
      ingress_rules = [
        {
          description = "Allow HTTPS from VPC"
          from_port   = 443
          to_port     = 443
          protocol    = "tcp"
          cidr_blocks = [var.network_config.vpc_cidr]
        },
        {
          description = "Allow container communication"
          from_port   = 2376
          to_port     = 2376
          protocol    = "tcp"
          cidr_blocks = [var.network_config.vpc_cidr]
        }
      ]
    }
  }

  security_zones = {
    field_network = {
      name = "field-network"
      description = "Secure zone for drone and robot communication"
      cidr_blocks = [var.network_config.vpc_cidr]
      rules = {
        drone_communication = {
          type = "ingress"
          ports = [8883]  # MQTT TLS
          protocols = ["tcp"]
          sources = ["drone_fleet"]
        }
      }
    }
  }

  flow_logs_config = {
    enabled = true
    traffic_type = "ALL"
    retention_days = 30
  }

  tags = var.tags
}

# Kubernetes cluster configuration for edge computing
module "kubernetes" {
  source = "./modules/kubernetes"
  depends_on = [module.network]

  cluster_name = "agricultural-edge-cluster"
  kubernetes_version = var.kubernetes_version
  
  node_pools = {
    compute = {
      name = "jetson-compute"
      instance_type = "nvidia-jetson-orin"
      min_size = 1
      max_size = 3
      labels = {
        "nvidia.com/gpu" = "true"
        "purpose" = "edge-compute"
      }
      taints = [{
        key = "nvidia.com/gpu"
        value = "true"
        effect = "NoSchedule"
      }]
    }
  }

  container_runtime = var.container_runtime
  nvidia_driver_version = "535.104.05"  # Latest stable version for Jetson Orin
}

# Storage configuration for persistent data
module "storage" {
  source = "./modules/storage"
  depends_on = [module.kubernetes]

  storage_classes = {
    timescaledb = {
      name = "timescaledb-storage"
      type = "local-nvme"
      parameters = {
        fsType = "ext4"
        volumeMode = "Filesystem"
      }
    }
    minio = {
      name = "minio-storage"
      type = "local-ssd"
      parameters = {
        fsType = "xfs"
        volumeMode = "Filesystem"
      }
    }
  }

  persistent_volumes = {
    timescaledb = {
      storage_class = "timescaledb-storage"
      size = var.storage_config.timescaledb.storage_size
      access_modes = ["ReadWriteOnce"]
    }
    minio = {
      storage_class = "minio-storage"
      size = var.storage_config.minio.storage_size
      access_modes = ["ReadWriteMany"]
    }
  }

  backup_config = var.backup_config
}

# Application deployment using Helm
module "applications" {
  source = "./modules/applications"
  depends_on = [module.kubernetes, module.storage]

  deepstream_config = {
    repository = "nvcr.io/nvidia/deepstream"
    tag = "6.2-triton"
    gpu_enabled = true
    resources = {
      requests = {
        nvidia.com/gpu = 1
      }
      limits = {
        nvidia.com/gpu = 1
      }
    }
  }

  ros2_config = {
    repository = "ros"
    tag = "jazzy"
    namespace = "robotics"
    persistence = {
      enabled = true
      storage_class = module.storage.storage_classes["timescaledb"].name
    }
  }

  monitoring_enabled = var.monitoring_enabled
  high_availability = var.high_availability
}

# Security configuration
module "security" {
  source = "./modules/security"
  depends_on = [module.kubernetes]

  network_policies_enabled = var.security_config.network_policies_enabled
  
  encryption_config = {
    enabled = var.security_config.enable_encryption
    kms_key_rotation_days = var.security_config.kms_key_rotation
  }

  certificate_config = {
    provider = var.security_config.ssl_certificate.provider
    email = var.security_config.ssl_certificate.email
  }
}

# Outputs
output "cluster_endpoint" {
  description = "Kubernetes cluster endpoint"
  value = module.kubernetes.cluster_endpoint
}

output "storage_classes" {
  description = "Available storage classes"
  value = module.storage.storage_classes
}

output "network_security_groups" {
  description = "Network security group configurations"
  value = module.network.security_groups
  sensitive = true
}