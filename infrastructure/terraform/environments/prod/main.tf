# Configure Terraform and required providers
terraform {
  required_version = ">= 1.5.0"
  required_providers {
    kubernetes = {
      source  = "hashicorp/kubernetes" # version: ~> 2.23.0
      version = "~> 2.23.0"
    }
    helm = {
      source  = "hashicorp/helm" # version: ~> 2.11.0
      version = "~> 2.11.0"
    }
  }

  backend "s3" {
    bucket         = "aams-terraform-state"
    key            = "prod/terraform.tfstate"
    region         = "us-west-2"
    encrypt        = true
    dynamodb_table = "aams-terraform-locks"
  }
}

# Local variables for production environment
locals {
  environment = "production"
  region      = "us-west-2"

  tags = {
    Environment = local.environment
    Project     = "agricultural-management-system"
    ManagedBy   = "terraform"
  }

  # High availability settings
  ha_config = {
    replica_count           = 2
    backup_retention_days   = 90
    metrics_retention_days  = 15
    monitoring_enabled      = true
    encryption_enabled      = true
  }
}

# Network module for production environment
module "network" {
  source = "../../modules/network"

  vpc_cidr = "10.0.0.0/16"
  
  subnet_configuration = [
    {
      name              = "public-1a"
      cidr_block        = "10.0.0.0/24"
      availability_zone = "${local.region}a"
      zone_type         = "public"
      is_public         = true
      route_table_tags  = { Type = "public" }
    },
    {
      name              = "private-1a"
      cidr_block        = "10.0.1.0/24"
      availability_zone = "${local.region}a"
      zone_type         = "private"
      is_public         = false
      route_table_tags  = { Type = "private" }
    },
    {
      name              = "field-1a"
      cidr_block        = "10.0.2.0/24"
      availability_zone = "${local.region}a"
      zone_type         = "field"
      is_public         = false
      route_table_tags  = { Type = "field" }
    }
  ]

  enable_nat_gateway = true

  field_network_config = {
    wifi_ssid           = "aams-field-network"
    wifi_security_type  = "WPA3"
    private_5g_enabled = true
    robot_network_cidr = "10.0.3.0/24"
    drone_network_cidr = "10.0.4.0/24"
    qos_policy = {
      video = {
        priority    = 1
        bandwidth   = 100
        latency_ms  = 50
      }
      telemetry = {
        priority    = 2
        bandwidth   = 50
        latency_ms  = 20
      }
    }
  }

  flow_logs_config = {
    enabled         = true
    log_destination = "cloudwatch"
    traffic_type    = "ALL"
    retention_days  = 90
    log_format      = "$${version} $${account-id} $${interface-id} $${srcaddr} $${dstaddr} $${srcport} $${dstport} $${protocol} $${packets} $${bytes} $${start} $${end} $${action} $${log-status}"
  }

  tags = local.tags
}

# Kubernetes module for production environment
module "kubernetes" {
  source = "../../modules/kubernetes"

  cluster_name       = "aams-prod"
  kubernetes_version = "1.27"
  gpu_node_count     = 2

  critical_namespaces = [
    "kube-system",
    "monitoring",
    "gpu-resources",
    "storage"
  ]

  kubeconfig_path    = "/etc/kubernetes/admin.conf"
  cluster_endpoint   = "https://k8s.aams-prod.local"
  kubeconfig_content = file("/etc/kubernetes/admin.conf")

  depends_on = [module.network]
}

# Storage module for production environment
module "storage" {
  source = "../../modules/storage"

  storage_class = "local-path"

  # TimescaleDB storage configuration
  timescaledb_storage_size      = "500Gi"
  timescaledb_chunk_interval_days = 1

  # MinIO object storage configuration
  minio_storage_size = "1000Gi"

  # Redis storage configuration
  redis_storage_size = "20Gi"

  # High availability configuration
  replication_enabled    = local.ha_config.replica_count > 1
  replica_count         = local.ha_config.replica_count
  backup_retention_days = local.ha_config.backup_retention_days
  encryption_enabled    = local.ha_config.encryption_enabled

  # Performance configuration
  storage_iops = 3000

  # Monitoring configuration
  metrics_retention_days = local.ha_config.metrics_retention_days

  # Backup configuration
  backup_schedule = "0 0 * * *"

  depends_on = [module.kubernetes]
}

# Output production environment configuration
output "network_config" {
  value = {
    vpc_id              = module.network.vpc_id
    public_subnet_ids   = module.network.public_subnet_ids
    private_subnet_ids  = module.network.private_subnet_ids
    field_network_ids   = module.network.field_network_subnet_ids
    security_groups     = module.network.security_group_ids
  }
  description = "Production network configuration"
}

output "kubernetes_config" {
  value = {
    cluster_endpoint = module.kubernetes.kubernetes_cluster.endpoint
    node_pools      = module.kubernetes.node_pools
  }
  description = "Production Kubernetes configuration"
}

output "storage_config" {
  value = {
    timescaledb = module.storage.timescaledb_storage
    minio       = module.storage.minio_storage
    redis       = module.storage.redis_storage
  }
  description = "Production storage configuration"
}