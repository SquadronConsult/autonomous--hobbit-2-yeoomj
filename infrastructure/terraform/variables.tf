# Core deployment environment variable
variable "environment" {
  type        = string
  description = "Deployment environment (dev/prod) for the Agricultural Management System"
  validation {
    condition     = contains(["dev", "prod"], var.environment)
    error_message = "Environment must be either 'dev' or 'prod'."
  }
}

# Geographic region for edge deployment
variable "region" {
  type        = string
  description = "Geographic region for edge deployment and resource allocation"
}

# Kubernetes version configuration
variable "kubernetes_version" {
  type        = string
  description = "Kubernetes version for container orchestration platform"
  default     = "1.26.0"
  validation {
    condition     = can(regex("^1\\.(2[4-6]|27)\\.\\d+$", var.kubernetes_version))
    error_message = "Kubernetes version must be between 1.24.0 and 1.27.x."
  }
}

# Node pool configuration for Jetson Orin clusters
variable "node_pool_config" {
  type = map(object({
    instance_type    = string
    min_size        = number
    max_size        = number
    disk_size       = number
    gpu_enabled     = bool
    labels          = map(string)
    taints          = list(object({
      key    = string
      value  = string
      effect = string
    }))
  }))
  description = "Configuration for Jetson Orin node pools including compute, GPU, and scaling parameters"

  validation {
    condition     = alltrue([for k, v in var.node_pool_config : v.disk_size >= 100])
    error_message = "Minimum disk size for each node pool must be at least 100GB."
  }
}

# Storage configuration for the system
variable "storage_config" {
  type = object({
    timescaledb = object({
      storage_size = number
      replica_count = number
      backup_retention_days = number
    })
    minio = object({
      storage_size = number
      replica_count = number
      bucket_configuration = map(string)
    })
    redis = object({
      memory_size = number
      replica_count = number
    })
  })
  description = "Storage configuration for databases and object storage components"
}

# Monitoring configuration
variable "monitoring_enabled" {
  type        = bool
  description = "Flag to enable deployment of monitoring stack (Prometheus/Grafana)"
  default     = true
}

# High availability configuration
variable "high_availability" {
  type        = bool
  description = "Enable high availability configuration for critical components"
  default     = true
}

# Container runtime configuration
variable "container_runtime" {
  type        = string
  description = "Container runtime configuration for NVIDIA GPU support"
  default     = "nvidia-container-runtime"
  validation {
    condition     = contains(["nvidia-container-runtime", "containerd"], var.container_runtime)
    error_message = "Container runtime must be either 'nvidia-container-runtime' or 'containerd'."
  }
}

# Resource tagging
variable "tags" {
  type        = map(string)
  description = "Tags to be applied to all infrastructure resources"
  default     = {
    "Project"     = "AgricultureManagementSystem"
    "Managed-by"  = "Terraform"
    "Environment" = "production"
  }
}

# Network configuration
variable "network_config" {
  type = object({
    vpc_cidr = string
    subnet_configuration = map(object({
      cidr_block = string
      zone       = string
      public     = bool
    }))
    enable_vpn = bool
    vpn_configuration = object({
      bandwidth = string
      protocol  = string
    })
  })
  description = "Network configuration including VPC, subnets, and VPN settings"
}

# Security configuration
variable "security_config" {
  type = object({
    enable_encryption = bool
    kms_key_rotation = number
    ssl_certificate = object({
      provider = string
      email    = string
    })
    network_policies_enabled = bool
  })
  description = "Security configuration for encryption, certificates, and network policies"
  default = {
    enable_encryption = true
    kms_key_rotation = 90
    ssl_certificate = {
      provider = "letsencrypt"
      email    = ""
    }
    network_policies_enabled = true
  }
}

# Backup configuration
variable "backup_config" {
  type = object({
    enabled = bool
    retention_days = number
    schedule = string
    storage_class = string
  })
  description = "Configuration for system backups"
  default = {
    enabled = true
    retention_days = 30
    schedule = "0 2 * * *"
    storage_class = "standard"
  }
}