# Cluster identification and naming
variable "cluster_name" {
  type        = string
  description = "Name of the Kubernetes cluster for agricultural management system"
  
  validation {
    condition     = can(regex("^[a-z0-9][a-z0-9-]*[a-z0-9]$", var.cluster_name))
    error_message = "Cluster name must be DNS compatible, containing only lowercase alphanumeric characters and hyphens."
  }
}

# Kubernetes version configuration
variable "kubernetes_version" {
  type        = string
  description = "Version of Kubernetes to use for the cluster, must be compatible with Jetson Orin"
  default     = "1.26.0"
}

# Node pool configurations
variable "node_pools" {
  type = map(object({
    name                = string
    instance_type      = string
    min_size          = number
    max_size          = number
    desired_size      = number
    gpu_enabled       = bool
    gpu_type          = string
    labels            = map(string)
    taints            = list(object({
      key    = string
      value  = string
      effect = string
    }))
    subnet_ids        = list(string)
  }))
  description = "Configuration for Kubernetes node pools including GPU support for DeepStream and TAO workloads"
}

# Container runtime configuration
variable "container_runtime" {
  type = object({
    type            = string
    version         = string
    nvidia_version  = string
    runtime_config  = map(string)
  })
  description = "NVIDIA container runtime configuration for GPU support"
  default = {
    type            = "containerd"
    version         = "1.6.20"
    nvidia_version  = "3.12.0"
    runtime_config  = {
      default_runtime = "nvidia"
      nvidia_runtime  = "/usr/bin/nvidia-container-runtime"
    }
  }
}

# Monitoring configuration
variable "monitoring_config" {
  type = object({
    prometheus_enabled = bool
    grafana_enabled   = bool
    retention_days    = number
    scrape_interval   = string
    alert_config      = map(string)
  })
  description = "Prometheus and Grafana monitoring configuration for cluster observability"
  default = {
    prometheus_enabled = true
    grafana_enabled   = true
    retention_days    = 30
    scrape_interval   = "15s"
    alert_config = {
      cpu_threshold    = "80"
      memory_threshold = "80"
      gpu_threshold    = "85"
    }
  }
}

# Resource limits and requests
variable "resource_limits" {
  type = map(object({
    cpu_request      = string
    cpu_limit        = string
    memory_request   = string
    memory_limit     = string
    gpu_request      = string
    gpu_limit        = string
  }))
  description = "Resource limits and requests for containers including GPU allocation"
  default = {
    deepstream = {
      cpu_request    = "4"
      cpu_limit      = "8"
      memory_request = "8Gi"
      memory_limit   = "16Gi"
      gpu_request    = "1"
      gpu_limit      = "1"
    }
    tao = {
      cpu_request    = "4"
      cpu_limit      = "8"
      memory_request = "16Gi"
      memory_limit   = "32Gi"
      gpu_request    = "1"
      gpu_limit      = "1"
    }
    ros = {
      cpu_request    = "2"
      cpu_limit      = "4"
      memory_request = "4Gi"
      memory_limit   = "8Gi"
      gpu_request    = "0"
      gpu_limit      = "0"
    }
  }
}

# Network configuration
variable "network_config" {
  type = object({
    pod_cidr         = string
    service_cidr     = string
    vpc_id           = string
    subnet_ids       = list(string)
    security_groups  = list(string)
    cni_provider     = string
    cni_version      = string
  })
  description = "Network configuration including CIDR ranges and VPC integration"
  
  validation {
    condition     = can(regex("^([0-9]{1,3}\\.){3}[0-9]{1,3}/[0-9]{1,2}$", var.network_config.pod_cidr))
    error_message = "Pod CIDR must be a valid IPv4 CIDR block."
  }
}

# High availability configuration
variable "high_availability" {
  type = object({
    control_plane_count = number
    multi_az           = bool
    etcd_backup        = object({
      enabled          = bool
      retention_days   = number
      schedule         = string
    })
    node_group_config  = object({
      max_unavailable  = number
      max_surge       = number
    })
  })
  description = "High availability configuration for 99.9% uptime"
  default = {
    control_plane_count = 3
    multi_az           = true
    etcd_backup = {
      enabled        = true
      retention_days = 7
      schedule       = "0 0 * * *"
    }
    node_group_config = {
      max_unavailable = 1
      max_surge      = 1
    }
  }
}