# Configure Terraform and required providers
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

# Kubernetes provider configuration
provider "kubernetes" {
  config_path = var.kubeconfig_path
}

provider "helm" {
  kubernetes {
    config_path = var.kubeconfig_path
  }
}

# Core Kubernetes cluster configuration
resource "kubernetes_namespace" "system" {
  metadata {
    name = "kube-system"
    labels = {
      "app.kubernetes.io/managed-by" = "terraform"
      "app.kubernetes.io/part-of"    = var.cluster_name
    }
  }
}

# GPU support configuration
resource "kubernetes_namespace" "gpu_resources" {
  metadata {
    name = "gpu-resources"
    labels = {
      "app.kubernetes.io/managed-by" = "terraform"
      "nvidia.com/gpu-supported"     = "true"
    }
  }
}

# NVIDIA device plugin deployment
resource "kubernetes_daemonset" "nvidia_device_plugin" {
  metadata {
    name      = "nvidia-device-plugin"
    namespace = kubernetes_namespace.gpu_resources.metadata[0].name
  }

  spec {
    selector {
      match_labels = {
        name = "nvidia-device-plugin"
      }
    }

    template {
      metadata {
        labels = {
          name = "nvidia-device-plugin"
        }
      }

      spec {
        container {
          name  = "nvidia-device-plugin-ctr"
          image = "nvcr.io/nvidia/k8s-device-plugin:v0.14.1" # Version compatible with Jetson Orin
          
          security_context {
            privileged = true
          }

          volume_mount {
            name       = "device-plugin"
            mount_path = "/var/lib/kubelet/device-plugins"
          }
        }

        volume {
          name = "device-plugin"
          host_path {
            path = "/var/lib/kubelet/device-plugins"
          }
        }

        toleration {
          key      = "nvidia.com/gpu"
          operator = "Exists"
          effect   = "NoSchedule"
        }
      }
    }
  }
}

# Node pool configuration for GPU workloads
resource "kubernetes_node_pool" "gpu_nodes" {
  metadata {
    name = "gpu-pool"
  }

  spec {
    replicas = var.gpu_node_count

    template {
      spec {
        node_selector = {
          "nvidia.com/gpu-enabled" = "true"
        }

        toleration {
          key      = "nvidia.com/gpu"
          operator = "Exists"
          effect   = "NoSchedule"
        }

        container {
          resources {
            limits = {
              "nvidia.com/gpu" = "1"
            }
          }
        }
      }
    }
  }
}

# Monitoring namespace and resources
resource "kubernetes_namespace" "monitoring" {
  metadata {
    name = "monitoring"
    labels = {
      "app.kubernetes.io/managed-by" = "terraform"
      "monitoring.grafana.com/cluster" = "true"
    }
  }
}

# Prometheus Operator deployment
resource "helm_release" "prometheus_operator" {
  name       = "prometheus-operator"
  namespace  = kubernetes_namespace.monitoring.metadata[0].name
  repository = "https://prometheus-community.github.io/helm-charts"
  chart      = "kube-prometheus-stack"
  version    = "45.7.1"

  set {
    name  = "prometheus.prometheusSpec.retention"
    value = "15d"
  }

  set {
    name  = "prometheus.prometheusSpec.resources.requests.cpu"
    value = "1000m"
  }

  set {
    name  = "prometheus.prometheusSpec.resources.requests.memory"
    value = "2Gi"
  }

  set {
    name  = "prometheus.prometheusSpec.storageSpec.volumeClaimTemplate.spec.storageClassName"
    value = "local-storage"
  }

  set {
    name  = "grafana.persistence.enabled"
    value = "true"
  }
}

# High availability configuration
resource "kubernetes_pod_disruption_budget" "critical_workloads" {
  for_each = toset(var.critical_namespaces)

  metadata {
    name      = "pdb-${each.value}"
    namespace = each.value
  }

  spec {
    min_available = "50%"
    selector {
      match_labels = {
        "app.kubernetes.io/part-of" = var.cluster_name
      }
    }
  }
}

# Resource quotas for namespaces
resource "kubernetes_resource_quota" "namespace_quotas" {
  for_each = toset(var.critical_namespaces)

  metadata {
    name      = "quota-${each.value}"
    namespace = each.value
  }

  spec {
    hard = {
      "requests.cpu"    = "4"
      "requests.memory" = "8Gi"
      "limits.cpu"      = "8"
      "limits.memory"   = "16Gi"
      "nvidia.com/gpu"  = "1"
    }
  }
}

# Network policies for secure communication
resource "kubernetes_network_policy" "default_deny" {
  for_each = toset(var.critical_namespaces)

  metadata {
    name      = "default-deny"
    namespace = each.value
  }

  spec {
    pod_selector {}
    policy_types = ["Ingress", "Egress"]
  }
}

# Local storage class for edge deployment
resource "kubernetes_storage_class" "local_storage" {
  metadata {
    name = "local-storage"
  }

  storage_provisioner = "kubernetes.io/no-provisioner"
  volume_binding_mode = "WaitForFirstConsumer"

  parameters = {
    fsType = "ext4"
  }
}

# Output the cluster configuration
resource "local_file" "kubeconfig" {
  content  = var.kubeconfig_content
  filename = "${path.module}/kubeconfig"
}

# Export cluster information
output "kubernetes_cluster" {
  value = {
    id        = kubernetes_namespace.system.id
    endpoint  = var.cluster_endpoint
    version   = var.kubernetes_version
  }
  description = "Kubernetes cluster configuration details"
}

output "node_pools" {
  value = {
    gpu_enabled  = true
    node_count   = var.gpu_node_count
    node_taints  = kubernetes_node_pool.gpu_nodes.spec[0].template[0].spec[0].toleration
    node_labels  = kubernetes_node_pool.gpu_nodes.spec[0].template[0].spec[0].node_selector
  }
  description = "Node pool configuration details"
}