# Core cluster information outputs
output "cluster_id" {
  description = "Unique identifier of the Kubernetes cluster"
  value       = kubernetes_namespace.system.id
}

output "cluster_endpoint" {
  description = "API endpoint URL for the Kubernetes cluster"
  value       = var.cluster_endpoint
  sensitive   = true
}

output "kubeconfig" {
  description = "Kubeconfig file content for cluster access"
  value       = local_file.kubeconfig.content
  sensitive   = true
}

# GPU node pool information
output "gpu_node_pools" {
  description = "Details of GPU-enabled node pools including specifications and status"
  value = {
    enabled     = true
    node_count  = kubernetes_node_pool.gpu_nodes.spec[0].replicas
    node_taints = kubernetes_node_pool.gpu_nodes.spec[0].template[0].spec[0].toleration
    node_labels = kubernetes_node_pool.gpu_nodes.spec[0].template[0].spec[0].node_selector
    gpu_spec = {
      device_plugin = kubernetes_daemonset.nvidia_device_plugin.metadata[0].name
      namespace     = kubernetes_namespace.gpu_resources.metadata[0].name
    }
  }
}

# Monitoring and observability endpoints
output "monitoring_endpoint" {
  description = "Endpoint for accessing Prometheus/Grafana monitoring stack"
  value = {
    prometheus = {
      namespace = kubernetes_namespace.monitoring.metadata[0].name
      operator  = helm_release.prometheus_operator.name
      endpoint  = "${var.cluster_endpoint}/api/v1/namespaces/${kubernetes_namespace.monitoring.metadata[0].name}/services/prometheus-operated:9090/proxy"
    }
    grafana = {
      endpoint = "${var.cluster_endpoint}/api/v1/namespaces/${kubernetes_namespace.monitoring.metadata[0].name}/services/prometheus-operator-grafana:80/proxy"
    }
  }
  sensitive = true
}

# Storage configuration
output "storage_classes" {
  description = "Available storage classes for persistent volumes"
  value = {
    local_storage = {
      name        = kubernetes_storage_class.local_storage.metadata[0].name
      provisioner = kubernetes_storage_class.local_storage.storage_provisioner
    }
  }
}

# High availability configuration
output "high_availability_config" {
  description = "High availability settings and configurations"
  value = {
    pod_disruption_budgets = {
      for ns in var.critical_namespaces : ns => {
        name          = kubernetes_pod_disruption_budget.critical_workloads[ns].metadata[0].name
        min_available = kubernetes_pod_disruption_budget.critical_workloads[ns].spec[0].min_available
      }
    }
    resource_quotas = {
      for ns in var.critical_namespaces : ns => {
        name = kubernetes_resource_quota.namespace_quotas[ns].metadata[0].name
        spec = kubernetes_resource_quota.namespace_quotas[ns].spec[0].hard
      }
    }
  }
}

# Network policies
output "network_policies" {
  description = "Network policies applied to critical namespaces"
  value = {
    for ns in var.critical_namespaces : ns => {
      name          = kubernetes_network_policy.default_deny[ns].metadata[0].name
      policy_types  = kubernetes_network_policy.default_deny[ns].spec[0].policy_types
    }
  }
}

# Cluster metadata
output "cluster_metadata" {
  description = "Additional cluster metadata and configuration details"
  value = {
    kubernetes_version = var.kubernetes_version
    critical_namespaces = var.critical_namespaces
    system_namespace = kubernetes_namespace.system.metadata[0].name
    monitoring_enabled = true
    gpu_support_enabled = true
  }
}