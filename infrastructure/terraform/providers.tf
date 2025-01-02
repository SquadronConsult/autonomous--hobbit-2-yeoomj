# Configure Terraform settings and required providers
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
    vault = {
      source  = "hashicorp/vault"
      version = "~> 3.20.0"
    }
  }
}

# Configure Kubernetes provider for container orchestration
provider "kubernetes" {
  # Read cluster endpoint from main configuration
  host = data.kubernetes_cluster.main.endpoint

  # Client certificate authentication
  client_certificate     = base64decode(data.kubernetes_cluster.main.client_certificate)
  client_key            = base64decode(data.kubernetes_cluster.main.client_key)
  cluster_ca_certificate = base64decode(data.kubernetes_cluster.main.cluster_ca_certificate)

  # Enable experimental features for Jetson Orin support
  experiments {
    manifest_resource = true
  }
}

# Configure Helm provider for package management
provider "helm" {
  kubernetes {
    host = data.kubernetes_cluster.main.endpoint
    
    client_certificate     = base64decode(data.kubernetes_cluster.main.client_certificate)
    client_key            = base64decode(data.kubernetes_cluster.main.client_key)
    cluster_ca_certificate = base64decode(data.kubernetes_cluster.main.cluster_ca_certificate)
  }

  # Configure environment-specific settings
  registry {
    url = "https://charts.${var.environment}.agricultural-system.io"
    username = data.vault_generic_secret.helm_registry.data["username"]
    password = data.vault_generic_secret.helm_registry.data["password"]
  }
}

# Configure HashiCorp Vault provider for secrets management
provider "vault" {
  # Vault server address based on environment
  address = "https://vault.${var.environment}.agricultural-system.io"
  
  # Authentication using Kubernetes service account
  auth_login {
    path = "auth/kubernetes/login"
    
    parameters = {
      role = "agricultural-system"
      jwt  = file("/var/run/secrets/kubernetes.io/serviceaccount/token")
    }
  }

  # Configure timeout and retry settings
  max_retries = 3
  timeout     = "30s"
}

# Data source for Kubernetes cluster configuration
data "kubernetes_cluster" "main" {
  name = "agricultural-system-${var.environment}-${var.region}"
}

# Data source for Helm registry credentials from Vault
data "vault_generic_secret" "helm_registry" {
  path = "secret/agricultural-system/${var.environment}/helm-registry"
}