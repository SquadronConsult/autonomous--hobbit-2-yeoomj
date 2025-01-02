# Core VPC CIDR configuration
variable "vpc_cidr" {
  type        = string
  description = "The CIDR block for the VPC network infrastructure"
  default     = "10.0.0.0/16"

  validation {
    condition     = can(regex("^([0-9]{1,3}\\.){3}[0-9]{1,3}/[0-9]{1,2}$", var.vpc_cidr))
    error_message = "VPC CIDR must be a valid IPv4 CIDR block."
  }
}

# Subnet configuration for different network zones
variable "subnet_configuration" {
  type = list(object({
    name               = string
    cidr_block         = string
    availability_zone  = string
    zone_type         = string
    is_public         = bool
    route_table_tags  = map(string)
  }))
  description = "Configuration for VPC subnets including public and private zones"

  validation {
    condition     = length([for s in var.subnet_configuration : s if s.zone_type == "public" || s.zone_type == "private" || s.zone_type == "field"]) == length(var.subnet_configuration)
    error_message = "Zone type must be one of: public, private, or field."
  }
}

# NAT Gateway configuration
variable "enable_nat_gateway" {
  type        = bool
  description = "Enable NAT Gateway for private subnet internet access"
  default     = true
}

# Security group definitions
variable "network_security_groups" {
  type = map(object({
    name        = string
    description = string
    vpc_id      = string
    ingress_rules = list(object({
      description      = string
      from_port       = number
      to_port         = number
      protocol        = string
      cidr_blocks     = list(string)
      security_groups = list(string)
    }))
    egress_rules = list(object({
      description      = string
      from_port       = number
      to_port         = number
      protocol        = string
      cidr_blocks     = list(string)
      security_groups = list(string)
    }))
  }))
  description = "Security group configurations for network access control"

  validation {
    condition     = alltrue([for sg in var.network_security_groups : length(sg.name) > 0])
    error_message = "Security group name cannot be empty."
  }
}

# Network ACL configurations
variable "network_acls" {
  type = map(object({
    name   = string
    vpc_id = string
    subnet_ids = list(string)
    ingress = list(object({
      rule_number = number
      protocol    = string
      action      = string
      cidr_block  = string
      from_port   = number
      to_port     = number
    }))
    egress = list(object({
      rule_number = number
      protocol    = string
      action      = string
      cidr_block  = string
      from_port   = number
      to_port     = number
    }))
  }))
  description = "Network ACL rules for subnet-level traffic control"

  validation {
    condition     = alltrue([for acl in var.network_acls : alltrue([for rule in acl.ingress : rule.rule_number >= 100 && rule.rule_number <= 32766])])
    error_message = "ACL rule numbers must be between 100 and 32766."
  }
}

# Security zone definitions
variable "security_zones" {
  type = map(object({
    name        = string
    description = string
    cidr_blocks = list(string)
    rules       = map(object({
      type        = string
      ports       = list(number)
      protocols   = list(string)
      sources     = list(string)
    }))
  }))
  description = "Security zone definitions for network segmentation"
  default = {
    public = {
      name        = "public"
      description = "Public facing zone for load balancers and bastion hosts"
      cidr_blocks = ["10.0.0.0/24"]
      rules       = {}
    }
    private = {
      name        = "private"
      description = "Private zone for application containers and services"
      cidr_blocks = ["10.0.1.0/24"]
      rules       = {}
    }
    field = {
      name        = "field"
      description = "Field network zone for drone and robot communication"
      cidr_blocks = ["10.0.2.0/24"]
      rules       = {}
    }
  }
}

# Field network configuration
variable "field_network_config" {
  type = object({
    wifi_ssid                = string
    wifi_security_type       = string
    private_5g_enabled      = bool
    robot_network_cidr      = string
    drone_network_cidr      = string
    qos_policy             = map(object({
      priority    = number
      bandwidth   = number
      latency_ms  = number
    }))
  })
  description = "Configuration for field network infrastructure supporting drone and robot communication"
}

# VPC Flow Logs configuration
variable "flow_logs_config" {
  type = object({
    enabled           = bool
    log_destination   = string
    traffic_type      = string
    retention_days    = number
    log_format        = string
  })
  description = "Configuration for VPC flow logs"
  default = {
    enabled         = true
    log_destination = "cloudwatch"
    traffic_type    = "ALL"
    retention_days  = 30
    log_format      = "$${version} $${account-id} $${interface-id} $${srcaddr} $${dstaddr} $${srcport} $${dstport} $${protocol} $${packets} $${bytes} $${start} $${end} $${action} $${log-status}"
  }
}

# Resource tagging
variable "tags" {
  type        = map(string)
  description = "Tags to be applied to all resources"
  default = {
    Environment = "production"
    Project     = "agricultural-management-system"
    Terraform   = "true"
  }
}