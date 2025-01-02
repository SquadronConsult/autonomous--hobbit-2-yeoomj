# Core VPC outputs
output "vpc_id" {
  description = "The ID of the VPC"
  value       = aws_vpc.main.id
}

output "vpc_cidr_block" {
  description = "The CIDR block of the VPC"
  value       = aws_vpc.main.cidr_block
}

# Subnet outputs organized by zone type
output "public_subnet_ids" {
  description = "List of public subnet IDs for load balancer and ingress configuration"
  value       = [for subnet in aws_subnet.subnets : subnet.id if subnet.tags["Zone"] == "public"]
}

output "private_subnet_ids" {
  description = "List of private subnet IDs for container and database deployment"
  value       = [for subnet in aws_subnet.subnets : subnet.id if subnet.tags["Zone"] == "private"]
}

output "edge_subnet_ids" {
  description = "List of edge computing subnet IDs for Jetson Orin deployment"
  value       = [for subnet in aws_subnet.subnets : subnet.id if subnet.tags["Zone"] == "edge"]
}

output "field_network_subnet_ids" {
  description = "List of field network subnet IDs for drone and robot communication"
  value       = [for subnet in aws_subnet.subnets : subnet.id if subnet.tags["Zone"] == "field"]
}

# Security group outputs
output "security_group_ids" {
  description = "Map of security group names to their IDs"
  value       = { for k, v in aws_security_group.security_groups : k => v.id }
}

# Network ACL outputs
output "network_acl_ids" {
  description = "Map of network ACL names to their IDs"
  value       = { for k, v in aws_network_acl.network_acls : k => v.id }
}

# NAT Gateway outputs
output "nat_gateway_ips" {
  description = "List of NAT Gateway public IPs for private subnet internet access"
  value       = var.enable_nat_gateway ? [aws_eip.nat[0].public_ip] : []
}

# Route table outputs
output "public_route_table_id" {
  description = "ID of the public route table"
  value       = aws_route_table.public.id
}

output "private_route_table_id" {
  description = "ID of the private route table"
  value       = aws_route_table.private.id
}

# Security zone mappings
output "security_zone_mappings" {
  description = "Mapping of security zones to their network resources"
  value = {
    for zone_name, zone in var.security_zones : zone_name => {
      subnets = [
        for subnet in aws_subnet.subnets : {
          id         = subnet.id
          cidr_block = subnet.cidr_block
          az         = subnet.availability_zone
        } if subnet.tags["Zone"] == zone_name
      ]
      security_groups = [
        for sg_key, sg in aws_security_group.security_groups : {
          id          = sg.id
          name        = sg.name
          description = sg.description
        } if contains([for rule in sg.ingress : rule.cidr_blocks], zone.cidr_blocks)
      ]
      network_acls = [
        for acl_key, acl in aws_network_acl.network_acls : {
          id   = acl.id
          name = acl.tags["Name"]
        } if contains(acl.subnet_ids, [for s in aws_subnet.subnets : s.id if s.tags["Zone"] == zone_name][0])
      ]
    }
  }
}

# Flow logs outputs
output "flow_logs_enabled" {
  description = "Indicates whether VPC flow logs are enabled"
  value       = var.flow_logs_config.enabled
}

output "flow_logs_group_name" {
  description = "Name of the CloudWatch log group for VPC flow logs"
  value       = var.flow_logs_config.enabled ? aws_cloudwatch_log_group.flow_logs[0].name : null
}

# Network metadata
output "network_metadata" {
  description = "Metadata about the network configuration"
  value = {
    availability_zones = distinct([for subnet in aws_subnet.subnets : subnet.availability_zone])
    total_subnets     = length(aws_subnet.subnets)
    security_groups   = length(aws_security_group.security_groups)
    network_acls      = length(aws_network_acl.network_acls)
    nat_gateways      = var.enable_nat_gateway ? 1 : 0
  }
}