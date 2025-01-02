# Provider and terraform configuration
terraform {
  required_version = ">= 1.5.0"
  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
  }
}

# Core VPC Resource
resource "aws_vpc" "main" {
  cidr_block           = var.vpc_cidr
  enable_dns_hostnames = true
  enable_dns_support   = true
  
  tags = merge(var.tags, {
    Name = "agricultural-management-vpc"
  })
}

# Internet Gateway for public subnets
resource "aws_internet_gateway" "main" {
  vpc_id = aws_vpc.main.id
  
  tags = merge(var.tags, {
    Name = "agricultural-management-igw"
  })
}

# Subnet creation for different network zones
resource "aws_subnet" "subnets" {
  for_each = { for idx, subnet in var.subnet_configuration : subnet.name => subnet }

  vpc_id            = aws_vpc.main.id
  cidr_block        = each.value.cidr_block
  availability_zone = each.value.availability_zone
  
  # Enable auto-assign public IP for public subnets
  map_public_ip_on_launch = each.value.is_public

  tags = merge(var.tags, {
    Name = "ams-${each.value.zone_type}-${each.value.name}"
    Zone = each.value.zone_type
  })
}

# NAT Gateway for private subnet internet access
resource "aws_nat_gateway" "main" {
  count = var.enable_nat_gateway ? 1 : 0

  allocation_id = aws_eip.nat[0].id
  subnet_id     = [for s in aws_subnet.subnets : s.id if s.tags["Zone"] == "public"][0]
  
  tags = merge(var.tags, {
    Name = "agricultural-management-nat"
  })

  depends_on = [aws_internet_gateway.main]
}

# Elastic IP for NAT Gateway
resource "aws_eip" "nat" {
  count = var.enable_nat_gateway ? 1 : 0
  domain = "vpc"

  tags = merge(var.tags, {
    Name = "agricultural-management-nat-eip"
  })
}

# Route Tables
resource "aws_route_table" "public" {
  vpc_id = aws_vpc.main.id

  route {
    cidr_block = "0.0.0.0/0"
    gateway_id = aws_internet_gateway.main.id
  }

  tags = merge(var.tags, {
    Name = "ams-public-rt"
  })
}

resource "aws_route_table" "private" {
  vpc_id = aws_vpc.main.id

  dynamic "route" {
    for_each = var.enable_nat_gateway ? [1] : []
    content {
      cidr_block = "0.0.0.0/0"
      nat_gateway_id = aws_nat_gateway.main[0].id
    }
  }

  tags = merge(var.tags, {
    Name = "ams-private-rt"
  })
}

# Route Table Associations
resource "aws_route_table_association" "subnet_associations" {
  for_each = aws_subnet.subnets

  subnet_id      = each.value.id
  route_table_id = each.value.tags["Zone"] == "public" ? aws_route_table.public.id : aws_route_table.private.id
}

# Security Groups
resource "aws_security_group" "security_groups" {
  for_each = var.network_security_groups

  name        = each.value.name
  description = each.value.description
  vpc_id      = aws_vpc.main.id

  dynamic "ingress" {
    for_each = each.value.ingress_rules
    content {
      description      = ingress.value.description
      from_port       = ingress.value.from_port
      to_port         = ingress.value.to_port
      protocol        = ingress.value.protocol
      cidr_blocks     = ingress.value.cidr_blocks
      security_groups = ingress.value.security_groups
    }
  }

  dynamic "egress" {
    for_each = each.value.egress_rules
    content {
      description      = egress.value.description
      from_port       = egress.value.from_port
      to_port         = egress.value.to_port
      protocol        = egress.value.protocol
      cidr_blocks     = egress.value.cidr_blocks
      security_groups = egress.value.security_groups
    }
  }

  tags = merge(var.tags, {
    Name = "ams-${each.key}-sg"
  })
}

# Network ACLs
resource "aws_network_acl" "network_acls" {
  for_each = var.network_acls

  vpc_id     = aws_vpc.main.id
  subnet_ids = each.value.subnet_ids

  dynamic "ingress" {
    for_each = each.value.ingress
    content {
      rule_number = ingress.value.rule_number
      protocol    = ingress.value.protocol
      action      = ingress.value.action
      cidr_block  = ingress.value.cidr_block
      from_port   = ingress.value.from_port
      to_port     = ingress.value.to_port
    }
  }

  dynamic "egress" {
    for_each = each.value.egress
    content {
      rule_number = egress.value.rule_number
      protocol    = egress.value.protocol
      action      = egress.value.action
      cidr_block  = egress.value.cidr_block
      from_port   = egress.value.from_port
      to_port     = egress.value.to_port
    }
  }

  tags = merge(var.tags, {
    Name = "ams-${each.key}-nacl"
  })
}

# VPC Flow Logs
resource "aws_flow_log" "main" {
  count = var.flow_logs_config.enabled ? 1 : 0

  iam_role_arn    = aws_iam_role.flow_logs[0].arn
  log_destination = aws_cloudwatch_log_group.flow_logs[0].arn
  traffic_type    = var.flow_logs_config.traffic_type
  vpc_id          = aws_vpc.main.id

  tags = merge(var.tags, {
    Name = "ams-vpc-flow-logs"
  })
}

# CloudWatch Log Group for Flow Logs
resource "aws_cloudwatch_log_group" "flow_logs" {
  count = var.flow_logs_config.enabled ? 1 : 0

  name              = "/aws/vpc/agricultural-management-flow-logs"
  retention_in_days = var.flow_logs_config.retention_days

  tags = merge(var.tags, {
    Name = "ams-vpc-flow-logs"
  })
}

# IAM Role for Flow Logs
resource "aws_iam_role" "flow_logs" {
  count = var.flow_logs_config.enabled ? 1 : 0

  name = "agricultural-management-flow-logs-role"

  assume_role_policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Action = "sts:AssumeRole"
        Effect = "Allow"
        Principal = {
          Service = "vpc-flow-logs.amazonaws.com"
        }
      }
    ]
  })

  tags = merge(var.tags, {
    Name = "ams-flow-logs-role"
  })
}

# IAM Role Policy for Flow Logs
resource "aws_iam_role_policy" "flow_logs" {
  count = var.flow_logs_config.enabled ? 1 : 0

  name = "agricultural-management-flow-logs-policy"
  role = aws_iam_role.flow_logs[0].id

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Action = [
          "logs:CreateLogGroup",
          "logs:CreateLogStream",
          "logs:PutLogEvents",
          "logs:DescribeLogGroups",
          "logs:DescribeLogStreams"
        ]
        Effect = "Allow"
        Resource = "*"
      }
    ]
  })
}