terraform {
  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
    archive = {
      source  = "hashicorp/archive"
      version = "~> 2.0"
    }
  }
}

provider "aws" {
  region = var.region
}

# Security Group for Isaac Sim WebRTC streaming
resource "aws_security_group" "isaac_sim" {
  name        = "isaac-sim-streaming"
  description = "Isaac Sim WebRTC streaming + SSH + DCV"

  # SSH
  ingress {
    description = "SSH"
    from_port   = 22
    to_port     = 22
    protocol    = "tcp"
    cidr_blocks = [var.my_ip_cidr]
  }

  # NICE DCV (fallback remote desktop)
  ingress {
    description = "NICE DCV"
    from_port   = 8443
    to_port     = 8443
    protocol    = "tcp"
    cidr_blocks = [var.my_ip_cidr]
  }

  # WebRTC signaling (TCP)
  ingress {
    description = "WebRTC signaling"
    from_port   = 49100
    to_port     = 49100
    protocol    = "tcp"
    cidr_blocks = [var.my_ip_cidr]
  }

  # WebRTC media (UDP)
  ingress {
    description = "WebRTC media"
    from_port   = 47998
    to_port     = 47998
    protocol    = "udp"
    cidr_blocks = [var.my_ip_cidr]
  }

  # All outbound traffic
  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = ["0.0.0.0/0"]
  }

  tags = {
    Name    = "isaac-sim-streaming"
    Project = "robotlab"
  }
}

# EC2 Instance - NVIDIA Isaac Sim Development Workstation
# Supports both on-demand and spot instances via use_spot variable
resource "aws_instance" "isaac_sim" {
  ami                  = var.isaac_sim_ami
  instance_type        = var.instance_type
  key_name             = var.key_name
  iam_instance_profile = aws_iam_instance_profile.isaac_sim.name

  vpc_security_group_ids = [aws_security_group.isaac_sim.id]

  # Bootstrap Claude Code on first boot
  user_data = templatefile("${path.module}/user-data.sh.tpl", {
    region          = var.region
    github_repo_url = var.github_repo_url
  })

  # Spot instance configuration (only applied when use_spot = true)
  dynamic "instance_market_options" {
    for_each = var.use_spot ? [1] : []
    content {
      market_type = "spot"
      spot_options {
        instance_interruption_behavior = "stop"             # Stop (not terminate) on interruption to preserve EBS
        spot_instance_type             = "persistent"       # Re-request spot when stopped
        max_price                      = var.spot_max_price # Optional price cap, defaults to on-demand price
      }
    }
  }

  root_block_device {
    volume_size           = var.root_volume_size
    volume_type           = "gp3"
    delete_on_termination = false # Preserve data on instance termination
  }

  tags = {
    Name    = "robotlab-isaac-sim${var.use_spot ? "-spot" : ""}"
    Project = "robotlab"
  }

  # Prevent accidental destruction
  lifecycle {
    prevent_destroy = false # Set to true after initial setup if desired
  }
}
