terraform {
  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
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
resource "aws_instance" "isaac_sim" {
  ami           = var.isaac_sim_ami
  instance_type = "g6e.2xlarge" # Only supported type for Isaac Sim streaming (L40s GPU with NVENC)
  key_name      = var.key_name

  vpc_security_group_ids = [aws_security_group.isaac_sim.id]

  root_block_device {
    volume_size           = var.root_volume_size
    volume_type           = "gp3"
    delete_on_termination = false # Preserve data on instance termination
  }

  tags = {
    Name    = "robotlab-isaac-sim"
    Project = "robotlab"
  }

  # Prevent accidental destruction
  lifecycle {
    prevent_destroy = false # Set to true after initial setup if desired
  }
}
