variable "region" {
  description = "AWS region for deployment"
  type        = string
  default     = "us-west-2"
}

variable "my_ip_cidr" {
  description = "Your IP address in CIDR notation (e.g., 1.2.3.4/32). Get yours with: curl -s ifconfig.me"
  type        = string
}

variable "key_name" {
  description = "Name of an existing AWS SSH key pair"
  type        = string
}

variable "isaac_sim_ami" {
  description = "AMI ID for NVIDIA Isaac Sim Development Workstation. Find in AWS Marketplace."
  type        = string
  # To find the AMI:
  # 1. Go to AWS Marketplace
  # 2. Search "NVIDIA Isaac Sim Development Workstation"
  # 3. Subscribe and get the AMI ID for your region
  # Example for us-west-2: ami-xxxxxxxxxxxxxxxxx
}

variable "root_volume_size" {
  description = "Size of root EBS volume in GB (200+ recommended for Isaac Sim)"
  type        = number
  default     = 200
}
