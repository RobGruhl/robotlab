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

variable "instance_type" {
  description = <<-EOT
    EC2 instance type. Options:
    - g6e.2xlarge: L40s GPU, best for streaming/interactive ($1.50/hr on-demand)
    - g5.xlarge:   A10G GPU, good for training + occasional streaming ($1.00/hr on-demand)
    - g4dn.xlarge: T4 GPU, budget option for lighter workloads ($0.50/hr on-demand)
  EOT
  type        = string
  default     = "g5.xlarge" # Good balance of cost and capability for dev/training
}

variable "use_spot" {
  description = "Use spot instances for ~70% cost savings. May be interrupted with 2 min warning."
  type        = bool
  default     = true # Default to spot for dev/training workloads
}

variable "spot_max_price" {
  description = "Max hourly price for spot instance. Leave empty to use on-demand price as cap."
  type        = string
  default     = "" # Empty = on-demand price (no cap)
}

variable "persist_volume_size" {
  description = "Size of persistent EBS volume for Claude Code state, repo, and rosbags (GB)"
  type        = number
  default     = 100
}

variable "github_repo_url" {
  description = "GitHub repository URL for robotlab (SSH or HTTPS)"
  type        = string
  default     = "https://github.com/YOUR_USERNAME/robotlab.git"
}
