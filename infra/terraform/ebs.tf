# Persistent EBS volume for Claude Code state, repo, and rosbags
# This survives instance stop/start and even termination

resource "aws_ebs_volume" "persist" {
  availability_zone = "${var.region}a"
  size              = var.persist_volume_size
  type              = "gp3"
  throughput        = 250  # MB/s, good for git operations
  iops              = 4000 # Sufficient for dev workloads

  tags = {
    Name    = "robotlab-persist"
    Project = "robotlab"
  }

  lifecycle {
    prevent_destroy = true # Critical: never accidentally delete this
  }
}

# Attach persistent volume to instance
resource "aws_volume_attachment" "persist" {
  device_name = "/dev/xvdf"
  volume_id   = aws_ebs_volume.persist.id
  instance_id = aws_instance.isaac_sim.id

  # Don't destroy attachment on terraform destroy (volume persists)
  skip_destroy = true
}

# IAM role for instance to access Secrets Manager
resource "aws_iam_role" "isaac_sim" {
  name = "robotlab-isaac-sim-role"

  assume_role_policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Action = "sts:AssumeRole"
        Effect = "Allow"
        Principal = {
          Service = "ec2.amazonaws.com"
        }
      }
    ]
  })

  tags = {
    Project = "robotlab"
  }
}

# Policy to allow reading Claude Code API key from Secrets Manager
resource "aws_iam_role_policy" "secrets_access" {
  name = "robotlab-secrets-access"
  role = aws_iam_role.isaac_sim.id

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect = "Allow"
        Action = [
          "secretsmanager:GetSecretValue"
        ]
        Resource = [
          "arn:aws:secretsmanager:${var.region}:*:secret:robotlab/*"
        ]
      }
    ]
  })
}

# Policy to allow instance to stop itself (deadman switch)
resource "aws_iam_role_policy" "self_stop" {
  name = "robotlab-self-stop"
  role = aws_iam_role.isaac_sim.id

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect = "Allow"
        Action = [
          "ec2:StopInstances",
          "ec2:DescribeInstances"
        ]
        Resource = "*"
        Condition = {
          StringEquals = {
            "ec2:ResourceTag/Project" = "robotlab"
          }
        }
      }
    ]
  })
}

# Instance profile to attach role to EC2
resource "aws_iam_instance_profile" "isaac_sim" {
  name = "robotlab-isaac-sim-profile"
  role = aws_iam_role.isaac_sim.name
}

# Store Claude Code API key in Secrets Manager
# NOTE: You must manually set the secret value after first apply:
#   aws secretsmanager put-secret-value \
#     --secret-id robotlab/claude-api-key \
#     --secret-string "sk-ant-xxx..."
resource "aws_secretsmanager_secret" "claude_api_key" {
  name        = "robotlab/claude-api-key"
  description = "Anthropic API key for Claude Code"

  tags = {
    Project = "robotlab"
  }

  # Allow recovery for 7 days if accidentally deleted
  recovery_window_in_days = 7
}
