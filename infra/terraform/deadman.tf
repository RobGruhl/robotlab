# Deadman Switch: Nightly Lambda + EventBridge + SNS
# Automatically stops the GPU instance at midnight to prevent forgotten billing.

locals {
  deadman_enabled = var.deadman_enabled && var.deadman_email != ""
}

# SNS Topic for notifications
resource "aws_sns_topic" "deadman_alerts" {
  count = local.deadman_enabled ? 1 : 0
  name  = "robotlab-deadman-alerts"

  tags = {
    Project = "robotlab"
  }
}

# Email subscription (requires manual confirmation)
resource "aws_sns_topic_subscription" "deadman_email" {
  count     = local.deadman_enabled ? 1 : 0
  topic_arn = aws_sns_topic.deadman_alerts[0].arn
  protocol  = "email"
  endpoint  = var.deadman_email
}

# IAM Role for Lambda
resource "aws_iam_role" "deadman_lambda" {
  count = local.deadman_enabled ? 1 : 0
  name  = "robotlab-deadman-lambda-role"

  assume_role_policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Action = "sts:AssumeRole"
        Effect = "Allow"
        Principal = {
          Service = "lambda.amazonaws.com"
        }
      }
    ]
  })

  tags = {
    Project = "robotlab"
  }
}

# Lambda execution policy
resource "aws_iam_role_policy" "deadman_lambda" {
  count = local.deadman_enabled ? 1 : 0
  name  = "robotlab-deadman-lambda-policy"
  role  = aws_iam_role.deadman_lambda[0].id

  policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Effect = "Allow"
        Action = [
          "logs:CreateLogGroup",
          "logs:CreateLogStream",
          "logs:PutLogEvents"
        ]
        Resource = "arn:aws:logs:*:*:*"
      },
      {
        Effect = "Allow"
        Action = [
          "ec2:DescribeInstances",
          "ec2:StopInstances",
          "ec2:CreateTags"
        ]
        Resource = "*"
      },
      {
        Effect   = "Allow"
        Action   = "sns:Publish"
        Resource = aws_sns_topic.deadman_alerts[0].arn
      }
    ]
  })
}

# Lambda function (inline Python)
resource "aws_lambda_function" "deadman_stop" {
  count         = local.deadman_enabled ? 1 : 0
  function_name = "robotlab-deadman-stop"
  role          = aws_iam_role.deadman_lambda[0].arn
  handler       = "index.handler"
  runtime       = "python3.12"
  timeout       = 30

  filename         = data.archive_file.deadman_lambda[0].output_path
  source_code_hash = data.archive_file.deadman_lambda[0].output_base64sha256

  environment {
    variables = {
      INSTANCE_ID = aws_instance.isaac_sim.id
      SNS_TOPIC   = aws_sns_topic.deadman_alerts[0].arn
      REGION      = var.region
    }
  }

  tags = {
    Project = "robotlab"
  }
}

# Lambda code archive
data "archive_file" "deadman_lambda" {
  count       = local.deadman_enabled ? 1 : 0
  type        = "zip"
  output_path = "${path.module}/.terraform/deadman_lambda.zip"

  source {
    content  = <<-PYTHON
import boto3
import os
from datetime import datetime

def handler(event, context):
    ec2 = boto3.client('ec2', region_name=os.environ['REGION'])
    sns = boto3.client('sns', region_name=os.environ['REGION'])
    instance_id = os.environ['INSTANCE_ID']
    sns_topic = os.environ['SNS_TOPIC']

    # Get instance state
    response = ec2.describe_instances(InstanceIds=[instance_id])
    state = response['Reservations'][0]['Instances'][0]['State']['Name']

    if state == 'running':
        # Stop the instance
        ec2.stop_instances(InstanceIds=[instance_id])

        # Tag with stop time
        ec2.create_tags(
            Resources=[instance_id],
            Tags=[{'Key': 'last-deadman-stop', 'Value': datetime.utcnow().isoformat()}]
        )

        # Send notification
        sns.publish(
            TopicArn=sns_topic,
            Subject='Robotlab GPU Instance Stopped (Deadman Switch)',
            Message=f'''Your robotlab GPU instance was automatically stopped by the deadman switch.

Instance ID: {instance_id}
Time (UTC): {datetime.utcnow().isoformat()}
Reason: Nightly scheduled shutdown

To start it again, run:
  cd infra/terraform && ./scripts/start-instance.sh

This helps prevent unexpected AWS charges from forgotten instances.
'''
        )

        return {
            'statusCode': 200,
            'body': f'Stopped instance {instance_id}'
        }
    else:
        return {
            'statusCode': 200,
            'body': f'Instance {instance_id} already {state}, no action taken'
        }
PYTHON
    filename = "index.py"
  }
}

# EventBridge rule for nightly trigger
resource "aws_cloudwatch_event_rule" "deadman_nightly" {
  count               = local.deadman_enabled ? 1 : 0
  name                = "robotlab-deadman-nightly"
  description         = "Trigger deadman switch at configured hour"
  schedule_expression = "cron(0 ${var.deadman_shutdown_hour_utc} * * ? *)"

  tags = {
    Project = "robotlab"
  }
}

# EventBridge target (Lambda)
resource "aws_cloudwatch_event_target" "deadman_lambda" {
  count     = local.deadman_enabled ? 1 : 0
  rule      = aws_cloudwatch_event_rule.deadman_nightly[0].name
  target_id = "DeadmanLambda"
  arn       = aws_lambda_function.deadman_stop[0].arn
}

# Permission for EventBridge to invoke Lambda
resource "aws_lambda_permission" "deadman_eventbridge" {
  count         = local.deadman_enabled ? 1 : 0
  statement_id  = "AllowEventBridgeInvoke"
  action        = "lambda:InvokeFunction"
  function_name = aws_lambda_function.deadman_stop[0].function_name
  principal     = "events.amazonaws.com"
  source_arn    = aws_cloudwatch_event_rule.deadman_nightly[0].arn
}

# AWS Budget for cost alerts
resource "aws_budgets_budget" "ec2_monthly" {
  count             = var.deadman_budget_limit > 0 ? 1 : 0
  name              = "robotlab-ec2-monthly"
  budget_type       = "COST"
  limit_amount      = var.deadman_budget_limit
  limit_unit        = "USD"
  time_unit         = "MONTHLY"
  time_period_start = "2024-01-01_00:00"

  cost_filter {
    name   = "Service"
    values = ["Amazon Elastic Compute Cloud - Compute"]
  }

  notification {
    comparison_operator        = "GREATER_THAN"
    threshold                  = 80
    threshold_type             = "PERCENTAGE"
    notification_type          = "ACTUAL"
    subscriber_email_addresses = var.deadman_email != "" ? [var.deadman_email] : []
  }

  notification {
    comparison_operator        = "GREATER_THAN"
    threshold                  = 100
    threshold_type             = "PERCENTAGE"
    notification_type          = "ACTUAL"
    subscriber_email_addresses = var.deadman_email != "" ? [var.deadman_email] : []
  }
}

# CloudWatch Log Group for Lambda (with retention)
resource "aws_cloudwatch_log_group" "deadman_lambda" {
  count             = local.deadman_enabled ? 1 : 0
  name              = "/aws/lambda/robotlab-deadman-stop"
  retention_in_days = 7

  tags = {
    Project = "robotlab"
  }
}
