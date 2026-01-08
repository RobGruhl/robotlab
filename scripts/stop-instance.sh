#!/bin/bash
# Stop the Isaac Sim EC2 instance to save costs
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TERRAFORM_DIR="$SCRIPT_DIR/../infra/terraform"

# Get instance ID from Terraform state
cd "$TERRAFORM_DIR"
INSTANCE_ID=$(terraform output -raw instance_id 2>/dev/null)

if [ -z "$INSTANCE_ID" ]; then
    echo "Error: Could not get instance ID. Have you run 'terraform apply'?"
    exit 1
fi

echo "Stopping instance $INSTANCE_ID..."
aws ec2 stop-instances --instance-ids "$INSTANCE_ID" > /dev/null

echo "Waiting for instance to stop..."
aws ec2 wait instance-stopped --instance-ids "$INSTANCE_ID"

echo ""
echo "========================================="
echo "Instance stopped. Billing has stopped."
echo "========================================="
echo ""
echo "Your data is preserved on the EBS volume."
echo "Run ./scripts/start-instance.sh to resume."
