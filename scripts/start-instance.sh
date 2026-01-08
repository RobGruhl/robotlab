#!/bin/bash
# Start the Isaac Sim EC2 instance and display connection info
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

echo "Starting instance $INSTANCE_ID..."
aws ec2 start-instances --instance-ids "$INSTANCE_ID" > /dev/null

echo "Waiting for instance to be running..."
aws ec2 wait instance-running --instance-ids "$INSTANCE_ID"

# Get the new public IP (may change after stop/start unless using Elastic IP)
PUBLIC_IP=$(aws ec2 describe-instances --instance-ids "$INSTANCE_ID" \
    --query 'Reservations[0].Instances[0].PublicIpAddress' --output text)

echo ""
echo "========================================="
echo "Instance is running!"
echo "========================================="
echo ""
echo "Public IP: $PUBLIC_IP"
echo ""
echo "SSH:       ssh -i ~/.ssh/YOUR_KEY.pem ubuntu@$PUBLIC_IP"
echo "WebRTC:    $PUBLIC_IP:49100"
echo "DCV:       https://$PUBLIC_IP:8443"
echo ""
echo "Next steps:"
echo "  1. SSH in and run: ./isaac-sim.sh --/app/livestream/enabled=true"
echo "  2. Connect Mac WebRTC client to $PUBLIC_IP:49100"
echo ""
echo "Cost: ~\$1.50/hr. Run ./scripts/stop-instance.sh when done!"
