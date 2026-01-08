output "instance_id" {
  description = "EC2 instance ID (used by start/stop scripts)"
  value       = aws_instance.isaac_sim.id
}

output "public_ip" {
  description = "Public IP address of the instance"
  value       = aws_instance.isaac_sim.public_ip
}

output "ssh_command" {
  description = "SSH command to connect to the instance"
  value       = "ssh -i ~/.ssh/${var.key_name}.pem ubuntu@${aws_instance.isaac_sim.public_ip}"
}

output "webrtc_url" {
  description = "WebRTC streaming URL for Mac client"
  value       = "${aws_instance.isaac_sim.public_ip}:49100"
}

output "dcv_url" {
  description = "NICE DCV URL (fallback remote desktop)"
  value       = "https://${aws_instance.isaac_sim.public_ip}:8443"
}

output "instance_type" {
  description = "Instance type and pricing mode"
  value       = "${var.instance_type} (${var.use_spot ? "spot" : "on-demand"})"
}

output "cost_warning" {
  description = "Cost reminder"
  value       = var.use_spot ? "Spot instance - ~70% cheaper but may be interrupted. Stop when done!" : "${var.instance_type} on-demand. Run ./scripts/stop-instance.sh when done!"
}
