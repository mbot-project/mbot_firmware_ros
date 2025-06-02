#!/bin/bash

set -e  # Quit on error.

SERVICE_NAME="mbot-microros-agent"

# Copy the service file
sudo cp ${SERVICE_NAME}.service /etc/systemd/system/${SERVICE_NAME}.service

# Reload systemd to recognize the new service
sudo systemctl daemon-reload

# Enable the service to start at boot and start it now
sudo systemctl enable --now ${SERVICE_NAME}.service

echo "${SERVICE_NAME}.service installed, enabled, and started."
