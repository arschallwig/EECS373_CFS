#!/bin/bash
# Driver bash script for Commuter First Solutions - Transit Usage Module

while true 
do
  sudo systemctl suspend
  echo Running detection
  python3 jetson-inference/python/examples/cfs-detection.py
done
