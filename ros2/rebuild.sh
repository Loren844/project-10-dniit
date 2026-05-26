#!/bin/bash
# rebuild.sh — Rebuild complet du workspace SCARA
# Usage : . ./rebuild.sh   (avec le point pour que source soit effectif dans le shell courant)

set -e
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE"

source /opt/ros/humble/setup.bash

echo "==> [0/4] Arret Gazebo/ROS en cours (si actif)..."
pkill -f 'gz_sim'  2>/dev/null || true
pkill -f 'gzserver' 2>/dev/null || true
sleep 1

echo "==> [1/4] Nettoyage build/ et install/..."
sudo rm -rf \
  build/scara_description   install/scara_description \
  build/scara_moveit_config install/scara_moveit_config \
  build/scara_visual_servoing install/scara_visual_servoing

echo "==> [2/4] Fix timestamps (transfert macOS -> Linux)..."
find src/scara_description src/scara_moveit_config src/scara_visual_servoing \
     -exec touch {} + 2>/dev/null || true

echo "==> [3/4] colcon build..."
colcon build \
  --packages-select scara_description scara_moveit_config scara_visual_servoing \
  --symlink-install

echo "==> [4/4] Source workspace..."
source install/setup.bash

echo ""
echo "Build OK. Commandes disponibles :"
echo ""
echo "  # Simulation legere (sans Gazebo) :"
echo "  ros2 launch scara_visual_servoing bringup.launch.py demo_mode:=conveyor"
echo ""
echo "  # Simulation complete Gazebo Sim :"
echo "  ros2 launch scara_visual_servoing gazebo_sim.launch.py"
echo "  ros2 launch scara_visual_servoing gazebo_sim.launch.py headless:=true"
echo "  ros2 launch scara_visual_servoing gazebo_sim.launch.py with_moveit:=true"
echo ""
echo "  # Visualisation URDF seule :"
echo "  ros2 launch scara_description display.launch.py"
