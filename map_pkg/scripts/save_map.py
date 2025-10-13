#!/usr/bin/env python3
import os
import subprocess
import sys
import yaml

# Simple script to save a map with a relative image path
# Usage: rosrun map_pkg save_map_relative_simple.py <map_name>

if len(sys.argv) < 2:
    print("Usage: save_map.py <map_name>")
    sys.exit(1)

map_name = sys.argv[1]

# Get the absolute path of the package
pkg_path = subprocess.getoutput("rospack find map_pkg")

# Define the folder where maps will be saved
maps_dir = os.path.join(pkg_path, "maps")
os.makedirs(maps_dir, exist_ok=True)

# 💡 Change the current directory to 'maps'
os.chdir(maps_dir)

# Save the map (files will be created inside 'maps/')
subprocess.run(["rosrun", "map_server", "map_saver", "-f", map_name])

# Read the YAML file and modify image path
yaml_file = map_name + ".yaml"

with open(yaml_file) as f:
    data = yaml.safe_load(f)

# Make image path relative
data["image"] = map_name + ".pgm"

with open(yaml_file, "w") as f:
    yaml.dump(data, f)

print("✅ Map saved in:", maps_dir)
print("   YAML:", yaml_file)
print("   Image path set to:", data["image"])
