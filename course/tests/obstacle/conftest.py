"""Pytest for the Obstacle course: pure Python, no ROS, no Gazebo. Finds my_obstacle in src/ without building it."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).parent))
for package in ('my_obstacle', 'robotek_course', 'robotek_common', 'robotek_evaluation'):
    sys.path.insert(0, str(ROOT / 'src' / package))
