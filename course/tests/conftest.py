"""Pytest for the course: pure Python, no ROS, no Gazebo. It finds the packages in src/ without building them."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).parent))
for package in ('my_open', 'robotek_course', 'robotek_common', 'robotek_evaluation'):
    sys.path.insert(0, str(ROOT / 'src' / package))
