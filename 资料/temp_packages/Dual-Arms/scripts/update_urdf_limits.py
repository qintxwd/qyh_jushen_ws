#!/usr/bin/env python3
"""
Update URDF joint limits: set effort and velocity values for Gazebo compatibility.
"""

import re

def update_urdf_joint_limits(input_file, output_file):
    """Update effort and velocity values in URDF joint limits."""
    
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Replace effort="0" with effort="100" (reasonable for robot arms)
    content = re.sub(r'effort="0"', 'effort="100"', content)
    
    # Replace velocity="0" with velocity="3.14" (reasonable angular velocity)
    content = re.sub(r'velocity="0"', 'velocity="3.14"', content)
    
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)
    
    print(f"✓ Updated URDF joint limits:")
    print(f"  - Set all effort values to 100 N⋅m")
    print(f"  - Set all velocity values to 3.14 rad/s")
    print(f"  - Output: {output_file}")

if __name__ == '__main__':
    import os
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    urdf_dir = os.path.join(package_dir, 'urdf')
    
    input_urdf = os.path.join(urdf_dir, 'Dual-Arms.urdf')
    output_urdf = os.path.join(urdf_dir, 'Dual-Arms-gazebo.urdf')
    
    update_urdf_joint_limits(input_urdf, output_urdf)
