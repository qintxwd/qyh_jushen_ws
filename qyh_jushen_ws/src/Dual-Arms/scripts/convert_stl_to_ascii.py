#!/usr/bin/env python3
"""
Convert binary STL files to ASCII STL format
"""
import struct
import os
import sys
from pathlib import Path

def read_binary_stl(filename):
    """Read a binary STL file and return triangles"""
    with open(filename, 'rb') as f:
        # Skip header (80 bytes)
        header = f.read(80)
        
        # Read number of triangles
        num_triangles = struct.unpack('I', f.read(4))[0]
        
        triangles = []
        for i in range(num_triangles):
            # Read normal (3 floats)
            normal = struct.unpack('fff', f.read(12))
            
            # Read 3 vertices (3 floats each)
            v1 = struct.unpack('fff', f.read(12))
            v2 = struct.unpack('fff', f.read(12))
            v3 = struct.unpack('fff', f.read(12))
            
            # Skip attribute byte count
            f.read(2)
            
            triangles.append((normal, v1, v2, v3))
        
        return triangles

def write_ascii_stl(filename, triangles, solid_name=""):
    """Write triangles to ASCII STL file"""
    with open(filename, 'w') as f:
        f.write(f'solid {solid_name}\n')
        
        for normal, v1, v2, v3 in triangles:
            f.write(f'  facet normal {normal[0]:e} {normal[1]:e} {normal[2]:e}\n')
            f.write('    outer loop\n')
            f.write(f'      vertex {v1[0]:e} {v1[1]:e} {v1[2]:e}\n')
            f.write(f'      vertex {v2[0]:e} {v2[1]:e} {v2[2]:e}\n')
            f.write(f'      vertex {v3[0]:e} {v3[1]:e} {v3[2]:e}\n')
            f.write('    endloop\n')
            f.write('  endfacet\n')
        
        f.write(f'endsolid {solid_name}\n')

def convert_stl(input_file, output_file):
    """Convert binary STL to ASCII STL"""
    print(f"Converting {input_file} to {output_file}...")
    
    try:
        triangles = read_binary_stl(input_file)
        solid_name = Path(input_file).stem
        write_ascii_stl(output_file, triangles, solid_name)
        print(f"  Successfully converted {len(triangles)} triangles")
        return True
    except Exception as e:
        print(f"  Error: {e}")
        return False

if __name__ == '__main__':
    if len(sys.argv) > 1:
        # Convert specific file
        input_file = sys.argv[1]
        output_file = sys.argv[2] if len(sys.argv) > 2 else input_file
        convert_stl(input_file, output_file)
    else:
        # Convert all STL files in meshes directory
        mesh_dir = Path(__file__).parent.parent / 'meshes'
        print(f"Converting all STL files in {mesh_dir}")
        
        stl_files = list(mesh_dir.glob('*.STL')) + list(mesh_dir.glob('*.stl'))
        
        success_count = 0
        for stl_file in stl_files:
            if convert_stl(str(stl_file), str(stl_file)):
                success_count += 1
        
        print(f"\nConverted {success_count}/{len(stl_files)} files successfully")
