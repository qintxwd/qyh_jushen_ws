#!/usr/bin/env python3
"""
Convert ASCII STL files to binary STL format for RViz2 compatibility.
"""

import struct
import re
import os

def read_ascii_stl(filename):
    """Read ASCII STL file and return list of triangles."""
    triangles = []
    
    with open(filename, 'r') as f:
        content = f.read()
        
    # Parse facets using regex
    facet_pattern = r'facet normal\s+([\d.e+-]+)\s+([\d.e+-]+)\s+([\d.e+-]+).*?vertex\s+([\d.e+-]+)\s+([\d.e+-]+)\s+([\d.e+-]+).*?vertex\s+([\d.e+-]+)\s+([\d.e+-]+)\s+([\d.e+-]+).*?vertex\s+([\d.e+-]+)\s+([\d.e+-]+)\s+([\d.e+-]+)'
    
    matches = re.finditer(facet_pattern, content, re.DOTALL)
    
    for match in matches:
        normal = [float(match.group(1)), float(match.group(2)), float(match.group(3))]
        v1 = [float(match.group(4)), float(match.group(5)), float(match.group(6))]
        v2 = [float(match.group(7)), float(match.group(8)), float(match.group(9))]
        v3 = [float(match.group(10)), float(match.group(11)), float(match.group(12))]
        
        triangles.append({
            'normal': normal,
            'vertices': [v1, v2, v3]
        })
    
    return triangles

def write_binary_stl(filename, triangles, header="Binary STL converted from ASCII"):
    """Write binary STL file."""
    with open(filename, 'wb') as f:
        # Write 80-byte header
        header_bytes = header.encode('utf-8')[:80].ljust(80, b'\0')
        f.write(header_bytes)
        
        # Write number of triangles
        f.write(struct.pack('<I', len(triangles)))
        
        # Write each triangle
        for tri in triangles:
            # Normal vector (3 floats)
            f.write(struct.pack('<fff', *tri['normal']))
            
            # Three vertices (9 floats)
            for vertex in tri['vertices']:
                f.write(struct.pack('<fff', *vertex))
            
            # Attribute byte count (uint16, usually 0)
            f.write(struct.pack('<H', 0))

def convert_file(input_file, output_file):
    """Convert single ASCII STL to binary STL."""
    print(f"Converting: {os.path.basename(input_file)}")
    triangles = read_ascii_stl(input_file)
    write_binary_stl(output_file, triangles)
    print(f"  ✓ Wrote {len(triangles)} triangles to binary format")

def main():
    # Get the meshes directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    meshes_dir = os.path.join(package_dir, 'meshes')
    
    # List of STL files to convert
    stl_files = [
        'base_link.STL',
        'l1.STL', 'l2.STL', 'l3.STL', 'l4.STL', 'l5.STL', 'l6.STL', 'l7.STL', 'lt.STL',
        'r1.STL', 'r2.STL', 'r3.STL', 'r4.STL', 'r5.STL', 'r6.STL', 'r7.STL', 'rt.STL'
    ]
    
    print("Converting ASCII STL files to binary format for RViz2...\n")
    
    for stl_file in stl_files:
        input_path = os.path.join(meshes_dir, stl_file)
        output_path = input_path  # Overwrite the original file
        
        if os.path.exists(input_path):
            convert_file(input_path, output_path)
        else:
            print(f"Warning: {stl_file} not found")
    
    print("\n✓ All conversions complete!")

if __name__ == '__main__':
    main()
