#!/bin/bash
# 在编译前创建 libmodbus 符号链接
# 用法：在 WSL 中运行此脚本

set -e

PACKAGES="qyh_shutdown qyh_waist_control qyh_lift_control qyh_gripper_control qyh_standard_robot"
ARCHS="x64 arm64"

echo "=== Creating libmodbus symlinks ==="

for pkg in $PACKAGES; do
    for arch in $ARCHS; do
        dir="src/$pkg/thirdparty/lib/linux/$arch"
        if [ -d "$dir" ]; then
            cd "$dir"
            # 删除旧的符号链接或文本文件
            rm -f libmodbus.so libmodbus.so.3
            # 创建新的符号链接
            ln -sf libmodbus.so.3.1.11 libmodbus.so.3
            ln -sf libmodbus.so.3 libmodbus.so
            echo "  ✓ Created symlinks in $pkg/$arch"
            cd - > /dev/null
        fi
    done
done

echo "=== All symlinks created successfully ==="
