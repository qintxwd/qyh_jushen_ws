#!/bin/bash
# 重组 thirdparty 目录结构
# 将通用的 include 和 lib 移动到 arm64 子目录下

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THIRDPARTY_DIR="$(dirname "$SCRIPT_DIR")/thirdparty"

cd "$THIRDPARTY_DIR"

echo "📁 当前目录: $(pwd)"
echo ""
echo "🔄 重组 thirdparty 目录结构..."
echo ""

# 1. 检查是否已经重组过
if [ -f "arm64/lib/libjakaAPI_2_3_0_13.so" ]; then
    echo "✅ arm64 目录已经包含库文件，跳过重组"
    exit 0
fi

# 2. 移动 include 到 arm64
if [ -d "include" ] && [ ! -d "arm64/include" ]; then
    echo "📦 移动 include/ 到 arm64/include/"
    mv include arm64/
else
    echo "⏭️  跳过 include 移动（已存在或源不存在）"
fi

# 3. 移动 lib 到 arm64
if [ -d "lib" ] && [ ! -d "arm64/lib" ]; then
    echo "📦 移动 lib/ 到 arm64/lib/"
    mv lib arm64/
else
    echo "⏭️  跳过 lib 移动（已存在或源不存在）"
fi

echo ""
echo "✅ 重组完成！"
echo ""
echo "📂 目录结构："
echo "thirdparty/"
echo "├── arm64/"
echo "│   ├── include/       (ARM64 头文件)"
echo "│   └── lib/           (ARM64 库: libjakaAPI_2_3_0_13.so)"
echo "└── x64/"
echo "    ├── include/       (x64 头文件)"
echo "    └── lib/           (x64 库: libjakaAPI_2_3_3.so)"
echo ""
echo "🔧 请在编译前确保已安装对应架构的依赖"
