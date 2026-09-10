#!/usr/bin/env bash
# 一键编译脚本：配置并构建 delta_bottom 工程
# 用法:
#   ./build.sh                 # Release 编译
#   ./build.sh --debug         # Debug 编译
#   ./build.sh --clean         # 先清空 build 目录再编译
#   ./build.sh --run           # 编译完成后直接运行
set -euo pipefail

# 切换到脚本所在目录（工程根目录）
cd "$(dirname "$0")"

BUILD_DIR="${BUILD_DIR:-build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
RUN=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug)
      BUILD_TYPE=Debug
      shift
      ;;
    --clean)
      rm -rf "$BUILD_DIR"
      shift
      ;;
    --run)
      RUN=1
      shift
      ;;
    *)
      echo "未知参数: $1" >&2
      echo "用法: $0 [--debug] [--clean] [--run]" >&2
      exit 1
      ;;
  esac
done

echo "==> 配置工程 (${BUILD_TYPE}) ..."
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

echo "==> 编译 (并行 $(nproc) 线程) ..."
cmake --build "$BUILD_DIR" -j"$(nproc)"

echo "==> 编译完成，产物: $BUILD_DIR/delta_bottom"

if [[ "$RUN" -eq 1 ]]; then
  echo "==> 运行 ..."
  "$BUILD_DIR/delta_bottom"
fi
