#!/usr/bin/env bash
# =============================================================================
# load_image.sh
# 从 cyberdog_race2026.tar 导入 Docker 镜像
#
# 用法：
#   bash docker/load_image.sh                        # 使用默认 tar 路径
#   bash docker/load_image.sh /path/to/custom.tar    # 指定 tar 文件路径
# =============================================================================
set -euo pipefail

# ── 默认 tar 文件路径（与实际文件位置一致）────────────────────
DEFAULT_TAR="$HOME/cyberdog_race2026.tar"
TAR_FILE="${1:-$DEFAULT_TAR}"

echo "======================================================"
echo " 导入 CyberDog Sim Docker 镜像"
echo " 源文件: $TAR_FILE"
echo "======================================================"
echo ""

# ── 检查文件存在 ─────────────────────────────────────────────
if [[ ! -f "$TAR_FILE" ]]; then
  echo "[ERROR] 未找到镜像文件: $TAR_FILE"
  echo "        请确认文件路径，或运行:"
  echo "        bash docker/load_image.sh /your/actual/path/cyberdog_race2026.tar"
  exit 1
fi

FILE_SIZE=$(du -sh "$TAR_FILE" | cut -f1)
echo "[INFO] 文件大小: $FILE_SIZE"
echo "[INFO] 开始导入（约需 1~5 分钟，请耐心等待）..."
echo ""

# ── 导入镜像 ─────────────────────────────────────────────────
time sudo docker load -i "$TAR_FILE"

echo ""
echo "[INFO] 当前 Docker 镜像列表:"
docker images | grep -E 'cyberdog|REPOSITORY'

echo ""
echo "======================================================"
echo " ✓ 镜像导入完成"
echo ""
echo " 下一步（后台启动容器 + VS Code 附加）："
echo "   bash docker/start_container.sh --detach"
echo "======================================================"
