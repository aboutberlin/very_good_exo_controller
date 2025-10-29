#!/bin/bash
# 一键同步并推送脚本：自动 pull -> add -> commit -> push
# 会忽略 .csv 文件

# 停止脚本在出错时继续执行
set -e

# Step 1. 切换到脚本所在目录（确保路径正确）
cd "$(dirname "$0")"

echo "🚀 开始同步 Git 仓库..."

# Step 2. 拉取远程仓库更新
echo "📥 正在拉取远程更新..."
git pull origin main --rebase

# Step 3. 确保 .gitignore 中忽略 csv
if ! grep -q "\*.csv" .gitignore 2>/dev/null; then
  echo "*.csv" >> .gitignore
  echo "📝 已将 '*.csv' 添加到 .gitignore"
fi

# Step 4. 添加所有更改
git add .

# Step 5. 提交更新（带时间戳）
git commit -m "Auto update on $(date '+%Y-%m-%d %H:%M:%S')" || echo "⚠️ 无更改可提交"

# Step 6. 推送到远程仓库
echo "📤 推送到 GitHub..."
git push origin main

echo "✅ 已完成同步和推送（CSV 文件被忽略）"
