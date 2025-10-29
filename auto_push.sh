#!/bin/bash
# 一键上传脚本：忽略 .csv 文件并推送到 GitHub

# Step 1. 确保 .gitignore 存在且包含忽略规则
echo "*.csv" > .gitignore

# Step 2. 添加所有文件（git 会自动忽略 csv）
git add .

# Step 3. 提交更新
git commit -m "Auto update on $(date '+%Y-%m-%d %H:%M:%S')"

# Step 4. 推送到 GitHub
git push origin main

echo "✅ 已自动推送到 GitHub（CSV 文件被忽略）"
