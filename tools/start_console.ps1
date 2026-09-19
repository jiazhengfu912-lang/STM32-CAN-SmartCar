param([int]$Port = 18765)
$ErrorActionPreference = 'Stop'
# 启动仅监听本机的控制台；本脚本不会烧录设备或使能电机。
python -B (Join-Path $PSScriptRoot '..\desktop\server.py') --port $Port
