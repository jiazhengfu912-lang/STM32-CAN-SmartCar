# 2026-09-19 仓库整理验证

本次仅构建、离线测试、证据校验及GitHub归档；未连接硬件进行新试验，未改变P/I或控制算法。

- `verification.json`：6条验证命令均返回0，STM32 App/Bootloader构建、主机测试和4个网页脚本测试通过。
- `build_report.json`：源码v125的实际构建报告；`test_report.json`：HOST_SOFTWARE_ONLY。
- `host-tests-full.log`：主机测试完整输出；其他log为对应命令的原始输出。个别命令输出编码可能与查看器不同，原样保留。
- `archive-check.json`：54份原始证据快照SHA256核验通过；本机配置值与凭据模式扫描未命中待提交文件。此扫描不代表全面安全审计。
- Markdown引用的本地目标均存在（排除流程文档中的示例占位路径）。

ESP32、EIDE及Keil IDE没有在本轮重新构建；既有MDK项目经过脚本构建同步前后字节未变，详见verification。未将旧版实物试验作为当前新验收。低速二次启动、超调和落地验证仍未完成。
