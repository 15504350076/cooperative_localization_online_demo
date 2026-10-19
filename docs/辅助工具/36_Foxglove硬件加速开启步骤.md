# 36 Foxglove 硬件加速开启步骤

## 结论

开启硬件加速通常不会改变消息、时间戳、曲线数值或离线分析结果，只改变 Foxglove 的绘制方式。对当前 IMU/UWB 2D 曲线不是必须；以后查看激光雷达点云、相机或 3D 面板时，建议开启。

## Edge 浏览器版（按你的环境操作）

1. 关闭正在播放的 Foxglove 页面（不必删除布局）。
2. 在 Edge 地址栏输入 `edge://settings/system`。
3. 打开“可用时使用图形加速”（英文为 **Use graphics acceleration when available**）。
4. 点击“重新启动”或完全退出后重新打开 Edge。
5. 重启后输入 `edge://gpu`，确认 `WebGL` 或 `WebGL2` 显示 `Hardware accelerated`。
6. 重新打开 Foxglove 和 `.mcap` 文件，原来的布局可以继续导入。

## Foxglove 桌面版

桌面版基于 Electron/Chromium，通常会自动使用可用的 GPU。若你是在 Edge 网页中打开 Foxglove，按上一节设置；若是 Foxglove 独立桌面程序，则按本节检查：

1. 先按上一节在 Edge 检查 GPU；若 Edge 也不是 `Hardware accelerated`，先安装或更新显卡驱动。
2. Linux 下确认当前会话确实运行在图形桌面，而不是纯 SSH/无显示环境。
3. 重新启动 Foxglove，再观察黄色提示是否消失。
4. 只有 Edge 的 3D 页面仍明显卡顿时，才在 `edge://flags` 中搜索并启用 **Override software rendering list**，重启后复查 `edge://gpu`。该选项可能导致不稳定，若出现黑屏或崩溃立即恢复默认。

## 验证是否生效

- 2D Plot、Raw Messages 能正常打开即可满足当前测试。
- 需要验证 3D/点云时，在 `chrome://gpu` 查看 `WebGL`/`WebGL2`，不要以曲线刷新速度判断传感器同步。
- 开启或关闭硬件加速都不会修复 IMU/UWB 时间偏差，也不会改变录包文件。

## 如果 Edge 仍显示 Software only（本机截图对应情况）

在 `edge://gpu` 中如果同时看到：

```text
WebGL: Disabled
WebGL2: Disabled
Canvas: Software only
Hardware acceleration disabled
```

说明不是 Edge 开关问题，而是 Linux 图形驱动没有提供可用的 GPU。截图中的 `GPU0: VMware`、`ANGLE ... SwiftShader` 表明当前桌面运行在 VMware 虚拟显卡/软件渲染环境中。

处理顺序：

1. 如果这是虚拟机：关机，在 VMware 设置中打开 **Accelerate 3D graphics（加速 3D 图形）**，增加足够的视频内存，再启动虚拟机。
2. 如果这是远程桌面或 SSH 转发的桌面：改用本机物理桌面登录；远程会话经常不会暴露真实 GPU。
3. 在 Linux 终端检查：

   ```bash
   sudo apt install mesa-utils
   glxinfo -B | grep --extended-regexp 'OpenGL renderer|direct rendering'
   ```

   `direct rendering: Yes` 且 renderer 是实际显卡名称，才具备继续启用的条件；若仍是 `llvmpipe`/`SwiftShader`，就是软件渲染。
4. 重启 Edge，再检查 `edge://gpu`。如果仍为 Software only，不要继续强制修改 flags；当前 IMU/UWB 2D 曲线照常使用即可。

仅在确认虚拟机 3D 加速和驱动正常后，才考虑 `edge://flags` 的 **Override software rendering list**。强制绕过黑名单可能黑屏或崩溃，不能作为常规修复。

Foxglove 官方说明：硬件加速主要改善 3D 面板的 GPU 渲染性能，桌面版在可用时会自动启用；基于 Chromium 的浏览器可在浏览器系统设置中开启并用 GPU 页面检查。[官方 3D 性能说明](https://docs.foxglove.dev/docs/visualization/panels/3d)
