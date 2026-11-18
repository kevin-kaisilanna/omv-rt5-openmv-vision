# OMV RT5 / OpenMV 兼容固件视觉工程

逐飞 OMV RT5 板上的 OpenMV 兼容 MicroPython 整机程序：钢球 / 循迹识别、UART 位置输出、逐飞 WiFi SPI 图传。

## 这个工程做什么

- `main.py` 是整机主循环：摄像头识别轨迹与钢球，把 `line_error`（循迹偏差）、`distance_raw`（钢球距离）、`cross_flag`（路口标志）打包成 6 字节帧，经 UART1 送给主控。
- 帧格式：`FF FE | line_error+120 | distance_raw | cross_flag | 前 5 字节校验和`，115200 波特率，P4/TX、P5/RX。
- `wifi_image_stream.py` 把逐飞 WiFi SPI 模块封装成 `SeekfreeWifiSPI` + `WifiImageStreamer`，按 `seekfree_assistant` CAMERA 帧协议把灰度图发给上位机。
- 只要 UART 数据、不需要图传时，把 `WIFI_SSID` 留空即可，程序会跳过 WiFi 初始化，不会因缺模块而卡死。
- 平台特征：`import sensor, image`，无 `media.*`、无 `nncase_runtime`；与 K230、RK3576 仓库不通用。

## 部署

1. **烧固件**：给 OMV RT5 烧录 OpenMV 兼容 MicroPython 固件（提供 `sensor` / `image` / `machine` 模块）。
2. **接线**（只要 UART 输出可跳过此步）：

   | 板子引脚 | WiFi 模块 | 说明 |
   | --- | --- | --- |
   | P0 / P1 / P2 | MOSI / MISO / SCLK | 硬件 SPI2，30 MHz |
   | P3 / P6 / P7 | CS / INT / RST | GPIO |
   | 5V / GND | 5V / GND | 供电 |
   | P4 / P5 | 主控 UART | UART1，115200 |

3. **拷文件**：把 `main.py` 和 `wifi_image_stream.py` 两个文件放到板子根目录。`main.py` 会被固件当作开机自启入口。
4. **填配置**：编辑 `main.py` 顶部 —— `WIFI_SSID` / `WIFI_PASSWORD` 填自己的网络（不图传就设 `""`），`SERVER_IP` / `SERVER_PORT`（默认 `8086`）/ `LOCAL_PORT`（默认 `6666`）填 PC 上位机地址。占位符说明见下方。
5. **开上位机**：PC 上启动逐飞 Assistant，监听 / 连接对应 TCP 端口，选择 CAMERA 图像通道。
6. **上电验证**：串口应打印 `OpenMV UART1 ready on P4/P5`；主控能收到 6 字节位置帧；上位机出现灰度图传画面。若打印 `WiFi SPI init failed`，检查接线与 SSID，UART 功能不受影响。

## 第三方内容（均未上传）

逐飞 `zf_device_wifi_spi.c` 原始 C 驱动与 SeekFree Assistant 协议来自逐飞科技开源资料；本仓库保留的是**自行重写的 MicroPython 移植版**，协议权利归逐飞。OpenMV 固件模块由烧录固件获得，厂商例程、调试截图与视频未分发。
真实 Wi-Fi 名称 / 密码、AP 密码与共享密钥已脱敏为 `YOUR_WIFI_SSID`、`YOUR_WIFI_PASSWORD`、`YOUR_AP_PASSWORD`、`YOUR_SHARED_SECRET`，**运行前必须填回**。

## 项目系列

同一竞赛项目按**硬件环境**拆分的独立仓库，各平台固件 API 与运行栈互不相通，代码不能混用。

| 硬件环境 / 运行栈 | 仓库 |
| --- | --- |
| 嘉楠 K230 · CanMV K230（MicroPython 固件） | [canmv-k230-vision](https://github.com/kevin-kaisilanna/canmv-k230-vision) |
| 逐飞 OMV RT5 · OpenMV 兼容固件 | **omv-rt5-openmv-vision** ← 本仓库 |
| 逐飞 OpenART Mini · 板级显示 / SPI | [openart-mini-display](https://github.com/kevin-kaisilanna/openart-mini-display) |
| KICKPI K7C · RK3576（Ubuntu + CPython/OpenCV） | [rk3576-k7c-vision](https://github.com/kevin-kaisilanna/rk3576-k7c-vision) |
| 竞赛设计报告（跨平台文档，无代码） | [rolling-ball-design-report](https://github.com/kevin-kaisilanna/rolling-ball-design-report) |

> 所有仓库均只收录自研代码；第三方开源库、厂商 SDK、平台生成代码与预训练模型未上传，来源见各仓库 README。

## 许可证

仅收录自研代码，尚未指定开源许可证（默认保留所有权利）；第三方组件权利归各自权利人。
