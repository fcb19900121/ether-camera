# ESP32-P4 以太网摄像头流媒体服务器

基于 ESP32-P4 的以太网摄像头项目，支持 MJPEG/H264 视频流、快照捕获和外部触发功能。

## 硬件配置

| 组件 | 规格 |
|------|------|
| MCU | ESP32-P4 (RISC-V 双核) |
| 摄像头 | OV2710 MIPI CSI |
| 分辨率 | 1280×720 @ 25fps |
| 网络 | 以太网（静态IP: 192.168.1.231） |
| 外部触发 | GPIO 17 |

## 网络配置

- **IP 地址**: `192.168.1.231`
- **子网掩码**: `255.255.255.0`
- **网关**: `192.168.1.1`
- **DNS**: `192.168.1.1`

## HTTP 端点

| 端点 | 方法 | 功能 | 说明 |
|------|------|------|------|
| `/` | GET | 首页 | Web界面，含视频预览和控制按钮 |
| `/stream` | GET | MJPEG 流 | 连续视频流，可直接在浏览器播放 |
| `/snapshot` | GET | 快照 | 获取当前最新一帧 JPEG 图片 |
| `/stream/h264` | GET | H264 流 | 原始 H264 视频流，适用于 VLC/ffplay |
| `/stream/info` | GET | 流信息 | JSON 格式的流状态信息 |

### 使用示例

#### 浏览器访问
```
http://192.168.1.231/          # 打开Web界面
http://192.168.1.231/stream    # 直接查看MJPEG视频流
http://192.168.1.231/snapshot  # 获取单张快照
```

#### 命令行工具
```bash
# 使用 ffplay 播放 MJPEG 流
ffplay http://192.168.1.231/stream

# 使用 ffplay 播放 H264 流
ffplay http://192.168.1.231/stream/h264

# 使用 VLC 播放
vlc http://192.168.1.231/stream

# 使用 curl 下载快照
curl -o snapshot.jpg http://192.168.1.231/snapshot

# 使用 wget 下载快照
wget http://192.168.1.231/snapshot -O snapshot.jpg

# 查看流信息
curl http://192.168.1.231/stream/info
```

#### Python 示例
```python
import requests

# 获取快照
response = requests.get('http://192.168.1.231/snapshot')
with open('snapshot.jpg', 'wb') as f:
    f.write(response.content)

# 获取流信息
info = requests.get('http://192.168.1.231/stream/info').json()
print(info)
```

### 响应头说明

#### /snapshot 响应头
| 头部 | 说明 |
|------|------|
| `Content-Type` | `image/jpeg` |
| `X-Timestamp-Us` | 帧时间戳（微秒） |
| `Cache-Control` | `no-cache` |

#### /stream/info 响应示例
```json
{
  "stream": {
    "active": true,
    "mode": "MJPEG",
    "resolution": "1280x720",
    "input_format": "RGB565",
    "jpeg_quality": 50,
    "h264_available": true,
    "buffer_size": 1843200
  },
  "endpoints": {
    "mjpeg": "/stream",
    "h264": "/stream/h264",
    "snapshot": "/snapshot",
    "info": "/stream/info"
  }
}
```

## 外部触发功能

使用 GPIO 17 作为外部触发输入，支持硬件中断触发帧捕获。

| 参数 | 值 |
|------|------|
| 触发引脚 | GPIO 17 |
| 触发方式 | 下降沿 |
| 内部上拉 | 已启用 |

### 触发行为
- 摄像头始终运行（25fps 连续出帧）
- 触发信号清空帧缓存并开始记录
- 环形缓冲区可缓存最近 5 帧（约 200ms）

## 视频编码

| 编码器 | 说明 |
|------|------|
| JPEG | 硬件编码，质量 50，YUV420 子采样 |
| H264 | 硬件编码（需要 YUV420 输入） |

## 延迟特性

| 场景 | 典型延迟 |
|------|----------|
| 快照请求 | 0-40ms |
| 触发到首帧 | 0-40ms |
| 网络传输 | ~5-10ms |

## 项目配置

在编译项目之前，请按照以下步骤配置摄像头 ID：

1. 在 ESP-IDF 终端运行配置命令：
   ```bash
   idf.py menuconfig
   ```
2. 依次进入菜单：`Example Configuration` -> `Select Camera ID`
3. 选择您连接的摄像头型号 ID
4. 按 `S` 键保存配置，然后按 `Esc` 键退出

## 编译和烧录

```bash
# 设置目标芯片
idf.py set-target esp32p4

# 编译
idf.py build

# 烧录
idf.py -p COM8 flash

# 监视输出
idf.py -p COM8 monitor
```

## 依赖组件

- `espressif/esp_video` - 视频框架
- `espressif/esp_cam_sensor` - 摄像头传感器驱动
- `espressif/esp_h264` - H264 编解码器
- `espressif/esp_ipa` - 图像处理算法

## 许可证

Apache-2.0
