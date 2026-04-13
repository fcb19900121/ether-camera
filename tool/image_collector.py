# -*- coding: utf-8 -*-
"""
图像采集程序（零丢帧重构版）

架构设计：
  每路摄像头一个独立线程，线程内直接执行：
    socket.recv() → MJPEG multipart Content-Length 解析 → JPEG 文件写盘 → 丢帧检测
  无中间队列，无 cv2.imdecode 在录制路径上，GIL 竞争最小化。

与旧版的关键区别：
  - 去掉 chunk_queue / frame_queue 双队列（旧版 drop-oldest 策略会截断 JPEG 导致后续帧全部错乱）
  - 去掉录制路径上的 cv2.imdecode（12路 decode 线程争抢 GIL 是丢帧主因）
  - 每帧存为独立 JPEG 文件（file.write 释放 GIL，12路真正并行）
  - 触发前就启动采集线程（线程阻塞在 recv 等第一帧，触发后立刻拿到数据）
  - 采集结束后可选将 JPEG 序列合成 MP4

功能：
  1. 串口发送 Trig/Stop 触发/停止摄像头
  2. 多路摄像头并行采集，每帧直写 JPEG 文件
  3. APP4 TSMP 嵌入时间戳解析 + 帧号连续性检测
  4. 采集结束后可选合成 MP4
"""

import os
import sys
import time
import datetime
import struct
import threading
import argparse
import glob
import signal
from typing import Optional, Tuple, List

import socket as _socket
import serial
import serial.tools.list_ports
import numpy as np
import cv2

# 全局停止事件：Ctrl+C / q / ESC 均可触发
g_stop_event = threading.Event()


# ═══════════════════════════════════════════════════════════════════════════
#  JPEG 时间戳提取
# ═══════════════════════════════════════════════════════════════════════════

def extract_jpeg_timestamp(jpeg_data: bytes) -> Tuple[bool, int, int]:
    """从 JPEG APP4 段提取嵌入的时间戳和帧号。

    格式: SOI(2) + APP4(2) + Length(2) + "TSMP"(4) + timestamp_us(8) + frame_num(4)
    返回: (成功, 时间戳微秒, 帧号)
    """
    if len(jpeg_data) < 22:
        return False, 0, 0
    if jpeg_data[0:2] != b'\xff\xd8':
        return False, 0, 0
    if jpeg_data[2:4] != b'\xff\xe4':
        return False, 0, 0
    if jpeg_data[6:10] != b'TSMP':
        return False, 0, 0
    ts_us = struct.unpack('>q', jpeg_data[10:18])[0]
    frame_num = struct.unpack('>I', jpeg_data[18:22])[0]
    return True, ts_us, frame_num


# ═══════════════════════════════════════════════════════════════════════════
#  串口触发器
# ═══════════════════════════════════════════════════════════════════════════

class SerialTrigger:
    """串口触发器：发送 Trig/Stop 指令"""

    def __init__(self, port: str, baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self._conn: Optional[serial.Serial] = None

    @staticmethod
    def list_ports():
        return [
            {'device': p.device, 'desc': p.description, 'hwid': p.hwid}
            for p in serial.tools.list_ports.comports()
        ]

    def connect(self) -> bool:
        try:
            if not self.port:
                ports = self.list_ports()
                if not ports:
                    print("[串口] 未找到可用串口")
                    return False
                self.port = ports[0]['device']
                print(f"[串口] 自动选择: {self.port}")
            self._conn = serial.Serial(
                self.port, self.baudrate, timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"[串口] {self.port} 已连接")
            return True
        except serial.SerialException as e:
            print(f"[串口] 连接失败: {e}")
            return False

    def disconnect(self):
        if self._conn and self._conn.is_open:
            self._conn.close()
            self._conn = None

    def send(self, cmd: str) -> Tuple[bool, float]:
        """发送指令，返回 (成功, 发送时刻 time.time())"""
        if not self._conn or not self._conn.is_open:
            return False, 0.0
        try:
            ts = time.time()
            self._conn.write(cmd.encode('utf-8'))
            self._conn.flush()
            return True, ts
        except serial.SerialException as e:
            print(f"[串口] 发送失败: {e}")
            return False, 0.0


# ═══════════════════════════════════════════════════════════════════════════
#  单路摄像头采集
# ═══════════════════════════════════════════════════════════════════════════

class CameraCapture:
    """单路摄像头：一个线程完成 recv → 解析 → 存盘。

    关键设计：
    - 无中间队列：recv 到的数据直接在本线程解析并写盘
    - 只用 Content-Length 定界帧（无 JPEG 结构解析，避免半帧截断）
    - socket.recv() 和 file.write() 都释放 GIL，12路线程真正并行
    - cv2.imdecode 仅用于预览（可选），不在录制路径上
    """

    def __init__(self, stream_url: str, camera_id: str):
        self.stream_url = stream_url
        self.camera_id = camera_id
        self.save_dir = ""
        self.trigger_timestamp = 0.0

        self._sock: Optional[_socket.socket] = None
        self._boundary = b''
        self._buf = bytearray()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._is_chunked = False  # 是否为 HTTP chunked 传输编码

        # 供主线程读取的最新 JPEG（仅用于预览，不影响录制）
        self._display_jpeg: Optional[bytes] = None
        self._display_lock = threading.Lock()

        # 统计
        self.total_frames = 0
        self.total_dropped = 0
        self.frame_size: Optional[Tuple[int, int]] = None
        self._last_frame_num = 0
        self.status = "初始化"  # 状态描述（主线程可读）

    # ── 连接 ────────────────────────────────────────────────────────

    def connect(self, timeout: float = 10.0) -> bool:
        """建立到 /stream 的 HTTP 连接，解析响应头。"""
        url = self.stream_url
        host_part = url.replace('http://', '').replace('https://', '')
        slash = host_part.find('/')
        host, path = (host_part[:slash], host_part[slash:]) if slash != -1 else (host_part, '/')
        if ':' in host:
            ip, port = host.rsplit(':', 1)[0], int(host.rsplit(':', 1)[1])
        else:
            ip, port = host, 80

        try:
            sock = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
            sock.setsockopt(_socket.IPPROTO_TCP, _socket.TCP_NODELAY, 1)
            sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_RCVBUF, 4 * 1024 * 1024)
            sock.settimeout(timeout)
            sock.connect((ip, port))

            request = (
                f"GET {path} HTTP/1.0\r\n"
                f"Host: {host}\r\n"
                f"Connection: close\r\n"
                f"\r\n"
            )
            sock.sendall(request.encode())

            # 读取 HTTP 响应头
            hbuf = bytearray()
            while b'\r\n\r\n' not in hbuf:
                d = sock.recv(4096)
                if not d:
                    raise ConnectionError("连接关闭")
                hbuf.extend(d)
                if len(hbuf) > 65536:
                    raise ValueError("响应头过长")

            first_line = hbuf[:hbuf.find(b'\r\n')].decode('latin-1', errors='replace')
            if '200' not in first_line:
                sock.close()
                raise ValueError(f"HTTP 错误: {first_line}")

            # 检测 Transfer-Encoding: chunked
            self._is_chunked = False
            for line in bytes(hbuf).split(b'\r\n'):
                low = line.lower()
                if low.startswith(b'transfer-encoding:') and b'chunked' in low:
                    self._is_chunked = True
                    print(f"[Cam {self.camera_id}] 警告: 检测到 chunked 编码")

            # 提取 MJPEG multipart boundary
            self._boundary = b''
            for line in bytes(hbuf).split(b'\r\n'):
                low = line.lower()
                if low.startswith(b'content-type:') and b'boundary=' in low:
                    bdry = line.split(b'boundary=', 1)[-1].strip().split(b';')[0].strip()
                    self._boundary = b'--' + bdry
                    break

            header_end = bytes(hbuf).find(b'\r\n\r\n') + 4
            self._buf = bytearray(hbuf[header_end:])
            self._sock = sock
            sock.settimeout(5)  # 采集期间：5秒超时

            self.status = "已连接"
            print(f"[Cam {self.camera_id}] 已连接 ({ip}:{port}), "
                  f"boundary={'有' if self._boundary else '无'}, "
                  f"residual={len(self._buf)}字节")
            return True

        except Exception as e:
            self.status = f"连接失败: {e}"
            print(f"[Cam {self.camera_id}] 连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接并停止线程"""
        self.stop()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    # ── 启动 / 停止 ──────────────────────────────────────────────────

    def start(self):
        """启动采集线程（非阻塞）。线程内部先跳过初始 boundary。"""
        if self._running:
            return
        self._running = True
        self.total_frames = 0
        self.total_dropped = 0
        self._last_frame_num = 0
        os.makedirs(self.save_dir, exist_ok=True)
        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True,
            name=f"cap-{self.camera_id}"
        )
        self._thread.start()

    def stop(self):
        """停止采集线程"""
        self._running = False
        # shutdown 解除 recv() 阻塞
        if self._sock:
            try:
                self._sock.shutdown(_socket.SHUT_RDWR)
            except OSError:
                pass
        if self._thread:
            self._thread.join(timeout=5.0)
            self._thread = None

    # ── 采集主循环（在独立线程中运行）────────────────────────────────

    def _capture_loop(self):
        meta_path = os.path.join(self.save_dir, 'meta.csv')
        meta_fp = open(meta_path, 'w', encoding='utf-8')
        meta_fp.write('local_idx,embed_frame_num,embed_ts_us,recv_ts,dropped\n')
        meta_fp.flush()  # 立即刷盘，确保 header 可见

        local_idx = 0
        drop_events = []

        self.status = "等待帧数据..."
        print(f"[Cam {self.camera_id}] 采集线程启动, 等待帧数据...")

        try:
            while self._running and not g_stop_event.is_set():
                jpeg = self._read_next_jpeg()
                if jpeg is None:
                    if self._running and not g_stop_event.is_set():
                        self.status = "流中断"
                        print(f"[Cam {self.camera_id}] 流中断 (recv 返回空)")
                    break

                recv_ts = time.time()

                # 首帧：打印调试信息
                if local_idx == 0:
                    self.status = "接收中"
                    print(f"[Cam {self.camera_id}] 首帧到达! "
                          f"JPEG大小={len(jpeg)}字节, "
                          f"头部={jpeg[:4].hex() if len(jpeg) >= 4 else 'N/A'}")

                # 提取嵌入元数据
                ok, embed_ts, embed_frame = extract_jpeg_timestamp(jpeg)

                # 首帧检测分辨率（只做一次 decode）
                if self.frame_size is None:
                    nparr = np.frombuffer(jpeg, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if img is not None:
                        self.frame_size = (img.shape[1], img.shape[0])
                        print(f"[Cam {self.camera_id}] 分辨率: {self.frame_size[0]}x{self.frame_size[1]}")

                # 丢帧检测
                dropped = 0
                if ok and self._last_frame_num > 0 and embed_frame > self._last_frame_num + 1:
                    dropped = embed_frame - self._last_frame_num - 1
                    self.total_dropped += dropped
                    drop_events.append((local_idx, self._last_frame_num + 1, embed_frame, dropped))
                    print(f"[Cam {self.camera_id}] 丢帧! 本地#{local_idx} "
                          f"期望帧号{self._last_frame_num + 1} 实际{embed_frame} 丢失{dropped}帧")
                if ok and embed_frame > 0:
                    self._last_frame_num = embed_frame

                # 写入 JPEG 文件（file.write 释放 GIL）
                filepath = os.path.join(self.save_dir, f'{local_idx:06d}.jpg')
                with open(filepath, 'wb') as f:
                    f.write(jpeg)

                # 写元数据行并立即刷盘
                meta_fp.write(f'{local_idx},{embed_frame if ok else -1},'
                              f'{embed_ts if ok else 0},{recv_ts:.6f},{dropped}\n')
                if local_idx % 10 == 0:
                    meta_fp.flush()

                # 更新预览帧引用（主线程按需 decode）
                with self._display_lock:
                    self._display_jpeg = jpeg

                local_idx += 1
                self.total_frames = local_idx

                if local_idx % 100 == 0:
                    d = f", 丢帧:{self.total_dropped}" if self.total_dropped else ""
                    print(f"[Cam {self.camera_id}] {local_idx} 帧{d}")

        except Exception as e:
            if self._running:
                self.status = f"异常: {e}"
                print(f"[Cam {self.camera_id}] 采集异常: {e}")
                import traceback
                traceback.print_exc()
        finally:
            meta_fp.flush()
            meta_fp.close()

        # 写入摘要
        summary_path = os.path.join(self.save_dir, 'summary.txt')
        with open(summary_path, 'w', encoding='utf-8') as f:
            f.write(f"camera_id: {self.camera_id}\n")
            f.write(f"stream_url: {self.stream_url}\n")
            f.write(f"trigger_ts: {self.trigger_timestamp}\n")
            f.write(f"total_frames: {self.total_frames}\n")
            f.write(f"total_dropped: {self.total_dropped}\n")
            f.write(f"frame_size: {self.frame_size}\n")
            if drop_events:
                f.write(f"\ndrop_events ({len(drop_events)}):\n")
                for li, expected, actual, cnt in drop_events:
                    f.write(f"  local#{li}: expected={expected}, actual={actual}, lost={cnt}\n")

        self.status = f"完成: {self.total_frames}帧"
        print(f"[Cam {self.camera_id}] 采集结束, 共 {self.total_frames} 帧, 丢帧 {self.total_dropped}")

    # ── MJPEG 帧读取 ─────────────────────────────────────────────────

    def _ensure_bytes(self, n: int) -> bool:
        """确保 self._buf 至少有 n 字节"""
        _wait_logged = False
        while len(self._buf) < n:
            if not self._running or g_stop_event.is_set():
                return False
            try:
                chunk = self._sock.recv(65536)
                if not chunk:
                    return False
                self._buf.extend(chunk)
            except _socket.timeout:
                if not self._running or g_stop_event.is_set():
                    return False
                if not _wait_logged:
                    _wait_logged = True
                    print(f"[Cam {self.camera_id}] 等待数据中... "
                          f"buf={len(self._buf)}/{n}字节, "
                          f"头部={bytes(self._buf[:40])!r}")
                continue  # 超时重试
            except OSError:
                return False  # socket 被 shutdown 或关闭
        return True

    def _skip_initial_boundary(self):
        """跳过服务端在帧数据之前发送的初始 boundary。

        ESP32 的 stream_handler 在等待触发之前就发送了一个裸 boundary
        (仅 '\\r\\n--boundary\\r\\n'，没有 Content-Type / Content-Length)。
        如果不跳过，_read_next_jpeg 会把它当成一个空 part，浪费一次迭代，
        在某些时序下可能与后续帧的 boundary 拼接导致解析错位。
        """
        if not self._boundary:
            return
        # 等待足够数据（boundary + 一些余量）
        need = len(self._boundary) + 20
        deadline = time.time() + 30  # 最多等 30 秒
        while len(self._buf) < need:
            if not self._running or g_stop_event.is_set():
                return
            try:
                chunk = self._sock.recv(4096)
                if not chunk:
                    return
                self._buf.extend(chunk)
            except _socket.timeout:
                if time.time() > deadline:
                    print(f"[Cam {self.camera_id}] 等待初始数据超时")
                    return
                if not self._running or g_stop_event.is_set():
                    return
                continue
            except OSError:
                return

        # 查找第一个 boundary
        pos = self._buf.find(self._boundary)
        if pos == -1:
            return  # 没有找到 boundary，不做处理

        # 检查这个 boundary 后面是否紧跟另一个 boundary（说明是裸初始 boundary）
        # 或者后面是否有 Content-Type 头（说明是有效帧 part）
        after_boundary = pos + len(self._boundary)
        # 确保有足够数据看后面内容
        self._ensure_bytes(after_boundary + 50)
        remaining = bytes(self._buf[after_boundary:after_boundary + 50])

        if remaining.lower().startswith(b'\r\n') and not remaining[2:].lower().startswith(b'content-'):
            # 是裸 boundary（后面不是 Content-Type），跳过整个初始 boundary
            # 找到 boundary 后面的换行
            skip_end = self._buf.find(b'\r\n', after_boundary)
            if skip_end != -1:
                del self._buf[:skip_end + 2]
            else:
                del self._buf[:after_boundary]
            print(f"[Cam {self.camera_id}] 跳过初始 boundary, 剩余 {len(self._buf)} 字节")
        elif pos > 0:
            # boundary 前有垃圾数据，清理
            del self._buf[:pos]

    def _read_next_jpeg(self) -> Optional[bytes]:
        """从 MJPEG multipart 流读取下一帧 JPEG。

        策略：直接搜索 Content-Length 头部，精确读取 N 字节，
        跳过所有 boundary / 空 part / 非 JPEG 数据。
        不依赖 boundary 定界，避免空 part 状态机的边界问题。
        """
        while self._running and not g_stop_event.is_set():
            # ── 等待足够数据才开始搜索 ────────────────────────────────
            if not self._ensure_bytes(16):
                return None

            buf = self._buf

            # ── 找 "Content-Length:" 头（大小写不敏感）────────────────
            cl_pos = -1
            search_upper = buf[:min(len(buf), 4096)].lower()
            cl_pos = search_upper.find(b'content-length:')

            if cl_pos == -1:
                # 没找到：可能数据不够，或者当前数据是纯 boundary/垃圾
                # 丢掉开头，留够一个 Content-Length 行的余量继续等
                if len(buf) > 256:
                    del buf[:len(buf) - 128]
                if not self._ensure_bytes(len(buf) + 512):
                    return None
                continue

            # ── 解析 Content-Length 值 ────────────────────────────────
            line_end = buf.find(b'\r\n', cl_pos)
            if line_end == -1:
                if not self._ensure_bytes(cl_pos + 128):
                    return None
                continue
            try:
                content_length = int(bytes(buf[cl_pos + 15: line_end]).strip())
            except ValueError:
                del buf[:cl_pos + 2]
                continue
            if content_length <= 0 or content_length > 2_000_000:
                del buf[:cl_pos + 2]
                continue

            # ── 找 headers 结束标志 \r\n\r\n ──────────────────────────
            hdr_end = buf.find(b'\r\n\r\n', cl_pos)
            if hdr_end == -1:
                if not self._ensure_bytes(cl_pos + 512):
                    return None
                continue
            data_start = hdr_end + 4

            # ── 精确读取 content_length 字节 ──────────────────────────
            needed = data_start + content_length
            if not self._ensure_bytes(needed):
                return None

            jpeg = bytes(buf[data_start:needed])
            del buf[:needed]

            # ESP32 在 part body 开头插入额外的 \r\n，Content-Length 包含这 2 字节
            jpeg = jpeg.lstrip(b'\r\n')

            # ── 验证 JPEG SOI ─────────────────────────────────────────
            if len(jpeg) >= 2 and jpeg[:2] == b'\xff\xd8':
                return jpeg
            # 不是有效 JPEG，继续找下一帧
            print(f"[Cam {self.camera_id}] 跳过非JPEG数据 "
                  f"content_length={content_length} 头部={jpeg[:4].hex()}")

        return None


    def _read_next_jpeg_fallback(self) -> Optional[bytes]:
        """无 boundary 时的 fallback：用 JPEG SOI (FFD8) 分界"""
        while self._running and not g_stop_event.is_set():
            soi = self._buf.find(b'\xff\xd8')
            if soi == -1:
                if len(self._buf) > 1:
                    del self._buf[:-1]
                if not self._ensure_bytes(2):
                    return None
                continue
            if soi > 0:
                del self._buf[:soi]

            # 找下一个 SOI
            next_soi = self._buf.find(b'\xff\xd8', 2)
            while next_soi == -1:
                if not self._ensure_bytes(len(self._buf) + 8192):
                    if len(self._buf) > 2:
                        jpeg = bytes(self._buf)
                        self._buf.clear()
                        return jpeg
                    return None
                next_soi = self._buf.find(b'\xff\xd8', 2)

            jpeg = bytes(self._buf[:next_soi])
            del self._buf[:next_soi]
            return jpeg

        return None

    # ── 预览接口 ──────────────────────────────────────────────────────

    def get_display_frame(self) -> Optional[np.ndarray]:
        """主线程调用：获取最新帧 BGR 图像用于预览（按需 decode，不影响录制）"""
        with self._display_lock:
            jpeg = self._display_jpeg
            self._display_jpeg = None
        if jpeg is None:
            return None
        nparr = np.frombuffer(jpeg, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        return img


# ═══════════════════════════════════════════════════════════════════════════
#  JPEG 序列 → MP4 转换
# ═══════════════════════════════════════════════════════════════════════════

def convert_jpegs_to_mp4(jpeg_dir: str, output_path: str, fps: int = 25):
    """将目录下按文件名排序的 .jpg 文件合成 MP4"""
    files = sorted(glob.glob(os.path.join(jpeg_dir, '*.jpg')))
    if not files:
        print(f"  {jpeg_dir}: 无 JPEG 文件")
        return

    first = cv2.imread(files[0])
    if first is None:
        print(f"  {jpeg_dir}: 首帧读取失败")
        return

    h, w = first.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))
    count = 0
    for f in files:
        img = cv2.imread(f)
        if img is not None:
            writer.write(img)
            count += 1
    writer.release()
    print(f"  MP4: {output_path} ({count} 帧)")


# ═══════════════════════════════════════════════════════════════════════════
#  主采集器
# ═══════════════════════════════════════════════════════════════════════════

class ImageCollector:
    """多路摄像头图像采集器"""

    def __init__(self, stream_urls: List[str], serial_port: str,
                 serial_baudrate: int = 1000000, fps: int = 25,
                 output_dir: str = './recordings'):
        self.fps = fps
        self.output_dir = output_dir
        self.serial = SerialTrigger(serial_port, serial_baudrate)
        self.cameras: List[CameraCapture] = []

        for url in stream_urls:
            try:
                cam_id = url.split('//')[1].split('/')[0].split('.')[-1]
            except (IndexError, AttributeError):
                cam_id = str(len(self.cameras))
            self.cameras.append(CameraCapture(stream_url=url, camera_id=cam_id))

    def run(self, duration: float = None) -> bool:
        # ── 1. 连接串口 ──────────────────────────────────────────────
        if not self.serial.connect():
            return False

        # ── 2. 并行连接所有摄像头 ────────────────────────────────────
        print(f"\n连接 {len(self.cameras)} 路摄像头...")
        connect_ok = [False] * len(self.cameras)
        threads = []

        def _connect(idx):
            connect_ok[idx] = self.cameras[idx].connect()

        for i in range(len(self.cameras)):
            t = threading.Thread(target=_connect, args=(i,))
            t.start()
            threads.append(t)
        for t in threads:
            t.join()

        connected = sum(connect_ok)
        if connected == 0:
            print("所有摄像头连接失败")
            self.serial.disconnect()
            return False
        print(f"\n{connected}/{len(self.cameras)} 路摄像头已连接")

        # ── 3. 创建本次采集目录 ──────────────────────────────────────
        ts_dir = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        session_dir = os.path.join(self.output_dir, ts_dir)
        os.makedirs(session_dir, exist_ok=True)
        print(f"输出目录: {session_dir}")

        # ── 4. 先启动所有采集线程，线程内部阻塞在 recv 等待第一帧 ────────
        for i, cam in enumerate(self.cameras):
            if connect_ok[i]:
                cam.save_dir = os.path.join(session_dir, f'cam_{cam.camera_id}')
                cam.trigger_timestamp = 0.0   # 触发后立即更新
                cam.start()

        # ── 5. 所有线程已就位后再发送触发指令 ─────────────────────
        time.sleep(0.05)  # 等线程就绪（进入 recv 阻塞）
        self.serial.send("Stop\r\n")
        time.sleep(0.1)
        ok, trigger_ts = self.serial.send("Trig\r\n")
        if not ok:
            trigger_ts = time.time()
            print("[触发] 指令发送失败，使用当前时间")
        else:
            ts_str = datetime.datetime.fromtimestamp(trigger_ts).strftime('%H:%M:%S.%f')
            print(f"[触发] 已发送 @ {ts_str}")

        # 更新所有摄像头的触发时间戳
        for cam in self.cameras:
            cam.trigger_timestamp = trigger_ts

        # 保存触发信息
        with open(os.path.join(session_dir, 'trigger_info.txt'), 'w', encoding='utf-8') as f:
            f.write(f"trigger_timestamp: {trigger_ts}\n")
            f.write(f"trigger_time: {datetime.datetime.fromtimestamp(trigger_ts).strftime('%Y-%m-%d %H:%M:%S.%f')}\n")
            f.write(f"cameras: {connected}\n")
            for i, cam in enumerate(self.cameras):
                f.write(f"  cam_{cam.camera_id}: {cam.stream_url} (connected={connect_ok[i]})\n")

        print(f"\n采集中...\n"
              f"  停止方式: OpenCV窗口中按 q/ESC, 或终端中按 Ctrl+C\n")

        # ── 6. 主线程：低帧率预览显示 ────────────────────────────────
        try:
            self._display_loop(duration)
        except KeyboardInterrupt:
            print("\n[Ctrl+C] 收到中断信号")
            g_stop_event.set()

        # ── 7. 停止 ────────────────────────────────────────────────
        g_stop_event.set()
        print("\n正在停止...")
        self.serial.send("Stop\r\n")

        stop_threads = []
        for cam in self.cameras:
            t = threading.Thread(target=cam.stop, daemon=True)
            t.start()
            stop_threads.append(t)
        for t in stop_threads:
            t.join(timeout=6.0)

        cv2.destroyAllWindows()

        # ── 8. 汇总 ──────────────────────────────────────────────────
        self._print_summary()

        # ── 9. 可选合成 MP4 ──────────────────────────────────────────
        self._offer_mp4_conversion(session_dir)

        # ── 清理 ─────────────────────────────────────────────────────
        for cam in self.cameras:
            cam.disconnect()
        self.serial.disconnect()

        return True

    def _display_loop(self, duration: float = None):
        """主线程预览循环：低帧率 decode + 网格显示。
        支持 q/ESC（OpenCV窗口焦点）和 Ctrl+C（终端焦点）两种停止方式。"""
        PREVIEW_W, PREVIEW_H = 480, 270
        GRID_COLS = 4
        n = len(self.cameras)
        grid_rows = (n + GRID_COLS - 1) // GRID_COLS
        blank = np.zeros((PREVIEW_H, PREVIEW_W, 3), dtype=np.uint8)

        # 初始化缓存
        cache = []
        for cam in self.cameras:
            b = blank.copy()
            cv2.putText(b, f"Cam {cam.camera_id}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
            cache.append(b)

        start_time = time.time()
        last_status_time = 0.0

        while not g_stop_event.is_set():
            if duration and (time.time() - start_time) >= duration:
                print(f"\n达到设定时长 {duration}s")
                break

            now = time.time()

            # 每 5 秒在终端打印状态（方便终端焦点时也能看到进度）
            if now - last_status_time >= 5.0:
                last_status_time = now
                statuses = [f"Cam{c.camera_id}:{c.total_frames}帧" for c in self.cameras if c.total_frames > 0]
                if statuses:
                    print(f"  [状态] {', '.join(statuses)}")
                else:
                    statuses = [f"Cam{c.camera_id}:{c.status}" for c in self.cameras]
                    print(f"  [状态] {', '.join(statuses[:4])}...")

            # 从每路摄像头获取最新帧（按需 decode，不影响录制线程）
            for i, cam in enumerate(self.cameras):
                img = cam.get_display_frame()
                if img is not None:
                    img = cv2.resize(img, (PREVIEW_W, PREVIEW_H))

                    # 叠加信息
                    cv2.putText(img, f"Cam {cam.camera_id}", (6, 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                    cv2.putText(img, f"#{cam.total_frames}", (6, 44),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                    if cam.total_dropped > 0:
                        cv2.putText(img, f"DROP:{cam.total_dropped}", (6, 64),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
                    cache[i] = img

            # 构建网格
            padded = cache + [blank] * (grid_rows * GRID_COLS - n)
            grid = np.vstack([
                np.hstack(padded[r * GRID_COLS:(r + 1) * GRID_COLS])
                for r in range(grid_rows)
            ])
            cv2.imshow("Image Collector", grid)

            key = cv2.waitKey(40) & 0xFF  # ~25fps 刷新率
            if key == ord('q') or key == 27:
                print("用户停止 (q/ESC)")
                break

    def _print_summary(self):
        print("\n" + "=" * 60)
        print("  采集汇总")
        print("=" * 60)
        total_f, total_d = 0, 0
        for cam in self.cameras:
            total_f += cam.total_frames
            total_d += cam.total_dropped
            d = f"  丢帧: {cam.total_dropped}" if cam.total_dropped else ""
            sz = f"  {cam.frame_size[0]}x{cam.frame_size[1]}" if cam.frame_size else ""
            print(f"  Cam {cam.camera_id}: {cam.total_frames} 帧{sz}{d}")
        print(f"  合计: {total_f} 帧, 丢帧 {total_d}")
        print("=" * 60)

    def _offer_mp4_conversion(self, session_dir: str):
        """询问是否合成 MP4"""
        has_frames = any(cam.total_frames > 0 for cam in self.cameras)
        if not has_frames:
            return

        print("\n合成 MP4? (y/n): ", end='', flush=True)
        try:
            ans = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            ans = 'n'
        if ans != 'y':
            return

        print("正在合成 MP4...")
        for cam in self.cameras:
            if cam.total_frames > 0 and cam.save_dir:
                mp4_path = os.path.join(session_dir, f'cam_{cam.camera_id}.mp4')
                convert_jpegs_to_mp4(cam.save_dir, mp4_path, self.fps)
        print("MP4 合成完成")


# ═══════════════════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════════════════

def main():
    # Ctrl+C 信号处理
    def _sigint_handler(sig, frame):
        print("\n[SIGINT] 正在停止...")
        g_stop_event.set()

    signal.signal(signal.SIGINT, _sigint_handler)

    parser = argparse.ArgumentParser(description='图像采集程序（零丢帧版）')
    parser.add_argument('--ip_range', default='101-112',
                        help='摄像头IP末段范围, 如 101-112')
    parser.add_argument('--ip_base', default='192.168.24.',
                        help='摄像头IP前缀')
    parser.add_argument('--port',
                        default='/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_6D8112695480-if00',
                        help='串口 (Windows 用 COM10 等)')
    parser.add_argument('--baudrate', type=int, default=1000000)
    parser.add_argument('--fps', type=int, default=25, help='标称帧率（用于 MP4 合成）')
    parser.add_argument('--output', default='./recordings', help='输出目录')
    parser.add_argument('--duration', type=float, default=None, help='采集时长(秒)')

    args = parser.parse_args()

    try:
        start_ip, end_ip = map(int, args.ip_range.split('-'))
    except ValueError:
        print(f"IP范围格式无效: {args.ip_range}")
        return 1

    urls = [f"http://{args.ip_base}{i}/stream" for i in range(start_ip, end_ip + 1)]

    print("=" * 60)
    print("  图像采集程序（零丢帧版）")
    print("=" * 60)
    print(f"  摄像头: {len(urls)} 路 ({args.ip_base}{start_ip} ~ {args.ip_base}{end_ip})")
    print(f"  串口: {args.port}")
    print(f"  帧率: {args.fps} FPS")
    print(f"  输出: {args.output}")
    print(f"  时长: {args.duration or '手动停止'}")
    print()

    collector = ImageCollector(
        stream_urls=urls,
        serial_port=args.port,
        serial_baudrate=args.baudrate,
        fps=args.fps,
        output_dir=args.output
    )

    try:
        collector.run(duration=args.duration)
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 正在退出...")
        g_stop_event.set()
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
