# -*- coding: utf-8 -*-
"""
图像采集测试程序
功能：
1. 通过串口发送触发指令 TRI
2. 记录触发时间戳
3. 从网络摄像头读取视频流
4. 解析帧嵌入的时间戳信息（APP4 TSMP 格式）
5. 以25帧率存储视频
"""

import cv2
import serial
import serial.tools.list_ports
import time
import datetime
import os
import threading
import queue
from dataclasses import dataclass
from typing import Optional, Tuple
import numpy as np
import requests
import socket as _socket
import struct


@dataclass
class FrameData:
    """帧数据结构"""
    frame: np.ndarray
    receive_timestamp: float      # 接收时间戳（系统时间）
    embedded_timestamp_us: int    # 嵌入的时间戳偏移（微秒，相对于触发点）
    embedded_frame_num: int       # 嵌入的帧号
    frame_index: int              # 本地帧索引


def extract_jpeg_timestamp(jpeg_data: bytes) -> Tuple[bool, int, int]:
    """
    从 JPEG 数据中提取嵌入的时间戳
    
    JPEG APP4 段格式:
    - SOI: 0xFF 0xD8
    - APP4: 0xFF 0xE4
    - Length: 2 bytes
    - Identifier: "TSMP" (4 bytes)
    - Timestamp: 8 bytes (大端序, int64, 微秒)
    - Frame Number: 4 bytes (大端序, uint32)
    
    返回: (成功标志, 时间戳微秒, 帧号)
    """
    if len(jpeg_data) < 22:
        return False, 0, 0
    
    # 检查 SOI (Start of Image)
    if jpeg_data[0] != 0xFF or jpeg_data[1] != 0xD8:
        return False, 0, 0
    
    # 检查 APP4 标记
    if jpeg_data[2] != 0xFF or jpeg_data[3] != 0xE4:
        return False, 0, 0
    
    # 检查标识符 "TSMP"
    if jpeg_data[6:10] != b'TSMP':
        return False, 0, 0
    
    # 提取时间戳 (大端序, 8字节)
    timestamp_us = struct.unpack('>q', jpeg_data[10:18])[0]
    
    # 提取帧号 (大端序, 4字节)
    frame_num = struct.unpack('>I', jpeg_data[18:22])[0]
    
    return True, timestamp_us, frame_num


class SerialTrigger:
    """串口触发器类"""
    
    def __init__(self, port: str = "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_6D8112695480-if00", baudrate: int = 1000000, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None
        
    @staticmethod
    def list_available_ports():
        """列出所有可用的串口"""
        ports = serial.tools.list_ports.comports()
        available_ports = []
        for port in ports:
            available_ports.append({
                'device': port.device,
                'description': port.description,
                'hwid': port.hwid
            })
        return available_ports
    
    def connect(self) -> bool:
        """连接串口"""
        try:
            if self.port is None:
                ports = self.list_available_ports()
                if not ports:
                    print("错误：未找到可用的串口设备")
                    return False
                self.port = ports[0]['device']
                print(f"自动选择串口: {self.port}")
            
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"串口 {self.port} 连接成功")
            return True
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("串口已断开")
    
    def send_trigger(self, command: str = "TRI") -> tuple[bool, float]:
        """
        发送触发指令
        返回: (是否成功, 发送时间戳)
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("错误：串口未连接")
            return False, 0.0
        
        try:
            # 发送指令并记录时间戳
            trigger_timestamp = time.time()
            self.serial_conn.write(command.encode('utf-8'))
            self.serial_conn.flush()
            
            trigger_time_str = datetime.datetime.fromtimestamp(trigger_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')
            print(f"触发指令 '{command}' 已发送，时间戳: {trigger_time_str}")
            
            return True, trigger_timestamp
        except serial.SerialException as e:
            print(f"发送触发指令失败: {e}")
            return False, 0.0
    
    def send_stop(self, command: str = "STP") -> bool:
        """
        发送停止指令
        返回: 是否成功
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("错误：串口未连接")
            return False
        
        try:
            self.serial_conn.write(command.encode('utf-8'))
            self.serial_conn.flush()
            print(f"停止指令 '{command}' 已发送")
            return True
        except serial.SerialException as e:
            print(f"发送停止指令失败: {e}")
            return False


class NetworkCameraCapture:
    """网络摄像头采集类 - 使用 HTTP 流式读取 MJPEG"""
    
    def __init__(self, stream_url: str = "http://192.168.24.100/stream", 
                 output_fps: int = 25,
                 output_dir: str = "./recordings"):
        self.stream_url = stream_url
        self.output_fps = output_fps
        self.output_dir = output_dir
        self.is_capturing = False
        # 减小队列大小以降低延迟。之前是200(约8秒延迟)。
        # 如果写入速度慢于采集速度，队列会满。消费者取到的是队列头(最旧数据)。
        # 设置为 5 意味着最大延迟控制在 0.2秒左右。
        self.frame_queue = queue.Queue(maxsize=5)
        # chunk_queue: I/O线程 → 解析+解码线程，存储原始TCP字节块
        # 300块 × 512KB ≈ 150MB 峰值缓冲，确保 I/O 线程永远不被解析阻塞
        self.chunk_queue: queue.Queue = queue.Queue(maxsize=300)
        self.capture_thread: Optional[threading.Thread] = None   # I/O线程（只做 recv）
        self.decode_thread: Optional[threading.Thread] = None    # 解析+解码线程
        self.trigger_timestamp: float = 0.0
        # 原始 TCP socket — socket.recv() 会完全释放 GIL
        self._sock: Optional[_socket.socket] = None
        self._sock_buf = bytearray()  # 连接建立期间读到的 headers 之后的残余数据
        self._boundary = b''          # MJPEG multipart boundary，如 b'--frame'
        self.frame_size: Optional[Tuple[int, int]] = None
        self._buffer = b''  # 兼容旧路径（_read_next_frame），运行时不再使用
        self._stream_iter = None
        self.frame_callback = None  # 帧回调函数，用于直接连接录制器
        
        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)
    
    def set_callback(self, callback):
        """设置帧回调函数"""
        self.frame_callback = callback
    
    def connect(self, retry_count: int = 3, retry_delay: float = 1.0) -> bool:
        """连接网络摄像头（原始 TCP socket，避免 requests/urllib3 的 GIL 开销）"""
        if self._sock is not None:
            print("检测到旧连接，先断开...")
            self.disconnect()
            time.sleep(0.3)

        # 解析 URL
        url = self.stream_url
        host_part = url.replace('http://', '').replace('https://', '')
        slash = host_part.find('/')
        if slash == -1:
            host, path = host_part, '/'
        else:
            host, path = host_part[:slash], host_part[slash:]
        ip, port = (host.rsplit(':', 1)[0], int(host.rsplit(':', 1)[1])) \
                   if ':' in host else (host, 80)

        for attempt in range(retry_count):
            sock = None
            try:
                if attempt > 0:
                    print(f"第 {attempt + 1} 次重试连接...")
                    time.sleep(retry_delay)

                print(f"正在连接摄像头: {self.stream_url}")
                sock = _socket.socket(_socket.AF_INET, _socket.SOCK_STREAM)
                sock.setsockopt(_socket.IPPROTO_TCP, _socket.TCP_NODELAY, 1)
                # 设置接收缓冲区为 4MB 减少内核唤醒次数
                sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_RCVBUF, 4 * 1024 * 1024)
                sock.settimeout(10)
                sock.connect((ip, port))
                sock.settimeout(30)  # 读超时

                # 发送 HTTP/1.0 请求 —— 响应不含 chunked 编码，字节流直接透传
                request = (
                    f"GET {path} HTTP/1.0\r\n"
                    f"Host: {host}\r\n"
                    f"Connection: close\r\n"
                    f"\r\n"
                )
                sock.sendall(request.encode())

                # 读取并跳过 HTTP 响应头
                hbuf = bytearray()
                while b'\r\n\r\n' not in hbuf:
                    data = sock.recv(4096)
                    if not data:
                        raise ConnectionError("连接在读取响应头时关闭")
                    hbuf.extend(data)
                    if len(hbuf) > 65536:
                        raise ValueError("响应头过长")

                first_line = hbuf[:hbuf.find(b'\r\n')].decode('latin-1', errors='replace')
                if '200' not in first_line:
                    raise ValueError(f"HTTP 错误: {first_line}")

                # 从 Content-Type 头提取 MJPEG multipart boundary
                self._boundary = b''
                for hline in bytes(hbuf).split(b'\r\n'):
                    if hline.lower().startswith(b'content-type:') and b'boundary=' in hline.lower():
                        bdry = hline.split(b'boundary=', 1)[-1].strip().split(b';')[0].strip()
                        self._boundary = b'--' + bdry
                        break
                if self._boundary:
                    print(f"  MJPEG boundary: {self._boundary}")
                else:
                    print("  未找到 MJPEG boundary，将使用 JPEG 结构解析（fallback）")

                # 头后面可能已经紧跟了 MJPEG 数据
                header_end = bytes(hbuf).find(b'\r\n\r\n') + 4
                self._sock = sock
                self._sock_buf = hbuf[header_end:]  # 残余数据（已是 bytearray）
                self._buffer = b''

                print(f"摄像头连接成功，等待触发...")
                print(f"  输出帧率: {self.output_fps}")
                return True

            except Exception as e:
                print(f"连接摄像头失败 (尝试 {attempt + 1}/{retry_count}): {e}")
                if sock:
                    try: sock.close()
                    except: pass

        print(f"连接摄像头失败，已重试 {retry_count} 次")
        return False
    
    def _cleanup_connection(self):
        """清理连接资源（用于重试）"""
        self._stream_iter = None
        self._buffer = b''
        self._sock_buf = bytearray()
        self._boundary = b''
        if self._sock:
            try: self._sock.close()
            except: pass
            self._sock = None
    
    def _fill_buffer(self, min_bytes: int) -> bool:
        """向缓冲区追加数据，直到缓冲区长度 >= min_bytes 或流结束。
        返回 True 表示成功获取到了更多数据，False 表示流已结束或出错。"""
        while len(self._buffer) < min_bytes:
            try:
                chunk = next(self._stream_iter, None)
                if chunk is None:
                    return False
                self._buffer += chunk
            except Exception as e:
                partial_data = None
                if hasattr(e, 'partial'):
                    partial_data = e.partial
                else:
                    for arg in e.args:
                        if hasattr(arg, 'partial'):
                            partial_data = arg.partial
                            break
                if partial_data:
                    self._buffer += partial_data
                    print(f"警告: 网络中断，但成功挽救 {len(partial_data)} 字节数据。")
                    return False
                print(f"读取流错误: {e}")
                return False
        return True

    def _read_next_frame(self) -> Optional[bytes]:
        """从流中读取下一帧完整的JPEG数据（基于JPEG段结构解析，避免误判payload内的0xFF 0xD9）

        根本原因说明：
          朴素的 buffer.find(b'\\xff\\xd9') 会命中 JPEG 熵编码数据内部合法存在的 0xFF 0xD9 字节，
          导致帧被截断。截断帧的 APP4 头部可以正常解析出帧号，但实际图像数据不完整，
          下一次搜索 SOI 时会跳过一个真正的帧，从而产生"客户端检测到丢帧"的假报警。
          本方法通过逐段解析 JPEG marker 定位真正的 EOI，消除误判。
        """
        if self._stream_iter is None:
            return None

        # ── 无标记段（非标准字节），丢弃至下一个 SOI ──────────────────
        # Markers that have NO length field (standalone 2-byte markers)
        NO_LENGTH_MARKERS = {
            0xD8,  # SOI
            0xD9,  # EOI
            0x01,  # TEM
        }
        # RST0-RST7 (0xD0-0xD7) also have no length field
        RST_RANGE = range(0xD0, 0xD8)

        try:
            while True:
                # 1. 找 SOI (0xFF 0xD8)
                while True:
                    if not self._fill_buffer(2):
                        return None
                    soi_pos = self._buffer.find(b'\xff\xd8')
                    if soi_pos != -1:
                        # 丢弃 SOI 之前的无效字节
                        if soi_pos > 0:
                            self._buffer = self._buffer[soi_pos:]
                        break
                    # 保留最后1字节（可能是0xFF的前半），丢弃其余
                    self._buffer = self._buffer[-1:]
                    if not self._fill_buffer(2):
                        return None

                # 2. 逐段走 JPEG 结构，直到遇到真正的 EOI
                pos = 0  # 相对于 self._buffer 开头的偏移
                frame_complete = False
                while not frame_complete:
                    # 需要至少 2 字节读 marker
                    if not self._fill_buffer(pos + 2):
                        return None

                    if self._buffer[pos] != 0xFF:
                        # 不应发生（结构紊乱），从头找下一个 SOI
                        self._buffer = self._buffer[pos + 1:]
                        break

                    marker_byte = self._buffer[pos + 1]

                    # 跳过填充字节（0xFF 0xFF ...）
                    if marker_byte == 0xFF:
                        pos += 1
                        continue

                    if marker_byte == 0xD9:
                        # 真正的 EOI
                        pos += 2
                        jpeg_data = bytes(self._buffer[:pos])
                        self._buffer = self._buffer[pos:]
                        return jpeg_data

                    if marker_byte in NO_LENGTH_MARKERS or marker_byte in RST_RANGE:
                        pos += 2
                        continue

                    if marker_byte == 0xDA:
                        # SOS: 先读 SOS 头部长度字段，然后扫描熵编码数据至下一个非填充 marker
                        if not self._fill_buffer(pos + 4):
                            return None
                        sos_length = struct.unpack('>H', self._buffer[pos + 2: pos + 4])[0]
                        pos += 2 + sos_length  # 跳过 SOS marker + 头部

                        # 扫描熵编码数据：遇到 0xFF 后跟非零、非填充字节时才是下一个 marker
                        while True:
                            if not self._fill_buffer(pos + 2):
                                return None
                            if self._buffer[pos] == 0xFF:
                                next_byte = self._buffer[pos + 1]
                                if next_byte == 0x00:
                                    # 0xFF 0x00 是转义，代表数据中的 0xFF，跳过
                                    pos += 2
                                elif next_byte in RST_RANGE:
                                    # RSTn marker，继续熵编码数据
                                    pos += 2
                                else:
                                    # 真正的 marker（包括 EOI 0xD9）
                                    break
                            else:
                                pos += 1
                        # 回到外层循环处理下一个 marker（可能是 EOI）
                        continue

                    # 普通段：读取长度字段并跳过
                    if not self._fill_buffer(pos + 4):
                        return None
                    seg_length = struct.unpack('>H', self._buffer[pos + 2: pos + 4])[0]
                    if seg_length < 2:
                        # 长度字段损坏，丢弃到此并重新找 SOI
                        self._buffer = self._buffer[pos + 2:]
                        break
                    pos += 2 + seg_length  # marker(2) + length_field_value

        except StopIteration:
            return None
        except Exception as e:
            print(f"读取流错误: {e}")
            return None
    
    def read_frame(self) -> Optional[FrameData]:
        """读取一帧（初始化首帧用），直接从 socket 读取，不经过双线程管道。
        优先使用 Content-Length（可靠），fallback 到 JPEG 结构解析。"""
        if self._sock is None:
            return None
        buf = self._sock_buf
        boundary = self._boundary
        deadline = time.time() + 10.0

        def _try_extract(buf) -> Optional[bytes]:
            """从 buf 中用 boundary+Content-Length 提取一帧，失败返回 None。"""
            if not boundary:
                return self._try_extract_jpeg(buf)
            b_pos = buf.find(boundary)
            if b_pos == -1:
                return None
            if b_pos > 0:
                del buf[:b_pos]
            line_end = buf.find(b'\r\n', len(boundary))
            if line_end == -1:
                return None
            pos = line_end + 2
            content_length = -1
            while True:
                hdr_end = buf.find(b'\r\n', pos)
                if hdr_end == -1:
                    return None
                line = bytes(buf[pos:hdr_end])
                pos = hdr_end + 2
                if not line:
                    break
                if line.lower().startswith(b'content-length:'):
                    try:
                        content_length = int(line.split(b':', 1)[1].strip())
                    except:
                        pass
            if content_length < 0:
                del buf[:pos]
                return self._try_extract_jpeg(buf)
            if len(buf) < pos + content_length:
                return None
            jpeg = bytes(buf[pos:pos + content_length])
            del buf[:pos + content_length]
            return jpeg

        while time.time() < deadline:
            jpeg_bytes = _try_extract(buf)
            if jpeg_bytes is not None:
                self._sock_buf = buf
                return self._decode_jpeg(jpeg_bytes, time.time())
            try:
                chunk = self._sock.recv(65536)
                if not chunk:
                    break
                buf.extend(chunk)
            except _socket.timeout:
                continue
            except Exception as e:
                print(f"read_frame 错误: {e}")
                break
        return None

    def _decode_jpeg(self, jpeg_data: bytes, receive_timestamp: float) -> Optional[FrameData]:
        """解码 JPEG 字节为 FrameData（可在任意线程调用）"""
        try:
            success, embedded_ts_us, embedded_frame_num = extract_jpeg_timestamp(jpeg_data)
            if not success:
                embedded_ts_us = 0
                embedded_frame_num = 0

            nparr = np.frombuffer(jpeg_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                return None

            if self.frame_size is None:
                self.frame_size = (frame.shape[1], frame.shape[0])
                print(f"  分辨率: {self.frame_size[0]}x{self.frame_size[1]}")

            return FrameData(
                frame=frame,
                receive_timestamp=receive_timestamp,
                embedded_timestamp_us=embedded_ts_us,
                embedded_frame_num=embedded_frame_num,
                frame_index=0
            )
        except Exception as e:
            print(f"解析帧错误: {e}")
            return None
    
    def disconnect(self):
        """断开摄像头连接"""
        self.stop_capture()
        self._stream_iter = None
        self._buffer = b''
        self._sock_buf = bytearray()
        self._boundary = b''
        if self._sock:
            try: self._sock.close()
            except: pass
            self._sock = None
        # 重置帧尺寸，以便下次连接重新检测
        self.frame_size = None
        print("摄像头已断开")
    
    def _io_worker(self):
        """I/O线程：纯 socket.recv() 循环。
        socket.recv() 是 C 级系统调用，阻塞期间完全释放 GIL，
        12 路摄像头的 recv 可真正并发执行，不再相互抢占 GIL。"""
        sock = self._sock
        if sock is None:
            return

        def _put(chunk):
            try:
                self.chunk_queue.put_nowait(chunk)
            except queue.Full:
                try:
                    self.chunk_queue.get_nowait()
                    self.chunk_queue.put_nowait(chunk)
                except Exception:
                    pass

        # 先把 read_frame() 读取残余后剩下的初始数据投入队列
        if self._sock_buf:
            _put(bytes(self._sock_buf))
            self._sock_buf = bytearray()

        try:
            while self.is_capturing:
                chunk = sock.recv(65536)   # ← GIL 在此完全释放
                if not chunk:
                    break
                _put(chunk)
        except OSError:
            pass  # socket 被 disconnect() 关闭，正常退出
        except Exception as e:
            if self.is_capturing:
                print(f"I/O线程错误 ({self.stream_url}): {e}")

    @staticmethod
    def _try_extract_jpeg(buf: bytearray) -> Optional[bytes]:
        """尝试从 bytearray 中原地提取一帧完整 JPEG。
        找到则返回 bytes 并从 buf 中删除该帧；数据不足则返回 None（buf 不修改或仅丢弃头部无效数据）。"""
        NO_LENGTH = {0xD8, 0xD9, 0x01}
        RST = set(range(0xD0, 0xD8))

        # 找 SOI
        while True:
            soi = buf.find(b'\xff\xd8')
            if soi == -1:
                if len(buf) > 1:
                    del buf[:-1]   # 保留末尾1字节（可能是0xFF前半）
                return None
            if soi > 0:
                del buf[:soi]
            break

        pos = 0
        blen = len(buf)

        while True:
            if pos + 2 > blen:
                return None  # 数据不足，等待更多块

            if buf[pos] != 0xFF:
                del buf[:pos + 1]  # 结构错误，跳过
                return None

            mb = buf[pos + 1]

            if mb == 0xFF:          # 填充字节
                pos += 1
                continue

            if mb == 0xD9:          # 真正的 EOI
                pos += 2
                jpeg = bytes(buf[:pos])
                del buf[:pos]
                return jpeg

            if mb in NO_LENGTH or mb in RST:
                pos += 2
                continue

            if mb == 0xDA:          # SOS：跳过头部后扫描熵编码数据
                if pos + 4 > blen:
                    return None
                sl = (buf[pos + 2] << 8) | buf[pos + 3]
                pos += 2 + sl
                while True:
                    if pos + 2 > blen:
                        return None
                    if buf[pos] == 0xFF:
                        nb = buf[pos + 1]
                        if nb == 0x00:
                            pos += 2
                        elif nb in RST:
                            pos += 2
                        else:
                            break   # 遇到真正的 marker
                    else:
                        pos += 1
                continue

            # 普通带长度段
            if pos + 4 > blen:
                return None
            sl = (buf[pos + 2] << 8) | buf[pos + 3]
            if sl < 2:
                del buf[:pos + 2]
                return None
            pos += 2 + sl

    def _decode_worker(self):
        """解析+解码线程：优先用 MJPEG multipart Content-Length 精确提取帧，
        无 boundary 时 fallback 到 JPEG 结构解析。所有字节操作在本线程，不与其他摄像头竞争 GIL。"""
        buf = bytearray()
        frame_index = 0
        boundary = self._boundary  # bytes, e.g. b'--frame'

        def drain_one(buf) -> Optional[bytes]:
            """从 buf 提取下一帧完整 JPEG；不足则返回 None（buf 可能被截断头部垃圾）。"""
            if not boundary:
                return self._try_extract_jpeg(buf)

            # ── 找 boundary ──────────────────────────────────────────────
            b_pos = buf.find(boundary)
            if b_pos == -1:
                # 保留末尾可能是 boundary 前缀的字节，丢弃其余
                keep = len(boundary) - 1
                if len(buf) > keep:
                    del buf[:len(buf) - keep]
                return None
            if b_pos > 0:
                del buf[:b_pos]  # 丢弃 boundary 前的垃圾

            # ── 读 boundary 行结尾 ────────────────────────────────────────
            line_end = buf.find(b'\r\n', len(boundary))
            if line_end == -1:
                return None  # boundary 行尚未到达
            pos = line_end + 2

            # ── 解析 MIME 部件头部 ────────────────────────────────────────
            content_length = -1
            while True:
                hdr_end = buf.find(b'\r\n', pos)
                if hdr_end == -1:
                    return None  # 头部未完整
                line = bytes(buf[pos:hdr_end])
                pos = hdr_end + 2
                if not line:  # 空行 → 头部结束
                    break
                if line.lower().startswith(b'content-length:'):
                    try:
                        content_length = int(line.split(b':', 1)[1].strip())
                    except:
                        pass

            if content_length < 0:
                # 无 Content-Length → fallback 到 JPEG 结构解析
                del buf[:pos]
                return self._try_extract_jpeg(buf)

            # ── 等待 Content-Length 字节全部到达 ─────────────────────────
            if len(buf) < pos + content_length:
                return None

            jpeg = bytes(buf[pos:pos + content_length])
            del buf[:pos + content_length]
            return jpeg

        while self.is_capturing or not self.chunk_queue.empty():
            try:
                chunk = self.chunk_queue.get(timeout=0.2)
                buf.extend(chunk)
            except queue.Empty:
                if not self.is_capturing:
                    break
                continue

            # 尽量提取 buf 中所有完整帧
            while True:
                jpeg_data = drain_one(buf)
                if jpeg_data is None:
                    break

                receive_timestamp = time.time()
                frame_data = self._decode_jpeg(jpeg_data, receive_timestamp)
                if frame_data is None:
                    continue

                frame_data.frame_index = frame_index

                if self.frame_callback:
                    try:
                        self.frame_callback(frame_data)
                    except Exception as e:
                        print(f"回调错误: {e}")

                try:
                    self.frame_queue.put_nowait(frame_data)
                except queue.Full:
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put_nowait(frame_data)
                    except Exception:
                        pass

                frame_index += 1

    def _capture_worker(self):
        """兼容旧接口，不再使用。"""
        pass
    
    def start_capture(self, trigger_timestamp: float):
        """开始采集：启动 I/O 线程 + 解码线程"""
        if self.is_capturing:
            print("采集已在进行中")
            return

        self.trigger_timestamp = trigger_timestamp
        self.is_capturing = True

        # 清空队列
        for q in (self.frame_queue, self.chunk_queue):
            while not q.empty():
                try:
                    q.get_nowait()
                except queue.Empty:
                    break

        self.capture_thread = threading.Thread(
            target=self._io_worker, daemon=True, name=f"io-{self.stream_url}")
        self.decode_thread = threading.Thread(
            target=self._decode_worker, daemon=True, name=f"dec-{self.stream_url}")
        self.capture_thread.start()
        self.decode_thread.start()
        print("开始采集 (I/O + 解码线程已启动)...")
    
    def stop_capture(self):
        """停止采集：等待 I/O 线程和解码线程退出（不打印，由调用方统一汇报）"""
        self.is_capturing = False
        if self.capture_thread:
            self.capture_thread.join(timeout=1.0)
            self.capture_thread = None
        if self.decode_thread:
            self.decode_thread.join(timeout=2.0)
            self.decode_thread = None
    
    def get_frame(self, timeout: float = 1.0) -> Optional[FrameData]:
        """
        获取一帧
        Args:
            timeout: 等待超时时间(秒)
        """
        try:
            return self.frame_queue.get(timeout=timeout)
        except queue.Empty:
            return None


class VideoRecorder:
    """视频录制类"""
    
    def __init__(self, output_dir: str = "./recordings", fps: int = 25, camera_id: str = ""):
        self.output_dir = output_dir
        self.fps = fps
        self.camera_id = camera_id
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.is_recording = False
        self.frame_count = 0
        self.timestamps = []
        self.trigger_timestamp = 0.0
        self.output_filename = ""
        
        # 异步写入
        self.write_queue = queue.Queue(maxsize=100) # 允许缓冲100帧(约4秒)
        self.write_thread: Optional[threading.Thread] = None
        self.running = False

        # 丢帧检测
        self.last_embedded_frame_num = 0
        self.dropped_frames = []  # 记录丢帧信息
        self.total_dropped = 0
        
        os.makedirs(self.output_dir, exist_ok=True)
    
    def _write_worker(self):
        """写入线程"""
        while self.running or not self.write_queue.empty():
            try:
                frame_data = self.write_queue.get(timeout=0.1)
            except queue.Empty:
                continue
                
            if self.video_writer:
                self.video_writer.write(frame_data.frame)
                
                # 计算相对于触发时间的延迟（系统时间）
                sys_delay_ms = (frame_data.receive_timestamp - self.trigger_timestamp) * 1000
                
                # 嵌入的时间戳（微秒转毫秒）
                embedded_delay_ms = frame_data.embedded_timestamp_us / 1000.0
                
                # 丢帧检测（基于嵌入帧号）
                dropped_info = ""
                if frame_data.embedded_frame_num > 0 and self.last_embedded_frame_num > 0:
                    expected_frame = self.last_embedded_frame_num + 1
                    if frame_data.embedded_frame_num > expected_frame:
                        dropped_count = frame_data.embedded_frame_num - expected_frame
                        self.total_dropped += dropped_count
                        dropped_info = f" [丢帧! 期望={expected_frame}, 实际={frame_data.embedded_frame_num}, 丢失{dropped_count}帧]"
                        self.dropped_frames.append({
                            'local_frame': self.frame_count,
                            'expected': expected_frame,
                            'actual': frame_data.embedded_frame_num,
                            'dropped': dropped_count,
                            'timestamp_ms': embedded_delay_ms
                        })
                        print(f"警告(Cam{self.camera_id}): 检测到丢帧! 本地帧{self.frame_count}, 期望{expected_frame}, 实际{frame_data.embedded_frame_num}")
                
                if frame_data.embedded_frame_num > 0:
                    self.last_embedded_frame_num = frame_data.embedded_frame_num
                
                timestamp_info = (
                    f"帧 {self.frame_count:06d}: "
                    f"嵌入帧号={frame_data.embedded_frame_num}, "
                    f"嵌入延迟={embedded_delay_ms:.3f}ms, "
                    f"系统延迟={sys_delay_ms:.3f}ms, "
                    f"差值={sys_delay_ms - embedded_delay_ms:.3f}ms"
                    f"{dropped_info}"
                )
                self.timestamps.append(timestamp_info)
                
                self.frame_count += 1
                
                # 每100帧打印一次进度
                if self.frame_count % 100 == 0:
                    dropped_str = f", 累计丢帧: {self.total_dropped}" if self.total_dropped > 0 else ""
                    print(f"Cam{self.camera_id} 已录制 {self.frame_count} 帧{dropped_str} (Q:{self.write_queue.qsize()})")

    def start_recording(self, frame_size: tuple, trigger_timestamp: float) -> str:
        """开始录制"""
        self.trigger_timestamp = trigger_timestamp
        
        # 生成文件名
        timestamp_str = datetime.datetime.fromtimestamp(trigger_timestamp).strftime('%Y%m%d_%H%M%S_%f')
        suffix = f"_{self.camera_id}" if self.camera_id else ""
        self.output_filename = os.path.join(self.output_dir, f"recording_{timestamp_str}{suffix}.mp4")
        # timestamp_log_file = os.path.join(self.output_dir, f"timestamps_{timestamp_str}{suffix}.txt") # 这行在原代码中没有被使用来创建文件，是在 stop_recording 里通过 replace 创建的
        
        # 创建视频写入器
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(
            self.output_filename,
            fourcc,
            self.fps,
            frame_size
        )
        
        if not self.video_writer.isOpened():
            print("错误：无法创建视频文件")
            return ""
        
        self.is_recording = True
        self.frame_count = 0
        self.timestamps = []
        
        # 重置丢帧检测
        self.last_embedded_frame_num = 0
        self.dropped_frames = []
        self.total_dropped = 0
        
        # 记录触发时间戳
        self.timestamps.append(f"触发时间戳: {trigger_timestamp}")
        self.timestamps.append(f"触发时间: {datetime.datetime.fromtimestamp(trigger_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')}")
        self.timestamps.append("-" * 50)
        
        # 启动写入线程
        self.running = True
        # 清空队列
        while not self.write_queue.empty():
            try: self.write_queue.get_nowait()
            except: pass
            
        self.write_thread = threading.Thread(target=self._write_worker, daemon=True)
        self.write_thread.start()

        print(f"开始录制: {self.output_filename}")
        return self.output_filename
    
    def write_frame(self, frame_data: FrameData):
        """写入一帧 (异步)"""
        if not self.is_recording:
            return
            
        try:
            self.write_queue.put(frame_data, timeout=0.005)
        except queue.Full:
            # 队列满了，记录
            print(f"警告(Cam{self.camera_id}): 写入队列已满，丢弃一帧!")
    
    def stop_recording(self) -> str:
        """停止录制"""
        if not self.is_recording:
            return ""
        
        self.is_recording = False
        
        # 停止线程
        self.running = False
        if self.write_thread:
            self.write_thread.join(timeout=5.0) # 等待队列写完
            self.write_thread = None
        
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        
        # 添加丢帧统计到日志
        if self.total_dropped > 0:
            self.timestamps.append("-" * 50)
            self.timestamps.append(f"丢帧统计: 总共丢失 {self.total_dropped} 帧")
            self.timestamps.append(f"丢帧详情:")
            for drop in self.dropped_frames:
                self.timestamps.append(
                    f"  本地帧 {drop['local_frame']:06d}: "
                    f"期望嵌入帧号={drop['expected']}, "
                    f"实际={drop['actual']}, "
                    f"丢失{drop['dropped']}帧, "
                    f"时间点={drop['timestamp_ms']:.3f}ms"
                )
        
        # 保存时间戳日志
        if self.output_filename:
            timestamp_log_file = self.output_filename.replace('.mp4', '_timestamps.txt')
            with open(timestamp_log_file, 'w', encoding='utf-8') as f:
                f.write('\n'.join(self.timestamps))
            print(f"时间戳日志已保存: {timestamp_log_file}")
        
        print(f"录制完成，共 {self.frame_count} 帧")
        if self.total_dropped > 0:
            print(f"警告: 累计丢失 {self.total_dropped} 帧, 共 {len(self.dropped_frames)} 次丢帧事件")
        print(f"视频文件: {self.output_filename}")
        
        return self.output_filename


class ImageCollector:
    """图像采集器主类"""
    
    def __init__(self, 
                 stream_urls: list,
                 serial_port: str = "COM10",
                 serial_baudrate: int = 1000000,
                 output_fps: int = 25,
                 output_dir: str = "./recordings"):
        
        self.serial_trigger = SerialTrigger(port=serial_port, baudrate=serial_baudrate)

        
        self.cameras = []
        self.recorders = []
        self.output_fps = output_fps
        
        for url in stream_urls:
            # 提取IP最后一段作为ID
            try:
                ip_part = url.split('//')[1].split('/')[0].split('.')[-1]
            except:
                ip_part = str(len(self.cameras))
                
            self.cameras.append(NetworkCameraCapture(stream_url=url, output_fps=output_fps, output_dir=output_dir))
            self.recorders.append(VideoRecorder(output_dir=output_dir, fps=output_fps, camera_id=ip_part))
        
    def initialize(self) -> bool:
        """初始化"""
        # 1. 连接串口
        print(f"正在连接串口 {self.serial_trigger.port} ...")
        if not self.serial_trigger.connect():
            print("串口连接失败")
            return False
            
        # 2. 连接所有摄像头
        connect_success = True
        for i, camera in enumerate(self.cameras):
            print(f"正在连接摄像头 {i+1}/{len(self.cameras)}: {camera.stream_url}")
            if not camera.connect():
                print(f"摄像头 {camera.stream_url} 连接失败")
                connect_success = False
        
        if not connect_success:
            print("警告: 部分摄像头连接失败")
            # 视需求决定是否退出，这里允许部分连接
            # return False 
            
        return True
    
    def start_collection(self, duration: float = None) -> bool:
        """
        开始采集
        Args:
            duration: 采集时长(秒)，None表示手动停止
        """
        print("\n" + "=" * 50)
        print("开始图像采集")
        print("=" * 50)
        success, trigger_timestamp = self.serial_trigger.send_trigger("Stop\r\n")

        time.sleep(0.1)  # 确保触发指令发送完成
        success, trigger_timestamp = self.serial_trigger.send_trigger("Trig\r\n")

        if not success:
            print("触发失败，但尝试继续采集...")
            trigger_timestamp = time.time()
        else:
            print(f"触发成功，时间戳: {trigger_timestamp}")
            
        print("等待第一帧...")

        if not success:
            trigger_timestamp = time.time()
            print(f"使用当前时间作为触发时间戳: {trigger_timestamp}")
        
        import concurrent.futures

        # 定义并行读取首帧的函数
        def init_camera(idx):
            if idx >= len(self.cameras): return False
            camera = self.cameras[idx]
            recorder = self.recorders[idx]
            
            # 读取第一帧以获取尺寸和嵌入时间戳
            first_frame_data = camera.read_frame()
            if first_frame_data is None:
                print(f"错误：摄像头 {idx} 无法读取视频帧")
                return False
            
            frame_size = camera.frame_size
            print(f"摄像头 {idx} 帧尺寸: {frame_size[0]}x{frame_size[1]}")
            
            # 开始录制
            output_file = recorder.start_recording(frame_size, trigger_timestamp)
            if not output_file:
                return False
            
            # 绑定回调: 采集线程直接写入录制队列，绕过主线程
            camera.set_callback(recorder.write_frame)

            # 写入第一帧
            first_frame_data.frame_index = 0
            recorder.write_frame(first_frame_data)
            
            # 开始采集
            camera.start_capture(trigger_timestamp)
            return True

        # 并行执行初始化
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(self.cameras)) as executor:
            futures = [executor.submit(init_camera, i) for i in range(len(self.cameras))]
            results = [f.result() for f in futures]
            
        if not any(results): # 如果全部失败
            print("所有摄像头启动失败")
            return False

        print("\n采集进行中... (按 'q' 停止)")

        # 每路摄像头的缩略图尺寸（宽, 高）
        preview_size = (480, 270)   # 16:9，4×3 格局时总显示区域 1920×810
        GRID_COLS = 4               # 每行显示的摄像头数量
        n_cams = len(self.cameras)
        grid_rows = (n_cams + GRID_COLS - 1) // GRID_COLS  # 向上取整

        # 初始化显示缓存，避免无数据时闪烁
        last_display_frames = []
        for i in range(n_cams):
            # 初始为空白帧，用灰色标示 Cam ID
            blk = np.zeros((preview_size[1], preview_size[0], 3), dtype=np.uint8)
            cv2.putText(blk, f"Cam {i}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
            last_display_frames.append(blk)
        
        start_time = time.time()
        PREVIEW_BUFFER_FRAMES = 10          # 预览开始前需要缓冲的帧数
        PREVIEW_INTERVAL = 1.0 / 25        # 25fps → 每 40ms 刷新一次预览
        buffered_frames = 0                 # 已收到的帧数（用于缓冲判断）
        last_preview_time = 0.0            # 上次刷新预览的时间
        
        try:
            frame_counter = 0
            while True:
                if duration and (time.time() - start_time) >= duration:
                    print(f"\n达到设定时长 {duration} 秒")
                    break
                
                display_frames = []
                frame_counter += 1
                
                # 遍历处理每个摄像头
                for i, camera in enumerate(self.cameras):
                    # 获取最新帧用于显示 (注意：录制已由回调处理，这里只需处理显示)
                    frame_data = camera.get_frame(timeout=0.001)
                    if frame_data:
                        buffered_frames += 1

                        # 缓冲满后，按 25fps 节拍更新预览帧
                        now = time.time()
                        if buffered_frames >= PREVIEW_BUFFER_FRAMES and (now - last_preview_time) >= PREVIEW_INTERVAL:
                            display_frame = frame_data.frame.copy()
                            display_frame = cv2.resize(display_frame, preview_size)
                            
                            embedded_delay_ms = frame_data.embedded_timestamp_us / 1000.0
                            
                            cv2.putText(display_frame, f"Cam {i}", (6, 22),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                            cv2.putText(display_frame, f"{embedded_delay_ms:.1f}ms", (6, 44),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                                    
                            # 更新缓存
                            last_display_frames[i] = display_frame
                            if i == len(self.cameras) - 1:
                                last_preview_time = now  # 所有摄像头处理完后更新时间戳
                    
                    # 使用缓存的帧（无论是刚更新的还是旧的）
                    display_frames.append(last_display_frames[i])
                
                if display_frames and buffered_frames >= PREVIEW_BUFFER_FRAMES:
                    # 构建网格：每 GRID_COLS 路拼成一行，不足的用黑色空帧补齐
                    blank = np.zeros((preview_size[1], preview_size[0], 3), dtype=np.uint8)
                    padded = display_frames + [blank] * (grid_rows * GRID_COLS - len(display_frames))
                    rows = [
                        np.hstack(padded[c * GRID_COLS:(c + 1) * GRID_COLS])
                        for c in range(grid_rows)
                    ]
                    combined_display = np.vstack(rows)
                    cv2.imshow("Multi-Camera Collector", combined_display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    print("\n用户停止采集")
                    break
                    
        except KeyboardInterrupt:
            print("\n接收到中断信号")

        # ── 立即阻止所有记录器接受新帧 ──────────────────────────────────
        for recorder in self.recorders:
            recorder.is_recording = False

        # ── 并行停止所有摄像头线程 + 所有记录线程 ──────────────────────
        print("正在停止所有摄像头和记录器...")
        stop_threads = []
        for camera in self.cameras:
            t = threading.Thread(target=camera.stop_capture, daemon=True)
            t.start()
            stop_threads.append(t)
        for recorder in self.recorders:
            t = threading.Thread(target=recorder.stop_recording, daemon=True)
            t.start()
            stop_threads.append(t)
        for t in stop_threads:
            t.join(timeout=6.0)
        print(f"全部 {len(self.cameras)} 路摄像头采集已停止")

        # 发送停止指令
        self.serial_trigger.send_stop("Stop\r\n")

        cv2.destroyAllWindows()

        return True
    
    def cleanup(self):
        """清理资源"""
        print("\n清理资源...")
        for camera in self.cameras:
            try:
                camera.disconnect()
            except: pass
        self.serial_trigger.disconnect()
        print("清理完成")


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='图像采集测试程序')
    parser.add_argument('--ip_range', type=str, default='101-112',
                       help='摄像头IP最后一段的范围, 例如 101-112 表示 101到112')
    parser.add_argument('--ip_base', type=str, default='192.168.24.',
                       help='摄像头IP前缀，例如 192.168.24.')
    parser.add_argument('--port', type=str, default='/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_6D8112695480-if00',
                       help='串口端口号 (如 /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=1000000,
                       help='串口波特率')
    parser.add_argument('--fps', type=int, default=25,
                       help='输出视频帧率')
    parser.add_argument('--output', type=str, default='./recordings',
                       help='输出目录')
    parser.add_argument('--duration', type=float, default=None,
                       help='采集时长(秒)，不设置则手动停止')
    
    args = parser.parse_args()
    
    # 解析 IP 范围
    try:
        start_ip, end_ip = map(int, args.ip_range.split('-'))
        ip_list = [f"{i}" for i in range(start_ip, end_ip + 1)]
    except:
        print(f"错误：IP范围格式无效 '{args.ip_range}'，应为 'start-end'")
        return 1
        
    stream_urls = [f"http://{args.ip_base}{ip}/stream" for ip in ip_list]
    
    print("=" * 50)
    print("   图像采集测试程序 (多摄像头版)")
    print("=" * 50)
    print(f"\n配置信息:")
    print(f"  摄像头列表: {stream_urls}")
    print(f"  串口端口: {args.port or '自动检测'}")
    print(f"  波特率: {args.baudrate}")
    print(f"  输出帧率: {args.fps} FPS")
    print(f"  输出目录: {args.output}")
    print(f"  采集时长: {args.duration or '手动停止'}")
    
    # 创建采集器
    collector = ImageCollector(
        stream_urls=stream_urls,
        serial_port=args.port,
        serial_baudrate=args.baudrate,
        output_fps=args.fps,
        output_dir=args.output
    )
    
    try:
        # 初始化
        if not collector.initialize():
            print("初始化失败，程序退出")
            return 1
        
        # 开始采集
        collector.start_collection(duration=args.duration)
        
    except Exception as e:
        print(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        collector.cleanup()
    
    return 0


if __name__ == '__main__':
    exit(main())
