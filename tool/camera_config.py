#!/usr/bin/env python3
"""
ESP32-P4 Ethernet Camera Configuration Tool
============================================
Listens for UDP broadcast announcements from the camera and provides
a GUI to configure parameters over UDP.

Requirements: Python 3.8+  (stdlib only – no third-party packages needed)
Usage:  python camera_config.py
"""

import json
import io
import queue
import socket
import threading
import tkinter as tk
import time
import urllib.request
from tkinter import messagebox, ttk

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

if PIL_AVAILABLE:
    RESAMPLE_LANCZOS = Image.Resampling.LANCZOS if hasattr(Image, "Resampling") else Image.LANCZOS

UDP_PORT      = 4210
BROADCAST_IP  = "255.255.255.255"
RECV_TIMEOUT  = 1.0   # seconds – socket read timeout in listener thread


# ---------------------------------------------------------------------------
#  UDP helpers
# ---------------------------------------------------------------------------

def udp_send_command(ip: str, port: int, cmd: dict) -> dict | None:
    """Send a JSON command to the camera and return the parsed JSON reply."""
    payload = json.dumps(cmd).encode()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(3.0)
        try:
            s.sendto(payload, (ip, port))
            data, _ = s.recvfrom(1024)
            return json.loads(data.decode())
        except (socket.timeout, json.JSONDecodeError, OSError) as e:
            return {"ok": False, "error": str(e)}


# ---------------------------------------------------------------------------
#  Background broadcast listener
# ---------------------------------------------------------------------------

class BroadcastListener(threading.Thread):
    """Daemon thread that collects UDP announce packets into a queue."""

    def __init__(self, port: int, result_queue: queue.Queue):
        super().__init__(daemon=True)
        self._port = port
        self._queue = result_queue
        self._stop_evt = threading.Event()

    def stop(self):
        self._stop_evt.set()

    def run(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            except OSError:
                pass
            s.bind(("", self._port))
            s.settimeout(RECV_TIMEOUT)
            while not self._stop_evt.is_set():
                try:
                    data, addr = s.recvfrom(1024)
                    try:
                        pkt = json.loads(data.decode())
                        if pkt.get("type") == "announce":
                            pkt["_src_ip"] = addr[0]
                            self._queue.put(pkt)
                    except json.JSONDecodeError:
                        pass
                except socket.timeout:
                    continue
                except OSError:
                    break


# ---------------------------------------------------------------------------
#  Main GUI
# ---------------------------------------------------------------------------

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32-P4 摄像头配置工具")
        self.geometry("1680x1320")
        self.resizable(True, True)

        self._announce_queue: queue.Queue = queue.Queue()
        self._cameras: dict[str, dict] = {}   # ip -> last announce dict
        self._selected_ip: str | None = None
        self._preview_running = False
        self._preview_stop_evt = threading.Event()
        self._preview_thread: threading.Thread | None = None
        self._preview_image = None

        # Host-side receive statistics (computed from received JPEG stream)
        self._rx_frame_count = 0
        self._rx_dropped_frames = 0
        self._rx_last_frame_num = None
        self._rx_start_monotonic = 0.0
        self._rx_last_arrival_us = 0
        self._rx_avg_interval_us = 0
        self._rx_interval_samples = 0

        self._build_ui()
        self._listener = BroadcastListener(UDP_PORT, self._announce_queue)
        self._listener.start()
        self._poll_announces()

    # -----------------------------------------------------------------------
    #  UI construction
    # -----------------------------------------------------------------------

    def _build_ui(self):
        pad = {"padx": 8, "pady": 4}

        # ── top: discovered cameras ───────────────────────────────────────
        disc_frame = ttk.LabelFrame(self, text="已发现的摄像头")
        disc_frame.grid(row=0, column=0, columnspan=2, sticky="nsew", **pad)

        self._camera_list = tk.Listbox(disc_frame, height=4, width=50,
                                       exportselection=False)
        self._camera_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True,
                               padx=4, pady=4)
        self._camera_list.bind("<<ListboxSelect>>", self._on_camera_select)

        sb = ttk.Scrollbar(disc_frame, orient=tk.VERTICAL,
                           command=self._camera_list.yview)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._camera_list.configure(yscrollcommand=sb.set)

        # ── middle-left: broadcast info ───────────────────────────────────
        info_frame = ttk.LabelFrame(self, text="广播信息")
        info_frame.grid(row=1, column=0, sticky="nsew", **pad)

        self._info_text = tk.Text(info_frame, height=12, width=46,
                                  state=tk.DISABLED, font=("Courier New", 9))
        self._info_text.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # ── middle-right: config panel ────────────────────────────────────
        cfg_frame = ttk.LabelFrame(self, text="参数配置")
        cfg_frame.grid(row=1, column=1, rowspan=2, sticky="nsew", **pad)

        row = 0

        # IP address
        ttk.Label(cfg_frame, text="IP 地址:").grid(
            row=row, column=0, sticky=tk.W, **pad)
        self._ip_var = tk.StringVar()
        ttk.Entry(cfg_frame, textvariable=self._ip_var, width=18).grid(
            row=row, column=1, sticky=tk.W, **pad)
        row += 1

        # Netmask
        ttk.Label(cfg_frame, text="子网掩码:").grid(
            row=row, column=0, sticky=tk.W, **pad)
        self._mask_var = tk.StringVar(value="255.255.255.0")
        ttk.Entry(cfg_frame, textvariable=self._mask_var, width=18).grid(
            row=row, column=1, sticky=tk.W, **pad)
        row += 1

        # Gateway
        ttk.Label(cfg_frame, text="网关:").grid(
            row=row, column=0, sticky=tk.W, **pad)
        self._gw_var = tk.StringVar()
        ttk.Entry(cfg_frame, textvariable=self._gw_var, width=18).grid(
            row=row, column=1, sticky=tk.W, **pad)
        row += 1

        ttk.Button(cfg_frame, text="设置静态 IP",
                   command=self._cmd_setip).grid(
            row=row, column=0, columnspan=2, pady=(2, 6), sticky=tk.EW,
            padx=8)
        row += 1

        ttk.Button(cfg_frame, text="切换为 DHCP",
                   command=self._cmd_dhcp).grid(
            row=row, column=0, columnspan=2, pady=(0, 8), sticky=tk.EW,
            padx=8)
        row += 1

        ttk.Separator(cfg_frame, orient=tk.HORIZONTAL).grid(
            row=row, column=0, columnspan=2, sticky=tk.EW, padx=4, pady=4)
        row += 1

        # JPEG quality
        ttk.Label(cfg_frame, text="JPEG 质量 (1-100):").grid(
            row=row, column=0, sticky=tk.W, **pad)
        self._quality_var = tk.IntVar(value=60)
        quality_spin = ttk.Spinbox(cfg_frame, from_=1, to=100,
                                   textvariable=self._quality_var, width=8)
        quality_spin.grid(row=row, column=1, sticky=tk.W, **pad)
        row += 1

        ttk.Button(cfg_frame, text="设置 JPEG 质量",
                   command=self._cmd_set_quality).grid(
            row=row, column=0, columnspan=2, pady=(2, 6), sticky=tk.EW,
            padx=8)
        row += 1

        ttk.Separator(cfg_frame, orient=tk.HORIZONTAL).grid(
            row=row, column=0, columnspan=2, sticky=tk.EW, padx=4, pady=4)
        row += 1

        # Skip frames
        ttk.Label(cfg_frame, text="触发后跳过帧数 (0-255):").grid(
            row=row, column=0, sticky=tk.W, **pad)
        self._skip_var = tk.IntVar(value=0)
        skip_spin = ttk.Spinbox(cfg_frame, from_=0, to=255,
                                textvariable=self._skip_var, width=8)
        skip_spin.grid(row=row, column=1, sticky=tk.W, **pad)
        row += 1

        ttk.Button(cfg_frame, text="设置跳过帧数",
                   command=self._cmd_set_skip).grid(
            row=row, column=0, columnspan=2, pady=(2, 6), sticky=tk.EW,
            padx=8)
        row += 1

        ttk.Separator(cfg_frame, orient=tk.HORIZONTAL).grid(
            row=row, column=0, columnspan=2, sticky=tk.EW, padx=4, pady=4)
        row += 1

        btn_frame = ttk.Frame(cfg_frame)
        btn_frame.grid(row=row, column=0, columnspan=2, sticky=tk.EW, padx=8, pady=(0, 6))
        ttk.Button(btn_frame, text="UDP触发开始",
                   command=self._cmd_trigger_start).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 4))
        ttk.Button(btn_frame, text="UDP触发停止",
                   command=self._cmd_trigger_stop).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(4, 0))
        row += 1

        ttk.Separator(cfg_frame, orient=tk.HORIZONTAL).grid(
            row=row, column=0, columnspan=2, sticky=tk.EW, padx=4, pady=4)
        row += 1

        ttk.Button(cfg_frame, text="查询当前配置",
                   command=self._cmd_get_config).grid(
            row=row, column=0, columnspan=2, pady=(2, 6), sticky=tk.EW,
            padx=8)
        row += 1

        ttk.Button(cfg_frame, text="保存后重启设备",
                   command=self._cmd_reset).grid(
            row=row, column=0, columnspan=2, pady=(2, 6), sticky=tk.EW,
            padx=8)
        row += 1

        # ── preview panel ─────────────────────────────────────────────────
        preview_frame = ttk.LabelFrame(self, text="实时预览 (MJPEG)")
        preview_frame.grid(row=2, column=0, sticky="nsew", **pad)

        self._preview_canvas = tk.Canvas(preview_frame, width=1280, height=720, bg="#1e1e1e", highlightthickness=1)
        self._preview_canvas.pack(fill=tk.NONE, expand=False, padx=4, pady=4)
        self._preview_canvas.create_text(640, 360, text="未开始预览", fill="#dddddd", tags="placeholder")

        self._rx_stats_var = tk.StringVar(
            value="接收统计: frames=0, dropped=0, fps=0.00, avg_interval_us=0"
        )
        ttk.Label(preview_frame, textvariable=self._rx_stats_var, font=("Courier New", 9)).pack(anchor=tk.W, padx=6, pady=(0, 4))

        self._preview_status_var = tk.StringVar(value="状态: idle")
        ttk.Label(preview_frame, textvariable=self._preview_status_var).pack(anchor=tk.W, padx=6, pady=(0, 4))

        # ── bottom: log ───────────────────────────────────────────────────
        log_frame = ttk.LabelFrame(self, text="操作日志")
        log_frame.grid(row=3, column=0, columnspan=2, sticky="nsew", **pad)

        self._log = tk.Text(log_frame, height=6, state=tk.DISABLED,
                            font=("Courier New", 9))
        log_sb = ttk.Scrollbar(log_frame, orient=tk.VERTICAL,
                               command=self._log.yview)
        self._log.configure(yscrollcommand=log_sb.set)
        self._log.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4, pady=4)
        log_sb.pack(side=tk.RIGHT, fill=tk.Y)

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)

    @staticmethod
    def _extract_embedded_meta(jpg: bytes):
        """Extract capture timestamp/frame number from APP4 TSMP segment if present."""
        if len(jpg) < 22:
            return None, None
        if jpg[0] != 0xFF or jpg[1] != 0xD8:
            return None, None
        if jpg[2] != 0xFF or jpg[3] != 0xE4:
            return None, None
        if jpg[6:10] != b"TSMP":
            return None, None
        cap_us = int.from_bytes(jpg[10:18], byteorder="big", signed=True)
        frame_num = int.from_bytes(jpg[18:22], byteorder="big", signed=False)
        return cap_us, frame_num

    def _reset_rx_stats(self):
        self._rx_frame_count = 0
        self._rx_dropped_frames = 0
        self._rx_last_frame_num = None
        self._rx_start_monotonic = time.monotonic()
        self._rx_last_arrival_us = 0
        self._rx_avg_interval_us = 0
        self._rx_interval_samples = 0
        self._update_rx_stats_label()

    def _update_rx_stats_on_frame(self, frame_num):
        now_us = int(time.monotonic() * 1_000_000)

        self._rx_frame_count += 1
        if self._rx_last_frame_num is not None and frame_num is not None and frame_num > self._rx_last_frame_num + 1:
            self._rx_dropped_frames += (frame_num - self._rx_last_frame_num - 1)

        if self._rx_last_arrival_us > 0:
            interval = now_us - self._rx_last_arrival_us
            self._rx_interval_samples += 1
            if self._rx_interval_samples == 1:
                self._rx_avg_interval_us = interval
            else:
                self._rx_avg_interval_us = int(
                    (self._rx_avg_interval_us * (self._rx_interval_samples - 1) + interval) /
                    self._rx_interval_samples
                )

        self._rx_last_arrival_us = now_us
        if frame_num is not None:
            self._rx_last_frame_num = frame_num

    def _update_rx_stats_label(self):
        elapsed = max(time.monotonic() - self._rx_start_monotonic, 1e-6)
        fps = self._rx_frame_count / elapsed
        self._rx_stats_var.set(
            f"接收统计: frames={self._rx_frame_count}, dropped={self._rx_dropped_frames}, "
            f"fps={fps:.2f}, avg_interval_us={self._rx_avg_interval_us}"
        )

    # -----------------------------------------------------------------------
    #  Helpers
    # -----------------------------------------------------------------------

    def _log_msg(self, msg: str):
        self._log.configure(state=tk.NORMAL)
        self._log.insert(tk.END, msg + "\n")
        self._log.see(tk.END)
        self._log.configure(state=tk.DISABLED)

    def _update_info_text(self, data: dict):
        lines = [
            f"IP 地址   : {data.get('ip', '-')}",
            f"UDP 端口  : {data.get('port', '-')}",
            f"触发模式  : {data.get('trigger_mode', 'GPIO OR UDP')}",
            f"JPEG 质量 : {data.get('jpeg_quality', '-')}",
            f"跳过帧数  : {data.get('skip_frames', '-')}",
            f"分辨率    : {data.get('width', '-')} × {data.get('height', '-')}",
            "",
            "说明: 丢帧与接收统计由上位机基于接收图像自行计算",
        ]
        text = "\n".join(lines)
        self._info_text.configure(state=tk.NORMAL)
        self._info_text.delete("1.0", tk.END)
        self._info_text.insert(tk.END, text)
        self._info_text.configure(state=tk.DISABLED)

    def _populate_fields_from(self, data: dict):
        """Fill config fields from an announce / get_config reply."""
        ip = data.get("ip", "")
        self._ip_var.set(ip)
        # Derive gateway default from IP
        parts = ip.split(".")
        if len(parts) == 4:
            self._gw_var.set(f"{parts[0]}.{parts[1]}.{parts[2]}.1")
        if "jpeg_quality" in data:
            self._quality_var.set(int(data["jpeg_quality"]))
        if "skip_frames" in data:
            self._skip_var.set(int(data["skip_frames"]))
        # trigger_mode is display-only now; no UI control needed.

    def _set_preview_status(self, text: str):
        self._preview_status_var.set(f"状态: {text}")

    def _show_preview_frame(self, photo):
        self._preview_image = photo
        self._preview_canvas.delete("all")
        self._preview_canvas.create_image(640, 360, image=photo)
        self._update_rx_stats_label()

    def _start_preview(self):
        if self._preview_running:
            return
        if not self._selected_ip:
            return
        if not PIL_AVAILABLE:
            self._set_preview_status("需要安装 pillow: pip install pillow")
            self._preview_canvas.delete("all")
            self._preview_canvas.create_text(640, 360, text="缺少 pillow，无法显示预览", fill="#dddddd")
            return

        self._preview_stop_evt.clear()
        self._reset_rx_stats()
        self._preview_running = True
        self._preview_thread = threading.Thread(target=self._preview_loop, daemon=True)
        self._preview_thread.start()

    def _stop_preview(self):
        self._preview_running = False
        self._preview_stop_evt.set()
        self._preview_canvas.delete("all")
        self._preview_canvas.create_text(640, 360, text="预览已停止", fill="#dddddd")
        self._set_preview_status("stopped")

    def _preview_loop(self):
        ip = self._selected_ip
        if not ip:
            self.after(0, lambda: self._set_preview_status("未选择设备"))
            self._preview_running = False
            return

        url = f"http://{ip}/stream"
        self.after(0, lambda: self._set_preview_status(f"connecting {url}"))

        while not self._preview_stop_evt.is_set():
            try:
                with urllib.request.urlopen(url, timeout=5) as resp:
                    self.after(0, lambda: self._set_preview_status("streaming"))
                    buf = b""
                    while not self._preview_stop_evt.is_set():
                        chunk = resp.read(4096)
                        if not chunk:
                            break
                        buf += chunk
                        soi = buf.find(b"\xff\xd8")
                        eoi = buf.find(b"\xff\xd9")
                        if soi != -1 and eoi != -1 and eoi > soi:
                            jpg = buf[soi:eoi + 2]
                            buf = buf[eoi + 2:]

                            _capture_us, frame_num = self._extract_embedded_meta(jpg)
                            self._update_rx_stats_on_frame(frame_num)

                            image = Image.open(io.BytesIO(jpg))
                            image.thumbnail((1280, 720), RESAMPLE_LANCZOS)
                            photo = ImageTk.PhotoImage(image)
                            self.after(0, lambda p=photo: self._show_preview_frame(p))
            except Exception as e:
                self.after(0, lambda err=str(e): self._set_preview_status(f"reconnecting ({err})"))
                if self._preview_stop_evt.wait(1.0):
                    break

        self._preview_running = False

    # -----------------------------------------------------------------------
    #  Broadcast poll (Tkinter main-loop safe)
    # -----------------------------------------------------------------------

    def _poll_announces(self):
        try:
            while True:
                pkt = self._announce_queue.get_nowait()
                ip = pkt.get("ip") or pkt.get("_src_ip", "unknown")
                self._cameras[ip] = pkt

                # Rebuild listbox
                self._camera_list.delete(0, tk.END)
                for cam_ip, cam in self._cameras.items():
                    label = (f"{cam_ip}  "
                             f"质量={cam.get('jpeg_quality','-')}  "
                             f"跳帧={cam.get('skip_frames','-')}  "
                             f"{cam.get('width','-')}×{cam.get('height','-')}")
                    self._camera_list.insert(tk.END, label)

                # If no camera selected yet, auto-select the first one
                if self._selected_ip is None:
                    self._selected_ip = ip
                    self._camera_list.selection_set(0)
                    self._update_info_text(pkt)
                    self._populate_fields_from(pkt)
                elif ip == self._selected_ip:
                    self._update_info_text(pkt)
        except queue.Empty:
            pass
        self.after(500, self._poll_announces)

    def _on_camera_select(self, _event=None):
        sel = self._camera_list.curselection()
        if not sel:
            return
        ips = list(self._cameras.keys())
        if sel[0] < len(ips):
            prev_ip = self._selected_ip
            self._selected_ip = ips[sel[0]]

            # Requirement: when switching camera, stop trigger on previous camera.
            if prev_ip and prev_ip != self._selected_ip:
                self._log_msg(f"[发送] trigger_stop (previous camera {prev_ip})")
                self._send_async_to(prev_ip, {"cmd": "trigger_stop"}, suppress_error_popup=True)

            data = self._cameras[self._selected_ip]
            self._update_info_text(data)
            self._populate_fields_from(data)
            if self._preview_running:
                self._stop_preview()
                self._start_preview()

    # -----------------------------------------------------------------------
    #  Command senders (run in background threads to avoid blocking UI)
    # -----------------------------------------------------------------------

    def _require_selected(self) -> str | None:
        if not self._selected_ip:
            messagebox.showwarning("提示", "请先选择一台摄像头。")
        return self._selected_ip

    def _send_async(self, cmd: dict, on_done=None, suppress_error_popup=False):
        """Send UDP command in a daemon thread; call on_done(reply) in main thread."""
        ip = self._selected_ip
        port = UDP_PORT

        def _run():
            reply = udp_send_command(ip, port, cmd)
            self.after(0, lambda: self._handle_reply(cmd, reply, on_done, suppress_error_popup))

        threading.Thread(target=_run, daemon=True).start()

    def _send_async_to(self, ip: str, cmd: dict, on_done=None, suppress_error_popup=False):
        """Send UDP command to the specified camera IP."""
        port = UDP_PORT

        def _run():
            reply = udp_send_command(ip, port, cmd)
            self.after(0, lambda: self._handle_reply(cmd, reply, on_done, suppress_error_popup))

        threading.Thread(target=_run, daemon=True).start()

    def _handle_reply(self, cmd: dict, reply: dict | None, on_done=None, suppress_error_popup=False):
        if reply is None:
            self._log_msg(f"[错误] 无响应  cmd={cmd.get('cmd')}")
            if not suppress_error_popup:
                messagebox.showerror("错误", "摄像头无响应，请检查网络。")
            return
        if reply.get("ok"):
            self._log_msg(f"[OK] {json.dumps(reply, ensure_ascii=False)}")
        else:
            err = reply.get("error", "unknown error")
            self._log_msg(f"[FAIL] {err}")
            if not suppress_error_popup:
                messagebox.showerror("命令失败", err)
        if on_done:
            on_done(reply)

    def _cmd_setip(self):
        if not self._require_selected():
            return
        ip   = self._ip_var.get().strip()
        mask = self._mask_var.get().strip()
        gw   = self._gw_var.get().strip()
        if not ip:
            messagebox.showwarning("提示", "请输入 IP 地址。")
            return
        cmd = {"cmd": "setip", "ip": ip}
        if mask:
            cmd["netmask"] = mask
        if gw:
            cmd["gw"] = gw
        self._log_msg(f"[发送] setip  ip={ip} mask={mask} gw={gw}")
        self._send_async(cmd)

    def _cmd_dhcp(self):
        if not self._require_selected():
            return
        if not messagebox.askyesno("确认", "切换为 DHCP 后 IP 地址将改变，确认继续？"):
            return
        self._log_msg("[发送] dhcp")
        self._send_async({"cmd": "dhcp"})

    def _cmd_set_quality(self):
        if not self._require_selected():
            return
        q = self._quality_var.get()
        self._log_msg(f"[发送] set_quality  value={q}")
        self._send_async({"cmd": "set_quality", "value": q})

    def _cmd_set_skip(self):
        if not self._require_selected():
            return
        n = self._skip_var.get()
        self._log_msg(f"[发送] set_skip_frames  value={n}")
        self._send_async({"cmd": "set_skip_frames", "value": n})

    def _cmd_trigger_start(self):
        if not self._require_selected():
            return
        self._log_msg("[发送] trigger_start")

        def _after_start(start_reply):
            if start_reply and start_reply.get("ok"):
                self._start_preview()

        self._send_async({"cmd": "trigger_start"}, on_done=_after_start)

    def _cmd_trigger_stop(self):
        if not self._require_selected():
            return
        self._log_msg("[发送] trigger_stop")
        self._stop_preview()
        self._send_async({"cmd": "trigger_stop"})

    def _cmd_reset(self):
        if not self._require_selected():
            return
        if not messagebox.askyesno("确认重启", "发送 reset 指令后设备将立即重启，是否继续？"):
            return
        self._log_msg("[发送] reset")

        def _update(reply):
            if reply and reply.get("ok"):
                self._stop_preview()
                messagebox.showinfo("提示", "设备正在重启，请等待设备重新广播上线。")

        self._send_async({"cmd": "reset"}, on_done=_update)

    def _cmd_get_config(self):
        if not self._require_selected():
            return
        self._log_msg("[发送] get_config")

        def _update(reply):
            if reply and reply.get("ok"):
                self._update_info_text(reply)
                self._populate_fields_from(reply)

        self._send_async({"cmd": "get_config"}, on_done=_update)

    def destroy(self):
        self._stop_preview()
        self._listener.stop()
        super().destroy()


# ---------------------------------------------------------------------------
#  Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    app = App()
    app.mainloop()
