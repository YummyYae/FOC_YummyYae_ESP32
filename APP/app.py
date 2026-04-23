import io
import math
import time
import socket
import struct
import threading
import tkinter as tk
from tkinter import filedialog, ttk, messagebox

from PIL import Image, ImageTk, ImageGrab


WIDTH = 192
HEIGHT = 32
PAYLOAD_LEN = WIDTH * HEIGHT * 3
BLANK_GRAY_LEVEL = 127
BLANK_RGB = (BLANK_GRAY_LEVEL, BLANK_GRAY_LEVEL, BLANK_GRAY_LEVEL)

MAGIC_IMAGE = b"TCP1"
MAGIC_SET_RPM = b"RPM1"
MAGIC_TELEM = b"TEL1"
MAGIC_GAMMA = b"GAM1"
HEADER_LEN = 8

DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 5005


class App:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("FOC TCP Host")
        self.root.geometry("1080x560")

        self.sock = None
        self.sock_lock = threading.Lock()
        self.rx_stop = threading.Event()
        self.frame_id = 0
        self.current_image_rgb = None
        self.preview_photo = None
        self.is_streaming = False

        self._build_ui()
        self._update_status_disconnected()

    def _build_ui(self):
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="Host IP").pack(side="left")
        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        ttk.Entry(top, textvariable=self.host_var, width=16).pack(side="left", padx=4)

        ttk.Label(top, text="Port").pack(side="left")
        self.port_var = tk.StringVar(value=str(DEFAULT_PORT))
        ttk.Entry(top, textvariable=self.port_var, width=8).pack(side="left", padx=4)

        ttk.Button(top, text="Connect", command=self.connect_tcp).pack(side="left", padx=6)
        ttk.Button(top, text="Disconnect", command=self.disconnect_tcp).pack(side="left")

        self.conn_label = ttk.Label(top, text="Disconnected")
        self.conn_label.pack(side="left", padx=12)

        ttk.Separator(self.root, orient="horizontal").pack(fill="x", pady=6)

        mid = ttk.Frame(self.root, padding=8)
        mid.pack(fill="both", expand=True)

        left = ttk.Frame(mid)
        left.pack(side="left", fill="y")

        ttk.Button(left, text="Choose Image", command=self.choose_image).pack(fill="x", pady=4)
        ttk.Button(left, text="Send Image", command=self.send_image).pack(fill="x", pady=4)

        self.stream_btn_var = tk.StringVar(value="Start Stream")
        ttk.Button(left, textvariable=self.stream_btn_var, command=self.toggle_stream).pack(fill="x", pady=4)

        rpm_line = ttk.Frame(left)
        rpm_line.pack(fill="x", pady=8)
        ttk.Label(rpm_line, text="Target RPM").pack(side="left")
        self.target_rpm_var = tk.StringVar(value="300")
        ttk.Entry(rpm_line, textvariable=self.target_rpm_var, width=10).pack(side="left", padx=6)
        ttk.Button(rpm_line, text="Send", command=self.send_target_rpm).pack(side="left")

        gamma_line = ttk.Frame(left)
        gamma_line.pack(fill="x", pady=8)
        ttk.Label(gamma_line, text="Gamma").pack(side="left")
        self.gamma_var = tk.StringVar(value="1.0")
        ttk.Entry(gamma_line, textvariable=self.gamma_var, width=10).pack(side="left", padx=6)
        ttk.Button(gamma_line, text="Send", command=self.send_gamma).pack(side="left")

        self.info_var = tk.StringVar(value="No image selected")
        ttk.Label(left, textvariable=self.info_var, wraplength=260).pack(fill="x", pady=8)

        self.preview_label = ttk.Label(left)
        self.preview_label.pack(fill="both", expand=True, pady=8)

        right = ttk.Frame(mid)
        right.pack(side="left", fill="both", expand=True, padx=(12, 0))

        telem_box = ttk.LabelFrame(right, text="Telemetry (5Hz)", padding=8)
        telem_box.pack(fill="x")
        self.rpm_now_var = tk.StringVar(value="RPM: --")
        self.rpm_target_var = tk.StringVar(value="Target RPM: --")
        self.uq_var = tk.StringVar(value="Uq: --")
        self.shaft_var = tk.StringVar(value="Shaft (rad): --")
        self.elec_var = tk.StringVar(value="Elec (rad): --")
        ttk.Label(telem_box, textvariable=self.rpm_now_var).pack(anchor="w")
        ttk.Label(telem_box, textvariable=self.rpm_target_var).pack(anchor="w")
        ttk.Label(telem_box, textvariable=self.uq_var).pack(anchor="w")
        ttk.Label(telem_box, textvariable=self.shaft_var).pack(anchor="w")
        ttk.Label(telem_box, textvariable=self.elec_var).pack(anchor="w")

        ttk.Label(right, text="Log").pack(anchor="w", pady=(10, 0))
        self.log = tk.Text(right, height=18)
        self.log.pack(fill="both", expand=True)

    @staticmethod
    def _project_cylinder_16_9(src: Image.Image) -> Image.Image:
        src_w, src_h = src.size
        if src_w <= 0 or src_h <= 0:
            return Image.new("RGB", (WIDTH, HEIGHT), BLANK_RGB)

        target_ratio = 16.0 / 9.0
        if src_w / src_h > target_ratio:
            new_w = int(src_h * target_ratio)
            left = (src_w - new_w) // 2
            src = src.crop((left, 0, left + new_w, src_h))
        else:
            new_h = int(src_w / target_ratio)
            top = (src_h - new_h) // 2
            src = src.crop((0, top, src_w, top + new_h))

        sample_w = 256
        src = src.resize((sample_w, HEIGHT), Image.Resampling.LANCZOS)

        out_w = 96
        canvas = Image.new("RGB", (WIDTH, HEIGHT), BLANK_RGB)

        col_map = []
        for c in range(out_w):
            phi = math.pi * (c + 0.5) / out_w
            x_proj = -math.cos(phi)
            src_x = int((x_proj + 1.0) / 2.0 * (sample_w - 1))
            col_map.append(min(max(src_x, 0), sample_w - 1))

        src_pixels = src.load()
        canvas_pixels = canvas.load()

        for y in range(HEIGHT):
            for c in range(out_w):
                canvas_pixels[c, y] = src_pixels[col_map[c], y]

        return canvas

    def choose_image(self):
        path = filedialog.askopenfilename(
            title="Choose Image",
            filetypes=[("Image Files", "*.png;*.jpg;*.jpeg;*.bmp;*.gif"), ("All Files", "*.*")]
        )
        if not path:
            return

        src = Image.open(path).convert("RGB")
        img = self._project_cylinder_16_9(src)
        self.current_image_rgb = img

        preview = img.resize((WIDTH * 3, HEIGHT * 3), Image.Resampling.NEAREST)
        self.preview_photo = ImageTk.PhotoImage(preview)
        self.preview_label.configure(image=self.preview_photo)

        self.info_var.set(f"Loaded: {path}\nOutput: {WIDTH}x{HEIGHT} RGB888 (16:9 cylinder projection)")
        self._append_log(f"Image loaded and projected: {path}")

    def connect_tcp(self):
        if self.sock is not None:
            self._append_log("TCP already connected")
            return

        host = self.host_var.get().strip()
        port_text = self.port_var.get().strip()
        if not host or not port_text:
            messagebox.showwarning("Warning", "Please enter host IP and port")
            return

        try:
            port = int(port_text)
            sock = socket.create_connection((host, port), timeout=3.0)
            sock.settimeout(None)
        except Exception as e:
            messagebox.showerror("Connect failed", str(e))
            return

        self.sock = sock
        self.rx_stop.clear()
        threading.Thread(target=self._rx_thread, daemon=True).start()
        self._append_log(f"TCP connected: {host}:{port}")
        self._update_status_connected(host, port)

    def disconnect_tcp(self):
        self.is_streaming = False
        try:
            self.stream_btn_var.set("Start Stream")
        except Exception:
            pass
        self.rx_stop.set()
        with self.sock_lock:
            if self.sock is not None:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None
        self._append_log("TCP disconnected")
        self._update_status_disconnected()

    def _send_packet(self, magic: bytes, frame_id: int, payload: bytes):
        if self.sock is None:
            raise RuntimeError("TCP not connected")
        if len(magic) != 4:
            raise RuntimeError("Magic length must be 4")
        if len(payload) > 0xFFFF:
            raise RuntimeError("Payload too long")

        frame = io.BytesIO()
        frame.write(magic)
        frame.write(struct.pack("<H", frame_id & 0xFFFF))
        frame.write(struct.pack("<H", len(payload)))
        frame.write(payload)
        data = frame.getvalue()

        with self.sock_lock:
            if self.sock is None:
                raise RuntimeError("TCP not connected")
            self.sock.sendall(data)

    def send_image(self):
        if self.current_image_rgb is None:
            messagebox.showwarning("Warning", "Please choose an image first")
            return
        payload = self.current_image_rgb.tobytes()
        if len(payload) != PAYLOAD_LEN:
            messagebox.showerror("Error", f"Invalid payload length: {len(payload)}")
            return

        try:
            self._send_packet(MAGIC_IMAGE, self.frame_id, payload)
            self._append_log(f"Image sent: frame_id={self.frame_id}, bytes={len(payload)}")
            self.frame_id = (self.frame_id + 1) & 0xFFFF
        except Exception as e:
            messagebox.showerror("Send failed", str(e))
            self.disconnect_tcp()

    def toggle_stream(self):
        if not self.is_streaming:
            if self.sock is None:
                messagebox.showwarning("Warning", "Please connect TCP first")
                return
            self.is_streaming = True
            self.stream_btn_var.set("Stop Stream")
            self._append_log("Start screen streaming...")
            threading.Thread(target=self._stream_thread, daemon=True).start()
        else:
            self.is_streaming = False
            self.stream_btn_var.set("Start Stream")
            self._append_log("Stop streaming")

    def _stream_thread(self):
        while self.is_streaming:
            start_t = time.time()
            try:
                screen = ImageGrab.grab()
                img = self._project_cylinder_16_9(screen)
                payload = img.tobytes()

                self._send_packet(MAGIC_IMAGE, self.frame_id, payload)
                self.frame_id = (self.frame_id + 1) & 0xFFFF

                preview = img.resize((WIDTH * 3, HEIGHT * 3), Image.Resampling.NEAREST)
                self.root.after(0, self._update_preview, preview)
            except Exception as e:
                self.root.after(0, self._append_log, f"Screen stream error: {e}")
                self.is_streaming = False
                self.root.after(0, self.stream_btn_var.set, "Start Stream")
                break

            elapsed = time.time() - start_t
            sleep_t = 0.1 - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _update_preview(self, preview_img):
        self.preview_photo = ImageTk.PhotoImage(preview_img)
        self.preview_label.configure(image=self.preview_photo)

    def send_target_rpm(self):
        text = self.target_rpm_var.get().strip()
        try:
            target = float(text)
        except ValueError:
            messagebox.showwarning("Warning", "Target RPM must be numeric")
            return

        payload = struct.pack("<f", target)
        try:
            self._send_packet(MAGIC_SET_RPM, 0, payload)
            self._append_log(f"Target RPM sent: {target:.2f} rpm")
        except Exception as e:
            messagebox.showerror("Send failed", str(e))
            self.disconnect_tcp()

    def send_gamma(self):
        text = self.gamma_var.get().strip()
        try:
            gamma = float(text)
        except ValueError:
            messagebox.showwarning("Warning", "Gamma must be numeric")
            return

        payload = struct.pack("<f", gamma)
        try:
            self._send_packet(MAGIC_GAMMA, 0, payload)
            self._append_log(f"Gamma sent: {gamma:.2f}")
        except Exception as e:
            messagebox.showerror("Send failed", str(e))
            self.disconnect_tcp()

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            with self.sock_lock:
                s = self.sock
            if s is None:
                raise ConnectionError("socket closed")
            chunk = s.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("socket closed by peer")
            buf.extend(chunk)
        return bytes(buf)

    def _rx_thread(self):
        try:
            while not self.rx_stop.is_set():
                hdr = self._recv_exact(HEADER_LEN)
                magic = hdr[0:4]
                _fid = struct.unpack("<H", hdr[4:6])[0]
                plen = struct.unpack("<H", hdr[6:8])[0]
                payload = self._recv_exact(plen) if plen > 0 else b""

                if magic == MAGIC_TELEM and plen == 20:
                    rpm, target, uq, shaft, elec = struct.unpack("<5f", payload)
                    self.root.after(0, self._update_telem, rpm, target, uq, shaft, elec)
                else:
                    self.root.after(0, self._append_log, f"Unknown RX packet: magic={magic!r} len={plen}")
        except Exception as e:
            if not self.rx_stop.is_set():
                self.root.after(0, self._append_log, f"Connection closed: {e}")
                self.root.after(0, self.disconnect_tcp)

    def _update_telem(self, rpm: float, target: float, uq: float, shaft: float, elec: float):
        self.rpm_now_var.set(f"RPM: {rpm:.2f} rpm")
        self.rpm_target_var.set(f"Target RPM: {target:.2f} rpm")
        self.uq_var.set(f"Uq: {uq:.3f}")
        self.shaft_var.set(f"Shaft (rad): {shaft:.3f}")
        self.elec_var.set(f"Elec (rad): {elec:.3f}")

    def _update_status_connected(self, host: str, port: int):
        self.conn_label.configure(text=f"Connected: {host}:{port}")

    def _update_status_disconnected(self):
        self.conn_label.configure(text="Disconnected")

    def _append_log(self, text: str):
        self.log.insert("end", text + "\n")
        self.log.see("end")


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.disconnect_tcp(), root.destroy()))
    root.mainloop()
