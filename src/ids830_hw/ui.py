#!/usr/bin/env python3
"""IDS830 Motor Test - Tkinter GUI."""

import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time

# Import the driver from the same package
from test_motor import IDS830CAN, setup_can

LEFT_ID = 0x004
RIGHT_ID = 0x002
POLL_HZ = 5  # encoder refresh rate


class MotorUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("IDS830 Motor Test")
        self.root.resizable(False, False)

        self.can: IDS830CAN | None = None
        self.stop_event = threading.Event()
        self.speed = tk.IntVar(value=300)
        self.connected = tk.BooleanVar(value=False)

        # Encoder / speed display vars
        self.enc_l = tk.StringVar(value="---")
        self.enc_r = tk.StringVar(value="---")
        self.spd_l = tk.StringVar(value="---")
        self.spd_r = tk.StringVar(value="---")

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ UI --

    def _build_ui(self):
        PAD = dict(padx=8, pady=4)

        # ── Top bar ──────────────────────────────────────────────────────────
        top = ttk.Frame(self.root)
        top.pack(fill="x", **PAD)

        self.status_label = ttk.Label(top, text="Disconnected", foreground="red", width=20)
        self.status_label.pack(side="left")

        ttk.Button(top, text="Connect", command=self._connect).pack(side="left", padx=4)
        ttk.Button(top, text="Disconnect", command=self._disconnect).pack(side="left")

        ttk.Separator(self.root, orient="horizontal").pack(fill="x", pady=2)

        # ── Motor feedback ───────────────────────────────────────────────────
        fb = ttk.Frame(self.root)
        fb.pack(fill="x", **PAD)

        for col, title, enc_var, spd_var in [
            (0, "MOTOR TRÁI  (0x004)", self.enc_l, self.spd_l),
            (1, "MOTOR PHẢI  (0x002)", self.enc_r, self.spd_r),
        ]:
            grp = ttk.LabelFrame(fb, text=title, padding=6)
            grp.grid(row=0, column=col, padx=6, sticky="nsew")
            ttk.Label(grp, text="Encoder:").grid(row=0, column=0, sticky="w")
            ttk.Label(grp, textvariable=enc_var, width=14, anchor="e",
                      font=("Courier", 11, "bold")).grid(row=0, column=1, sticky="e")
            ttk.Label(grp, text="Speed:").grid(row=1, column=0, sticky="w")
            ttk.Label(grp, textvariable=spd_var, width=14, anchor="e",
                      font=("Courier", 11, "bold")).grid(row=1, column=1, sticky="e")

        fb.columnconfigure(0, weight=1)
        fb.columnconfigure(1, weight=1)

        ttk.Separator(self.root, orient="horizontal").pack(fill="x", pady=2)

        # ── Speed control ────────────────────────────────────────────────────
        spd_frame = ttk.LabelFrame(self.root, text="Speed (RPM)", padding=6)
        spd_frame.pack(fill="x", **PAD)

        self.slider = ttk.Scale(spd_frame, from_=1, to=3000, orient="horizontal",
                                variable=self.speed, command=self._slider_moved)
        self.slider.pack(side="left", fill="x", expand=True, padx=(0, 8))

        self.speed_entry = ttk.Entry(spd_frame, textvariable=self.speed, width=6,
                                     justify="center")
        self.speed_entry.pack(side="left")
        self.speed_entry.bind("<Return>", lambda _: self._apply_speed())

        ttk.Button(spd_frame, text="Set", command=self._apply_speed).pack(side="left", padx=4)

        # ── Drive buttons ────────────────────────────────────────────────────
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill="x", **PAD)

        self.btn_fwd = ttk.Button(btn_frame, text="▲  FORWARD  (W)",
                                  command=self._forward, width=18)
        self.btn_fwd.pack(side="left", padx=4)

        self.btn_stop = ttk.Button(btn_frame, text="■  STOP  (X)",
                                   command=self._stop_motors, width=14)
        self.btn_stop.pack(side="left", padx=4)

        self.btn_rev = ttk.Button(btn_frame, text="▼  REVERSE  (S)",
                                  command=self._reverse, width=18)
        self.btn_rev.pack(side="left", padx=4)

        # ── Log ──────────────────────────────────────────────────────────────
        ttk.Separator(self.root, orient="horizontal").pack(fill="x", pady=2)
        self.log = scrolledtext.ScrolledText(self.root, height=8, state="disabled",
                                             font=("Courier", 9))
        self.log.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        # Keyboard shortcuts
        self.root.bind("w", lambda _: self._forward())
        self.root.bind("s", lambda _: self._reverse())
        self.root.bind("x", lambda _: self._stop_motors())

        self._set_controls_state("disabled")

    # ---------------------------------------------------------------- helpers

    def _log(self, msg: str):
        self.log.configure(state="normal")
        self.log.insert("end", msg + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _set_controls_state(self, state: str):
        for w in (self.btn_fwd, self.btn_stop, self.btn_rev, self.slider, self.speed_entry):
            w.configure(state=state)

    def _slider_moved(self, _):
        # Round to int
        self.speed.set(int(self.speed.get()))

    def _apply_speed(self):
        try:
            v = int(self.speed_entry.get())
            v = max(1, min(3000, v))
            self.speed.set(v)
            self._log(f"  Speed set to {v} RPM")
        except ValueError:
            pass

    # ------------------------------------------------------------ connection

    def _connect(self):
        if self.can:
            return
        self.status_label.configure(text="Connecting…", foreground="orange")
        threading.Thread(target=self._do_connect, daemon=True).start()

    def _do_connect(self):
        try:
            self.root.after(0, self._log, "[1/3] Setting up CAN interface…")
            setup_can()
            self.root.after(0, self._log, "[2/3] Connecting…")
            can = IDS830CAN()
            self.root.after(0, self._log, "[3/3] Unlocking PC mode…")
            results = []
            for cid, name in [(LEFT_ID, "TRÁI"), (RIGHT_ID, "PHẢI")]:
                ok = can.unlock_pc_mode(cid)
                can.set_speed_mode(cid)
                can.enable(cid)
                results.append(f"  Motor {name} (0x{cid:03X}): {'✅ OK' if ok else '❌ FAIL'}")
            self.can = can
            self.stop_event.clear()
            threading.Thread(target=self._poll_loop, daemon=True).start()
            for r in results:
                self.root.after(0, self._log, r)
            self.root.after(0, self._on_connected)
        except Exception as e:
            self.root.after(0, self._log, f"  ❌ Error: {e}")
            self.root.after(0, self.status_label.configure,
                            {"text": "Error", "foreground": "red"})

    def _on_connected(self):
        self.connected.set(True)
        self.status_label.configure(text="Connected ✓", foreground="green")
        self._set_controls_state("normal")
        self._log("Ready.")

    def _disconnect(self):
        if not self.can:
            return
        self.stop_event.set()
        try:
            for cid in [LEFT_ID, RIGHT_ID]:
                self.can.set_speed(cid, 0)
                self.can.disable(cid)
            self.can.close()
        except Exception:
            pass
        self.can = None
        self.connected.set(False)
        self.status_label.configure(text="Disconnected", foreground="red")
        self._set_controls_state("disabled")
        for v in (self.enc_l, self.enc_r, self.spd_l, self.spd_r):
            v.set("---")
        self._log("Disconnected.")

    # ---------------------------------------------------------- encoder poll

    def _poll_loop(self):
        while not self.stop_event.is_set() and self.can:
            try:
                enc_l = self.can.read_encoder(LEFT_ID)
                enc_r = self.can.read_encoder(RIGHT_ID)
                spd_l = self.can.read_speed(LEFT_ID)
                spd_r = self.can.read_speed(RIGHT_ID)

                self.root.after(0, self.enc_l.set,
                                f"{enc_l:>10d}" if enc_l is not None else "---")
                self.root.after(0, self.enc_r.set,
                                f"{enc_r:>10d}" if enc_r is not None else "---")
                self.root.after(0, self.spd_l.set,
                                f"{spd_l:>7.1f} RPM" if spd_l is not None else "---")
                self.root.after(0, self.spd_r.set,
                                f"{spd_r:>7.1f} RPM" if spd_r is not None else "---")
            except Exception:
                pass
            self.stop_event.wait(1.0 / POLL_HZ)

    # -------------------------------------------------------------- commands

    def _forward(self):
        if not self.can:
            return
        spd = self.speed.get()
        self.can.set_speed(LEFT_ID, spd)
        self.can.set_speed(RIGHT_ID, -spd)
        self._log(f">>> FORWARD {spd} RPM")

    def _reverse(self):
        if not self.can:
            return
        spd = self.speed.get()
        self.can.set_speed(LEFT_ID, -spd)
        self.can.set_speed(RIGHT_ID, spd)
        self._log(f">>> REVERSE {spd} RPM")

    def _stop_motors(self):
        if not self.can:
            return
        self.can.set_speed(LEFT_ID, 0)
        self.can.set_speed(RIGHT_ID, 0)
        self._log(">>> STOP")

    # --------------------------------------------------------------- cleanup

    def _on_close(self):
        self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    MotorUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()