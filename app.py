#!/usr/bin/env python3
"""
Sistem Tertanam: Smart Neonatal Incubator Sentinel (v2)
Backend: Flask REST API + Multithreaded Hardware Engine
Platform: Raspberry Pi 5

Aturan operasional:
  1. Status aman            -> LED Hijau menyala tetap.
  2. Peringatan/masalah     -> LED Merah berkedip cepat, LED Hijau mati.
  3. Suhu / kelembapan di luar batas -> HANYA LED + peringatan web (buzzer boks hening).
  4. Bayi tidak bergerak >= 2 menit, jadwal susu, atau CODE BLUE -> buzzer boks bunyi.
  5. Buzzer boks HANYA bisa dimatikan dengan menahan tombol fisik di inkubator.

Opsional: set token untuk API kontrol  ->  export INCUBATOR_TOKEN="rahasia"
"""

from __future__ import annotations

import hmac
import logging
import os
import random
import signal
import sys
import threading
import time
from collections import deque
from datetime import datetime
from enum import Enum

from flask import Flask, jsonify, render_template, request

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(threadName)s: %(message)s")
logger = logging.getLogger("sentinel")
logging.getLogger("werkzeug").setLevel(logging.WARNING)


# =========================== KONFIGURASI ===========================
class Config:
    PIN_LED_MERAH = 24
    PIN_LED_HIJAU = 27
    PIN_BUZZER = 25
    PIN_TOMBOL = 22
    PIN_PIR = 17
    PIN_SOUND = 23  
    PIN_DHT = "D4"

    # Polaritas modul. Ubah bila modul Anda berperilaku kebalikan
    # (jalankan test_sensors.py untuk memastikan).
    BUZZER_ACTIVE_HIGH = True    # False = modul buzzer "low-level trigger"
    PIR_ACTIVE_HIGH = True       # HC-SR501 normalnya HIGH saat ada gerak
    SOUND_ACTIVE_HIGH = True     # banyak modul LM393: DO jadi LOW saat ada suara -> False

    LOOP_INTERVAL = 0.1
    DHT_INTERVAL = 2.0
    DHT_STALE_AFTER = 10.0
    INACTIVITY_WARNING = 90.0    # peringatan dini di web (detik)
    INACTIVITY_TIMEOUT = 120.0   # buzzer pemeriksaan/stimulasi (2 menit)
    MOTION_DISPLAY_HOLD = 3.0    # status "AKTIF" di web ditahan agar pulsa PIR singkat tetap terlihat
    CRY_HOLD = 3.0               # status tangisan ditahan agar tidak berkedip
    MASK_SOUND_DURING_BUZZER = True   # abaikan sensor suara saat buzzer boks bunyi (cegah self-trigger)
    BUTTON_HOLD = 0.5            # tombol fisik harus ditahan (detik) agar tidak mati karena tersenggol

    HYSTERESIS = 1.0             # histeresis suhu (°C)
    TEMP_DEFAULT_HIGH = 34.0     # ambang bersifat tetap (tidak bisa diubah dari web)
    TEMP_DEFAULT_LOW = 27.0

    HUM_HYSTERESIS = 5.0         # DHT11 hanya akurat ±5% RH
    HUM_DEFAULT_HIGH = 90.0
    HUM_DEFAULT_LOW = 40.0

    AUTH_TOKEN = os.environ.get("INCUBATOR_TOKEN", "")

    LOG_SIZE = 50
    HOST = "0.0.0.0"
    PORT = 5000


# ============================ STATUS FSM ============================
class State(str, Enum):
    IDLE_NORMAL = "IDLE_NORMAL"
    CRYING_DISTRESS = "CRYING_DISTRESS"
    SENSOR_FAULT = "SENSOR_FAULT"
    HUMIDITY_HAZARD = "HUMIDITY_HAZARD"
    THERMAL_HAZARD = "THERMAL_HAZARD"
    FEEDING_WAKE = "FEEDING_WAKE"
    INACTIVITY_ALERT = "INACTIVITY_ALERT"
    EMERGENCY = "CODE_BLUE"


# Hanya status ini yang membunyikan buzzer di dalam boks inkubator.
BUZZER_STATES = {State.INACTIVITY_ALERT, State.FEEDING_WAKE, State.EMERGENCY}


# ====================== LAPISAN PERANGKAT KERAS ======================
SIMULATION = os.environ.get("INCUBATOR_SIM", "0") == "1"

if not SIMULATION:
    try:
        import adafruit_dht
        import board
        from gpiozero import LED, Button, Buzzer, DigitalInputDevice
    except (ImportError, NotImplementedError, RuntimeError) as exc:
        logger.warning("Library GPIO tidak tersedia (%s) -> MODE SIMULASI DIAKTIFKAN.", exc)
        SIMULATION = True


class _SimOutput:
    def __init__(self, name):
        self.name, self.mode = name, "off"
    def on(self): self.mode = "on"
    def off(self): self.mode = "off"
    def blink(self, on_time=1.0, off_time=1.0, n=None, background=True):
        self.mode = f"blink({on_time}/{off_time})"
    beep = blink
    def close(self): self.off()


class _SimInput:
    def __init__(self, p_active=0.0):
        self._p = p_active
        self.when_pressed = None
        self.when_held = None
        self.when_activated = None
    @property
    def is_active(self): return random.random() < self._p
    def close(self): pass


class _SimDHT:
    def __init__(self): self._t, self._h = 33.0, 60.0
    @property
    def temperature(self):
        self._t = min(max(self._t + random.uniform(-0.2, 0.2), 29.0), 36.0)
        return round(self._t, 1)
    @property
    def humidity(self):
        self._h = min(max(self._h + random.uniform(-1, 1), 40.0), 80.0)
        return round(self._h)
    def exit(self): pass


class Hardware:
    def __init__(self):
        if SIMULATION:
            self.led_merah = _SimOutput("led_merah")
            self.led_hijau = _SimOutput("led_hijau")
            self.buzzer = _SimOutput("buzzer")
            self.tombol = _SimInput()
            self.pir = _SimInput(0.05)
            self.sound = _SimInput(0.01)
            self.dht = _SimDHT()
        else:
            self._check_pin_conflicts()
            self.led_merah = LED(Config.PIN_LED_MERAH)
            self.led_hijau = LED(Config.PIN_LED_HIJAU)
            self.buzzer = Buzzer(Config.PIN_BUZZER, active_high=Config.BUZZER_ACTIVE_HIGH,
                                 initial_value=False)
            self.tombol = Button(Config.PIN_TOMBOL, bounce_time=0.05,
                                 hold_time=max(Config.BUTTON_HOLD, 0.05))
            # PIR & sensor suara dibaca sebagai input digital biasa (tanpa smoothing),
            # supaya pulsa singkat tidak hilang dan polaritas bisa diatur.
            self.pir = DigitalInputDevice(Config.PIN_PIR, pull_up=not Config.PIR_ACTIVE_HIGH)
            self.sound = DigitalInputDevice(Config.PIN_SOUND, pull_up=not Config.SOUND_ACTIVE_HIGH)
            self.dht = adafruit_dht.DHT11(getattr(board, Config.PIN_DHT), use_pulseio=False)

    @staticmethod
    def _check_pin_conflicts():
        """Pastikan tidak ada dua perangkat yang memakai pin GPIO yang sama."""
        used = {}
        for name in ("PIN_LED_MERAH", "PIN_LED_HIJAU", "PIN_BUZZER",
                     "PIN_TOMBOL", "PIN_PIR", "PIN_SOUND"):
            pin = getattr(Config, name)
            used.setdefault(pin, []).append(name)
        conflicts = {p: n for p, n in used.items() if len(n) > 1}
        if conflicts:
            detail = "; ".join(f"GPIO{p} dipakai oleh {', '.join(n)}"
                               for p, n in conflicts.items())
            raise SystemExit(f"[KONFIGURASI PIN BENTROK] {detail}. "
                             f"Perbaiki nilai PIN_* di class Config.")

    def read_dht(self):
        try:
            t, h = self.dht.temperature, self.dht.humidity
        except RuntimeError:
            return None
        except Exception as exc:
            logger.warning("Pengecualian pembacaan DHT: %s", exc)
            return None
        if t is None or h is None or not (0 <= t <= 50 and 0 <= h <= 100):
            return None
        return float(t), float(h)

    def apply_pattern(self, state: State):
        """
        - IDLE_NORMAL : LED Hijau menyala tetap.
        - Semua peringatan lain : LED Merah berkedip cepat, LED Hijau mati.
        - Buzzer boks hanya untuk INACTIVITY_ALERT, FEEDING_WAKE, EMERGENCY.
          Suhu/kelembapan sengaja hening di boks (alarm hanya di web).
        """
        g, r, b = self.led_hijau, self.led_merah, self.buzzer
        g.off(); r.off(); b.off()

        if state is State.IDLE_NORMAL:
            g.on()
        elif state is State.EMERGENCY:
            r.blink(on_time=0.1, off_time=0.1)
            b.beep(on_time=0.1, off_time=0.1)
        elif state in (State.INACTIVITY_ALERT, State.FEEDING_WAKE):
            r.blink(on_time=0.15, off_time=0.15)
            b.beep(on_time=0.4, off_time=0.4)
        else:  # THERMAL, HUMIDITY, SENSOR_FAULT, CRYING
            r.blink(on_time=0.15, off_time=0.15)

    def chirp(self):
        self.buzzer.beep(on_time=0.08, off_time=0.08, n=1)

    def close(self):
        for dev in (self.led_merah, self.led_hijau, self.buzzer,
                    self.tombol, self.pir, self.sound):
            try:
                if hasattr(dev, "off"): dev.off()
                dev.close()
            except Exception:
                pass
        try:
            self.dht.exit()
        except Exception:
            pass


# ============================ LOGIKA FSM ============================
class ControlError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


class IncubatorSentinel:
    def __init__(self, hw: Hardware):
        self.hw = hw
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.started_at = time.monotonic()

        self.temperature = None
        self.humidity = None
        self.last_dht_ok = None

        self.pir_raw = False
        self._pir_prev = False
        self.motion = False
        self.motion_events = 0
        self.last_motion = time.monotonic()

        self.crying = False
        self.last_cry = None
        self._mask_until = 0.0

        self.th_high, self.th_low = Config.TEMP_DEFAULT_HIGH, Config.TEMP_DEFAULT_LOW
        self.hum_high, self.hum_low = Config.HUM_DEFAULT_HIGH, Config.HUM_DEFAULT_LOW

        self.thermal_alarm = False
        self.humidity_alarm = False
        self.sensor_fault = False
        self.inactivity = False
        self.feeding_wake = False
        self.cry_active = False
        self.emergency = False

        self.state = State.IDLE_NORMAL
        self._chirp_requested = False
        self._applied_state = None

        self.logs = deque(maxlen=Config.LOG_SIZE)
        self.log("info", "Sistem berhasil diinisialisasi" + (" (MODE SIMULASI)." if SIMULATION else "."))
        if not SIMULATION:
            self.log("info", "PIR HC-SR501 butuh pemanasan 30–60 detik; gerakan awal bisa tidak stabil.")

        hw.tombol.when_held = self._on_button_confirmed
        hw.pir.when_activated = self._on_motion_detected
        hw.sound.when_activated = self._on_sound_detected

    def log(self, level, msg):
        entry = {"time": datetime.now().strftime("%H:%M:%S"), "level": level, "msg": msg}
        with self.lock:
            self.logs.appendleft(entry)
        {"info": logger.info, "warn": logger.warning, "alarm": logger.error}[level](msg)

    # ---------- CALLBACK GPIO ----------
    def _sound_masked(self, now):
        return Config.MASK_SOUND_DURING_BUZZER and (self.state in BUZZER_STATES or now < self._mask_until)

    def _on_motion_detected(self):
        with self.lock:
            self.last_motion = time.monotonic()

    def _on_sound_detected(self):
        with self.lock:
            now = time.monotonic()
            if not self._sound_masked(now):
                self.last_cry = now

    def _on_button_confirmed(self):
        """Tombol fisik (ditahan BUTTON_HOLD detik): satu-satunya cara mematikan buzzer boks."""
        with self.lock:
            if self.inactivity or self.feeding_wake or self.emergency:
                self.inactivity = self.feeding_wake = self.emergency = False
                self.last_motion = time.monotonic()
                self._chirp_requested = True
                self.log("info", "VERIFIKASI FISIK: Perawat hadir di lokasi. Buzzer inkubator dinonaktifkan.")
            else:
                self._chirp_requested = True
                self.log("info", "PERAWAT: Konfirmasi pemeriksaan rutin fisik di lokasi.")

    # ---------- THREAD ENGINE UTAMA ----------
    def run(self):
        next_dht = 0.0
        while not self.stop_event.is_set():
            try:
                now = time.monotonic()
                reading = None
                if now >= next_dht:
                    reading = self.hw.read_dht()
                    next_dht = now + Config.DHT_INTERVAL

                pir_now = bool(self.hw.pir.is_active)
                sound_now = bool(self.hw.sound.is_active)

                with self.lock:
                    self._update_locked(now, reading, pir_now, sound_now)
                    current_st = self.state
                    chirp, self._chirp_requested = self._chirp_requested, False
                    if chirp:
                        self._mask_until = now + 0.6

                if current_st != self._applied_state:
                    self.hw.apply_pattern(current_st)
                    self._applied_state = current_st

                if chirp:
                    self.hw.chirp()

            except Exception:
                logger.exception("Kesalahan eksekusi pada hardware engine")
            self.stop_event.wait(Config.LOOP_INTERVAL)

    @staticmethod
    def _band_alarm(active, value, low, high, hyst):
        """Alarm menyala bila di luar [low, high]; padam hanya setelah masuk pita dalam (histeresis)."""
        if not active:
            return value >= high or value <= low
        return not (low + hyst <= value <= high - hyst)

    def _update_locked(self, now, reading, pir_now, sound_now):
        # 1. Telemetri DHT11 & deteksi kerusakan sensor
        if reading is not None:
            self.temperature, self.humidity = reading
            self.last_dht_ok = now

        ref_time = self.last_dht_ok if self.last_dht_ok is not None else self.started_at
        fault = (now - ref_time) > Config.DHT_STALE_AFTER
        if fault != self.sensor_fault:
            self.sensor_fault = fault
            if fault:
                self.log("alarm", "SENSOR: DHT11 tidak merespons! Periksa kabel.")
            else:
                self.log("info", "SENSOR: Komunikasi DHT11 pulih normal.")

        # 2. Suhu & kelembapan (dengan histeresis)
        if self.temperature is not None and not self.sensor_fault:
            t, h = self.temperature, self.humidity
            new_t = self._band_alarm(self.thermal_alarm, t, self.th_low, self.th_high, Config.HYSTERESIS)
            if new_t != self.thermal_alarm:
                self.thermal_alarm = new_t
                if new_t:
                    self.log("alarm", f"SUHU KRITIS: {t:.1f}°C (aman {self.th_low:.1f}–{self.th_high:.1f}°C). "
                                      "Peringatan hanya di LED dan web.")
                else:
                    self.log("info", f"SUHU NORMAL: kembali stabil di {t:.1f}°C.")

            new_h = self._band_alarm(self.humidity_alarm, h, self.hum_low, self.hum_high, Config.HUM_HYSTERESIS)
            if new_h != self.humidity_alarm:
                self.humidity_alarm = new_h
                if new_h:
                    self.log("alarm", f"KELEMBAPAN KRITIS: {h:.0f}% (aman {self.hum_low:.0f}–{self.hum_high:.0f}%). "
                                      "Peringatan hanya di LED dan web.")
                else:
                    self.log("info", f"KELEMBAPAN NORMAL: kembali stabil di {h:.0f}%.")

        # 3. Gerak (PIR) & inaktivitas 2 menit
        self.pir_raw = pir_now
        if pir_now:
            if not self._pir_prev:
                self.motion_events += 1
            self.last_motion = now
        self._pir_prev = pir_now
        inactivity_s = now - self.last_motion
        self.motion = inactivity_s <= Config.MOTION_DISPLAY_HOLD

        if inactivity_s >= Config.INACTIVITY_TIMEOUT and not self.inactivity:
            self.inactivity = True
            self.log("alarm", f"INAKTIVITAS: Tidak ada gerak {int(inactivity_s)} detik! "
                              "Buzzer boks berbunyi, perawat wajib memeriksa bayi.")

        # 4. Tangisan (sensor suara) — diabaikan saat buzzer boks sendiri berbunyi
        masked = self._sound_masked(now)
        if sound_now and not masked:
            self.last_cry = now
        cry = self.last_cry is not None and (now - self.last_cry) <= Config.CRY_HOLD
        if cry and not self.cry_active:
            self.log("warn", "DISTRES: Tangisan/suara bayi terdeteksi oleh sensor suara.")
        self.cry_active = cry
        self.crying = cry

        # 5. Resolusi status FSM
        new_state = self._resolve_state()
        if new_state is not self.state:
            old_st, self.state = self.state, new_state
            if new_state is State.IDLE_NORMAL:
                self.log("info", f"SISTEM STABIL: Inkubator kembali normal (sebelumnya {old_st.value}).")

    def _resolve_state(self):
        if self.emergency:      return State.EMERGENCY
        if self.inactivity:     return State.INACTIVITY_ALERT
        if self.feeding_wake:   return State.FEEDING_WAKE
        if self.thermal_alarm:  return State.THERMAL_HAZARD
        if self.humidity_alarm: return State.HUMIDITY_HAZARD
        if self.sensor_fault:   return State.SENSOR_FAULT
        if self.cry_active:     return State.CRYING_DISTRESS
        return State.IDLE_NORMAL

    def stop(self):
        self.stop_event.set()

    # ---------- PERINTAH KONTROL DARI WEB ----------
    def trigger_feeding_wake(self, source):
        """Bunyikan buzzer boks untuk membangunkan bayi (jadwal susu). Mati hanya via tombol fisik."""
        with self.lock:
            if self.emergency:
                raise ControlError("CODE BLUE sedang aktif.", 409)
            if self.inactivity or self.feeding_wake:
                raise ControlError("Buzzer stimulasi sudah aktif. Tekan tombol fisik di inkubator.", 409)
            self.feeding_wake = True
            msg = f"JADWAL SUSU: Buzzer boks dibunyikan dari {source} untuk membangunkan bayi."
            self.log("warn", msg)
            return msg

    def trigger_emergency(self, source):
        with self.lock:
            if self.emergency:
                raise ControlError("Panggilan darurat CODE BLUE sudah aktif.", 409)
            self.emergency = True
            msg = f"CODE BLUE: Panggilan darurat diaktifkan oleh {source}!"
            self.log("alarm", msg)
            return msg

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            alerts = []
            if self.emergency:      alerts.append("CODE BLUE")
            if self.inactivity:     alerts.append("Inaktivitas bayi")
            if self.feeding_wake:   alerts.append("Stimulasi jadwal susu")
            if self.thermal_alarm:  alerts.append("Suhu")
            if self.humidity_alarm: alerts.append("Kelembapan")
            if self.sensor_fault:   alerts.append("Sensor DHT11")
            if self.cry_active:     alerts.append("Bayi menangis")
            return {
                "state": self.state.value,
                "active_alerts": alerts,
                "temperature": self.temperature,
                "humidity": self.humidity,
                "sensor_ok": not self.sensor_fault,
                "sensor_age_seconds": (round(now - self.last_dht_ok, 1) if self.last_dht_ok is not None else None),
                "motion": self.motion,
                "pir_raw": self.pir_raw,
                "motion_events": self.motion_events,
                "last_motion_ago": int(now - self.last_motion),
                "inactivity_seconds": int(now - self.last_motion),
                "crying": self.cry_active,
                "last_cry_ago": (int(now - self.last_cry) if self.last_cry is not None else None),
                "thermal_alarm": self.thermal_alarm,
                "humidity_alarm": self.humidity_alarm,
                "inactivity_alarm": self.inactivity,
                "feeding_wake": self.feeding_wake,
                "emergency": self.emergency,
                "temp_threshold_high": self.th_high,
                "temp_threshold_low": self.th_low,
                "hum_threshold_high": self.hum_high,
                "hum_threshold_low": self.hum_low,
                "auth_required": bool(Config.AUTH_TOKEN),
                "config": {
                    "hysteresis": Config.HYSTERESIS,
                    "inactivity_warning": Config.INACTIVITY_WARNING,
                    "inactivity_timeout": Config.INACTIVITY_TIMEOUT,
                },
                "simulation": SIMULATION,
                "logs": list(self.logs)[:20],
                "server_time": datetime.now().isoformat(timespec="seconds"),
            }


# ======================== WEB SERVER (FLASK) ========================
app = Flask(__name__)
hardware = Hardware()
sentinel = IncubatorSentinel(hardware)


@app.after_request
def no_cache(resp):
    # Nonaktifkan cache untuk semua halaman agar browser selalu memuat HTML/API terbaru
    resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    resp.headers["Pragma"] = "no-cache"
    resp.headers["Expires"] = "0"
    return resp


@app.route("/")
def index():
    return render_template("index.html")


@app.get("/api/status")
def get_status():
    return jsonify(sentinel.snapshot())


def _authorized():
    token = Config.AUTH_TOKEN
    if not token:
        return True
    supplied = request.headers.get("X-Auth-Token", "")
    return hmac.compare_digest(supplied.encode("utf-8"), token.encode("utf-8"))


@app.post("/api/control")
def post_control():
    if not _authorized():
        return jsonify({"status": "error", "message": "Token tidak valid."}), 401

    data = request.get_json(silent=True)
    if not isinstance(data, dict):
        return jsonify({"status": "error", "message": "Format data JSON tidak valid."}), 400

    src = "Nurse Web Dashboard"
    action = data.get("action")

    if action == "wake_baby":
        handler = lambda: sentinel.trigger_feeding_wake(src)
    elif action in ("update_thresholds", "update_humidity_thresholds"):
        return jsonify({
            "status": "error",
            "message": "Ambang suhu dan kelembapan bersifat default dan tidak dapat diubah dari monitor."
        }), 403
    elif action == "emergency_alert":
        handler = lambda: sentinel.trigger_emergency(src)
    elif action in ("mute", "unmute", "stop_buzzer", "clear_emergency"):
        return jsonify({
            "status": "error",
            "message": "Protokol Keselamatan: buzzer inkubator hanya dapat dimatikan via tombol fisik di boks!"
        }), 403
    else:
        return jsonify({"status": "error", "message": f"Aksi tidak dikenal: {action!r}"}), 400

    try:
        message = handler()
    except ControlError as exc:
        return jsonify({"status": "error", "message": str(exc)}), exc.status

    return jsonify({"status": "success", "message": message, "data": sentinel.snapshot()})


def main():
    if not Config.AUTH_TOKEN:
        logger.warning("INCUBATOR_TOKEN belum di-set: API kontrol terbuka untuk semua perangkat di jaringan!")
    engine = threading.Thread(target=sentinel.run, name="hardware-engine", daemon=True)
    engine.start()
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    try:
        app.run(host=Config.HOST, port=Config.PORT, debug=False, use_reloader=False, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        logger.info("Mematikan server dan subsistem...")
        sentinel.stop()
        engine.join(timeout=2.0)
        hardware.close()


if __name__ == "__main__":
    main()
