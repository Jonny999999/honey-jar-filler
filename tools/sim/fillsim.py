"""Session runner: drives the REAL firmware strategy code (libfillsim.so, built
by build.sh) with the closed-loop plant model from plant.py.

    from fillsim import run_session
    res = run_session("adaptive-heuristic", "thin_dummy", n_fills=9, bucket_g=2300,
                      params=dict(target_grams=300, max_gate_pct=50, ...))

Each returned fill dict holds the firmware's own fill_summary plus simulator
ground truth (true flow, gate activity, bucket level). A "reboot" event loads a
fresh copy of the library (all static/RAM state lost) while the file-backed NVS
shim keeps whatever the strategy persisted.
"""

from __future__ import annotations

import ctypes
import json
import os
import shutil
import tempfile
from dataclasses import dataclass, field
from pathlib import Path

from plant import Plant, Medium, MEDIA, SAMPLE_DT_S, SIM_DT_S, medium as make_medium

HERE = Path(__file__).resolve().parent
LIB = HERE / "build" / "libfillsim.so"

FILL, DRIP_WAIT, VERIFY_TARGET, FIND_SLOT, FAULT = 3, 5, 6, 1, 8
STATE_NAMES = {0: "IDLE", 1: "FIND_SLOT", 2: "VERIFY_EMPTY", 3: "FILL", 4: "SLOT_SETTLE",
               5: "DRIP_WAIT", 6: "VERIFY_TARGET", 7: "DONE", 8: "FAULT"}
FAULT_NAMES = {0: "NONE", 1: "NO_JAR", 2: "WEIGHT_RANGE", 3: "MOTOR_TIMEOUT", 4: "SERVO_TIMEOUT",
               5: "SCALE_TIMEOUT", 6: "EMPTY_HONEY", 7: "USER_ABORT"}


class Firmware:
    """One 'boot' of the firmware strategy library."""

    def __init__(self, strategy: str, preset: int, params: dict, log_path: str | None, nvs_path: str):
        self._tmp = tempfile.NamedTemporaryFile(suffix=".so", delete=False)
        self._tmp.close()
        shutil.copyfile(LIB, self._tmp.name)          # fresh copy => fresh static state
        lib = ctypes.CDLL(self._tmp.name)
        lib.sim_init.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p]
        lib.sim_param_set.argtypes = [ctypes.c_char_p, ctypes.c_double]
        lib.sim_param_get.argtypes = [ctypes.c_char_p]
        lib.sim_param_get.restype = ctypes.c_double
        lib.sim_begin_fill.argtypes = [ctypes.c_longlong, ctypes.c_float]
        lib.sim_tick.argtypes = [ctypes.c_longlong, ctypes.c_int, ctypes.c_float]
        lib.sim_gate.restype = ctypes.c_float
        lib.sim_gate_label.restype = ctypes.c_char_p
        lib.sim_take_summary.argtypes = [ctypes.c_char_p, ctypes.c_int]
        lib.sim_sample_json.argtypes = [ctypes.c_char_p, ctypes.c_int]
        self.lib = lib
        if not lib.sim_init(strategy.encode(), preset, (log_path or "").encode(), nvs_path.encode()):
            raise ValueError(f"unknown strategy {strategy!r} (rebuild after adding it?)")
        for k, v in params.items():
            if not lib.sim_param_set(k.encode(), float(v)):
                raise KeyError(f"unknown param {k}")
        self.buf = ctypes.create_string_buffer(16384)

    def summary(self) -> dict | None:
        if self.lib.sim_take_summary(self.buf, len(self.buf)):
            return json.loads(self.buf.value.decode())
        return None

    def sample(self) -> dict:
        self.lib.sim_sample_json(self.buf, len(self.buf))
        return json.loads(self.buf.value.decode())

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass


@dataclass
class Event:
    fill: int                    # 1-based fill index the event applies to
    kind: str                    # refill_bucket | medium | reboot | refill_midfill | param
    value: object = None
    at_s: float = 0.0            # for *_midfill: seconds after the fill started


def run_session(strategy: str, medium: str | Medium, n_fills: int, bucket_g: float,
                params: dict | None = None, events: list[Event] | None = None, seed: int = 1,
                preset: int = 1, log_path: str | None = None, nvs_dir: str | None = None,
                telemetry_path: str | None = None, max_fill_s: float = 900.0,
                stop_on_fault: bool = False) -> list[dict]:
    params = dict(params or {})
    events = list(events or [])
    med = make_medium(medium) if isinstance(medium, str) else medium
    own_nvs = nvs_dir is None
    nvs_dir = nvs_dir or tempfile.mkdtemp(prefix="fillsim_nvs_")
    nvs_path = os.path.join(nvs_dir, "nvs")
    fw = Firmware(strategy, preset, params, log_path, nvs_path)
    plant = Plant(med, bucket_g, seed=seed)
    tel = open(telemetry_path, "w") if telemetry_path else None
    now = 0.0
    next_sample = 0.0
    results = []

    def emit(obj):
        if tel:
            tel.write(json.dumps(obj) + "\n")

    try:
        for i in range(1, n_fills + 1):
            for ev in events:
                if ev.fill != i or ev.kind.endswith("_midfill"):
                    continue
                if ev.kind == "refill_bucket":
                    plant.add_to_bucket(float(ev.value))
                elif ev.kind == "medium":
                    v = ev.value
                    plant.set_medium(make_medium(v) if isinstance(v, str) else v)
                elif ev.kind == "reboot":
                    fw.close()
                    fw = Firmware(strategy, preset, params, log_path, nvs_path)
                elif ev.kind == "param":
                    k, v = ev.value
                    params[k] = v
                    fw.lib.sim_param_set(k.encode(), float(v))
            # carousel move + slot settle: gate closed, drips land on the carousel
            for _ in range(int(3.0 / SIM_DT_S)):
                plant.command(0.0)
                plant.step()
                now += SIM_DT_S
            plant.swap_jar()
            for _ in range(int(1.0 / SIM_DT_S)):
                plant.step()
                now += SIM_DT_S
            base = plant.read_scale()
            bucket_start = plant.bucket_g
            disp_start = plant.dispensed_g
            jar_start_t = now
            fw.lib.sim_begin_fill(int(now * 1e6), base)
            emit({"kind": "fill_start", "ts_us": int(now * 1e6), "run_id": 1, "slot_idx": i % 6,
                  "strategy_name": strategy, "preset_name": f"sim:{plant.m.name}",
                  "target_g": int(fw.lib.sim_param_get(b"target_grams")), "base_weight_g": base,
                  "scale_period_ms_cfg": 100, "fsm_period_ms_cfg": 10,
                  "params": {f"VAR({k})": fw.lib.sim_param_get(k.encode()) for k in (
                      "target_grams", "target_tol_low_g", "target_tol_high_g", "fill_timeout_ms",
                      "slow_remaining_g", "slow_gate_pct", "max_gate_pct", "close_remaining_g",
                      "drip_delay_ms")}})
            state = FILL
            gate_prev = 0.0
            gate_changes = 0
            gates_seen: list[float] = []
            max_gate = 0.0
            open_s = 0.0
            phase_s = {"fast": 0.0, "reduced": 0.0, "refill": 0.0, "closed": 0.0}
            first_flow_t = None
            phase_now = "fast"
            midfill = [ev for ev in events if ev.fill == i and ev.kind.endswith("_midfill")]
            summary = None
            last_state = state
            while True:
                t_in = now - jar_start_t
                for ev in list(midfill):
                    if t_in >= ev.at_s:
                        if ev.kind == "refill_midfill":
                            plant.add_to_bucket(float(ev.value))
                        elif ev.kind == "press_midfill":
                            # value = (grams, seconds): someone pushes on the scale
                            g, dur = ev.value
                            plant.ext_g = float(g)
                            midfill.append(Event(ev.fill, "release_midfill", None, at_s=ev.at_s + float(dur)))
                        elif ev.kind == "release_midfill":
                            plant.ext_g = 0.0
                        elif ev.kind == "medium_midfill":
                            plant.set_medium(make_medium(ev.value) if isinstance(ev.value, str) else ev.value)
                        midfill.remove(ev)
                new = now + 1e-9 >= next_sample
                grams = plant.read_scale() if new else 0.0
                if new:
                    next_sample += SAMPLE_DT_S
                state = fw.lib.sim_tick(int(now * 1e6), 1 if new else 0, grams)
                gate = float(fw.lib.sim_gate())
                if abs(gate - gate_prev) > 1e-3:
                    if gate <= 0.0 < gate_prev:
                        plant.close_impact()
                    if gate > 0.0:
                        gate_changes += 1
                        gates_seen.append(round(gate, 1))
                    emit({"kind": "gate", "ts_us": int(now * 1e6), "run_id": 1, "slot_idx": i % 6,
                          "gate_pct": gate, "text": fw.lib.sim_gate_label().decode()})
                    gate_prev = gate
                plant.command(gate)
                if gate > 0.0:
                    open_s += SIM_DT_S
                max_gate = max(max_gate, gate)
                if new and state == FILL:
                    ph = fw.sample().get("gate_phase") or phase_now
                    phase_now = ph
                if state == FILL:
                    if first_flow_t is None and plant.jar_mass > 0.5:
                        first_flow_t = now - jar_start_t
                    if first_flow_t is not None:
                        phase_s[phase_now] = phase_s.get(phase_now, 0.0) + SIM_DT_S
                if tel and new:
                    s = fw.sample()
                    rec = {"kind": "sample", "ts_us": int(now * 1e6), "run_id": 1, "slot_idx": i % 6,
                           "state": state, "target_g": int(fw.lib.sim_param_get(b"target_grams")),
                           "weight_g": grams, "relative_fill_g": grams - base, "gate_pct": gate,
                           "sim_true_flow_gps": plant.q, "sim_bucket_g": plant.bucket_g}
                    if s.get("valid"):
                        rec.update({k: v for k, v in s.items() if k != "valid"})
                    emit(rec)
                if state != last_state:
                    emit({"kind": "state", "ts_us": int(now * 1e6), "run_id": 1, "slot_idx": i % 6,
                          "state": state, "text": STATE_NAMES.get(state, "?")})
                    last_state = state
                s = fw.summary()
                if s:
                    summary = s
                    emit(s)
                if state not in (FILL, DRIP_WAIT, VERIFY_TARGET):
                    break
                plant.step()
                now += SIM_DT_S
                if now - jar_start_t > max_fill_s:
                    break
            fault = FAULT_NAMES.get(fw.lib.sim_fault(), "?") if state == FAULT else ""
            true_mass = plant.jar_mass
            target = fw.lib.sim_param_get(b"target_grams")
            res = {
                "fill": i,
                "medium": plant.m.name,
                "state": STATE_NAMES.get(state, str(state)),
                "fault": fault,
                "true_mass_g": true_mass,
                "true_err_g": true_mass - target,
                "jar_time_s": now - jar_start_t,
                "open_s": open_s,
                "dead_s": first_flow_t or 0.0,
                "fast_s": phase_s.get("fast", 0.0),
                "slow_s": phase_s.get("reduced", 0.0) + phase_s.get("refill", 0.0),
                "gate_changes": gate_changes,
                "gates": gates_seen,
                "max_gate": max_gate,
                "bucket_start_g": bucket_start,
                "dispensed_g": plant.dispensed_g - disp_start,
                "summary": summary or {},
            }
            results.append(res)
            if state == FAULT and stop_on_fault:
                break
            if state == FAULT:
                # operator clears the fault and restarts; the strategy state is
                # re-entered fresh on the next fill like on the real machine
                pass
    finally:
        fw.close()
        if tel:
            tel.close()
        if own_nvs:
            shutil.rmtree(nvs_dir, ignore_errors=True)
    return results


def table(results: list[dict], extra: tuple[str, ...] = ()) -> str:
    hdr = (f"{'#':>2} {'medium':12s} {'end':6s} {'err':>6} {'jar_s':>6} {'open_s':>6} {'dead':>5} {'fast':>5} {'slow':>5} {'moves':>5} "
           f"{'refl':>4} {'L':>5} {'post':>5} {'r_f':>5} {'r_s':>5} {'bucket':>6}  gates")
    lines = [hdr]
    for r in results:
        s = r["summary"]
        end = r["fault"] or s.get("text", r["state"])
        lines.append(
            f"{r['fill']:2d} {r['medium'][:12]:12s} {end[:6]:6s} {r['true_err_g']:6.1f} {r['jar_time_s']:6.1f} "
            f"{r['open_s']:6.1f} {r['dead_s']:5.1f} {r['fast_s']:5.1f} {r['slow_s']:5.1f} {r['gate_changes']:5d} {int(s.get('refill_count', 0)):4d} "
            f"{s.get('measured_dead_time_s', 0):5.2f} {s.get('measured_post_close_gain_g', 0):5.1f} "
            f"{s.get('measured_fast_rate_gps', 0):5.1f} {s.get('measured_slow_rate_gps', 0):5.1f} "
            f"{r['bucket_start_g']:6.0f}  {_gates(r['gates'])}")
    return "\n".join(lines)


def _gates(gs: list[float]) -> str:
    if len(gs) > 8:
        return " ".join(f"{g:g}" for g in gs[:4]) + f" ..({len(gs) - 8}).. " + " ".join(f"{g:g}" for g in gs[-4:])
    return " ".join(f"{g:g}" for g in gs)


def gate_table(results: list[dict]) -> str:
    lines = [f"{'#':>2} {'g_f':>5} {'g_s':>5} -> {'g_f':>5} {'g_s':>5} | {'r*_f':>5} {'obs_f':>5} | {'r*_s':>5} {'obs_s':>5} | "
             f"{'corr':>4} {'rgm':>3} {'close':>5} {'near':>5} {'post':>5} {'err':>5}"]
    for r in results:
        s = r["summary"]
        if not s:
            continue
        lines.append(f"{r['fill']:2d} {s.get('used_fast_gate_pct',0):5.1f} {s.get('used_slow_gate_pct',0):5.1f} -> "
                     f"{s.get('next_fast_gate_pct',0):5.1f} {s.get('next_slow_gate_pct',0):5.1f} | "
                     f"{s.get('target_fast_rate_gps',0):5.1f} {s.get('obs_fast_rate_gps',0):5.1f} | "
                     f"{s.get('target_slow_rate_gps',0):5.1f} {s.get('obs_slow_rate_gps',0):5.1f} | "
                     f"{int(s.get('gate_corrections',0)):4d} {int(s.get('regime_change',0)):3d} "
                     f"{s.get('used_close_early_g',0):5.1f} {s.get('used_near_close_g',0):5.1f} "
                     f"{s.get('measured_post_close_gain_g',0):5.1f} {r['true_err_g']:5.1f}")
    return "\n".join(lines)
