"""Safely tune the live five-tendon joint-angle PI controller via the open GUI.

The tool intentionally does not open either serial port.  It drives the already
running Tk GUI, starts one independent continuous-record CSV per candidate, and
monitors that CSV while the hand moves.  This avoids competing with the upper
computer for COM6/COM7.

Physical motion is disabled unless ``--execute`` is supplied.
"""

from __future__ import annotations

import argparse
import csv
import ctypes
from ctypes import wintypes
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import time
from typing import Iterable, Optional, Sequence

try:
    from pywinauto import Application, keyboard
except ImportError as exc:  # pragma: no cover - environment setup error
    raise SystemExit("live_pi_autotune.py 需要 pywinauto：python -m pip install pywinauto") from exc


USER32 = ctypes.windll.user32
SW_MAXIMIZE = 3
KEYEVENTF_KEYUP = 0x0002
KEYEVENTF_UNICODE = 0x0004
MOUSEEVENTF_LEFTDOWN = 0x0002
MOUSEEVENTF_LEFTUP = 0x0004
VK_CONTROL = 0x11
VK_A = 0x41


@dataclass(frozen=True)
class Candidate:
    kp: float
    ki: float


@dataclass
class ResponseMetric:
    target_deg: float
    joint: int
    start_deg: float
    rmse_deg: float
    steady_error_deg: float
    overshoot_deg: float
    rise_time_s: Optional[float]
    settling_time_s: Optional[float]


@dataclass
class CandidateResult:
    kp: float
    ki: float
    raw_csv: str
    score: float
    mean_rmse_deg: float
    mean_steady_error_deg: float
    mean_overshoot_deg: float
    mean_settling_time_s: Optional[float]
    unsettled_joint_steps: int
    peak_tension_n: float
    metrics: list[ResponseMetric]


class SafetyAbort(RuntimeError):
    pass


class POINT(ctypes.Structure):
    _fields_ = [("x", wintypes.LONG), ("y", wintypes.LONG)]


def _enum_visible_windows() -> list[tuple[int, str]]:
    found: list[tuple[int, str]] = []
    callback_type = ctypes.WINFUNCTYPE(ctypes.c_bool, wintypes.HWND, wintypes.LPARAM)

    def callback(hwnd: int, _lparam: int) -> bool:
        if not USER32.IsWindowVisible(hwnd):
            return True
        length = USER32.GetWindowTextLengthW(hwnd)
        if length <= 0:
            return True
        buf = ctypes.create_unicode_buffer(length + 1)
        USER32.GetWindowTextW(hwnd, buf, length + 1)
        found.append((int(hwnd), buf.value))
        return True

    USER32.EnumWindows(callback_type(callback), 0)
    return found


def find_encoder_window() -> int:
    for hwnd, title in _enum_visible_windows():
        if "Encoder / Servo" in title:
            return hwnd
    raise RuntimeError("找不到已打开的 Encoder / Servo 上位机窗口。")


def window_rect(hwnd: int) -> wintypes.RECT:
    rect = wintypes.RECT()
    if not USER32.GetWindowRect(hwnd, ctypes.byref(rect)):
        raise ctypes.WinError()
    return rect


def foreground(hwnd: int) -> wintypes.RECT:
    USER32.ShowWindow(hwnd, SW_MAXIMIZE)
    USER32.SetForegroundWindow(hwnd)
    time.sleep(0.25)
    if int(USER32.GetForegroundWindow()) != int(hwnd):
        raise RuntimeError("无法取得上位机窗口焦点，已拒绝发送鼠标/键盘输入。")
    return window_rect(hwnd)


def click_relative(hwnd: int, x: int, y: int) -> None:
    rect = foreground(hwnd)
    USER32.SetCursorPos(rect.left + x, rect.top + y)
    USER32.mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0)
    USER32.mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0)
    time.sleep(0.12)


def _key(vk: int, up: bool = False) -> None:
    USER32.keybd_event(vk, 0, KEYEVENTF_KEYUP if up else 0, 0)


def ctrl_a() -> None:
    _key(VK_CONTROL)
    _key(VK_A)
    _key(VK_A, True)
    _key(VK_CONTROL, True)


def type_unicode(text: str) -> None:
    for char in text:
        code = ord(char)
        USER32.keybd_event(0, code, KEYEVENTF_UNICODE, 0)
        USER32.keybd_event(0, code, KEYEVENTF_UNICODE | KEYEVENTF_KEYUP, 0)


class LiveGui:
    # Tk widgets are exposed as unnamed UIA Panes.  These verified points are
    # used only to select the smallest UIA element under the point; click_input
    # then invokes that element rather than emitting an unverified global click.
    KP_ENTRY = (343, 469)
    KP_APPLY = (508, 469)
    KI_ENTRY = (968, 469)
    KI_APPLY = (1132, 469)
    TARGET_ENTRY = (1909, 224)
    TARGET_ALL = (2298, 224)
    RECORD_TOGGLE = (2470, 224)
    STOP = (227, 224)
    STEP_POSES_ENTRY = (1400, 565)

    def __init__(self, hwnd: int) -> None:
        self.hwnd = hwnd
        rect = foreground(hwnd)
        if rect.right - rect.left < 2400 or rect.bottom - rect.top < 1300:
            raise RuntimeError("上位机未能最大化到预期的 2560x1440 布局。")

        self.window = Application(backend="uia").connect(handle=hwnd).window(handle=hwnd)
        self._control_cache: dict[tuple[int, int], object] = {}

    def control_at(self, point: tuple[int, int]):
        cached = self._control_cache.get(point)
        if cached is not None:
            return cached
        x, y = point
        candidates = []
        for control in self.window.descendants():
            rect = control.rectangle()
            if rect.left < x < rect.right and rect.top < y < rect.bottom:
                area = max(1, rect.right - rect.left) * max(1, rect.bottom - rect.top)
                candidates.append((area, control))
        if not candidates:
            raise RuntimeError(f"UI Automation 未找到坐标 {point} 对应的控件。")
        control = min(candidates, key=lambda item: item[0])[1]
        self._control_cache[point] = control
        return control

    def replace_entry(self, point: tuple[int, int], text: str) -> None:
        self.window.set_focus()
        entry = self.control_at(point)
        entry.click_input()
        keyboard.send_keys("^a", pause=0.03)
        keyboard.send_keys(text, pause=0.01, with_spaces=True)
        time.sleep(0.1)

    def click_control(self, point: tuple[int, int]) -> None:
        self.window.set_focus()
        self.control_at(point).click_input()
        time.sleep(0.15)

    def set_gains(self, kp: float, ki: float) -> None:
        # Force an integral-state reset even if the candidate Ki equals the
        # previously applied value.
        self.replace_entry(self.KI_ENTRY, "0")
        self.click_control(self.KI_APPLY)
        time.sleep(0.25)
        self.replace_entry(self.KP_ENTRY, f"{kp:.4f}")
        self.click_control(self.KP_APPLY)
        time.sleep(0.25)
        self.replace_entry(self.KI_ENTRY, f"{ki:.4f}")
        self.click_control(self.KI_APPLY)
        time.sleep(0.4)

    def send_joint_target(self, target: float) -> None:
        self.send_joint_targets([target] * 4)

    def send_joint_targets(self, targets: Sequence[float]) -> None:
        if len(targets) != 4:
            raise ValueError("exactly four joint targets are required")
        text = ";".join(f"{float(target):.3f}" for target in targets)
        self.replace_entry(self.TARGET_ENTRY, text)
        self.click_control(self.TARGET_ALL)

    def toggle_record(self) -> None:
        self.click_control(self.RECORD_TOGGLE)

    def stop_control(self) -> None:
        self.click_control(self.STOP)

    def restore_step_pose_text(self, text: str) -> None:
        self.replace_entry(self.STEP_POSES_ENTRY, text)


def latest_record(directory: Path, newer_than: float = 0.0) -> Optional[Path]:
    paths = [
        p
        for p in directory.glob("continuous_record_*.csv")
        if not p.name.endswith("_events.csv") and p.stat().st_mtime >= newer_than
    ]
    return max(paths, key=lambda p: p.stat().st_mtime) if paths else None


def wait_for_new_record(directory: Path, newer_than: float, timeout_s: float = 4.0) -> Path:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        path = latest_record(directory, newer_than)
        if path is not None:
            return path
        time.sleep(0.1)
    raise RuntimeError("点击连续记录后没有创建新的 CSV；已停止调参。")


def read_rows(path: Path) -> list[dict[str, str]]:
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as fp:
            return list(csv.DictReader(fp))
    except PermissionError:
        return []


def as_float(row: dict[str, str], key: str) -> Optional[float]:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def check_last_row(path: Path, *, max_tension_n: float) -> None:
    rows = read_rows(path)
    if not rows:
        return
    row = rows[-1]
    if row.get("force_complete") not in {"1", "1.0", "True", "true"}:
        raise SafetyAbort("五路力数据不完整")
    tensions: list[float] = []
    for motor in range(5):
        force_n = as_float(row, f"tension_m{motor:02d}_n")
        if force_n is None:
            raise SafetyAbort(f"M{motor:02d} 张力无效")
        tensions.append(-force_n)
        online = row.get(f"servo_m{motor:02d}_online", "")
        if online not in {"1", "1.0", "True", "true"}:
            raise SafetyAbort(f"M{motor:02d} 离线")
    if max(tensions) >= max_tension_n:
        raise SafetyAbort(f"张力达到提前中止线：{max(tensions):.1f} N >= {max_tension_n:.1f} N")
    for joint in range(4):
        if row.get(f"joint_j{joint:02d}_valid") not in {"1", "1.0", "True", "true"}:
            raise SafetyAbort(f"J{joint:02d} 编码器无效")
        angle = as_float(row, f"joint_j{joint:02d}_deg")
        if angle is None or not (-2.0 <= angle <= 25.0):
            raise SafetyAbort(f"J{joint:02d} 超出调参姿态保护范围：{angle}")
    j01 = as_float(row, "joint_j01_deg")
    if j01 is None or j01 <= 0.5:
        raise SafetyAbort(f"J01 未严格保持正角度：{j01}")


def monitored_hold(path: Path, duration_s: float, *, max_tension_n: float) -> None:
    deadline = time.monotonic() + duration_s
    missing = 0
    while time.monotonic() < deadline:
        before = path.stat().st_size if path.exists() else 0
        time.sleep(0.45)
        after = path.stat().st_size if path.exists() else 0
        if after <= before:
            missing += 1
            if missing >= 5:
                raise SafetyAbort("连续记录超过 2 s 未增长，可能通信中断")
        else:
            missing = 0
        check_last_row(path, max_tension_n=max_tension_n)


def segment_metrics(
    rows: list[dict[str, str]], target: float, *, skip_initial_baseline: bool = False
) -> list[ResponseMetric]:
    matches: list[int] = []
    for index, row in enumerate(rows):
        values = [as_float(row, f"target_j{joint:02d}_deg") for joint in range(4)]
        if all(value is not None and abs(value - target) < 0.05 for value in values):
            if index == 0:
                matches.append(index)
            else:
                previous = [as_float(rows[index - 1], f"target_j{joint:02d}_deg") for joint in range(4)]
                if not all(value is not None and abs(value - target) < 0.05 for value in previous):
                    matches.append(index)
    output: list[ResponseMetric] = []
    for start_index in matches:
        # The first 10-degree block is a stationary baseline, not a response.
        # Later 10-degree blocks are the two return-to-center responses and
        # must be scored; omitting them can incorrectly reward a controller
        # that reaches +/-5 degrees but returns to the working point poorly.
        if skip_initial_baseline and start_index == 0:
            continue
        end_index = start_index + 1
        while end_index < len(rows):
            values = [as_float(rows[end_index], f"target_j{joint:02d}_deg") for joint in range(4)]
            if not all(value is not None and abs(value - target) < 0.05 for value in values):
                break
            end_index += 1
        segment = rows[start_index:end_index]
        if len(segment) < 10:
            continue
        t0 = as_float(segment[0], "elapsed_s") or 0.0
        prior = rows[max(0, start_index - 10):start_index]
        for joint in range(4):
            samples: list[tuple[float, float]] = []
            for row in segment:
                t = as_float(row, "elapsed_s")
                y = as_float(row, f"joint_j{joint:02d}_deg")
                if t is not None and y is not None:
                    samples.append((t - t0, y))
            prior_values = [
                value
                for row in prior
                if (value := as_float(row, f"joint_j{joint:02d}_deg")) is not None
            ]
            if len(samples) < 10:
                continue
            start_deg = statistics.mean(prior_values) if prior_values else samples[0][1]
            errors = [value - target for _, value in samples]
            rmse = math.sqrt(statistics.mean(error * error for error in errors))
            final_start = max(0, len(samples) - max(5, int(3.0 / 0.1)))
            steady_error = statistics.mean(abs(value - target) for _, value in samples[final_start:])
            direction = 1.0 if target >= start_deg else -1.0
            # The user-defined acceptable band is +/-1 degree. Motion inside
            # that band is treated as target attainment rather than overshoot.
            raw_overshoot = max(0.0, max(direction * (value - target) for _, value in samples))
            overshoot = max(0.0, raw_overshoot - 1.0)
            amplitude = abs(target - start_deg)
            rise_time: Optional[float] = None
            if amplitude > 0.2:
                threshold10 = start_deg + 0.1 * (target - start_deg)
                threshold90 = start_deg + 0.9 * (target - start_deg)
                t10 = next((t for t, value in samples if direction * (value - threshold10) >= 0.0), None)
                t90 = next((t for t, value in samples if direction * (value - threshold90) >= 0.0), None)
                if t10 is not None and t90 is not None and t90 >= t10:
                    rise_time = t90 - t10
            settling: Optional[float] = None
            # Require a full two-second window inside +/-1 degree.
            for idx, (t, _value) in enumerate(samples):
                window_end = t + 2.0
                window = [value for tt, value in samples[idx:] if tt <= window_end]
                if len(window) >= 10 and all(abs(value - target) <= 1.0 for value in window):
                    settling = t
                    break
            output.append(
                ResponseMetric(
                    target_deg=target,
                    joint=joint,
                    start_deg=start_deg,
                    rmse_deg=rmse,
                    steady_error_deg=steady_error,
                    overshoot_deg=overshoot,
                    rise_time_s=rise_time,
                    settling_time_s=settling,
                )
            )
    return output


def evaluate_candidate(
    path: Path,
    candidate: Candidate,
    *,
    center_deg: float = 10.0,
    amplitude_deg: float = 5.0,
    low_amplitude_deg: Optional[float] = None,
) -> CandidateResult:
    low_amplitude = amplitude_deg if low_amplitude_deg is None else low_amplitude_deg
    rows = read_rows(path)
    metrics = (
        segment_metrics(rows, center_deg + amplitude_deg)
        + segment_metrics(rows, center_deg, skip_initial_baseline=True)
        + segment_metrics(rows, center_deg - low_amplitude)
    )
    if len(metrics) < 16:
        raise RuntimeError(f"{path.name} 有效阶跃指标不足：{len(metrics)}/16")
    peak_tension = 0.0
    for row in rows:
        for motor in range(5):
            value = as_float(row, f"tension_m{motor:02d}_n")
            if value is not None:
                peak_tension = max(peak_tension, -value)
    mean_rmse = statistics.mean(item.rmse_deg for item in metrics)
    mean_steady = statistics.mean(item.steady_error_deg for item in metrics)
    mean_overshoot = statistics.mean(item.overshoot_deg for item in metrics)
    settled = [item.settling_time_s for item in metrics if item.settling_time_s is not None]
    unsettled = len(metrics) - len(settled)
    mean_settle = statistics.mean(settled) if settled else None
    score = mean_rmse + 0.8 * mean_steady + 0.4 * mean_overshoot + 0.75 * unsettled
    if mean_settle is not None:
        score += 0.03 * mean_settle
    score += max(0.0, peak_tension - 40.0) * 0.05
    return CandidateResult(
        kp=candidate.kp,
        ki=candidate.ki,
        raw_csv=str(path),
        score=score,
        mean_rmse_deg=mean_rmse,
        mean_steady_error_deg=mean_steady,
        mean_overshoot_deg=mean_overshoot,
        mean_settling_time_s=mean_settle,
        unsettled_joint_steps=unsettled,
        peak_tension_n=peak_tension,
        metrics=metrics,
    )


def write_report(
    output_dir: Path,
    results: Iterable[CandidateResult],
    *,
    center_deg: float = 10.0,
    amplitude_deg: float = 5.0,
    low_amplitude_deg: Optional[float] = None,
) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = output_dir / f"pi_autotune_{stamp}.json"
    markdown_path = output_dir / f"pi_autotune_{stamp}.md"
    ordered = sorted(results, key=lambda item: item.score)
    low_amplitude = amplitude_deg if low_amplitude_deg is None else low_amplitude_deg
    json_path.write_text(
        json.dumps([asdict(item) for item in ordered], ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    lines = [
        f"<!-- center={center_deg:g} deg, high=+{amplitude_deg:g} deg, low=-{low_amplitude:g} deg -->",
        f"# {center_deg:g}°工作点联合 +{amplitude_deg:g}°/-{low_amplitude:g}° PI自动调参",
        "",
        "|排名|Kp|Ki (1/s)|得分|平均RMSE°|末段绝对误差°|平均有效超调(±1°死区)°|平均稳定时间s|未稳定关节阶跃|峰值张力N|",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for rank, item in enumerate(ordered, 1):
        settle = "--" if item.mean_settling_time_s is None else f"{item.mean_settling_time_s:.2f}"
        lines.append(
            f"|{rank}|{item.kp:.3f}|{item.ki:.3f}|{item.score:.3f}|"
            f"{item.mean_rmse_deg:.3f}|{item.mean_steady_error_deg:.3f}|"
            f"{item.mean_overshoot_deg:.3f}|{settle}|{item.unsettled_joint_steps}|"
            f"{item.peak_tension_n:.1f}|"
        )
    lines.extend(["", f"推荐参数：`Kp={ordered[0].kp:.3f}`，`Ki={ordered[0].ki:.3f} 1/s`。", ""])
    markdown_path.write_text("\n".join(lines), encoding="utf-8")
    return json_path, markdown_path


def run(args: argparse.Namespace) -> int:
    hwnd = find_encoder_window()
    rect = foreground(hwnd)
    print(f"Encoder/Servo HWND={hwnd}, rect=({rect.left},{rect.top},{rect.right},{rect.bottom})")
    if not args.execute:
        print("探测成功。未提供 --execute，因此没有发送任何物理控制命令。")
        return 0

    record_dir = Path(args.record_dir).resolve()
    candidates = [Candidate(*values) for values in args.candidate]
    gui = LiveGui(hwnd)
    results: list[CandidateResult] = []
    record_active = False
    dangerous_abort = False
    try:
        for index, candidate in enumerate(candidates, 1):
            print(f"[{index}/{len(candidates)}] Kp={candidate.kp:.3f}, Ki={candidate.ki:.3f}", flush=True)
            gui.set_gains(candidate.kp, candidate.ki)
            gui.send_joint_target(args.center_deg)
            time.sleep(0.5)
            started = time.time()
            gui.toggle_record()
            record_active = True
            path = wait_for_new_record(record_dir, started - 0.2)
            print(f"  CSV: {path}", flush=True)
            monitored_hold(path, args.baseline_s, max_tension_n=args.abort_tension_n)
            for target, hold_s in (
                (args.center_deg + args.amplitude_deg, args.step_s),
                (args.center_deg, args.center_s),
                (args.center_deg - args.low_amplitude_deg, args.step_s),
                (args.center_deg, args.center_s),
            ):
                print(f"  target={target:.1f}°, hold={hold_s:.1f}s", flush=True)
                gui.send_joint_target(target)
                monitored_hold(path, hold_s, max_tension_n=args.abort_tension_n)
            gui.toggle_record()
            record_active = False
            time.sleep(0.8)
            result = evaluate_candidate(
                path,
                candidate,
                center_deg=args.center_deg,
                amplitude_deg=args.amplitude_deg,
                low_amplitude_deg=args.low_amplitude_deg,
            )
            results.append(result)
            print(
                f"  score={result.score:.3f}, RMSE={result.mean_rmse_deg:.3f}°, "
                f"steady={result.mean_steady_error_deg:.3f}°, peakT={result.peak_tension_n:.1f}N",
                flush=True,
            )
        best = min(results, key=lambda item: item.score)
        gui.set_gains(best.kp, best.ki)
        gui.send_joint_target(args.center_deg)
        json_path, markdown_path = write_report(
            Path(args.output_dir),
            results,
            center_deg=args.center_deg,
            amplitude_deg=args.amplitude_deg,
            low_amplitude_deg=args.low_amplitude_deg,
        )
        print(f"BEST Kp={best.kp:.3f}, Ki={best.ki:.3f}, score={best.score:.3f}")
        print(f"JSON: {json_path}")
        print(f"Markdown: {markdown_path}")
        return 0
    except SafetyAbort as exc:
        dangerous_abort = True
        print(f"SAFETY ABORT: {exc}", flush=True)
        raise
    finally:
        if record_active:
            try:
                gui.toggle_record()
            except Exception as exc:  # pragma: no cover - emergency best effort
                print(f"停止记录失败: {exc}")
        if dangerous_abort:
            try:
                gui.stop_control()
            except Exception as exc:  # pragma: no cover - emergency best effort
                print(f"发送STOP失败: {exc}")
        elif results:
            # A non-safety software failure returns to the known center target.
            try:
                gui.send_joint_target(args.center_deg)
            except Exception as exc:  # pragma: no cover
                print(f"返回10°失败: {exc}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="实际操作已打开的上位机并移动硬件")
    parser.add_argument(
        "--candidate",
        nargs=2,
        type=float,
        metavar=("KP", "KI"),
        action="append",
        default=None,
        help="候选参数，可重复；默认测试四组低风险候选",
    )
    parser.add_argument("--baseline-s", type=float, default=5.0)
    parser.add_argument("--step-s", type=float, default=18.0)
    parser.add_argument("--center-s", type=float, default=10.0)
    parser.add_argument("--center-deg", type=float, default=10.0)
    parser.add_argument("--amplitude-deg", type=float, default=5.0)
    parser.add_argument("--low-amplitude-deg", type=float, default=None)
    parser.add_argument("--abort-tension-n", type=float, default=55.0)
    parser.add_argument("--record-dir", default="run_data/continuous_records")
    parser.add_argument("--output-dir", default="run_data/autotune")
    args = parser.parse_args()
    if args.candidate is None:
        args.candidate = [(0.30, 0.02), (0.45, 0.02), (0.60, 0.02), (0.45, 0.05)]
    for kp, ki in args.candidate:
        if not (0.0 <= kp <= 1.0 and 0.0 <= ki <= 0.2):
            parser.error("本工具的安全候选范围限定为 0<=Kp<=1、0<=Ki<=0.2")
    if args.abort_tension_n > 65.0:
        parser.error("提前中止线不得高于65 N")
    if args.low_amplitude_deg is None:
        args.low_amplitude_deg = args.amplitude_deg
    if args.amplitude_deg <= 0.0 or args.low_amplitude_deg <= 0.0:
        parser.error("amplitude-deg must be positive")
    if args.center_deg - args.low_amplitude_deg <= 0.5:
        parser.error("the low target must remain above 0.5 deg (J01 safety margin)")
    if args.center_deg + args.amplitude_deg > 25.0:
        parser.error("the high target must not exceed the 25 deg autotune safety bound")
    return args


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
