"""Hood diagnostic analysis — step response, settling time, overshoot, stationary accuracy.

Usage:
    python analyze_hood.py <csv_file>
    python analyze_hood.py                  # defaults to latest hood_diag_*.csv in parent dir
"""
import csv, sys, os, glob
import numpy as np


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def find_step_responses(cols, min_step_deg=1.0, settle_tolerance=2.0, settle_window=5):
    """Find setpoint step changes and measure response characteristics."""
    sp = cols["hood_setpoint"]
    act = cols["hood_actual"]
    t = cols["timestamp"]
    cur = cols["stator_current"]
    n = len(sp)

    # Detect step changes in setpoint
    sp_diff = np.abs(np.diff(sp))
    step_indices = np.where(sp_diff > min_step_deg)[0]

    # Merge consecutive step indices (same step change)
    steps = []
    if len(step_indices) > 0:
        start = step_indices[0]
        for i in range(1, len(step_indices)):
            if step_indices[i] - step_indices[i - 1] > 5:
                steps.append(start)
                start = step_indices[i]
        steps.append(start)

    results = []
    for idx in steps:
        if idx + 10 >= n:
            continue

        initial_pos = act[idx]
        target_pos = sp[idx + 1]
        step_size = target_pos - initial_pos
        direction = "UP" if step_size > 0 else "DOWN"
        abs_step = abs(step_size)

        if abs_step < min_step_deg:
            continue

        # Find settling time: first time |error| stays within tolerance for settle_window samples
        settle_time = None
        for j in range(idx + 1, min(n, idx + 500)):
            err = abs(act[j] - target_pos)
            if err < settle_tolerance:
                settled = True
                for k in range(j, min(n, j + settle_window)):
                    if abs(act[k] - target_pos) >= settle_tolerance:
                        settled = False
                        break
                if settled:
                    settle_time = t[j] - t[idx]
                    break

        # Rise time: 10% to 90% of step
        threshold_10 = initial_pos + 0.1 * step_size
        threshold_90 = initial_pos + 0.9 * step_size
        t_10, t_90 = None, None
        for j in range(idx + 1, min(n, idx + 500)):
            if direction == "UP":
                if t_10 is None and act[j] >= threshold_10:
                    t_10 = t[j] - t[idx]
                if t_90 is None and act[j] >= threshold_90:
                    t_90 = t[j] - t[idx]
            else:
                if t_10 is None and act[j] <= threshold_10:
                    t_10 = t[j] - t[idx]
                if t_90 is None and act[j] <= threshold_90:
                    t_90 = t[j] - t[idx]
            if t_10 is not None and t_90 is not None:
                break

        rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None

        # Overshoot: max deviation beyond target in direction of travel
        search_end = min(n, idx + 500)
        window = act[idx + 1:search_end]
        if direction == "UP":
            overshoot = max(0, np.max(window) - target_pos)
        else:
            overshoot = max(0, target_pos - np.min(window))
        overshoot_pct = (overshoot / abs_step * 100) if abs_step > 0 else 0

        # Peak current during step
        peak_current = np.max(np.abs(cur[idx:search_end]))

        results.append({
            "time": t[idx],
            "initial": initial_pos,
            "target": target_pos,
            "step_size": step_size,
            "direction": direction,
            "rise_time": rise_time,
            "settle_time": settle_time,
            "overshoot_deg": overshoot,
            "overshoot_pct": overshoot_pct,
            "peak_current": peak_current,
        })

    return results


def find_stationary_windows(cols, min_samples=50, sp_change_threshold=0.1):
    """Find windows where setpoint is stable."""
    sp = cols["hood_setpoint"]
    t = cols["timestamp"]
    n = len(sp)

    windows, start = [], None
    for i in range(1, n):
        if abs(sp[i] - sp[i - 1]) < sp_change_threshold:
            if start is None:
                start = i
        else:
            if start is not None and (i - start) >= min_samples:
                windows.append((start, i))
            start = None
    if start is not None and (n - start) >= min_samples:
        windows.append((start, n))

    return windows


def analyze_stationary(cols, windows):
    """Analyze position accuracy during stationary windows."""
    act = cols["hood_actual"]
    sp = cols["hood_setpoint"]
    err = cols["hood_error"]
    t = cols["timestamp"]

    results = []
    all_errs = []
    for s, e in windows:
        # Skip first 25 samples for settling
        settle = min(25, (e - s) // 4)
        sl = slice(s + settle, e)
        if len(err[sl]) < 10:
            continue

        w_err = err[sl]
        w_abs = np.abs(w_err)
        w_act = act[sl]

        all_errs.extend(w_abs.tolist())
        results.append({
            "setpoint": np.mean(sp[sl]),
            "duration": t[e - 1] - t[s],
            "err_mean": np.mean(w_abs),
            "err_p95": np.percentile(w_abs, 95),
            "bias": np.mean(w_err),
            "act_p2p": np.ptp(w_act),
        })

    all_errs = np.array(all_errs) if all_errs else np.array([0])
    return results, all_errs


def main():
    # Find CSV file
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        parent = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        candidates = sorted(glob.glob(os.path.join(parent, "hood_diag_*.csv")))
        if not candidates:
            print("No hood_diag_*.csv found. Pass a path as argument.")
            sys.exit(1)
        csv_path = candidates[-1]
        print(f"Using: {csv_path}")

    cols = load(csv_path)
    n = len(cols["timestamp"])
    duration = cols["timestamp"][-1] - cols["timestamp"][0]
    dt = np.diff(cols["timestamp"])

    print()
    print("=" * 70)
    print("  HOOD DIAGNOSTIC ANALYSIS")
    print("=" * 70)
    print(f"  File: {os.path.basename(csv_path)}")
    print(f"  Samples: {n}  Duration: {duration:.1f}s  Avg dt: {np.mean(dt)*1000:.1f}ms")
    print()

    # --- Step Responses ---
    steps = find_step_responses(cols)

    print("=" * 70)
    print("  STEP RESPONSES")
    print("=" * 70)

    if not steps:
        print("  No step responses detected (no setpoint changes > 1 deg)")
    else:
        print(f"  {'#':>3s} {'Dir':>4s} {'Step':>7s} {'Rise':>8s} {'Settle':>8s} "
              f"{'Overshoot':>10s} {'OS%':>6s} {'Ipeak':>7s}")
        print(f"  {'':>3s} {'':>4s} {'(deg)':>7s} {'(ms)':>8s} {'(ms)':>8s} "
              f"{'(deg)':>10s} {'':>6s} {'(A)':>7s}")
        print(f"  {'-'*3} {'-'*4} {'-'*7} {'-'*8} {'-'*8} {'-'*10} {'-'*6} {'-'*7}")

        for i, s in enumerate(steps):
            rise_str = f"{s['rise_time']*1000:.0f}" if s['rise_time'] is not None else "N/A"
            settle_str = f"{s['settle_time']*1000:.0f}" if s['settle_time'] is not None else "N/A"
            print(f"  {i+1:3d} {s['direction']:>4s} {s['step_size']:+7.1f} "
                  f"{rise_str:>8s} {settle_str:>8s} "
                  f"{s['overshoot_deg']:10.3f} {s['overshoot_pct']:5.1f}% "
                  f"{s['peak_current']:6.1f}")

        # Separate up vs down
        up_steps = [s for s in steps if s["direction"] == "UP"]
        down_steps = [s for s in steps if s["direction"] == "DOWN"]

        print()
        print("  SUMMARY BY DIRECTION:")
        for label, group in [("UP", up_steps), ("DOWN", down_steps)]:
            if not group:
                continue
            settles = [s["settle_time"] for s in group if s["settle_time"] is not None]
            rises = [s["rise_time"] for s in group if s["rise_time"] is not None]
            overshoots = [s["overshoot_pct"] for s in group]
            print(f"    {label:5s}: "
                  f"avg rise={np.mean(rises)*1000:.0f}ms  " if rises else f"    {label:5s}: avg rise=N/A  ",
                  end="")
            print(f"avg settle={np.mean(settles)*1000:.0f}ms  " if settles else "avg settle=N/A  ", end="")
            print(f"avg overshoot={np.mean(overshoots):.1f}%")

    # --- Stationary Analysis ---
    print()
    print("=" * 70)
    print("  STATIONARY ACCURACY")
    print("=" * 70)

    windows = find_stationary_windows(cols)
    win_results, all_errs = analyze_stationary(cols, windows)

    if not win_results:
        print("  No stationary windows detected (no stable setpoint for >1 second)")
    else:
        print(f"  {'Win':>4s} {'Setpt':>7s} {'Dur':>5s} {'|Err|':>8s} {'P95':>8s} "
              f"{'Bias':>8s} {'Act p2p':>8s}")
        print(f"  {'-'*4} {'-'*7} {'-'*5} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")
        for i, w in enumerate(win_results):
            print(f"  {i:4d} {w['setpoint']:6.1f}d {w['duration']:4.1f}s "
                  f"{w['err_mean']:7.3f}d {w['err_p95']:7.3f}d "
                  f"{w['bias']:+7.3f}d {w['act_p2p']:7.3f}d")

        print()
        print(f"  Overall |error| mean: {np.mean(all_errs):.3f} deg")
        print(f"  Overall |error| P95:  {np.percentile(all_errs, 95):.3f} deg")

    # --- Current Limiting ---
    print()
    print("=" * 70)
    print("  CURRENT LIMITING")
    print("=" * 70)
    cur = np.abs(cols["stator_current"])
    near_limit = np.sum(cur >= 19.0)
    at_limit = np.sum(cur >= 20.0)
    print(f"  Samples near limit (>=19A): {near_limit} ({near_limit/n*100:.1f}%)")
    print(f"  Samples at limit (>=20A):   {at_limit} ({at_limit/n*100:.1f}%)")
    print(f"  Peak stator current:        {np.max(cur):.1f} A")

    # --- Verdict ---
    print()
    print("=" * 70)
    print("  VERDICT")
    print("=" * 70)
    print()

    issues = []

    if steps:
        avg_settle = np.mean([s["settle_time"] for s in steps
                              if s["settle_time"] is not None]) * 1000
        avg_os = np.mean([s["overshoot_pct"] for s in steps])

        if avg_settle < 200:
            print("  Settling time:      EXCELLENT (<200ms)")
        elif avg_settle < 500:
            print(f"  Settling time:      GOOD ({avg_settle:.0f}ms)")
        else:
            print(f"  Settling time:      SLOW ({avg_settle:.0f}ms)")
            issues.append("Increase Motion Magic acceleration or reduce kD")

        if avg_os < 2:
            print(f"  Overshoot:          MINIMAL ({avg_os:.1f}%)")
        elif avg_os < 10:
            print(f"  Overshoot:          MODERATE ({avg_os:.1f}%)")
        else:
            print(f"  Overshoot:          EXCESSIVE ({avg_os:.1f}%)")
            issues.append("Reduce kP or increase kD")

    if win_results:
        overall_err = np.mean(all_errs)
        if overall_err < 0.5:
            print(f"  Stationary accuracy: EXCELLENT ({overall_err:.3f} deg)")
        elif overall_err < 1.0:
            print(f"  Stationary accuracy: GOOD ({overall_err:.3f} deg)")
        else:
            print(f"  Stationary accuracy: NEEDS WORK ({overall_err:.3f} deg)")
            issues.append("Increase kI or kS for better steady-state")

    if near_limit > n * 0.05:
        print(f"  Current limiting:    DETECTED ({near_limit/n*100:.0f}% of samples)")
        issues.append("Consider increasing stator current limit from 20A")
    else:
        print("  Current limiting:    NONE")

    # Check for up/down asymmetry (gravity compensation)
    if steps:
        up_settles = [s["settle_time"] for s in steps
                      if s["direction"] == "UP" and s["settle_time"] is not None]
        down_settles = [s["settle_time"] for s in steps
                        if s["direction"] == "DOWN" and s["settle_time"] is not None]
        if up_settles and down_settles:
            ratio = np.mean(up_settles) / np.mean(down_settles)
            if ratio > 1.5 or ratio < 0.67:
                worse = "UP" if ratio > 1.0 else "DOWN"
                print(f"  Gravity asymmetry:   PRESENT ({worse} steps {max(ratio,1/ratio):.1f}x slower)")
                issues.append(f"Tune kG (gravity feedforward) - {worse} direction is slower")
            else:
                print("  Gravity asymmetry:   BALANCED")

    print()
    if issues:
        print("  RECOMMENDATIONS:")
        for issue in issues:
            print(f"    -> {issue}")
    else:
        print("  No tuning issues detected.")
    print()


if __name__ == "__main__":
    main()
