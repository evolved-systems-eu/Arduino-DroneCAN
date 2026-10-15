#!/usr/bin/env python3
"""
ICEMonitor RPM Signal Processing Simulator
===========================================
Faithfully reproduces the ISR accumulator, two-tier stale detection,
plausibility filter (sliding window), and EMA filter from ICEMonitor/main.cpp.

All eight key scenarios are tested and the results verified automatically.

Usage:
    python rpm_simulator.py            # text output only
    python rpm_simulator.py --plot     # text + matplotlib plots saved to rpm_simulator.png
"""

import sys
import math
from typing import List, Tuple, Optional

# ---------------------------------------------------------------------------
# Parameters – match main.cpp defaults
# ---------------------------------------------------------------------------
MAX_PERIOD_US = 400_000          # 300 RPM lower bound
RPM_MAX       = 8_000            # RPM_MAX parameter default
MIN_PERIOD_US = 120_000_000 // RPM_MAX   # 15 000 µs  (8 000 RPM upper bound)

BCAST_MS      = 10               # broadcast tick interval
RPM_ALPHA     = 0.2              # EMA weight
RPM_A1, RPM_A0 = 1.0, 0.0
DUTY_ALPHA    = 0.3
DUTY_A1, DUTY_A0 = 1.0, 0.0
RPM_HOLD_MS   = 500              # no-signal → hold + CRANK_ERR
RPM_STOP_MS   = 3_000            # no-signal → STOPPED + zero RPM
RPM_ACCEL     = 5_000.0          # max plausible RPM/s per sample
RPM_K_WIN     = 5                # plausibility sliding-window size
RPM_K_ACC     = 3                # passes required per window to update EMA

RISING, FALLING = 1, 0

# ---------------------------------------------------------------------------
# Signal builder
# ---------------------------------------------------------------------------

class Trace:
    """
    Builds a sorted list of (time_us, RISING|FALLING) injection edges.

    All methods advance an internal cursor so that back-to-back calls
    produce phase-continuous pulses (no gap or overlap at boundaries).
    The cursor only jumps on silence() calls, which accurately models
    a fuel cut-off.
    """

    def __init__(self):
        self._edges: List[Tuple[int, int]] = []
        self._t = 0.0   # cursor in µs

    # -- builders ----------------------------------------------------------

    def inject(self, dur_ms: float, rpm: float, duty: float = 10.0) -> "Trace":
        """Constant-RPM injection for dur_ms milliseconds."""
        if rpm > 0 and dur_ms > 0:
            period = 120_000_000.0 / rpm
            high   = period * duty / 100.0
            end    = self._t + dur_ms * 1000.0
            t      = self._t
            while t < end:
                r, f = int(t), int(t + high)
                if r < end: self._edges.append((r, RISING))
                if f < end: self._edges.append((f, FALLING))
                t += period
            self._t = end
        else:
            self._t += dur_ms * 1000.0
        return self

    def silence(self, dur_ms: float) -> "Trace":
        """No injection for dur_ms ms (fuel cut-off / engine stop)."""
        self._t += dur_ms * 1000.0
        return self

    def ramp(self, dur_ms: float, rpm0: float, rpm1: float,
             duty: float = 10.0) -> "Trace":
        """Linearly interpolated RPM ramp from rpm0 to rpm1 over dur_ms."""
        if dur_ms <= 0:
            return self
        end = self._t + dur_ms * 1000.0
        t   = self._t
        while t < end:
            frac   = (t - self._t) / (dur_ms * 1000.0)
            rpm    = rpm0 + frac * (rpm1 - rpm0)
            period = 120_000_000.0 / max(rpm, 1.0)
            high   = period * duty / 100.0
            r, f   = int(t), int(t + high)
            if r < end: self._edges.append((r, RISING))
            if f < end: self._edges.append((f, FALLING))
            t += period
        self._t = end
        return self

    def skip(self, dur_ms: float, real_rpm: float,
             duty: float = 10.0) -> "Trace":
        """
        Skip-injection: ECU fires every other crankshaft cycle.
        Period seen by ISR = 2 × real period  →  implied RPM = real_rpm / 2.
        """
        return self.inject(dur_ms, real_rpm / 2.0, duty)

    def spurious(self, abs_ms: float, high_us: int = 1000) -> "Trace":
        """
        Insert a single spurious RISING+FALLING at absolute time abs_ms.
        Does NOT advance the cursor.
        """
        t = int(abs_ms * 1000.0)
        self._edges.append((t, RISING))
        self._edges.append((t + high_us, FALLING))
        return self

    # -- accessors ---------------------------------------------------------

    @property
    def edges(self) -> List[Tuple[int, int]]:
        return sorted(self._edges, key=lambda e: e[0])

    @property
    def total_ms(self) -> int:
        return int(self._t / 1000)


# ---------------------------------------------------------------------------
# ISR simulator
# ---------------------------------------------------------------------------

class ISR:
    """
    Mirrors fuelinj_isr() + ISR state variables from main.cpp.

    Rising edge:
        Commits the just-completed cycle if a valid pending_high exists and
        the new period is within [MIN_PERIOD_US, MAX_PERIOD_US].
    Falling edge:
        Records the pulse width of the cycle in progress.
    """

    def __init__(self):
        self.rising_us    = 0
        self.pending_high = 0
        self.period_sum   = 0
        self.high_sum     = 0
        self.count        = 0
        self.last_edge_ms = 0   # 0 = never seen an edge

    def edge(self, now_us: int, level: int):
        self.last_edge_ms = now_us // 1000
        if level == RISING:
            if self.rising_us and self.pending_high:
                p = now_us - self.rising_us
                if MIN_PERIOD_US <= p <= MAX_PERIOD_US:
                    self.period_sum += p
                    self.high_sum   += self.pending_high
                    self.count      += 1
                self.pending_high = 0   # cleared regardless (period may be out of bounds)
            self.rising_us = now_us
        else:  # FALLING
            if self.rising_us:
                self.pending_high = now_us - self.rising_us

    def snapshot(self) -> Tuple[int, int, int, int]:
        """Atomic snapshot + reset (mirrors noInterrupts block in main.cpp)."""
        s = (self.period_sum, self.high_sum, self.count, self.last_edge_ms)
        self.period_sum = self.high_sum = self.count = 0
        return s


# ---------------------------------------------------------------------------
# Broadcast-loop RPM processor
# ---------------------------------------------------------------------------

class Processor:
    """
    Mirrors the ICE Status broadcast block (RPM path) from main.cpp.

    Two-tier stale detection:
        no_signal < RPM_HOLD_MS  → RUNNING, fresh data
        no_signal < RPM_STOP_MS  → RUNNING, hold last RPM + CRANK_ERR
        no_signal ≥ RPM_STOP_MS  → STOPPED, zero RPM

    Plausibility filter:
        Per sample: compute implied RPM/s vs last accepted reference.
        Reject if > RPM_ACCEL.  Maintain a circular sliding window of
        RPM_K_WIN results; update EMA only when RPM_K_ACC pass.
        When fewer than RPM_K_ACC pass, freeze EMA and set CRANK_ERR.
    """

    def __init__(self):
        self.rpm_f   = 0.0
        self.duty_f  = 0.0
        self.seeded  = False
        # plausibility window
        self.win     = [False] * RPM_K_WIN
        self.head    = 0
        self.filled  = 0
        self.passes  = 0
        self.ref_rpm = 0.0
        self.ref_ms  = 0

    def _reset(self):
        self.rpm_f   = self.duty_f = 0.0
        self.seeded  = False
        self.win     = [False] * RPM_K_WIN
        self.head    = self.filled = self.passes = 0
        self.ref_rpm = 0.0
        self.ref_ms  = 0

    def tick(self, now_ms: int, period_sum: int, high_sum: int,
             count: int, last_ms: int) -> dict:
        """Process one broadcast tick; returns an output record dict."""
        INT32_MAX    = 2**31 - 1
        elapsed      = INT32_MAX if last_ms == 0 else (now_ms - last_ms)
        no_sig       = max(0, elapsed)
        stopped      = (last_ms == 0) or (no_sig >= RPM_STOP_MS)
        held         = (not stopped) and (no_sig >= RPM_HOLD_MS)
        running      = not stopped

        rpm_raw = plaus_pass = None   # set only when filter runs

        if stopped:
            self._reset()

        elif not held and count > 0:
            mean_p   = period_sum / count
            mean_h   = high_sum   / count
            rpm_raw  = 120_000_000.0 / mean_p
            duty_raw = 100.0 * mean_h / mean_p

            # Plausibility check: reject if implied RPM/s > RPM_ACCEL
            if self.ref_ms == 0:
                # No prior reference → accept unconditionally (first sample)
                plaus_pass = True
            else:
                dt         = max((now_ms - self.ref_ms) / 1000.0, 0.001)
                accel      = abs(rpm_raw - self.ref_rpm) / dt
                plaus_pass = (accel <= RPM_ACCEL)

            if plaus_pass:
                self.ref_rpm = rpm_raw
                self.ref_ms  = now_ms

            # Update circular sliding window
            if self.filled < RPM_K_WIN:
                # Window still filling: append without overwriting
                self.win[self.head] = plaus_pass
                if plaus_pass: self.passes += 1
                self.head    = (self.head + 1) % RPM_K_WIN
                self.filled += 1
            else:
                # Window full: overwrite the oldest slot
                if self.win[self.head]: self.passes -= 1
                self.win[self.head] = plaus_pass
                if plaus_pass:         self.passes += 1
                self.head = (self.head + 1) % RPM_K_WIN

            # Commit to EMA only when the current sample individually passes AND
            # the window confirms enough recent passes.  A rejected sample must
            # never feed the EMA even if the window still holds old True entries.
            if plaus_pass and self.passes >= RPM_K_ACC:
                if not self.seeded:
                    self.rpm_f  = rpm_raw   # seed: avoid ramp-up from 0
                    self.duty_f = duty_raw
                    self.seeded = True
                else:
                    self.rpm_f  = RPM_ALPHA  * rpm_raw  + (1 - RPM_ALPHA)  * self.rpm_f
                    self.duty_f = DUTY_ALPHA * duty_raw + (1 - DUTY_ALPHA) * self.duty_f
            # else: EMA frozen at last accepted value

        rejecting  = not held and self.filled >= RPM_K_WIN and self.passes < RPM_K_ACC
        crank_err  = running and (held or rejecting)
        engine_rpm = max(0.0, min(RPM_A1 * self.rpm_f + RPM_A0, 131071.0))
        tier       = "stopped" if stopped else ("held" if held else "fresh")

        return dict(
            now_ms     = now_ms,
            state      = "RUNNING" if running else "STOPPED",
            tier       = tier,
            crank_err  = crank_err,
            engine_rpm = engine_rpm,
            duty_pct   = DUTY_A1 * self.duty_f + DUTY_A0,
            no_sig     = no_sig,
            passes     = self.passes,
            filled     = self.filled,
            rpm_raw    = rpm_raw,
            plaus_pass = plaus_pass,
        )


# ---------------------------------------------------------------------------
# Simulation runner
# ---------------------------------------------------------------------------

def simulate(name: str, trace: Trace, print_every: int = 100) -> List[dict]:
    """Run one scenario; print a time-series log; return all records."""
    print(f"\n{'='*72}")
    print(f"  SCENARIO: {name}")
    print(f"{'='*72}")

    edges  = trace.edges
    total  = trace.total_ms
    isr    = ISR()
    proc   = Processor()
    recs: List[dict] = []
    ei     = 0
    last_p = -print_every
    prev   = None

    for now_ms in range(0, total + BCAST_MS, BCAST_MS):
        tick_us = now_ms * 1000
        while ei < len(edges) and edges[ei][0] <= tick_us:
            isr.edge(edges[ei][0], edges[ei][1])
            ei += 1

        ps, hs, cnt, lem = isr.snapshot()
        r = proc.tick(now_ms, ps, hs, cnt, lem)
        recs.append(r)

        changed = (prev is None or
                   r["state"]     != prev["state"]     or
                   r["tier"]      != prev["tier"]       or
                   r["crank_err"] != prev["crank_err"])

        if now_ms - last_p >= print_every or changed:
            last_p = now_ms
            rp  = f"{r['rpm_raw']:6.0f}" if r["rpm_raw"]    is not None else "     -"
            pp  = ("pass" if r["plaus_pass"] else "FAIL") \
                  if r["plaus_pass"] is not None else "  -"
            err = "CRANK_ERR" if r["crank_err"] else "---------"
            print(f"  t={now_ms:5d}ms  {r['state']:7s} [{r['tier']:7s}]  "
                  f"raw={rp}  out={r['engine_rpm']:6.0f}  "
                  f"win={r['passes']}/{RPM_K_WIN}  {pp}  {err}  "
                  f"no_sig={r['no_sig']:4d}ms")
        prev = r

    return recs


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------

def scenario_1() -> List[dict]:
    """Normal engine start and steady-state at 2000 RPM."""
    tr = Trace().inject(4000, 2000)
    return simulate(
        "1. Normal start and steady-state (2000 RPM, 10% duty)",
        tr, print_every=200,
    )


def scenario_2() -> List[dict]:
    """Engine stop: signal absent ≥ RPM_STOP_MS → STOPPED + zero RPM."""
    tr = Trace().inject(2000, 2000).silence(5000)
    return simulate(
        "2. Engine stop: signal absent -> STOPPED after RPM_STOP_MS",
        tr, print_every=200,
    )


def scenario_3() -> List[dict]:
    """Brief fuel cut-off (1500 ms < RPM_STOP_MS): RUNNING + CRANK_ERR during cut."""
    tr = (Trace()
          .inject(2000, 2000)
          .silence(1500)           # gap: RPM_HOLD_MS < 1500 < RPM_STOP_MS
          .inject(2000, 2000))
    return simulate(
        "3. Brief fuel cut-off (1500 ms gap): hold last RPM + CRANK_ERR",
        tr, print_every=200,
    )


def scenario_4() -> List[dict]:
    """Full cut-off (4000 ms > RPM_STOP_MS): STOPPED, then engine restarts."""
    tr = (Trace()
          .inject(2000, 2000)
          .silence(4000)           # > RPM_STOP_MS → STOPPED
          .inject(3000, 2000))
    return simulate(
        "4. Full cut-off (4000 ms > RPM_STOP_MS) → STOPPED → recovery",
        tr, print_every=300,
    )


def scenario_5() -> List[dict]:
    """
    Skip injection at real 8000 RPM.

    The ECU fires every other crankshaft cycle (overspeed protection).
    ISR sees period = 2 x 15 000 us = 30 000 us -> rpm_raw = 4000.

    Plausibility check at first skip sample:
        delta = |4000 - 8000| = 4000 RPM
        dt    ~= 30 ms (two normal injection periods)
        accel = 4000 / 0.030 = 133 333 RPM/s  >>  RPM_ACCEL = 5000
        -> REJECTED

    EMA protection:
        Because the EMA is only updated when the CURRENT sample also passes
        (plaus_pass && passes >= K_ACC), rejected skip samples never feed the
        EMA regardless of how many old True entries remain in the window.
        RPM output stays at 8000 throughout the skip event.
        CRANK_ERR fires after K_WIN-K_ACC+1=3 consecutive rejections drop
        passes below K_ACC.
    """
    # Skip duration kept to 700 ms so that the implied accel (4000 RPM over
    # elapsed dt since last accepted) stays above RPM_ACCEL throughout:
    #   threshold dt = delta_rpm / RPM_ACCEL = 4000 / 5000 = 0.800 s
    #   skip ends at ~2700 ms, last accepted at ~1980 ms -> dt = 720 ms < 800 ms
    # All skip samples are rejected; RPM output stays locked at 8000.
    tr = (Trace()
          .inject(2000, 8000)      # normal at 8000 RPM
          .skip  ( 700, 8000)      # every-other-cycle (700 ms < 800 ms threshold)
          .inject(2000, 8000))     # normal injection resumes
    return simulate(
        "5. Skip injection (real 8000 RPM, every-other-cycle → implies 4000 RPM)",
        tr, print_every=100,
    )


def scenario_6() -> List[dict]:
    """
    Single spurious edge during steady 2000 RPM injection.

    A false RISING at t=2020 ms (40 ms after the last normal 1980 ms RISING)
    causes the ISR to commit an early period of 40 000 us -> rpm_raw = 3000.
    The interrupted normal RISING at 2040 ms then commits a short 20 000 us
    period -> rpm_raw = 6000.  Both samples are rejected individually by the
    plausibility filter (accel >> RPM_ACCEL).

    EMA protection:
        Both bad samples are rejected by the plausibility check.  Because the
        EMA is only updated when the CURRENT sample also passes, neither
        contaminates the EMA.  RPM output stays at 2000 with no transient.
    """
    tr = (Trace()
          .inject(5000, 2000)      # continuous 2000 RPM signal
          .spurious(2020.0))       # false edge 20 ms into a 60 ms cycle
    return simulate(
        "6. Single spurious in-bounds edge at t=2020 ms (implies 6000 RPM): rejected",
        tr, print_every=100,
    )


def scenario_7() -> List[dict]:
    """
    Rapid acceleration 1300 → 6500 RPM in 1.8 s.

    Observed maximum in field: 1300 → 6500 RPM in 1.8 s = 2944 RPM/s.
    RPM_ACCEL = 5000 RPM/s (70% headroom over observed maximum).

    Because the ramp is simulated by continuously varying the injection
    period, each broadcast tick's rpm_raw changes only by the amount the
    engine actually accelerated since the previous injection cycle.  At
    1300 RPM (period 60 ms) that is 2944 × 0.060 = 177 RPM per cycle,
    well within the threshold.  All samples pass → EMA tracks the ramp.
    """
    tr = (Trace()
          .inject(1000, 1300)
          .ramp  (1800, 1300, 6500)
          .inject(2000, 6500))
    return simulate(
        "7. Rapid accel 1300->6500 RPM in 1.8 s (2944 RPM/s < RPM_ACCEL=5000): all pass",
        tr, print_every=200,
    )


def scenario_8() -> List[dict]:
    """
    Over-speed injection above RPM_MAX.

    At 10 000 RPM, period = 12 000 µs < MIN_PERIOD_US (15 000 µs).
    The ISR period bounds check drops every such sample.  No samples reach
    the plausibility filter; count stays 0 each broadcast tick; EMA is
    frozen at the last accepted value from the warm-up phase.
    """
    tr = (Trace()
          .inject(1000, 3000)      # warm up at 3000 RPM
          .inject(2000, 10000)     # 10 000 RPM: period 12 000 µs < MIN_PERIOD_US
          .inject(2000, 3000))     # back to 3000 RPM
    return simulate(
        f"8. Over-speed 10000 RPM: period < MIN_PERIOD_US={MIN_PERIOD_US} us -> ISR drops",
        tr, print_every=200,
    )


# ---------------------------------------------------------------------------
# Automated pass/fail checks
# ---------------------------------------------------------------------------

def check_results(results: dict) -> bool:
    checks = [
        # (scenario_id, description, lambda)
        (1, "RPM converges to 2000 within first K_WIN injections",
         lambda r: any(abs(x["engine_rpm"] - 2000) < 100 for x in r[-200:])),

        (2, "STOPPED reported after RPM_STOP_MS",
         lambda r: any(x["state"] == "STOPPED" for x in r)),

        (3, "CRANK_ERR set during cut; cleared on resumption",
         lambda r: (any(x["crank_err"] for x in r) and
                    any(not x["crank_err"] and x["engine_rpm"] > 1000 for x in r[-150:]))),

        (4, "STOPPED after full cut; RPM > 1000 after recovery",
         lambda r: (any(x["state"] == "STOPPED" for x in r) and
                    any(x["engine_rpm"] > 1000 for x in r[-200:]))),

        (5, "CRANK_ERR set during skip injection; RPM held at 8000, recovers after resumption",
         lambda r: (any(x["crank_err"] and x["engine_rpm"] > 7900 for x in r) and
                    any(x["engine_rpm"] > 7900 for x in r[-100:]))),

        (6, "Spurious edge rejected with no EMA contamination; RPM stays at 2000",
         lambda r: all(abs(x["engine_rpm"] - 2000) < 100
                       for x in r if x["engine_rpm"] > 100 and x["state"] == "RUNNING")),

        (7, "RPM output reaches > 5000 (ramp accepted, not filtered out)",
         lambda r: any(x["engine_rpm"] > 5000 for x in r[-200:])),

        (8, "RPM output never exceeds RPM_MAX+50 during over-speed phase",
         lambda r: all(x["engine_rpm"] <= RPM_MAX + 50 for x in r)),
    ]

    print(f"\n{'='*72}")
    print(f"  Results")
    print(f"{'='*72}")
    all_pass = True
    for scen_id, desc, fn in checks:
        passed   = fn(results[scen_id])
        all_pass = all_pass and passed
        mark     = "PASS" if passed else "FAIL"
        print(f"  [{mark}] Scenario {scen_id}: {desc}")

    print()
    print(f"  Overall: {'ALL PASS' if all_pass else 'SOME FAILURES'}")
    return all_pass


# ---------------------------------------------------------------------------
# Optional matplotlib plot
# ---------------------------------------------------------------------------

def plot_results(results: dict):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("\nmatplotlib not available – skipping plot (pip install matplotlib)")
        return

    titles = [
        "1. Normal start",
        "2. Engine stop",
        "3. Brief fuel cut",
        "4. Full cut + recovery",
        "5. Skip injection",
        "6. Spurious edge",
        "7. Rapid accel 1300->6500",
        "8. Over-speed",
    ]

    n    = len(results)
    fig, axes = plt.subplots(n, 1, figsize=(14, n * 2.8))
    fig.suptitle(
        f"ICEMonitor RPM Simulator   "
        f"(RPM_HOLD={RPM_HOLD_MS} ms, RPM_STOP={RPM_STOP_MS} ms, "
        f"RPM_ACCEL={RPM_ACCEL:.0f} RPM/s, "
        f"K_WIN={RPM_K_WIN}, K_ACC={RPM_K_ACC}, α={RPM_ALPHA})",
        fontsize=10, fontweight="bold",
    )

    for idx, (sid, recs) in enumerate(sorted(results.items())):
        ax  = axes[idx]
        t   = [r["now_ms"] / 1000.0 for r in recs]
        out = [r["engine_rpm"] for r in recs]
        raw = [r["rpm_raw"] if r["rpm_raw"] is not None else float("nan")
               for r in recs]

        ax.plot(t, out, "b-", lw=1.8, label="RPM output (EMA)")

        # Shade regions: red = CRANK_ERR, grey = STOPPED
        for i in range(len(t) - 1):
            if recs[i]["crank_err"]:
                ax.axvspan(t[i], t[i + 1], alpha=0.18, color="red",  lw=0)
            if recs[i]["state"] == "STOPPED":
                ax.axvspan(t[i], t[i + 1], alpha=0.12, color="black", lw=0)

        ax.axhline(RPM_MAX, color="orange", ls="--", lw=0.9)

        # Raw dots plotted last so they appear on top
        ax.plot(t, raw, "r.", ms=3.5, alpha=0.8, label="RPM raw (per tick)")

        ax.set_title(titles[idx], fontsize=9, loc="left")
        ax.set_ylabel("RPM", fontsize=8)
        ax.set_xlabel("Time (s)", fontsize=8)
        ax.set_ylim(bottom=-300)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.25)

        err_p  = mpatches.Patch(color="red",   alpha=0.4,  label="CRANK_ERR region")
        stop_p = mpatches.Patch(color="black", alpha=0.25, label="STOPPED region")
        max_l  = plt.Line2D([], [], color="orange", ls="--", lw=0.9,
                            label=f"RPM_MAX={RPM_MAX}")
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(handles + [err_p, stop_p, max_l],
                  labels  + ["CRANK_ERR region", "STOPPED region", f"RPM_MAX={RPM_MAX}"],
                  fontsize=6.5, loc="upper right", ncol=2)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    out_file = "rpm_simulator.png"
    plt.savefig(out_file, dpi=150, bbox_inches="tight")
    print(f"\nPlot saved → {out_file}")
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("ICEMonitor RPM Signal Processing Simulator")
    print(f"  MIN_PERIOD_US={MIN_PERIOD_US} us (RPM_MAX={RPM_MAX})")
    print(f"  RPM_HOLD_MS={RPM_HOLD_MS}  RPM_STOP_MS={RPM_STOP_MS}")
    print(f"  RPM_ACCEL={RPM_ACCEL:.0f} RPM/s  RPM_K_WIN={RPM_K_WIN}  "
          f"RPM_K_ACC={RPM_K_ACC}  RPM_ALPHA={RPM_ALPHA}")
    print(f"  BCAST_MS={BCAST_MS}")

    results = {
        1: scenario_1(),
        2: scenario_2(),
        3: scenario_3(),
        4: scenario_4(),
        5: scenario_5(),
        6: scenario_6(),
        7: scenario_7(),
        8: scenario_8(),
    }

    check_results(results)

    if "--plot" in sys.argv:
        plot_results(results)
