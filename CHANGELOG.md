# orcp_ros2 — Changelog

---

## 0.2.0 — 2026-08-27

### ⚠️ Requires `orcp>=0.2.0` — a correctness floor, not a preference

Declared in `setup.py`, `package.xml`, the `Dockerfile` and the README, because
a pin in only one of them is not a pin.

`orcp` 0.1.x used `serial.readline()`, which returns a **partial line** when its
timeout expires — so a telemetry push split mid-line and its tail was handed to a
waiting command as that command's response. **This node streams continuously and
polls `STATUS` on a timer, so it runs permanently in the exact condition that
triggers it**, and the failure gets worse with latency: worst over WiFi, on a
real robot.

Previously the package declared no dependency on `orcp` at all, so an install
could silently pick up a broken client.

### Added

* **`~/coast`** (`std_srvs/Trigger`) — stop by coasting to rest, after which the
  controller parks itself. A controller without the extension degrades to a
  brake, which is safe.
* **`~/hold`** (`std_srvs/Trigger`) — stop and actively hold position.
  A refused hold returns `success=False` with `"stopped, but NOT holding:
  <reason>"`. ⚠️ **The wording is deliberate: the STOP happened.** Only the hold
  was refused, and a message implying the robot is still moving would be worse
  than useless.

  ⚠️ **Position hold is a convenience, not a safety function.** It needs power, a
  live controller and working encoders, and releases on any power-stage fault.
  Do not use it as a parking brake on a gradient.

* **`/diagnostics` reports `hold` and `coast`**, and raises **ERROR** with
  `"position hold RELEASED (fault or timeout)"` when `hold=2`. The controller
  *was* holding position and is not any more, so on a gradient the robot may now
  be moving. ERROR rather than WARN because a released hold is not a degraded
  state — it is the absence of the thing being relied on.

  ⚠️ Both fields appear **only when the controller reports them**. Publishing
  `hold: 0` for a controller that cannot hold would read as "not holding right
  now", which is a different claim.

* `dropped_lines` surfaced in diagnostics when non-zero, so a link quietly
  delivering garbage is visible before it becomes a mystery.
