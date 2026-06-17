# FLOAT-2026 Requirements & Checklist

Target task: **2026 MATE ROV — "MATE Floats! Under the Ice"** (vertical profiling float, non-ROV device, 70 pts).

This is a planning/tracking document only. It does not change any code.

Scope split:
- **A. Float firmware** — the program that runs on the Pico (one `setup()`/`loop()`).
- **B. Receiver + graph tool** — separate software on a shore computer.
- **C. Hardware + documentation** — physical build and required paperwork (not code).

---

## A. Float firmware (on-board)

### A1. Mission sequence (buoyancy engine only — no thrusters)
- [ ] Wait for a `START` command over the radio before doing anything.
- [ ] **Descend to 2.5 m (±33 cm)** by changing buoyancy (pump fluid/air), then hold.
- [ ] **Hold 30 s, counting time only while inside the ±33 cm band** — reset the 30 s clock the instant depth leaves the band.
- [ ] **Ascend to 0.40 m (±33 cm)** and hold the same way.
- [ ] **Do NOT break the surface or touch the ice** during the ascent hold (−5 penalty). Needs an ascent stop/floor guard.
- [ ] Repeat for **two full profiles** (descend → hold → ascend → hold, ×2).

### A2. Sensing & control
- [ ] Read the MS5837; produce **depth (m)** and **pressure (kPa)** (convert from the library's mbar default).
- [ ] **Zero/tare pressure to atmospheric at the surface on startup** so depth = 0 at the surface.
- [ ] **Apply the sensor mounting offset** so reported depth matches the rules' reference point (bottom of float at 2.5 m, top at 40 cm). Record the offset to tell the judge.
- [ ] PID holds depth tightly inside ±33 cm with **integral anti-windup** and output limits (no band/surface overshoot).
- [ ] Tune PID gains in water (current `{1,1,1}` are placeholders).

### A3. Data logging
- [ ] Capture depth+time at least every 5 s during holds (need points at 0,5,10,15,20,25,30 s = **7 per hold**).
- [ ] Capture **≥20 packets total** across the mission for the graph.
- [ ] Store packets in a **bounded buffer** (not an ever-growing `String` — heap-exhaustion risk on the Pico).

### A4. Packet format
- [ ] Each packet contains **company number + time (formatted, e.g. HH:MM:SS) + pressure (kPa) and/or depth (m), with units**.
  - Example target: `EX01 1:51:42 9.8 kPa 1.00 m`

### A5. Transmit (after recovery)
- [ ] Transmit packets **discretely** (one at a time, delimited), each **tagged with the company number** so the receiver can filter only this float while other teams transmit simultaneously.
- [ ] Replace the `DONE`-state blast loop that resends the whole blob on every received byte.

### A6. Safety / robustness
- [ ] On sensor-read failure or inability to hold: **surface and stop the pump** rather than hang.

### A7. Firmware structure (build blockers to clear)
- [ ] **One** program with a single `setup()`/`loop()` (mission currently lives in uncalled `setupSensor()`/`loopSensor()`).
- [ ] Remove `#include "PID.cpp"`; make a proper `PID.h` / `PID.cpp` exposing only `PID()` (no second `setup`/`loop`, no duplicate `sensor`).
- [ ] Remove/replace duplicate `setup()`/`loop()` in `transmitter.cpp` and the empty `jsonEncode.cc` / missing `jsonEncode.h`.
- [ ] Fix pin conflicts: `LED 2` vs `pumpPin2 2`; `pumpPin1 1` vs HC-12 `RX_PIN 1` / `TX_PIN 0`.
- [ ] Decide what to do with `TBP/main.cpp` (parallel naive version; uses `==` float equality that never triggers) — keep as reference, exclude from build.

---

## B. Receiver + graph tool (shore computer — separate program)

- [ ] Receive packets over the radio link.
- [ ] **Filter to only this float's company number** (other teams transmit at the same time).
- [ ] Capture/store received packets.
- [ ] **Graph depth (Y) vs time (X)** on a computer, ≥20 packets (no hand-drawing).
- [ ] Fallback noted: MATE-provided data can be used, but then the "communicate all packets" and "graph your own data" points are forfeited.

---

## C. Hardware + documentation (not code)

### C1. Physical
- [ ] Buoyancy engine (changes float density by moving fluid/air) — thrusters explicitly don't count.
- [ ] All air stored onboard — no tether, airline, or rope to surface/bottom.
- [ ] **No cameras** (ELEC-NRD-002).
- [ ] Size: **< 1 m tall** (incl. antenna), **< 18 cm diameter/width**.
- [ ] **Recovery feature** the ROV can grab: ≥5 cm wide, protruding ≥5 cm (e.g. #310-or-larger U-bolt / rope loop). May exceed the 18 cm limit.
- [ ] **Pressure relief** (ELEC-NRD-006): ≥2.5 cm friction-fit hole OR pop-off end cap (≥2.5 cm sealing dia). No fasteners/valves/clamped caps holding the housing closed.

### C2. Power / electrical
- [ ] Onboard batteries only, **≤12 VDC, ≤5 A**.
- [ ] **NiMH or AGM only** (no alkaline, LiPo, or standard 12 V outdoor rechargeables).
- [ ] **Single fuse that kills all power**, within 5 cm of the battery positive terminal, visible through a clear housing.
- [ ] Fuse sizing from battery mAh (mAh ÷ 1000 = max A → nearest standard fuse). Max by type: AA 2.0 A, C/D 5.0 A, 9V 200 mA, 12V brick 5.0 A.
- [ ] Fuse type: ATO/MINI blade (1–5 A, color-coded, ≥32 VDC) or stamped cartridge.

### C3. Documentation (submit before competition)
- [ ] **DOC-004 non-ROV device document** (max 2 pages): photo/diagram, battery type, battery-pack photos, fuse photo, full-load-amps in **waiting mode** and **buoyancy-change mode**, plus buoyancy-engine, comms, and battery-pack descriptions.
- [ ] **1-page SID** (system integration diagram) with a standard fuse symbol and the FLA measurements.

---

## Scoring reference (70 pts)
- Per profile: descend & hold 2.5 m (5) + ascend & hold 40 cm (5) + completed via buoyancy engine (10) = 20 pts × 2 profiles.
- Surface/ice contact on ascent hold: **−5**.
- Transmit data packets to shore receiver: up to **10**.
- Graph depth vs time (≥20 packets, on computer): **10**.
- "Holding depth" must be provable: **7 sequential in-band packets** at 0/5/10/15/20/25/30 s; drifting out of band resets the 30 s clock.
