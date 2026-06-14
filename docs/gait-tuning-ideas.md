# Gait Tuning Ideas

Notes on potential improvements to explore for the tripod gait. Motivated by an observed artifact on hardware (not visible in Webots): the front legs appear to slide rapidly inward right before liftoff.

## Background — Why it happens on the front legs

Stance moves the foot in a straight line along the body-frame X axis. The front legs are mounted at ±30° from forward, so body-X translation is *not* aligned with the leg's natural plane — it has both a "leg-forward" and a "leg-inward" component. As stance progresses, hip-local `legX` shrinks (foot tucks closer to the hip).

For FRONT_RIGHT walking forward with stride=0.04, standingReach=0.091:
- Start of stance: `legX ≈ 0.108`, `legY ≈ +0.01`
- End of stance:   `legX ≈ 0.074`, `legY ≈ -0.01` — ~30% reduction in outward distance

Middle legs don't see this — their hip points exactly outward, so body-X stride moves only along `legY`.

COXA angle is `atan2(legY, legX)`. The sensitivity of `atan2` grows as `legX` shrinks, so small Cartesian motion produces large angular changes when the foot is tucked. At gaitSpeed=1:
- Start of stance: COXA moves at ~17.6 °/s
- End of stance:   COXA moves at ~37.7 °/s (more than 2× faster)

Webots tracks this perfectly. Real servos with backlash, calibration error, and ground friction do not — deviation manifests right where the angular rate peaks, which is right before liftoff on the front legs.

## Ideas to explore

### 1. Lift the foot earlier in the cycle (raise swing duty)
Increase swing duty above 50% (e.g., 0.55) so liftoff happens before the COXA angular peak. Currently hardcoded to 0.5 in `Hexapod::applyGaitToLeg` ([Hexapod.cpp:329](../lib/HexapodCore/src/Hexapod.cpp#L329)). Tradeoff: shorter stance duty means each leg has less time on the ground, which can reduce stability margin (especially at higher gait speeds where the swinging tripod is moving fast).

### 2. Non-linear stance profile (ease-out at the end of stance)
Replace the linear `strideProgress` ramp during stance with an ease-out curve so the foot decelerates as it approaches the tucked configuration. The current profile in `GaitMath::calculatePhaseProgress` is purely linear in phase. Could try:
- Cosine ease: `strideProgress = 0.5 * cos(stanceProgress * π)` for a smooth in-and-out.
- Asymmetric ease: linear early, cubic ease-out late.

Watch out for: changing the stance velocity profile means the body's forward velocity is no longer constant during the cycle, which affects the swinging tripod's ground-relative trajectory.

### 3. Reduce stride at higher gait speeds
The geometric effect scales with stride length — smaller strides keep the foot in the "linear-ish" zone of the leg's workspace. Could add a stride/speed scaling rule (e.g., max stride decreases as gaitSpeed increases).

### 4. Calibrate front-leg COXAs first
The most-tucked configuration amplifies COXA/FEMUR offset error. A 2° COXA offset matters much more at end-of-stance than mid-stance. Worth running a careful calibration pass on the front legs specifically and verifying with a slow walk on a flat surface.

### 5. Mechanical / friction
- Rubber feet to reduce sliding when load shifts to the other tripod.
- Stiffer servos (or higher-quality digital servos) to reduce backlash snap when load reverses at the swing transition.

These are symptom treatments — the geometric COXA acceleration is still there — but they help mask it.

### 6. (Speculative) Plan the foot path in leg-local frame instead of body frame
Today the stride is a straight line in body X, then transformed into leg-local coordinates. An alternative: define the stride directly in each leg's local frame so each leg's foot path is symmetric in its own workspace. This would eliminate the per-leg geometric asymmetry but changes the meaning of "heading" and "turn rate" — needs a different formulation of `calculateFootDelta`. Significant rewrite; only worth it if the simpler tweaks above don't work.

### 7. Shape the swing trajectory for clean ground clearance (implemented)
Motivated by a *different* artifact than the front-leg slide above: on short carpet the swinging foot grazes the pile tops near the end of swing while still translating forward. That forward drag is a horizontal reaction on the body, and the only thing resisting it is friction on the stance tripod — so the stance legs slip.

The lever that matters is the **horizontal foot velocity at the moment of surface contact**. The original swing profile in `GaitMath::calculatePhaseProgress` was *linear* horizontal (constant forward velocity, including at touchdown) plus a *symmetric* sine lift. Reshaped to:
- **Horizontal: cosine ease-in-out** (`-0.5 + 0.5(1 - cos(s·π))`). Forward velocity is ~0 at both liftoff and touchdown; the foot does its forward travel mid-swing while high, then arrives moving almost purely downward. Endpoints (±0.5) are preserved so it still meshes with the linear stance ramp. This is the part that actually kills the scuffing drag.
- **Vertical: trapezoidal "flat-top" arc.** Quarter-sine rise, a flat hold at peak across the middle `SWING_PLATEAU` fraction of swing, then a quarter-cosine fall. Steep at *both* liftoff and touchdown for good clearance, with the forward translation happening while the foot is held high.

> **History / why not an asymmetric apex:** an earlier version used `SWING_APEX` — peak placed late in swing (e.g. 0.7) to get a steep descent. But that's a *trade*, not a win: a late apex steepens the descent only by making the *rise* shallow, so the foot crawls out of the pile at liftoff and can scuff on the way up just as it did on the way down. The flat-top profile gets steep clearance at both ends with no such bias. The asymmetric apex was removed; it can be reintroduced if a *mild* late bias is ever wanted (defensible because a touchdown scuff brakes the leg right as it takes load, whereas a liftoff scuff is on an unloading leg).

Tunables: `SWING_PLATEAU` in [GaitMath.h](../lib/HexapodCore/include/GaitMath.h) — fraction of swing held at peak lift, in `[0, 1)`. Larger → steeper ramps + more time at full clearance; `0` reduces to a symmetric sine. Covered by `TestSwingTrajectoryShape` in [tests/test_gait_math.cpp](../tests/test_gait_math.cpp).

Tradeoffs: a large plateau means steeper (harder-impact) ramps; ease it down if the plant gets jarring. There's an intentional horizontal-velocity discontinuity at the swing→stance handoff — the foot plants at ~zero body-relative velocity, which is desirable. A simpler complementary lever is raising `LIFT_HEIGHT` so the foot clears the pile with more margin everywhere; try that first, as it may make the trajectory shaping unnecessary.

## How to evaluate

- Slow walk on a flat surface, side view video, watch a single front leg through stance → liftoff.
- Compare the same gait/speed/stride on hardware vs. Webots to confirm the artifact disappears in sim.
- If experimenting with swing duty or stance profile, sweep one variable at a time and look at: (a) does the slide visibly reduce, (b) does forward velocity stay reasonable, (c) any new instability on the swinging side.
