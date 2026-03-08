# Turret Safe Testing Checklist

**Date:** 2026-03-07
**Prerequisites:** Deploy latest code. Have e-stop ready at all times.

## Test Mode Button Map

| Button | Action |
|--------|--------|
| A (hold) | Apply +1V open-loop (test direction) - stops on release |
| B (hold) | Apply -1V open-loop (test direction) - stops on release |
| X (press) | Nudge +10 degrees from current position (closed-loop) |
| Y (press) | Nudge -10 degrees from current position (closed-loop) |
| LB (press) | Emergency stop |

## Dashboard Values to Watch

All under the `Turret/` prefix in SmartDashboard/Shuffleboard:

- **Position Degrees** - fused motor position in degrees
- **CANCoder Degrees** - absolute CANCoder position in degrees
- **CANCoder Raw Rotations** - raw CANCoder value (useful for debugging offset)
- **Motor Raw Rotations** - raw motor position value
- **Motor Stator Current** - current draw in Amps (should stay under 20A)
- **Motor Voltage** - voltage being applied to motor
- **Target Degrees** - where the turret is trying to go
- **At Position** - true when turret is within 1 degree of target

---

## Step 1: Verify Sensor Readings (no motor power)

1. Deploy code and enter **Test Mode** in Driver Station
2. **Do NOT press any buttons yet**
3. Open SmartDashboard/Shuffleboard and find the `Turret/` values
4. **Spin the turret slowly by hand** and observe:
   - [ ] `CANCoder Degrees` reads approximately 0 at mechanical zero
   - [ ] `CANCoder Raw Rotations` reads approximately 0.0 at mechanical zero
   - [ ] `Position Degrees` matches `CANCoder Degrees`
   - [ ] Turning one direction makes numbers increase, the other decreases
   - [ ] Values do NOT jump or wrap unexpectedly near zero
5. **Write down which physical direction is "positive"** (increasing numbers):
   - Positive direction: Counter-Clock-wise

## Step 2: Verify Motor Direction (open-loop, 1V only)

**This is the critical safety check. Have e-stop ready.**

1. **Hold A button** - motor gets +1V
   - [ ] Note which direction the turret physically spins: Clock-wise
   - [ ] Release A - motor stops
2. **Compare to Step 1:** Did the motor spin in the SAME direction that makes CANCoder INCREASE?
   - [ ] **YES** - Direction is correct. Proceed to Step 3.
   - [ ] **NO** - **This is the bug.** The motor fights itself. STOP HERE and fix:
     - In `Turret.java` line 122, flip `InvertedValue.CounterClockwise_Positive` to `InvertedValue.Clockwise_Positive` (or vice versa)
     - Redeploy and repeat Step 2
3. **Hold B button** to confirm the opposite direction works correctly
   - [ ] Motor spins the opposite way and stops on release

## Step 3: Test Closed-Loop Nudge (small movements)

**Only proceed if Step 2 confirmed correct direction.**

1. Have someone on e-stop
2. Press **X** once - turret should slowly move +10 degrees and hold
   - [ ] Turret moved in the correct direction
   - [ ] Movement was slow and controlled
   - [ ] `Motor Stator Current` stayed well under 20A
   - [ ] `Position Degrees` is approximately 10 degrees more than before
3. Press **Y** once - turret should slowly move -10 degrees back
   - [ ] Turret returned toward original position
   - [ ] Movement was slow and controlled
4. Try pressing X and Y a few more times
   - [ ] Turret consistently moves the correct direction
   - [ ] No oscillation or vibration when holding position

## Step 4: Results and Next Steps

**If everything passed:**
- The turret is working correctly with safe settings
- Gradually increase constants in future sessions:
  - Cruise velocity: 0.1 -> 0.25 -> 0.5 -> 1.0 rot/s
  - P gain: 10 -> 25 -> 50
  - Current limits: 20A -> 40A -> 60A -> 80A
- Test at each stage before increasing further

**If Step 2 failed (direction mismatch):**
- This was the root cause of the turret spinning out of control
- The motor was fighting the sensor in a positive feedback loop
- After flipping the inversion and confirming Step 2 passes, continue with Steps 3-4

**Notes from testing:**

_____________________________________________

_____________________________________________

_____________________________________________
