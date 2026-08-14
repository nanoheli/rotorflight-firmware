# CH-47 Chinook (Tandem Rotor) Mixer

This document explains how to configure Rotorflight's generic mixer to fly a CH-47 Chinook
style **tandem rotor** helicopter with:

- 2x DShot ESCs / motors (front rotor = `M1`, rear rotor = `M2`)
- 2x 120° CCPM swashplates (3 servos each, 6 servos total)
- All 6 servos wired to the FC's **SBUS output** (serial bus servos), not PWM pins

It also shows the CLI commands needed to enable the **dual-motor governor**, so both rotors
are held at the exact same headspeed.

## 1. How the Rotorflight mixer engine works

Rotorflight does not use fixed "mixer presets" (TRI/QUAD/HELI120/...) like old Cleanflight.
Instead it has a small generic mixing engine:

- **Inputs** – named signals such as stabilized roll/pitch/yaw/collective/throttle
  (`SR`, `SP`, `SY`, `SC`, `ST`, `ST2`, ...), RC channels, etc. Each input has a `rate`
  (scale) and `min`/`max` limit, configurable with `mixer rate` / `mixer limit`.
- **Outputs** – servo channels `S1`...`S26` and motor channels `M1`...`M4`.
- **Rules** – up to 32 entries of the form
  `mixer rule <index> <SET|ADD|MUL> <input> <output> <weight> <offset>`,
  each rule multiplies one input by a weight (and adds an offset), then
  `SET`s, `ADD`s or `MUL`s it into one output. Multiple rules can accumulate onto the
  same output (first rule `set`, following rules `add`), which is exactly what lets us
  build two independent swashplates from simple linear terms.

For a normal single-rotor helicopter, `swash_type` auto-generates the rules for you
(collective + cyclic roll/pitch onto 3-4 servos, throttle onto `M1`). A tandem rotor
aircraft has **two** swashplates that must move in specific combinations for each control
axis, so we turn the automatic swash mixing **off** (`swash_type = NONE`) and build the
whole mixer by hand with `mixer rule`.

## 2. CH-47 control theory

A tandem rotor helicopter has no tail rotor. Both rotors are collective+cyclic (120° CCPM)
and counter-rotate to cancel torque. The 4 flight axes map onto the two swashplates like
this (the same scheme used on real Chinooks and most RC tandem-rotor models):

| Stick        | Effect                                                                 |
| ------------ | ----------------------------------------------------------------------- |
| Collective   | Same collective pitch on **both** rotors → climb/descend                |
| Roll         | Same lateral cyclic tilt on **both** rotors (front and rear tilt the same way) → roll, just like a single-rotor helicopter |
| Pitch        | **Differential collective**: front rotor collective increases while rear decreases (or vice-versa) → pitch moment, because the rotors are separated fore/aft |
| Yaw          | **Differential lateral cyclic**: front and rear tilt sideways in *opposite* directions → yaw moment, again because of the fore/aft separation |

So each swashplate servo receives:

```
servo = collective_common ± pitch_differential   (uniform/"collective" component)
      ± roll_common ± yaw_differential            (tilt component, only on 2 of the 3 servos)
```

with the sign of the differential terms flipped between the front and rear swashplate.

This is mechanically identical to a standard 120° CCPM swash (same `0.5` / `0.866` factors
used by the built-in `swash_type = CP120`), we are simply feeding the *pitch stick* into the
"collective" term and the *yaw stick* into the "roll/tilt" term of the rear swash with the
opposite sign.

## 3. Servo layout used below

Each swashplate uses the same servo roles as a normal 120° CCPM setup:

- **Servo A** – arm on the roll axis (no roll/yaw authority, only collective/pitch)
- **Servo B** – arm at +120° (positive roll & yaw contribution)
- **Servo C** – arm at −120° (negative roll & yaw contribution)

Since all 6 servos are driven over **SBUS**, they must use servo slots `S9`-`S26`
(slots `S1`-`S8` are PWM-pin only, see [`Mixer.md`](Mixer.md) / [`Serial.md`](Serial.md)).
This example uses `S9`-`S11` for the front swash and `S12`-`S14` for the rear swash,
which correspond to SBUS-output channels 0-5:

| Swashplate | Servo A | Servo B | Servo C |
| ---------- | ------- | ------- | ------- |
| Front      | S9      | S10     | S11     |
| Rear       | S12     | S13     | S14     |

If you are instead driving some servos from PWM pins, just replace the servo names with
`S1`-`S8` accordingly - the `mixer rule` values themselves don't change.

## 4. CLI configuration

### 4.1 Enable SBUS servo output

```
serial <UART_id> 262144 115200 115200 115200 115200
set sbus_out_frame_rate = 100
```

- `262144` is the `SBUS_OUT` serial function bit - assign it to whichever UART your bus
  servos are wired to (see `serial` in [`Cli.md`](Cli.md) for the full syntax).
- `bus_servo_source_type` selects, per SBUS channel (0-17), whether that channel is driven
  by the mixer (`0`) or passed through from the RX (`1`). Channels 0-5 (`S9`-`S14`) default
  to mixer-driven already, so no change is needed for this setup.

### 4.2 Turn off the built-in single-swash mixer

```
set swash_type = NONE
set tail_rotor_mode = VARIABLE
```

(`tail_rotor_mode` is irrelevant for a Chinook since there's no tail rotor, but leaving it
at its default keeps the built-in tail logic - which we don't use - fully disabled.)

### 4.3 Reset and build the custom mixer rules

```
mixer rule reset
```

**Front swashplate (`M1`'s rotor) - servos S9/S10/S11:**

```
mixer rule  0 set SC S9   500    0   # Servo A: collective
mixer rule  1 add SP S9   500    0   # Servo A: + differential pitch (front)

mixer rule  2 set SC S10  500    0   # Servo B: collective
mixer rule  3 add SP S10  500    0   # Servo B: + differential pitch (front)
mixer rule  4 add SR S10  866    0   # Servo B: + roll (common)
mixer rule  5 add SY S10  866    0   # Servo B: + differential yaw (front)

mixer rule  6 set SC S11  500    0   # Servo C: collective
mixer rule  7 add SP S11  500    0   # Servo C: + differential pitch (front)
mixer rule  8 add SR S11 -866    0   # Servo C: - roll (common)
mixer rule  9 add SY S11 -866    0   # Servo C: - differential yaw (front)
```

**Rear swashplate (`M2`'s rotor) - servos S12/S13/S14 - same weights, pitch and yaw signs flipped:**

```
mixer rule 10 set SC S12  500    0   # Servo A: collective
mixer rule 11 add SP S12 -500    0   # Servo A: - differential pitch (rear)

mixer rule 12 set SC S13  500    0   # Servo B: collective
mixer rule 13 add SP S13 -500    0   # Servo B: - differential pitch (rear)
mixer rule 14 add SR S13  866    0   # Servo B: + roll (common)
mixer rule 15 add SY S13 -866    0   # Servo B: - differential yaw (rear)

mixer rule 16 set SC S14  500    0   # Servo C: collective
mixer rule 17 add SP S14 -500    0   # Servo C: - differential pitch (rear)
mixer rule 18 add SR S14 -866    0   # Servo C: - roll (common)
mixer rule 19 add SY S14  866    0   # Servo C: + differential yaw (rear)
```

**Motors - front ESC on `M1`, rear ESC on `M2`:**

```
mixer rule 20 set ST  M1 1000    0   # Front rotor throttle (governor instance 0)
mixer rule 21 set ST2 M2 1000    0   # Rear rotor throttle  (governor instance 1)
```

`ST` and `ST2` are the two independent governor throttle outputs (see next section) -
this is what allows each rotor to be governed to its own RPM feedback while both target
the same headspeed.

### 4.4 ESC / motor setup

```
set motor_pwm_protocol = DSHOT600
set dshot_bidir = ON
set motor_poles = 14,14        # set to your actual motor pole count
```

Both ESCs must support bidirectional DShot (eRPM telemetry) - the governor needs a real RPM
feedback signal from **both** motors.

### 4.5 Dual-motor governor

```
set gov_mode = ELECTRIC
set gov_dual_motor = ON
set gov_headspeed = 1500       # target headspeed, both rotors
set gov_gain = 100
set gov_p_gain = 40
set gov_i_gain = 50
set gov_d_gain = 10
set gov_f_gain = 10
```

`gov_dual_motor = ON` runs a **second, fully independent** governor instance for `M2`
(motor index 1), reading its own RPM telemetry, while sharing the exact same headspeed
target and PID gains as the primary governor on `M1`. Since both instances are governed to
the same target with identical tuning, both rotors converge to the same RPM even though
each ESC is driven independently. See [`Governor.md`](Governor.md) for details on the
individual `gov_*` parameters.

### 4.6 Save

```
save
```

## 5. Verifying and tuning

1. With props/blades off, arm and check each stick individually:
   - Collective: all 6 servos move the same way.
   - Roll: all 6 servos tilt for roll, both swashplates the same direction.
   - Pitch: front and rear swashplates move collective in **opposite** directions.
   - Yaw: front and rear swashplates tilt in **opposite** directions.
2. If an axis moves the wrong way, flip the sign of the corresponding `weight` in the
   `mixer rule` entries for that axis (e.g. change `866` to `-866`).
3. Use `mixer rule` (no arguments) or `mixer status` to inspect the currently active rules
   and live output values while tuning.
4. Overall throw/expo per axis can still be tuned the same way as on any other
   Rotorflight heli, with `mixer rate SC/SR/SP/SY <value>` and `mixer limit ...`.
5. Confirm both ESCs report valid RPM (`status` / OSD / blackbox) before relying on
   `gov_dual_motor` in flight - if either motor loses its RPM signal, arming is blocked
   exactly like the single-motor governor safety check.
