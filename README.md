# Contribution 1: Use dynamic reconfigure in whistle detector

**Contribution Number:** 1  
**Student:** Huy Hoang  
**Issue:** https://github.com/bit-bots/bitbots_main/issues/776  
**Status:** Phase III Complete

---

## Why I Chose This Issue

I chose this issue because it is a well-scoped enhancement to a real robotics codebase used by the Hamburg Bit-Bots RoboCup team. The goal is to add dynamic reconfiguration to the whistle detector module so that parameters like the threshold for whistle energy vs. overall energy can be adjusted at runtime without restarting the node. This makes the issue appealing because the expected outcome is concrete and the scope is bounded to a single module.

This issue also matches my goals for this contribution cycle because it is labeled as a good first issue and gives me hands-on experience with ROS 2 and the `dynamic_reconfigure` pattern used in robotics software. I want to learn how parameter management works in a production ROS 2 project and how to make a meaningful contribution to an active open-source robotics team.

---

## Understanding the Issue

### Problem Description

The `bitbots_whistle_detector` node detects whistles by comparing the energy in the whistle frequency band (2000–4500 Hz) to total audio energy using FFT. When this ratio exceeds a threshold, it publishes a detection event. However, the threshold (`0.6`) and frequency band bounds (`2000`, `4500`) are all hardcoded constants in the source file. There is no way to change these values without editing the Python source and rebuilding the package.

### Expected Behavior

A robot operator or developer should be able to adjust the detection threshold (and optionally the frequency band) at runtime without modifying source code or restarting the node — for example, with `ros2 param set /whistle_detector whistle_energy_ratio_threshold 0.8`.

### Current Behavior

The node exposes no user-facing ROS 2 parameters. Running `ros2 param list /whistle_detector` shows only the built-in ROS internals. Attempting `ros2 param set /whistle_detector whistle_energy_ratio_threshold 0.8` fails with:
```
Setting parameter failed: parameter 'whistle_energy_ratio_threshold' is not declared
```

### Affected Components

- **Primary file:** `src/bitbots_misc/bitbots_whistle_detector/bitbots_whistle_detector/whistle_detector.py`
  - `WhistleDetector.__init__()` — where parameters should be declared
  - `WhistleDetector.detect_whistle()` — where hardcoded values `0.6`, `2000`, `4500` are used

---

## Reproduction Process

### Environment Setup

The project uses **pixi** for dependency management — no system-wide ROS 2 installation or Docker required. ROS 2 Jazzy is the supported version.

1. Install pixi (if not already installed):
   ```bash
   curl -fsSL https://pixi.sh/install.sh | bash
   # Restart terminal or run: export PATH="$HOME/.pixi/bin:$PATH"
   ```
2. Clone your fork and build:
   ```bash
   git clone https://github.com/hwii-florescent/bitbots_main.git
   cd bitbots_main
   pixi run build   # Downloads all ROS 2 dependencies automatically
   ```
3. Activate the pixi shell for subsequent ROS 2 commands:
   ```bash
   pixi shell
   ```

### Steps to Reproduce

1. After completing environment setup, launch the whistle detector node:
   ```bash
   ros2 run bitbots_whistle_detector whistle_detector
   ```
2. In a second terminal (with pixi shell active), list its parameters:
   ```bash
   ros2 param list /whistle_detector
   ```
   Observe: only built-in ROS 2 internals are listed — no custom parameters.

3. Attempt to set the detection threshold at runtime:
   ```bash
   ros2 param set /whistle_detector whistle_energy_ratio_threshold 0.8
   ```
   **Observed result:** `Setting parameter failed: parameter 'whistle_energy_ratio_threshold' is not declared`

4. Open `src/bitbots_misc/bitbots_whistle_detector/bitbots_whistle_detector/whistle_detector.py` and locate the `detect_whistle()` method. The threshold is hardcoded on the final line:
   ```python
   return ratio > 0.6  # hardcoded — cannot be changed at runtime
   ```
   Similarly, the frequency band bounds `2000` and `4500` are hardcoded two lines above.

### Reproduction Evidence

- **Branch:** https://github.com/hwii-florescent/bitbots_main/tree/fix-issue-776
- **Screenshots/logs:** See step 3 above — `ros2 param set` failure message confirms no parameters declared
- **My findings:** All detection parameters (`0.6` ratio threshold, `2000–4500 Hz` band) are Python constants defined directly in `whistle_detector.py`. The node never calls `declare_parameter()`, so ROS 2 has no knowledge of them and they cannot be changed at runtime.

---

## Solution Approach

### Analysis

The root cause is in `whistle_detector.py`. The `WhistleDetector.__init__()` method never calls `declare_parameter()`, so ROS 2's parameter system is unaware of the detection settings. The hardcoded values live directly in `detect_whistle()` as Python literals (`0.6`, `2000`, `4500`).

### Proposed Solution

Use ROS 2's native parameter system: declare named parameters with defaults matching the current hardcoded values, read them into instance attributes on startup, and register a callback so that changes made via `ros2 param set` are applied immediately without restarting the node.

### Implementation Plan

Using UMPIRE framework (adapted):

**Understand:** The `detect_whistle()` method computes the FFT of a 512-sample audio buffer, sums energy in the 2000–4500 Hz band, divides by total energy, and compares to `0.6`. All three numeric values are literals — inaccessible to ROS 2's parameter system and unchangeable at runtime.

**Match:** After reviewing issue comments from the maintainer (Flova), the correct approach is the **`generate_parameter_library`** (piknik) library, not plain `declare_parameter`. The `bitbots_ball_filter` package in this repo is the canonical Python example — it uses `ParamListener` generated from a YAML definition file to handle parameter loading and live updates.

**Plan:**
1. Create `config/whistle_detector_parameters.yaml` defining four parameters with types, defaults, descriptions, and validation bounds: `whistle_energy_ratio_threshold` (double, 0.0–1.0), `whistle_frequency_min_hz` (int), `whistle_frequency_max_hz` (int), and `chunk_size` (int, read-only)
2. Update `setup.py` to call `generate_parameter_module("whistle_detector_parameters", "config/whistle_detector_parameters.yaml")` — this generates the Python parameter class at build time
3. Add `<depend>generate_parameter_library</depend>` to `package.xml`
4. In `whistle_detector.py`: import the generated module, replace `self.chunk_size = 512` with `self.param_listener = parameters.ParamListener(self)` + `self.config = self.param_listener.get_params()`, add `is_old()` / `refresh_dynamic_parameters()` / `get_params()` refresh at the top of `process_audio()`, and replace all hardcoded values with `self.config.*`

**Implement:** https://github.com/hwii-florescent/bitbots_main/tree/fix-issue-776

**Review:** Run `pixi run format` before committing. Confirm the commit message follows the repo's conventions (checked via recent commit history). Verify no CONTRIBUTING.md rules are violated.

**Evaluate:**
- `ros2 param list /whistle_detector` shows `whistle_energy_ratio_threshold`, `whistle_frequency_min_hz`, `whistle_frequency_max_hz`
- `ros2 param set /whistle_detector whistle_energy_ratio_threshold 0.8` succeeds and the node logger confirms the update
- Default behavior is identical to before when no parameters are overridden
- `pixi run test --pkg bitbots_whistle_detector` passes with no regressions

---

## Testing Strategy

### Unit Tests

- [x] Test default parameter values match hardcoded originals (threshold=0.6, freq_min=2000, freq_max=4500, chunk_size=512)
- [ ] Test that `detect_whistle` returns True when ratio exceeds threshold, False when below
- [ ] Test that `detect_whistle` returns False when total_energy is 0

### Integration Tests

- [x] Launch node, verify `ros2 param list` exposes all four parameters
- [x] Set `whistle_energy_ratio_threshold` to 0.8 via `ros2 param set`, confirm node picks up the new value without restart
- [x] Confirm `chunk_size` is read-only — `ros2 param set /whistle_detector chunk_size 256` fails as expected
- [x] Confirm validation bounds — `ros2 param set /whistle_detector whistle_energy_ratio_threshold 1.5` fails as expected
- [x] Confirm value persists — `ros2 param get /whistle_detector whistle_energy_ratio_threshold` returns `0.8` after setting it

### Manual Testing

Ran the full verification suite locally with `pixi run build` + `pixi shell` + Zenoh router:

```
$ ros2 param list /whistle_detector
  chunk_size
  start_type_description_service
  use_sim_time
  whistle_energy_ratio_threshold
  whistle_frequency_max_hz
  whistle_frequency_min_hz

$ ros2 param set /whistle_detector whistle_energy_ratio_threshold 0.8
Set parameter successful

$ ros2 param set /whistle_detector chunk_size 256
Setting parameter failed: parameter 'chunk_size' is read-only

$ ros2 param set /whistle_detector whistle_energy_ratio_threshold 1.5
Setting parameter failed

$ ros2 param get /whistle_detector whistle_energy_ratio_threshold
Double value is: 0.8
```

All checks passed. Default behavior is identical to before — no regressions.

---

## Implementation Notes

### Week 1 Progress

Completed Phase III implementation. Key challenge: the maintainer (Flova) specified using the `generate_parameter_library` (piknik) library rather than plain `declare_parameter` — discovered this by reading the issue comments carefully. Used `bitbots_ball_filter` as the reference Python implementation to understand the pattern (`ParamListener`, `is_old()`, `refresh_dynamic_parameters()`, `get_params()`).

Decision: exposed `chunk_size` as `read_only: True` based on a comment from jaagut — it can be set at launch time but not live-swapped, since runtime buffer resizing would add unnecessary complexity for a first contribution.

### Code Changes

- **Files modified:**
  - `src/bitbots_misc/bitbots_whistle_detector/config/whistle_detector_parameters.yaml` *(created)*
  - `src/bitbots_misc/bitbots_whistle_detector/setup.py`
  - `src/bitbots_misc/bitbots_whistle_detector/package.xml`
  - `src/bitbots_misc/bitbots_whistle_detector/bitbots_whistle_detector/whistle_detector.py`
- **Key commits:** https://github.com/hwii-florescent/bitbots_main/tree/fix-issue-776
- **Approach decisions:** Used `generate_parameter_library` per maintainer request; `chunk_size` marked `read_only` to avoid runtime buffer-resize complexity

### Code Changes

- **Files modified:** [List]
- **Key commits:** [Links to important commits]
- **Approach decisions:** [Why you chose certain approaches]

---

## Pull Request

**PR Link:** https://github.com/bit-bots/bitbots_main/pull/932

**PR Description:** [Draft or final PR description - much of the content above can be adapted]

**Maintainer Feedback:**
- [Date]: [Summary of feedback received]
- [Date]: [How you addressed it]

**Status:** [Awaiting review / Iterating / Approved / Merged]

---

## Learnings & Reflections

### Technical Skills Gained

[What you learned technically]

### Challenges Overcome

[What was hard and how you solved it]

### What I'd Do Differently Next Time

[Reflection on your process]

---

## Resources Used

- [Link to helpful documentation]
- [Tutorial or Stack Overflow post that helped]
- [GitHub issues or discussions that helped]
