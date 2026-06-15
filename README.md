# Contribution 1: Use dynamic reconfigure in whistle detector

**Contribution Number:** 1  
**Student:** Huy Hoang  
**Issue:** https://github.com/bit-bots/bitbots_main/issues/776  
**Status:** Phase II Complete

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

**Match:** ROS 2 nodes natively support runtime parameters via `declare_parameter()` and live updates via `add_on_set_parameters_callback()`. This pattern requires no new dependencies — `rcl_interfaces` ships with every ROS 2 installation and is already available in this workspace.

**Plan:**
1. Add import: `from rcl_interfaces.msg import SetParametersResult` and `from rclpy.parameter import Parameter` in `whistle_detector.py`
2. In `WhistleDetector.__init__()`, after `super().__init__("whistle_detector")`, declare three parameters:
   - `self.declare_parameter("whistle_energy_ratio_threshold", 0.6)`
   - `self.declare_parameter("whistle_frequency_min_hz", 2000)`
   - `self.declare_parameter("whistle_frequency_max_hz", 4500)`
3. Load initial values into instance attributes (`self.threshold`, `self.freq_min`, `self.freq_max`) via `get_parameter(...).value`
4. Register `self.add_on_set_parameters_callback(self._on_params_change)`
5. Add `_on_params_change(self, params)` method that validates each param by name and type, updates the corresponding instance attribute, and returns `SetParametersResult(successful=True)`
6. In `detect_whistle()`, replace `2000`, `4500`, and `0.6` with `self.freq_min`, `self.freq_max`, and `self.threshold`

**Implement:** https://github.com/hwii-florescent/bitbots_main/tree/fix-issue-776 *(code changes in Phase III)*

**Review:** Run `pixi run format` before committing. Confirm the commit message follows the repo's conventions (checked via recent commit history). Verify no CONTRIBUTING.md rules are violated.

**Evaluate:**
- `ros2 param list /whistle_detector` shows `whistle_energy_ratio_threshold`, `whistle_frequency_min_hz`, `whistle_frequency_max_hz`
- `ros2 param set /whistle_detector whistle_energy_ratio_threshold 0.8` succeeds and the node logger confirms the update
- Default behavior is identical to before when no parameters are overridden
- `pixi run test --pkg bitbots_whistle_detector` passes with no regressions

---

## Testing Strategy

### Unit Tests

- [ ] Test case 1: [Description]
- [ ] Test case 2: [Description]
- [ ] Test case 3: [Description]

### Integration Tests

- [ ] Integration scenario 1
- [ ] Integration scenario 2

### Manual Testing

[What you tested manually and results]

---

## Implementation Notes

### Week [X] Progress

[What you built this week, challenges faced, decisions made]

### Week [Y] Progress

[Continue documenting as you work]

### Code Changes

- **Files modified:** [List]
- **Key commits:** [Links to important commits]
- **Approach decisions:** [Why you chose certain approaches]

---

## Pull Request

**PR Link:** [GitHub PR URL when submitted]

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
