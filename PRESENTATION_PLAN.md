# Presentation Plan

This is an ideation-stage outline for the project presentation. It is not meant to be the final deck. The goal here is to build a stronger narrative around what the project actually became.

## Core Story

The cleanest story is no longer:

- compare our controller to a human pilot
- compare our controller to a feedforward heuristic

The stronger and more accurate story is:

- a conventional fly-by-wire mapping assumes all control surfaces are available
- a rudder failure breaks that mapping
- the project goal became to reconfigure the control logic so the aircraft can still follow heading commands using the remaining control authority
- the final contribution is a closed-loop, X-Plane-integrated fault-tolerant control architecture, not just one tuned gain set

## Recommended Baseline Framing

Since human-pilot and feedforward comparisons are no longer part of the project, the best baseline options are:

### Recommended primary baseline

- proportional-only closed-loop cascade
- specifically: the early packaged controller from April 2, 2026, which preserved the prototype structure from `actual_code.py`

Why this is the strongest baseline:

- it is documented in repository history
- it uses the same overall architecture as the later controller
- improvements can be attributed to better control design and tuning, not to switching to a totally different system

### Recommended conceptual baseline

- rudder-failure case with no compensation controller enabled

Why this helps:

- it motivates the entire project at the physical system level
- it gives the audience an intuitive “without reconfiguration, the aircraft does not track the command well” reference point

### Optional third baseline if needed

- reduced-depth controller:
  - yaw damper only
  - or yaw damper + roll damper
  - compared against full heading-hold + roll-damper + yaw-damper cascade

This works if you want to show why the final cascade architecture was necessary.

## Recommended Slide Order

### 1. Title / One-Line Thesis

- Title idea:
  - Fault-Tolerant Fly-By-Wire Control for Rudder-Failure Compensation in X-Plane
- One-line thesis:
  - We designed and iteratively improved a closed-loop controller that re-maps pilot intent into a new control path after rudder failure so the aircraft can still achieve commanded heading changes.

Visual:

- X-Plane screenshot during a failure-recovery run

### 2. Motivation: Why Reconfiguration Is Needed

- Original fly-by-wire assumption:
  - pilot intent is mapped to aircraft response assuming all control surfaces function normally
- Failure case:
  - if the rudder is lost, that original mapping is no longer valid
- Project question:
  - can we redesign the control logic so the aircraft still turns toward the commanded heading using the remaining surfaces?

Storytelling note:

- This is where you can reuse the original proposal’s right-turn example, but update the wording so it supports the new story rather than promising a human-vs-controller comparison.

Visual:

- simple before/after diagram:
  - normal aircraft: pilot command -> nominal actuator mapping
  - failure case: pilot command -> reconfigured control law -> remaining actuators

### 3. Updated Project Scope

- What the project originally proposed:
  - fault-tolerant control under control-surface failure
- What the project actually focused on:
  - building a working closed-loop compensation controller in X-Plane
  - integrating telemetry, command transmission, plotting, and logging
  - tuning a reconfigurable control architecture for heading recovery and hold
- What changed from the original proposal:
  - no longer comparing against a human pilot
  - no longer using feedforward/open-loop as the main comparison axis
  - the main comparison is now early-controller baseline vs improved controller designs

### 4. Project Overview And Experiment Stack

- Simulator platform:
  - X-Plane 12
- Software stack from the repository:
  - X-Plane UDP discovery and telemetry
  - packaged Python client
  - control modules
  - live plotting
  - CSV recording
  - offline analysis
- Main engineering contribution:
  - creating a repeatable loop:
    - read state -> compute control -> send command -> log results -> compare runs

Visual:

- block diagram:
  - X-Plane -> telemetry -> Python controller -> DATA packet -> X-Plane
  - logging and plotting attached to the same runtime

### 5. Baseline You Picked And Improvement You Targeted

- Primary technical baseline:
  - proportional-only closed-loop cascade from the early prototype / April 2 packaged controller
- Conceptual baseline:
  - rudder-failure case without compensation
- Improvement targeted:
  - better heading tracking to a commanded target
  - lower overshoot and oscillation
  - less aileron saturation
  - more stable transient response
  - more reproducible testing and analysis workflow

Recommended phrasing:

- “We are not claiming to outperform a human pilot. Instead, we measure improvement relative to the first working closed-loop fault-tolerant controller we built.”

### 6. Control Method Chosen

- Chosen method:
  - cascaded feedback controller
  - heading hold -> roll damper -> yaw damper -> final aileron command
- Why this method makes sense:
  - heading error is a high-level objective, not a direct actuator command
  - the cascade progressively converts a heading objective into a stabilizing control signal
  - this lets the system use remaining control authority in a structured way after rudder failure
- Final controller family to present:
  - yaw damper: PI
  - roll damper: PI
  - heading hold: PID
  - measurement-based derivative action for heading hold
  - roll-damper output clamp used as a yaw-rate-command limiter

Visual:

- labeled control block diagram with:
  - target heading
  - heading error
  - heading-hold output
  - roll-damper output / yaw-rate command
  - yaw feedback
  - final aileron command

### 7. Iteration 0: Build The Infrastructure First (March 21, 2026)

- Goal of this stage:
  - make the experiment pipeline usable before serious controller tuning
- Repository-backed work:
  - X-Plane discovery
  - telemetry ingestion
  - terminal output
  - CSV logging
  - live plotting
  - modular package structure
- Outcome:
  - the team created a repeatable test environment instead of relying on one monolithic script
- Lesson learned:
  - without reliable logging and plotting, controller tuning would be anecdotal rather than defensible

Visuals:

- package/runtime architecture diagram
- screenshot of live plotter or logging workflow

### 8. Iteration 1: First Working Closed-Loop Baseline (April 2, 2026)

- What changed:
  - the control path from `actual_code.py` was migrated into the packaged runtime
  - the `MasterController` was introduced
  - three control depths became available:
    - yaw damper only
    - yaw + roll damper
    - heading hold + roll damper + yaw damper
- Technical reasoning:
  - start with the simplest working closed-loop version of the architecture
  - verify signal signs, actuator path, and end-to-end X-Plane integration
- Outcome:
  - the controller worked in closed loop, but some runs showed aggressive transients and actuator saturation
- Candidate artifact files:
  - `artifacts/session-20260402-174356.csv`
  - `artifacts/session-20260402-174643.csv`
- Lesson learned:
  - the architecture was viable, but proportional-only control was not enough for good transient performance

Plots to show:

- heading vs time
- roll angle vs time
- yaw rate vs time
- aileron command vs time

### 9. Iteration 2: Add Integral Action (April 5, 2026)

- What changed:
  - PI control added to yaw damper, roll damper, and heading hold
  - loop timing `dt` measured from telemetry spacing
  - integrator support and windup limiting added
- Technical reasoning:
  - proportional control alone can leave persistent error or weak convergence
  - integral action was intended to improve tracking and reduce residual bias
- Documented repository outcome:
  - “everything works but very long transient time”
- Interpretation:
  - the system gained error-correction authority, but became slower and harder to tune cleanly
- Lesson learned:
  - better steady-state behavior can come at the cost of poor transient response

Evidence to include:

- baseline proportional response vs PI response
- command trace showing more sustained controller effort
- optional X-Plane clip showing longer settling

### 10. Iteration 3: Fix The Structure, Not Just The Gains (April 12, 2026)

- What changed:
  - heading hold became the canonical module
  - PD/PID support added for heading hold
  - derivative taken on measurement instead of target error change
  - roll-damper output clamp added as a yaw-rate-command limiter
  - a first anti-windup idea for yaw damper was attempted, then reverted
- Technical reasoning:
  - derivative on measurement avoids derivative kick when the heading command steps
  - limiting the roll-damper output makes more sense than limiting the upstream heading block because that signal is what actually enters the yaw-damper stage
  - this stage shows that controller structure matters as much as controller gains
- Candidate artifact files:
  - unstable / saturated examples:
    - `artifacts/session-20260412-153103.csv`
    - `artifacts/session-20260412-154356.csv`
  - improved examples:
    - `artifacts/session-20260412-160709.csv`
    - `artifacts/session-20260412-161348.csv`
- Lesson learned:
  - the biggest improvements came from placing the limiter correctly and handling derivative action carefully, not from adding complexity blindly

### 11. Iteration 4: Latest Tuned Runs / Candidate Final Result (April 16, 2026)

- Best current candidate for the final result slide:
  - `artifacts/session-20260416-011437.csv`
- Additional candidate:
  - `artifacts/session-20260416-013924.csv`
- Why these are promising:
  - they appear to show strong reduction in heading error
  - they do not show obvious heavy aileron saturation in the quick CSV scan
  - roll and yaw-rate peaks look more moderate than earlier unstable runs
- Suggested claim:
  - by the latest tuning stage, the controller was able to capture a commanded heading with smoother transient behavior than the early proportional baseline
- Important caution:
  - the CSV schema does not store gain metadata or explicit experiment labels
  - before the final presentation, verify the exact run conditions for the “final” plot so the comparison is fair

### 12. Baseline Vs Final Comparison Slide

- Best comparison framing:
  - compare the proportional-only closed-loop baseline to the later tuned PI/PI/PID cascade with yaw-rate clamp
- Improvement claims to emphasize:
  - better heading convergence
  - less extreme roll behavior
  - less actuator saturation
  - smoother transient response
  - stronger experimental workflow and reproducibility

Recommended visuals:

- overlay: heading response baseline vs final
- overlay: aileron command baseline vs final
- overlay: yaw-rate response baseline vs final
- side-by-side X-Plane stills or a short video comparison

If you want one quantitative summary table:

- final heading error
- peak absolute roll angle
- peak absolute yaw rate
- percent of samples with `|aileron_cmd| >= 0.95`

### 13. X-Plane Implementation Slide

- Show the current packaged run path from `README.md`:
  - `python3 -m python_client.cli.run_all --hz 15 --history-seconds 60 --yaw-roll-heading-hold --yaw-controller-type pi --roll-controller-type pi --heading-controller-type pid --roll-damper-max-yaw-rate-deg-s 5.0 --yaw-damper-gain 3.5 --yaw-damper-integral-gain 0.03 --roll-damper-gain 1.0 --roll-damper-integral-gain 0.01 --heading-hold-gain 0.7 --heading-hold-integral-gain 0.01 --heading-hold-derivative-gain 0.4 --target-heading-deg 30.0`
- What this proves:
  - the controller is implemented inside a real simulator loop
  - telemetry, plotting, recording, and command transmission run together
  - the project became a reusable experimental platform, not just a one-off script

Visuals:

- X-Plane screenshot
- live plot screenshot
- optional short snippet from `master_controller.py`

### 14. Conclusion / Main Takeaways

- The final project is best understood as a fault-tolerant control reconfiguration problem.
- The most meaningful baseline is the first working closed-loop controller, not a human or feedforward comparator.
- The useful final controller emerged through multiple iterations in architecture, not just gain tuning.
- The strongest improvements came from:
  - adding the right loops
  - handling derivative action correctly
  - limiting the right intermediate signal
  - making experiments repeatable through logging and plotting

## Suggested Answers To The Required Questions

### The baseline you picked and the improvement you targeted

- Baseline:
  - primary: proportional-only closed-loop cascaded controller from the early packaged implementation
  - conceptual: rudder-failure behavior without compensation
- Improvement targeted:
  - improved heading tracking and recovery after rudder failure
  - reduced oscillation, overshoot, and actuator saturation
  - better transient behavior and more repeatable evaluation

### The control method you chose

- Cascaded feedback control:
  - heading hold -> roll damper -> yaw damper -> aileron command
- Final tuned version:
  - yaw PI
  - roll PI
  - heading PID
  - measurement-based derivative action
  - roll-damper yaw-rate clamp

### The outcomes of each iteration and the lessons learned

- Infrastructure stage:
  - outcome: created a usable experiment pipeline
  - lesson: repeatable logging/plotting was required before serious tuning
- First proportional baseline:
  - outcome: proved the architecture could work in X-Plane
  - lesson: raw proportional control produced poor transients in some runs
- PI expansion:
  - outcome: improved error-correction capability but caused very long transients
  - lesson: integral action helps one problem while creating another
- PID/clamp restructuring:
  - outcome: better-behaved heading capture and less damaging transients
  - lesson: structural choices and signal placement matter as much as gains
- Latest tuning:
  - outcome: candidate runs show practical heading capture with more moderate control behavior
  - lesson: the final controller quality came from iterative refinement of both architecture and tuning

### The final result and its comparison against the baseline

- Final result:
  - the tuned controller can drive the aircraft toward and hold commanded headings in the packaged X-Plane loop after the control mapping is reconfigured for the failure scenario
- Improvement over the baseline:
  - less saturation
  - smaller extreme roll excursions
  - better heading convergence
  - smoother transient behavior
  - more robust experiment workflow
- Evidence to show:
  - heading, yaw-rate, roll, and aileron-command plots
  - X-Plane screenshots or video
  - the packaged implementation command and architecture diagram

## What To Verify Before Building The Final Deck

- Confirm whether the final presentation should explicitly mention “rudder failure with controller off” as a conceptual baseline.
- Match the baseline and final runs to similar initial conditions and the same target heading.
- Confirm which April 16 artifact corresponds to the README’s current working controller command.
- Export final plots from the selected CSVs.
- Capture one short X-Plane clip for:
  - early/baseline response
  - final/tuned response
- If needed, rerun the baseline and final controllers under matched conditions so the comparison is defensible.
