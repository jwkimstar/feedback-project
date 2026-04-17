# Presentation Content Draft

This file converts the ideation-stage plan into a slide-by-slide presentation draft. The wording is meant to be close to usable in PowerPoint, while still leaving room for trimming once the final plots and videos are selected.

## Slide 1: Title

**Title**

Fault-Tolerant Fly-By-Wire Control for Rudder-Failure Compensation in X-Plane

**Subtitle**

ASE 370C Feedback Control Systems  
Spring 2026 Design Project

**On-slide points**

- Autonomous flight will require fault-tolerant control, not just nominal control
- Closed-loop reconfiguration after rudder failure
- Implemented, tested, and iteratively improved inside X-Plane

**Visual**

- Full-slide X-Plane screenshot during a turn or heading-capture run

**Speaker notes**

- As aviation moves toward higher levels of autonomy, safety-critical control systems need to remain controllable even when part of the aircraft stops behaving normally.
- The central idea of the project is that a fly-by-wire system normally assumes all control surfaces are available.
- Once that assumption breaks, the original control mapping is no longer valid.
- Our project became an effort to redesign that mapping in closed loop so the aircraft could still follow heading commands after rudder failure.

## Slide 2: Motivation

**Title**

Why Fault-Tolerant Reconfiguration Is Needed

**On-slide points**

- Autonomous flight depends on reliable closed-loop control in off-nominal conditions.
- A nominal fly-by-wire mapping assumes all control surfaces are functioning.
- If the rudder fails, the aircraft loses direct heading control through the intended channel.
- A safe system should recover controllability using the remaining control authority.

**Visual**

- Before/after diagram:
  - normal: pilot command -> nominal fly-by-wire mapping -> aircraft response
  - failure: pilot command -> reconfigured control law -> remaining actuators -> aircraft response

**Speaker notes**

- The long-term relevance of this project is autonomy and safety.
- A future autonomous aircraft cannot depend on every actuator always behaving perfectly.
- The original project proposal already identified this problem correctly.
- What changed during the semester is the comparison story, not the motivation.
- We are still solving the same high-level problem: how to preserve useful aircraft response after losing a control surface.

## Slide 3: Problem Statement

**Title**

Project Question

**On-slide points**

- After rudder failure, the aircraft loses direct heading controllability through the nominal path.
- Can a PID-based controller restore usable heading control using the remaining surfaces?
- Can the aircraft still turn toward and hold a commanded heading?
- Can that controller be implemented and evaluated in a repeatable simulator workflow?

**Visual**

- Simple right-turn example:
  - commanded turn
  - nominal rudder-assisted response unavailable
  - compensation must come from remaining surfaces and new control logic

**Speaker notes**

- Our final project is best described as a fault-tolerant control reconfiguration problem.
- The baseline problem is the loss of direct heading control in the first place.
- The question is whether feedback control can recover enough controllability to make heading commands trackable again.

## Slide 4: Scope Update

**Title**

How The Project Evolved

**On-slide points**

- Original proposal:
  - fault-tolerant control under control-surface failure
  - possible comparisons against human pilot or open-loop heuristic
- Final project focus:
  - safety and controllability in the context of autonomous flight
  - closed-loop PID-based controller design
  - X-Plane integration
  - telemetry logging and offline analysis
  - controller-to-controller iteration and improvement

**On-slide conclusion**

We no longer compare against a human pilot or feedforward baseline.  
Our main baseline is loss of direct heading control after rudder failure.

**Speaker notes**

- This slide is important because it prevents the audience from expecting a human-vs-controller experiment that is no longer part of the work.
- It also makes the later baseline choice much more defensible: the baseline is uncompensated loss of heading controllability, and the controller iterations show how we improved from there.

## Slide 5: System Overview

**Title**

Experiment And Control Stack

**On-slide points**

- X-Plane 12 provides the aircraft simulation environment.
- Python client discovers X-Plane and receives telemetry over UDP.
- Controller computes commands from aircraft state.
- Commands are sent back into X-Plane.
- The same runtime also records CSV logs and supports plotting.

**Visual**

- Block diagram:
  - X-Plane -> telemetry -> Python runtime -> controller -> DATA packet -> X-Plane
  - logging and plotting attached to the runtime

**Speaker notes**

- A meaningful part of the project was building a repeatable test harness.
- This let us move from one-off observations to structured controller iterations backed by recorded runs.

## Slide 6: Baseline And Improvement Target

**Title**

Baseline And What We Tried To Improve

**On-slide points**

- Project baseline:
  - rudder-failure case with no compensation enabled
  - loss of direct heading control through the nominal control path
- Secondary engineering reference:
  - early proportional-only closed-loop controller from April 2, 2026
- Improvement targets:
  - restore controllability of heading
  - better heading tracking
  - lower overshoot and oscillation
  - less aileron saturation
  - smoother transient response

**Recommended statement**

We are not claiming to outperform a human pilot.  
Our baseline is the loss of direct heading control after rudder failure, and our controller is meant to restore that controllability.

**Speaker notes**

- At the project level, the baseline is the uncompensated failure case.
- At the controller-design level, the early proportional controller is still useful as an internal engineering reference for how the design improved.

## Slide 7: Control Method

**Title**

Chosen Control Method

**On-slide points**

- Cascaded feedback control
- Heading hold -> roll damper -> yaw damper -> final aileron command
- Final controller family:
  - heading hold: PID
  - roll damper: PI
  - yaw damper: PI
- Additional design features:
  - measurement-based derivative action
  - roll-damper output clamp as a yaw-rate-command limiter

**Visual**

- Labeled block diagram with:
  - target heading
  - heading error
  - intermediate signals
  - yaw feedback
  - final aileron output

**Speaker notes**

- The reason for using a cascade is that heading error is too high level to send directly to an actuator.
- The cascade converts that objective into progressively more local control signals until it becomes a command X-Plane can apply.

## Slide 8: Why This Architecture Makes Sense

**Title**

Why A Cascaded Controller

**On-slide points**

- Heading is the outer-loop objective.
- Roll and yaw dynamics are faster inner-loop behaviors.
- A cascade separates objectives by level:
  - outer loop: where we want to go
  - inner loops: how we stabilize the motion needed to get there
- This lets us reuse remaining control authority in a structured way after failure.

**Speaker notes**

- This is one of the most important technical ideas in the presentation.
- The controller is not just a gain multiplied by heading error.
- It is a structured re-mapping from pilot intent to feasible aircraft behavior after the nominal actuator relationship is broken.

## Slide 9: Iteration 0

**Title**

Iteration 0: Build The Infrastructure First

**On-slide points**

- Date: March 21, 2026
- Built the package architecture and experiment tooling
- Added:
  - X-Plane discovery and telemetry ingestion
  - terminal telemetry output
  - CSV recording
  - live plotting
  - offline analysis entry points
- Outcome:
  - repeatable experiments became possible

**Lesson learned**

Before tuning controllers, we needed a reliable way to observe, record, and compare runs.

**Visual**

- architecture diagram or screenshot of the telemetry/logging workflow

**Speaker notes**

- This stage may not look like “control design,” but it made every later control decision defensible.
- Without this infrastructure, the project would have remained an anecdotal prototype.

## Slide 10: Iteration 1

**Title**

Iteration 1: First Working Closed-Loop Controller

**On-slide points**

- Date: April 2, 2026
- Migrated the prototype control path from `actual_code.py` into the packaged runtime
- Introduced the `MasterController`
- Enabled three controller depths:
  - yaw damper only
  - yaw + roll damper
  - heading hold + roll damper + yaw damper

**Technical reasoning**

- Start with the simplest working closed-loop version
- Verify sign conventions, actuator path, and end-to-end simulator integration

**Outcome**

- Closed-loop control worked in X-Plane
- Some runs showed aggressive transients and saturation

**Supporting artifacts**

- `artifacts/session-20260402-174356.csv`
- `artifacts/session-20260402-174643.csv`

**Speaker notes**

- This is not the project-level baseline anymore.
- Instead, it is the first working controller that showed we could begin to restore heading controllability after the failure case.
- It also exposed that proportional-only control was too rough in some scenarios.

## Slide 11: Iteration 1 Evidence

**Title**

Baseline Behavior: What The Early Controller Showed

**On-slide points**

- The controller could produce heading changes and command the aircraft in closed loop.
- However, some runs showed:
  - large roll excursions
  - strong yaw-rate excursions
  - periods of actuator saturation
- This established the first useful engineering question:
  - how do we preserve the architecture while improving transient behavior?

**Plots to place**

- heading vs time
- roll angle vs time
- yaw rate vs time
- aileron command vs time

**Speaker notes**

- The key point is not that the baseline failed completely.
- The key point is that it gave us a functioning reference controller and exposed the next set of design problems clearly.

## Slide 12: Iteration 2

**Title**

Iteration 2: Add Integral Action

**On-slide points**

- Date: April 5, 2026
- Added PI support to:
  - yaw damper
  - roll damper
  - heading hold
- Added measured `dt` handling from telemetry timing
- Added integral-state support and windup limiting

**Technical reasoning**

- Proportional control alone can leave residual error or weak convergence.
- Integral action was added to improve tracking and error correction.

**Documented outcome**

“Everything works but very long transient time.”

**Speaker notes**

- This iteration shows a classic control tradeoff.
- A feature that helps steady-state correction can also degrade transient response if it is not introduced carefully.

## Slide 13: Iteration 2 Evidence And Lesson

**Title**

What PI Improved And What It Broke

**On-slide points**

- Improvement:
  - more persistent effort to drive error down
- New problem:
  - very long transient response
  - harder tuning
  - more sustained command activity

**Lesson learned**

Integral action solved one problem, but introduced another.  
Better steady-state behavior alone was not enough.

**Visuals**

- baseline proportional response vs PI response
- aileron command trace showing longer sustained effort
- optional X-Plane still or clip

**Speaker notes**

- This slide should make the audience feel the tuning problem.
- We were not done once the controller “worked.”
- We had to make it work well.

## Slide 14: Iteration 3

**Title**

Iteration 3: Fix The Structure, Not Just The Gains

**On-slide points**

- Date: April 12, 2026
- Added PD/PID support to heading hold
- Used derivative on measurement instead of target change
- Added roll-damper output clamp as a yaw-rate-command limiter
- Tried a yaw-damper anti-windup strategy, then reverted it

**Technical reasoning**

- Derivative on measurement avoids derivative kick on heading-step commands.
- Limiting the roll-damper output is more meaningful because that signal enters the yaw-damper stage directly.
- This iteration focused on better structure, not only different gains.

**Speaker notes**

- This is probably the most important “control thinking” slide in the deck.
- It shows that improvement did not come from blind tuning.
- It came from deciding where in the cascade the limiter and derivative action should live.

## Slide 15: Iteration 3 Evidence

**Title**

Effect Of Better Signal Placement And Derivative Handling

**On-slide points**

- Earlier April 12 runs still showed unstable or saturated behavior
- Later April 12 runs looked more controlled
- This suggested the architecture refinements were moving the design in the right direction

**Candidate unstable examples**

- `artifacts/session-20260412-153103.csv`
- `artifacts/session-20260412-154356.csv`

**Candidate improved examples**

- `artifacts/session-20260412-160709.csv`
- `artifacts/session-20260412-161348.csv`

**Lesson learned**

The biggest improvements came from putting the limiter in the correct place and handling derivative action carefully.

**Speaker notes**

- If the plots are chosen well, this becomes the pivot point of the whole presentation.
- It demonstrates that structural control decisions mattered more than simply increasing or decreasing gains.

## Slide 16: Iteration 4

**Title**

Iteration 4: Latest Tuned Controller

**On-slide points**

- Date: April 16, 2026
- Candidate final controller:
  - heading hold PID
  - roll damper PI
  - yaw damper PI
  - roll-damper yaw-rate clamp
- Candidate final-result artifacts:
  - `artifacts/session-20260416-011437.csv`
  - `artifacts/session-20260416-013924.csv`

**Observed behavior from quick artifact review**

- strong reduction in heading error
- no obvious heavy aileron saturation
- more moderate roll and yaw-rate peaks than early unstable runs

**Speaker notes**

- This slide should present the current best version as the culmination of the previous iterations.
- It should still be careful not to overclaim until the final comparison run is matched and verified.

## Slide 17: Final Result

**Title**

Final Result

**On-slide points**

- The tuned controller can drive the aircraft toward and hold commanded headings inside the packaged X-Plane loop.
- The final design preserves the same overall architecture as the baseline.
- The improvement came from:
  - adding integral action where useful
  - adding derivative action to heading hold
  - using derivative on measurement
  - limiting the correct intermediate signal

**Recommended claim**

The final controller achieved smoother heading capture and less damaging transient behavior than the original proportional-only baseline.

**Visuals**

- best final heading plot
- best final aileron-command plot
- screenshot or short clip from X-Plane

**Speaker notes**

- Keep this slide outcome-focused.
- The audience should leave with a clear sense that the final controller is not just different, but meaningfully better behaved.

## Slide 18: Baseline Vs Final Comparison

**Title**

Baseline Vs Final Controller

**On-slide points**

- Baseline:
  - rudder failure with loss of direct heading control
  - no compensation controller enabled
- Final:
  - tuned PI/PI/PID cascade with yaw-rate clamp
- Improvements observed:
  - restored heading controllability
  - better heading convergence
  - less extreme roll behavior
  - less actuator saturation
  - smoother transient response

**Recommended comparison visuals**

- overlay: uncompensated failure heading response vs final controlled heading response
- overlay: yaw-rate response without controller vs with controller
- overlay: aileron command for early controller vs final controller

**Optional metric table**

- final heading error
- peak absolute roll angle
- peak absolute yaw rate
- percent of samples with `|aileron_cmd| >= 0.95`

**Speaker notes**

- This is the comparison slide that replaces the old human/feedforward baseline concept.
- The main comparison is between uncompensated loss of heading control and the final tuned controller.
- If useful, we can still include an inset or secondary comparison showing early-controller behavior versus final-controller behavior.

## Slide 19: X-Plane Implementation

**Title**

Implementation In X-Plane

**On-slide points**

- The controller is integrated into a live simulator loop.
- One runtime handles:
  - telemetry
  - control computation
  - command transmission
  - logging
  - plotting
- Example packaged command:

```bash
python3 -m python_client.cli.run_all --hz 15 --history-seconds 60 --yaw-roll-heading-hold --yaw-controller-type pi --roll-controller-type pi --heading-controller-type pid --roll-damper-max-yaw-rate-deg-s 5.0 --yaw-damper-gain 3.5 --yaw-damper-integral-gain 0.03 --roll-damper-gain 1.0 --roll-damper-integral-gain 0.01 --heading-hold-gain 0.7 --heading-hold-integral-gain 0.01 --heading-hold-derivative-gain 0.4 --target-heading-deg 30.0
```

**Speaker notes**

- This slide proves the project is not just a conceptual controller design.
- It is implemented, runnable, and instrumented inside the simulator.

## Slide 20: Main Lessons

**Title**

What We Learned

**On-slide points**

- Fault-tolerant fly-by-wire control is fundamentally a reconfiguration problem.
- In an autonomy context, safety depends on maintaining controllability after failures.
- The most meaningful project baseline was the loss of direct heading control after rudder failure.
- The final improvement came from architecture decisions as much as gain tuning.
- Logging, plotting, and repeatable runs were essential to controller development.

**Closing sentence**

The final project became a reusable experimental platform for fault-tolerant control, not just a single prototype controller.

**Speaker notes**

- This is the right note to end on because it captures both the control insight and the engineering maturity of the project.

## Slide 21: Next Steps

**Title**

Next Steps

**On-slide points**

- Rerun baseline and final controllers under matched initial conditions
- Generate final comparison plots from selected CSV artifacts
- Add more explicit experiment metadata to recordings
- Extend the approach to additional failure cases or added control loops

**Speaker notes**

- This slide is optional.
- Keep it if the presentation format expects future work or limitations.

## Appendix: Suggested Plot Placement

Use these as the first candidates when building the deck:

- Baseline proportional examples:
  - `artifacts/session-20260402-174356.csv`
  - `artifacts/session-20260402-174643.csv`
- Mid-iteration unstable examples:
  - `artifacts/session-20260412-153103.csv`
  - `artifacts/session-20260412-154356.csv`
- Mid-iteration improved examples:
  - `artifacts/session-20260412-160709.csv`
  - `artifacts/session-20260412-161348.csv`
- Candidate final examples:
  - `artifacts/session-20260416-011437.csv`
  - `artifacts/session-20260416-013924.csv`

## Appendix: Minimum Plot Set For The Final Deck

If time is limited, generate these plots only:

1. Uncompensated failure vs final controlled heading response
2. Uncompensated failure vs final yaw-rate response
3. Early controller vs final aileron command
4. One X-Plane screenshot or short video for the final controller

## Appendix: Important Caution

The current CSV recordings do not store full experiment metadata such as controller gains, controller type, or explicit target labels. Before the final presentation, verify that the selected baseline and final runs correspond to comparable conditions.
