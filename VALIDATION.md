# Validation

- 21 local unit tests passed on Python 3.12 / Windows. Coverage includes RGB/HSV behavior, candidate filtering, camera geometry, acquisition/loss/reacquisition, command limits, input isolation, scene actuator bounds, and actual simulated motor movement.
- Full rendered evaluation completed: 30 trials, 7,500 frames, seeds 100–104, six groups, no discarded trials.
- Six 10-second MP4 replays and a preview were generated from the first seed in each group.
- Source hashes are recorded in `reports/evaluation.json`; the scoring pipeline never supplies target ground truth to the controller.
- Browser interaction verification and GitHub workflow status are pending publication checks. No remote CI success is claimed.

Published percentages are conditional on the synthetic protocol documented in the README. Lookalike identity errors and fast-motion failures are retained in the report.
