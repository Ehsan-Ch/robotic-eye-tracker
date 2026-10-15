# Validation

- 21 local unit tests passed on Python 3.12 / Windows. Coverage includes RGB/HSV behavior, candidate filtering, camera geometry, acquisition/loss/reacquisition, command limits, input isolation, scene actuator bounds, and actual simulated motor movement.
- Full rendered evaluation completed: 30 trials, 7,500 frames, seeds 100â€“104, six groups, no discarded trials.
- Six 10-second MP4 replays and a preview were generated from the first seed in each group.
- Source hashes are recorded in `reports/evaluation.json`; the scoring pipeline never supplies target ground truth to the controller.
- Browser verification passed: normal replay loads with readyState 4 and duration 10 seconds; playback advances; recorded time, state, and motor angles update. Occlusion and similar-object selection update the report. The camera view and report layout were visually inspected.
- [GitHub Actions run 38754678596](https://github.com/Ehsan-Ch/robotic-eye-tracker/actions/runs/38754678596) did not start the test job. GitHub reports: "The job was not started because your account is locked due to a billing issue." There were no executed steps. This is an account-side CI block, not a test assertion failure; remote tests remain unverified. Rerun the workflow after the account issue is resolved.
- The published source files match the source hashes embedded in the evaluation. The final ZIP and GitHub file contents are checked byte for byte using Git blob hashes.

Published percentages are conditional on the synthetic protocol documented in the README. Lookalike identity errors and fast-motion failures are retained in the report.
