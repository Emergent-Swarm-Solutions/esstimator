Tuning With The Visualizer
##########################

The EKF and UKF can publish tuning telemetry and open a matplotlib window that summarizes all input streams. The visualizer is intended to answer two practical questions:

1. Are the innovations behaving like the filter expects?
2. Where should the Mahalanobis rejection threshold sit for each stream?


Enabling The Visualizer
***********************

In the filter YAML, enable the visualizer:

.. code-block:: yaml

   tuning_visualizer_enabled: true
   tuning_visualizer_history_seconds: 60.0

The window is auto-launched only when a desktop display is available. If no ``DISPLAY`` or ``WAYLAND_DISPLAY`` is present, the filter still publishes the telemetry topics under ``tuning/``.

The Python visualizer also supports a ``sigma_window_seconds`` parameter, which controls the rolling window used for observed standard deviation plots. If it is not provided, the default is 8 seconds.


What Each Plot Means
********************

Top Grid: Mahalanobis Summary
=============================

Each sensor stream gets its own subplot. The blue line is the Mahalanobis distance for each incoming measurement. The orange dashed line is the configured rejection threshold. Red ``x`` markers are rejected measurements.

Use this view to answer:

* Which streams are regularly close to their gate?
* Which streams are producing occasional outliers?
* Which streams are rejected in bursts rather than as isolated spikes?

Do not compare the absolute Mahalanobis distance of a 1D stream directly against a 6D stream. A stream fusing more variables naturally has a larger multivariate distance baseline.


Measurement vs Predicted
========================

This plot compares the observed measurement component against the filter's predicted measurement for the selected stream.

Use it to answer:

* Is the sensor biased?
* Is the filter consistently lagging the measurement?
* Is one component behaving differently from the others in the same stream?


Innovation
==========

The innovation is ``measurement - prediction`` for each fused component. This is the raw residual before gating.

Use it to answer:

* Is the residual centered around zero?
* Is there a repeating oscillation, lag, or bias?
* Are spikes synchronized with a specific maneuver or timestamp problem?


Rolling Observed vs Predicted Sigma
===================================

This is the main covariance-tuning plot.

For each selected component:

* ``obs`` is the rolling observed standard deviation of the innovation
* ``pred`` is the rolling mean of ``sqrt(S_ii)``, where ``S`` is the innovation covariance
* ``norm`` is the rolling standard deviation of the normalized innovation ``innovation / sqrt(S_ii)``

Interpretation:

* ``obs`` close to ``pred`` means the filter's uncertainty model is roughly consistent with reality.
* ``obs`` much larger than ``pred`` means the filter is overconfident.
* ``obs`` much smaller than ``pred`` means the filter is conservative for that component.
* ``norm`` close to 1 means the residual is scaled correctly.

The normalized sigma is the most portable metric across components with different units.


Latest Covariance Diagonals
===========================

This bar chart compares:

* the measurement covariance diagonal
* the innovation covariance diagonal

Use it to answer:

* Is the sensor covariance unrealistically tiny?
* Is one component dominating the update because its covariance is much smaller than the rest?


Filter Covariance Snapshot
==========================

This bar chart shows:

* the current estimate covariance diagonal
* the process noise diagonal

Use it to answer:

* Which state variables the filter currently believes are uncertain
* Whether the process noise is unrealistically small or large for slow-converging states


Suggested Tuning Workflow
*************************

1. Start with conservative gates.

   Use thresholds that are unlikely to reject normal data while you first tune covariances. A rejected measurement does not tell you whether the sensor is bad or the covariance is simply under-modeled.

2. Pick one stream and inspect the selected-stream plots.

   Start with the stream that looks closest to the truth source you trust most.

3. Tune measurement covariance before tightening the gate.

   Use the rolling sigma plot. Aim for ``obs`` and ``pred`` to be in the same neighborhood, and for ``norm`` to settle near 1 over ordinary operating conditions.

4. Remove obvious modeling mistakes before touching thresholds.

   If the innovation has a mean offset, time lag, frame sign error, or angle wrap problem, fix that first. A Mahalanobis threshold should not be used to hide a deterministic error.

5. Set the gate using normal operating peaks, not failure peaks.

   Once the covariance is reasonable, watch the top Mahalanobis summary and the selected innovation plot. Choose a threshold that stays above ordinary peaks and below true outlier bursts.

6. Re-check other streams after every major change.

   One stream's covariance affects the filter state, which changes the innovation seen by other streams.

7. Tune process noise only after the main sensors are roughly matched.

   If the filter reacts too slowly even with good measurement covariances, the process noise may be too small. If the state becomes noisy and overreactive, it may be too large.


Practical Reading Rules
***********************

If ``obs`` is consistently larger than ``pred``:

* increase the relevant measurement covariance upstream, or
* increase process noise if the model is too stiff and cannot follow real motion

If ``obs`` is consistently smaller than ``pred``:

* the measurement covariance may be too large, or
* the process noise may be too large, causing an overly uncertain prediction

If ``norm`` is near 1 but Mahalanobis still spikes:

* the stream may be multi-dimensional, and the combined distance is reacting to joint excursions
* inspect which component spikes first in the innovation plot

If one component in a mixed stream is much worse than the others:

* tune that component's covariance upstream if possible, or
* separate the source into smaller streams if your system architecture allows it

If rejections happen in long bursts:

* suspect timing, frame transforms, stale data, differential mode misuse, or a real bias shift

If rejections are isolated single spikes:

* suspect intermittent outliers, packet corruption, or occasional sensor glitches


Threshold Guidance
******************

The rejection threshold in ``robot_localization`` is applied to the multivariate Mahalanobis distance for the pose, twist, or acceleration group being fused. It is not a direct per-component "3 sigma" test when several variables are fused together.

This means:

* compare a stream mainly against its own normal baseline
* use the per-component normalized sigma plot to judge covariance quality
* use the Mahalanobis summary to decide where the gate should sit for that specific stream

In practice:

* if normal operation sits comfortably below the threshold and outliers clearly exceed it, the threshold is probably fine
* if normal operation repeatedly grazes the threshold, fix covariance modeling before pushing the threshold upward
* if normal operation is very low relative to the threshold, the gate may be looser than needed
