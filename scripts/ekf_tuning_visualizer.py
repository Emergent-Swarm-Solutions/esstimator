#!/usr/bin/env python3

import math
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.widgets import RadioButtons
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from esstimator.msg import FilterStateDiagnostic
from esstimator.msg import InnovationDiagnostic


STATE_LABELS = [
    "X",
    "Y",
    "Z",
    "ROLL",
    "PITCH",
    "YAW",
    "VX",
    "VY",
    "VZ",
    "VROLL",
    "VPITCH",
    "VYAW",
    "AX",
    "AY",
    "AZ",
]

ANGLE_STATE_LABELS = {
    "ROLL",
    "PITCH",
    "YAW",
}

ANGULAR_RATE_STATE_LABELS = {
    "VROLL",
    "VPITCH",
    "VYAW",
}

LINEAR_STATE_LABELS = {
    "X",
    "Y",
    "Z",
    "VX",
    "VY",
    "VZ",
    "AX",
    "AY",
    "AZ",
}

ANGULAR_STATE_LABELS = {
    *ANGLE_STATE_LABELS,
    *ANGULAR_RATE_STATE_LABELS,
}


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def threshold_enabled(threshold: float) -> bool:
    return np.isfinite(threshold) and threshold < 1.0e6


def ellipsize(text: str, max_length: int) -> str:
    if len(text) <= max_length:
        return text
    if max_length <= 3:
        return text[:max_length]
    keep = max_length - 3
    left = keep // 2
    right = keep - left
    return f"{text[:left]}...{text[-right:]}"


def display_scale_for_label(label: str) -> float:
    if label in ANGULAR_STATE_LABELS:
        return 180.0 / math.pi
    return 1.0


def display_unit_for_label(label: str) -> str:
    if label in ANGLE_STATE_LABELS:
        return "deg"
    if label in ANGULAR_RATE_STATE_LABELS:
        return "deg/s"
    if label in {"X", "Y", "Z"}:
        return "m"
    if label in {"VX", "VY", "VZ"}:
        return "m/s"
    if label in {"AX", "AY", "AZ"}:
        return "m/s^2"
    return ""


def scale_series_for_display(label: str, values):
    return np.asarray(values, dtype=float) * display_scale_for_label(label)


def scale_variance_for_display(label: str, values):
    scale = display_scale_for_label(label)
    return np.asarray(values, dtype=float) * scale * scale


class StreamHistory:
    def __init__(self) -> None:
        self.times = deque()
        self.mahalanobis = deque()
        self.thresholds = deque()
        self.accepted = deque()
        self.state_indices = deque()
        self.measurements = deque()
        self.predicted_measurements = deque()
        self.innovations = deque()
        self.measurement_covariances = deque()
        self.innovation_covariances = deque()

    def append(self, msg: InnovationDiagnostic) -> None:
        self.times.append(stamp_to_seconds(msg.stamp))
        self.mahalanobis.append(float(msg.mahalanobis_distance))
        self.thresholds.append(float(msg.mahalanobis_threshold))
        self.accepted.append(bool(msg.accepted))
        self.state_indices.append(tuple(int(index) for index in msg.state_indices))
        self.measurements.append(np.array(msg.measurement, dtype=float))
        self.predicted_measurements.append(np.array(msg.predicted_measurement, dtype=float))
        self.innovations.append(np.array(msg.innovation, dtype=float))
        self.measurement_covariances.append(
            np.array(msg.measurement_covariance_diagonal, dtype=float)
        )
        self.innovation_covariances.append(
            np.array(msg.innovation_covariance_diagonal, dtype=float)
        )

    def prune(self, cutoff_time: float) -> None:
        while self.times and self.times[0] < cutoff_time:
            self.times.popleft()
            self.mahalanobis.popleft()
            self.thresholds.popleft()
            self.accepted.popleft()
            self.state_indices.popleft()
            self.measurements.popleft()
            self.predicted_measurements.popleft()
            self.innovations.popleft()
            self.measurement_covariances.popleft()
            self.innovation_covariances.popleft()

    def latest_indices(self):
        if not self.state_indices:
            return ()
        return self.state_indices[-1]

    def component_labels(self):
        return [STATE_LABELS[index] for index in self.latest_indices()]

    def latest_threshold(self) -> float:
        if not self.thresholds:
            return float("nan")
        return self.thresholds[-1]

    def latest_distance(self) -> float:
        if not self.mahalanobis:
            return float("nan")
        return self.mahalanobis[-1]

    def latest_accepted(self) -> bool:
        if not self.accepted:
            return True
        return self.accepted[-1]


class EkfTuningVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("ekf_tuning_visualizer")

        self.declare_parameter("history_seconds", 60.0)
        self.declare_parameter("sigma_window_seconds", 8.0)
        self.history_seconds = max(
            1.0, float(self.get_parameter("history_seconds").value)
        )
        self.sigma_window_seconds = min(
            self.history_seconds,
            max(1.0, float(self.get_parameter("sigma_window_seconds").value)),
        )

        self.stream_histories = {}
        self.configured_streams = set()
        self.stream_topics = {}
        self.configured_state_indices = {}
        self.selected_stream = None
        self.stream_order = []
        self.stream_metadata_signature = ()
        self.detail_mode = "Overview"

        self.filter_state_stamp = None
        self.filter_state = np.array([])
        self.estimate_covariance_diagonal = np.array([])
        self.process_noise_diagonal = np.array([])

        self.summary_axes_by_stream = {}
        self.summary_axes = []
        self.selector_ax = None
        self.stream_selector = None
        self.selector_label_to_stream = {}
        self.stream_to_selector_label = {}
        self.selector_active_index = None
        self.suppress_selector_callback = False
        self.detail_mode_ax = None
        self.detail_mode_selector = None
        self.detail_axes_positions = {}

        self.selected_info_ax = None
        self.measurement_ax = None
        self.measurement_secondary_ax = None
        self.innovation_ax = None
        self.innovation_secondary_ax = None
        self.sigma_ax = None
        self.sigma_norm_ax = None
        self.gate_metric_ax = None
        self.gate_metric_secondary_ax = None
        self.selected_covariance_ax = None
        self.filter_covariance_ax = None

        streams_qos = QoSProfile(depth=1)
        streams_qos.reliability = ReliabilityPolicy.RELIABLE
        streams_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            InnovationDiagnostic,
            "tuning/innovation_diagnostics",
            self.handle_innovation,
            200,
        )
        self.create_subscription(
            FilterStateDiagnostic,
            "tuning/filter_state",
            self.handle_filter_state,
            20,
        )
        self.create_subscription(
            String,
            "tuning/configured_streams",
            self.handle_configured_streams,
            streams_qos,
        )

        plt.ion()
        self.figure = plt.figure(figsize=(16, 10))
        manager = getattr(self.figure.canvas, "manager", None)
        if manager is not None:
            try:
                manager.set_window_title("esstimator EKF Tuning")
            except Exception:
                pass
        self.figure.canvas.mpl_connect("button_press_event", self.on_click)

        self.rebuild_axes()

    def handle_innovation(self, msg: InnovationDiagnostic) -> None:
        history = self.stream_histories.setdefault(msg.stream_name, StreamHistory())
        history.append(msg)
        self.prune_histories()
        if self.selected_stream is None:
            self.selected_stream = msg.stream_name

    def handle_filter_state(self, msg: FilterStateDiagnostic) -> None:
        self.filter_state_stamp = stamp_to_seconds(msg.stamp)
        self.filter_state = np.array(msg.state, dtype=float)
        self.estimate_covariance_diagonal = np.array(
            msg.estimate_error_covariance_diagonal, dtype=float
        )
        self.process_noise_diagonal = np.array(
            msg.process_noise_covariance_diagonal, dtype=float
        )

    def handle_configured_streams(self, msg: String) -> None:
        configured_streams = set()
        stream_topics = {}
        configured_state_indices = {}

        for raw_line in msg.data.splitlines():
            line = raw_line.strip()
            if not line:
                continue

            fields = line.split("\t")
            stream_name = fields[0] if len(fields) > 0 else ""
            source_topic = fields[1] if len(fields) > 1 else ""
            state_indices_csv = fields[2] if len(fields) > 2 else ""

            stream_name = stream_name.strip()
            source_topic = source_topic.strip()
            if not stream_name:
                continue

            configured_streams.add(stream_name)
            if source_topic:
                stream_topics[stream_name] = source_topic
            if state_indices_csv:
                configured_state_indices[stream_name] = tuple(
                    int(index_text)
                    for index_text in state_indices_csv.split(",")
                    if index_text.strip()
                )

        self.configured_streams = configured_streams
        self.stream_topics = stream_topics
        self.configured_state_indices = configured_state_indices

        if self.selected_stream is None:
            stream_names = self.all_stream_names()
            if stream_names:
                self.selected_stream = stream_names[0]

    def prune_histories(self) -> None:
        latest_time = self.latest_stream_time()
        if latest_time is None:
            return
        cutoff = latest_time - self.history_seconds
        for history in self.stream_histories.values():
            history.prune(cutoff)

    def latest_stream_time(self):
        latest_times = [
            history.times[-1] for history in self.stream_histories.values() if history.times
        ]
        if not latest_times:
            return None
        return max(latest_times)

    def all_stream_names(self):
        return sorted(set(self.stream_histories.keys()) | set(self.configured_streams))

    def current_stream_metadata_signature(self):
        return tuple(
            (
                stream_name,
                self.stream_topics.get(stream_name, ""),
                self.configured_state_indices.get(stream_name, ()),
            )
            for stream_name in self.all_stream_names()
        )

    def source_topic_for(self, stream_name: str) -> str:
        return self.stream_topics.get(stream_name, "")

    def configured_component_labels(self, stream_name: str):
        state_indices = self.configured_state_indices.get(stream_name, ())
        return [
            STATE_LABELS[index]
            for index in state_indices
            if 0 <= index < len(STATE_LABELS)
        ]

    def component_labels_for_stream(self, stream_name: str, history=None):
        if history is not None:
            history_labels = history.component_labels()
            if history_labels:
                return history_labels

        if history is None and stream_name in self.stream_histories:
            history_labels = self.stream_histories[stream_name].component_labels()
            if history_labels:
                return history_labels

        return self.configured_component_labels(stream_name)

    def stream_category_label(self, stream_name: str, component_labels):
        label_set = set(component_labels)
        if label_set and label_set.issubset({"ROLL", "PITCH", "YAW"}):
            return "orientation (RPY)"
        if label_set and label_set.issubset({"VROLL", "VPITCH", "VYAW"}):
            return "angular velocity"
        if label_set and label_set.issubset({"AX", "AY", "AZ"}):
            return "linear acceleration"
        if label_set and label_set.issubset({"X", "Y", "Z"}):
            return "position"
        if label_set and label_set.issubset({"VX", "VY", "VZ"}):
            return "linear velocity"

        suffix = stream_name.split("_", 1)[1] if "_" in stream_name else ""
        return {
            "pose": "pose",
            "twist": "twist",
            "acceleration": "acceleration",
        }.get(suffix, stream_name)

    def stream_display_name(self, stream_name: str, history=None) -> str:
        sensor_name = stream_name.split("_", 1)[0]
        component_labels = self.component_labels_for_stream(stream_name, history)
        category = self.stream_category_label(stream_name, component_labels)

        if category == stream_name:
            return stream_name
        if component_labels:
            return f"{sensor_name} {category} [{', '.join(component_labels)}]"
        return f"{sensor_name} {category}"

    def stream_brief_name(self, stream_name: str, history=None) -> str:
        sensor_name = stream_name.split("_", 1)[0]
        component_labels = self.component_labels_for_stream(stream_name, history)
        category = self.stream_category_label(stream_name, component_labels)
        category = {
            "orientation (RPY)": "orientation",
            "angular velocity": "ang vel",
            "linear acceleration": "lin accel",
        }.get(category, category)

        if category == stream_name:
            return stream_name
        return f"{sensor_name} {category}"

    def stream_heading(self, stream_name: str, history=None) -> str:
        display_name = self.stream_display_name(stream_name, history)
        source_topic = self.source_topic_for(stream_name)
        if source_topic:
            return f"{display_name} ({source_topic})"
        return display_name

    def selector_label_for_stream(self, stream_name: str) -> str:
        return self.stream_brief_name(stream_name)

    def stream_components_text(self, stream_name: str, history=None) -> str:
        component_labels = self.component_labels_for_stream(stream_name, history)
        if not component_labels:
            return "not available yet"
        return ", ".join(component_labels)

    def axis_label_for_component_family(self, component_labels, family: str) -> str:
        if family == "angular":
            label_set = set(component_labels)
            if label_set and label_set.issubset(ANGLE_STATE_LABELS):
                return "Angle (deg)"
            if label_set and label_set.issubset(ANGULAR_RATE_STATE_LABELS):
                return "Angular rate (deg/s)"
            return "Angular quantity (deg or deg/s)"

        label_set = set(component_labels)
        if label_set and label_set.issubset({"X", "Y", "Z"}):
            return "Position (m)"
        if label_set and label_set.issubset({"VX", "VY", "VZ"}):
            return "Linear velocity (m/s)"
        if label_set and label_set.issubset({"AX", "AY", "AZ"}):
            return "Linear acceleration (m/s^2)"
        return "Linear quantity"

    def display_unit_for_component_group(self, component_labels) -> str:
        label_set = set(component_labels)
        if label_set and label_set.issubset(ANGLE_STATE_LABELS):
            return "deg"
        if label_set and label_set.issubset(ANGULAR_RATE_STATE_LABELS):
            return "deg/s"
        if label_set and label_set.issubset({"X", "Y", "Z"}):
            return "m"
        if label_set and label_set.issubset({"VX", "VY", "VZ"}):
            return "m/s"
        if label_set and label_set.issubset({"AX", "AY", "AZ"}):
            return "m/s^2"
        return "mixed units"

    def on_click(self, event) -> None:
        axis = event.inaxes
        if axis in self.summary_axes_by_stream:
            self.selected_stream = self.summary_axes_by_stream[axis]

    def on_selector_change(self, label: str) -> None:
        if self.suppress_selector_callback:
            return
        self.selected_stream = self.selector_label_to_stream.get(label, label)
        if self.selected_stream in self.stream_order:
            self.selector_active_index = self.stream_order.index(self.selected_stream)

    def on_detail_mode_change(self, label: str) -> None:
        if label == self.detail_mode:
            return
        self.detail_mode = label
        self.rebuild_axes()

    def rebuild_axes(self) -> None:
        self.figure.clf()
        self.summary_axes_by_stream = {}
        self.summary_axes = []
        self.selector_ax = None
        self.stream_selector = None
        self.selector_label_to_stream = {}
        self.stream_to_selector_label = {}
        self.selector_active_index = None
        self.selected_info_ax = None
        self.detail_mode_ax = None
        self.detail_mode_selector = None
        self.detail_axes_positions = {}

        stream_names = self.all_stream_names()
        stream_count = max(1, len(stream_names))
        summary_columns = min(3, stream_count)
        summary_rows = int(math.ceil(stream_count / summary_columns))
        summary_height = 1.05 + 0.45 * max(summary_rows - 1, 0)

        outer_grid = self.figure.add_gridspec(
            4,
            1,
            height_ratios=[summary_height, 0.42, 1.85, 0.92],
            hspace=0.50,
        )

        if stream_names:
            summary_grid = outer_grid[0].subgridspec(
                summary_rows, summary_columns, hspace=0.55, wspace=0.30
            )
            for stream_index, stream_name in enumerate(stream_names):
                axis = self.figure.add_subplot(
                    summary_grid[
                        stream_index // summary_columns, stream_index % summary_columns
                    ]
                )
                self.summary_axes.append(axis)
                self.summary_axes_by_stream[axis] = stream_name
        else:
            self.summary_axes.append(self.figure.add_subplot(outer_grid[0]))

        self.selected_info_ax = self.figure.add_subplot(outer_grid[1])
        self.selected_info_ax.axis("off")

        detail_grid = outer_grid[2].subgridspec(2, 2, hspace=0.55, wspace=0.32)
        self.measurement_ax = self.figure.add_subplot(detail_grid[0, 0])
        self.measurement_secondary_ax = self.measurement_ax.twinx()
        self.innovation_ax = self.figure.add_subplot(detail_grid[0, 1])
        self.innovation_secondary_ax = self.innovation_ax.twinx()
        self.sigma_ax = self.figure.add_subplot(detail_grid[1, 0])
        self.sigma_norm_ax = self.sigma_ax.twinx()
        self.selected_covariance_ax = self.figure.add_subplot(detail_grid[1, 1])
        self.gate_metric_ax = self.figure.add_subplot(detail_grid[1, 1], label="gate_metric")
        self.gate_metric_secondary_ax = self.gate_metric_ax.twinx()
        self.filter_covariance_ax = self.figure.add_subplot(outer_grid[3])
        self.detail_axes_positions = {
            "measurement": self.measurement_ax.get_position().frozen(),
            "innovation": self.innovation_ax.get_position().frozen(),
            "sigma": self.sigma_ax.get_position().frozen(),
            "gate_metric": self.gate_metric_ax.get_position().frozen(),
            "covariance": self.selected_covariance_ax.get_position().frozen(),
        }
        self.figure.suptitle(
            "esstimator EKF Tuning",
            fontsize=15,
        )
        self.stream_order = stream_names
        self.stream_metadata_signature = self.current_stream_metadata_signature()
        self.figure.tight_layout(rect=(0.03, 0.03, 0.80, 0.95))
        self.build_selector_widget(stream_names)
        self.build_detail_mode_widget()
        self.apply_detail_mode_layout()

    def build_selector_widget(self, stream_names) -> None:
        if not stream_names:
            return

        if self.selected_stream not in stream_names:
            self.selected_stream = stream_names[0]

        selector_height = min(0.34, max(0.12, 0.035 * len(stream_names) + 0.05))
        selector_bottom = 0.93 - selector_height
        self.selector_ax = self.figure.add_axes([0.82, selector_bottom, 0.17, selector_height])
        self.selector_ax.set_title("Configured Streams", fontsize=10)
        self.selector_ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

        labels = []
        for stream_name in stream_names:
            label = self.selector_label_for_stream(stream_name)
            labels.append(label)
            self.selector_label_to_stream[label] = stream_name
            self.stream_to_selector_label[stream_name] = label

        active_index = stream_names.index(self.selected_stream)
        self.stream_selector = RadioButtons(self.selector_ax, labels, active=active_index)
        for label in self.stream_selector.labels:
            label.set_fontsize(8.5)

        self.stream_selector.on_clicked(self.on_selector_change)
        self.selector_active_index = active_index

    def build_detail_mode_widget(self) -> None:
        detail_modes = [
            "Overview",
            "Measurement",
            "Innovation",
            "Sigma",
            "Gate Metric",
            "Covariance",
        ]
        self.detail_mode_ax = self.figure.add_axes([0.82, 0.08, 0.17, 0.22])
        self.detail_mode_ax.set_title("Detail View", fontsize=10)
        self.detail_mode_ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

        active_index = detail_modes.index(self.detail_mode)
        self.detail_mode_selector = RadioButtons(
            self.detail_mode_ax, detail_modes, active=active_index
        )
        for label in self.detail_mode_selector.labels:
            label.set_fontsize(8.5)

        self.detail_mode_selector.on_clicked(self.on_detail_mode_change)

    def apply_detail_mode_layout(self) -> None:
        if not self.detail_axes_positions:
            return

        axes = {
            "measurement": self.measurement_ax,
            "innovation": self.innovation_ax,
            "sigma": self.sigma_ax,
            "gate_metric": self.gate_metric_ax,
            "covariance": self.selected_covariance_ax,
        }
        twin_axes = {
            "measurement": self.measurement_secondary_ax,
            "innovation": self.innovation_secondary_ax,
            "sigma": self.sigma_norm_ax,
            "gate_metric": self.gate_metric_secondary_ax,
        }

        if self.detail_mode == "Overview":
            for axis_name, axis in axes.items():
                axis.set_visible(axis_name != "covariance")
                axis.set_position(self.detail_axes_positions[axis_name])
            for axis_name, twin_axis in twin_axes.items():
                twin_axis.set_position(self.detail_axes_positions[axis_name])
                twin_axis.set_visible(axis_name == "gate_metric")
            self.selected_covariance_ax.set_visible(False)
            return

        left = min(position.x0 for position in self.detail_axes_positions.values())
        bottom = min(position.y0 for position in self.detail_axes_positions.values())
        right = max(position.x1 for position in self.detail_axes_positions.values())
        top = max(position.y1 for position in self.detail_axes_positions.values())
        expanded_bounds = [left, bottom, right - left, top - bottom]
        visible_axis_name = {
            "Measurement": "measurement",
            "Innovation": "innovation",
            "Sigma": "sigma",
            "Gate Metric": "gate_metric",
            "Covariance": "covariance",
        }.get(self.detail_mode, "measurement")

        for axis_name, axis in axes.items():
            axis.set_position(
                expanded_bounds if axis_name == visible_axis_name else self.detail_axes_positions[axis_name]
            )
            axis.set_visible(axis_name == visible_axis_name)

        for axis_name, twin_axis in twin_axes.items():
            twin_axis.set_position(
                expanded_bounds
                if axis_name == visible_axis_name
                else self.detail_axes_positions[axis_name]
            )
            twin_axis.set_visible(axis_name == visible_axis_name)

    def sync_selector_to_selection(self) -> None:
        if self.stream_selector is None:
            return
        if self.selected_stream not in self.stream_order:
            return

        desired_index = self.stream_order.index(self.selected_stream)
        if self.selector_active_index == desired_index:
            return

        self.suppress_selector_callback = True
        self.stream_selector.set_active(desired_index)
        self.suppress_selector_callback = False
        self.selector_active_index = desired_index

    def redraw(self) -> None:
        self.prune_histories()

        stream_names = self.all_stream_names()
        metadata_signature = self.current_stream_metadata_signature()
        if stream_names != self.stream_order or metadata_signature != self.stream_metadata_signature:
            self.rebuild_axes()

        if self.selected_stream not in self.all_stream_names() and self.all_stream_names():
            self.selected_stream = self.all_stream_names()[0]

        self.sync_selector_to_selection()

        latest_time = self.latest_stream_time()
        self.draw_summary_axes(latest_time)
        self.draw_selected_stream_info()
        self.draw_selected_stream(latest_time)
        self.draw_filter_state_covariance()
        self.figure.canvas.draw_idle()

    def draw_selected_stream_info(self) -> None:
        self.selected_info_ax.clear()
        self.selected_info_ax.axis("off")

        stream_names = self.all_stream_names()
        if self.selected_stream not in stream_names:
            self.selected_info_ax.text(
                0.01,
                0.50,
                "Select a stream from the right-hand list or click a summary plot.",
                ha="left",
                va="center",
                fontsize=10,
                color="0.35",
                transform=self.selected_info_ax.transAxes,
            )
            return

        history = self.stream_histories.get(self.selected_stream)
        brief_name = self.stream_brief_name(self.selected_stream, history)
        source_topic = self.source_topic_for(self.selected_stream) or "not reported"
        components_text = self.stream_components_text(self.selected_stream, history)

        if history is None or not history.times:
            status_text = "No innovation diagnostics received yet"
        else:
            accepted_text = "accepted" if history.latest_accepted() else "rejected"
            latest_threshold = history.latest_threshold()
            gate_text = (
                f"{latest_threshold:.2f}" if threshold_enabled(latest_threshold) else "disabled"
            )
            status_text = (
                f"Latest gate: d={history.latest_distance():.2f} / "
                f"{gate_text} | {accepted_text} | "
                f"samples={len(history.times)}"
            )

        info_text = (
            f"Selected stream: {brief_name}\n"
            f"Topic: {source_topic}\n"
            f"Components: {components_text}\n"
            f"{status_text}"
        )
        self.selected_info_ax.text(
            0.01,
            0.50,
            info_text,
            ha="left",
            va="center",
            fontsize=10,
            transform=self.selected_info_ax.transAxes,
            bbox=dict(boxstyle="round,pad=0.4", facecolor="#f8f8f8", edgecolor="0.80"),
        )

    def draw_summary_axes(self, latest_time) -> None:
        stream_names = self.all_stream_names()
        if not stream_names:
            axis = self.summary_axes[0]
            axis.clear()
            axis.set_title("Waiting for innovation diagnostics")
            axis.text(
                0.5,
                0.5,
                "Enable tuning_visualizer_enabled in the EKF YAML\nand start the filter.",
                ha="center",
                va="center",
                transform=axis.transAxes,
            )
            axis.set_xticks([])
            axis.set_yticks([])
            return

        for axis, stream_name in self.summary_axes_by_stream.items():
            axis.clear()
            history = self.stream_histories.get(stream_name)
            has_samples = history is not None and bool(history.times)
            brief_name = self.stream_brief_name(stream_name, history)
            component_text = self.stream_components_text(stream_name, history)

            title = brief_name
            if stream_name == self.selected_stream:
                title += " [selected]"
            source_topic = ellipsize(self.source_topic_for(stream_name), 42)

            if not has_samples:
                axis.set_facecolor("#f2f2f2")
                if stream_name == self.selected_stream:
                    axis.set_facecolor("#e8f1fb")
                axis.set_title(title, fontsize=10, loc="left")
                if source_topic:
                    axis.set_title(source_topic, fontsize=8, color="0.35", loc="right")
                axis.text(
                    0.5,
                    0.56,
                    "No diagnostics yet",
                    ha="center",
                    va="center",
                    transform=axis.transAxes,
                    fontsize=10,
                    color="0.35",
                )
                axis.text(
                    0.5,
                    0.36,
                    f"Components: {component_text}",
                    ha="center",
                    va="center",
                    transform=axis.transAxes,
                    fontsize=8.5,
                    color="0.45",
                )
                axis.set_xticks([])
                axis.set_yticks([])
                for spine in axis.spines.values():
                    spine.set_color("0.78")
                    spine.set_linewidth(1.5 if stream_name == self.selected_stream else 1.0)
                continue

            axis.set_facecolor("white")
            if stream_name == self.selected_stream:
                axis.set_facecolor("#eef6ff")
            for spine in axis.spines.values():
                spine.set_color("#4c78a8" if stream_name == self.selected_stream else "0.75")
                spine.set_linewidth(1.5 if stream_name == self.selected_stream else 0.9)

            times = np.array(history.times, dtype=float)
            relative_times = times - latest_time
            distances = np.array(history.mahalanobis, dtype=float)
            threshold = history.latest_threshold()

            axis.plot(relative_times, distances, color="tab:blue", linewidth=1.6)
            rejected_mask = np.logical_not(np.array(history.accepted, dtype=bool))
            if np.any(rejected_mask):
                axis.scatter(
                    relative_times[rejected_mask],
                    distances[rejected_mask],
                    color="tab:red",
                    marker="x",
                    s=28,
                )
            if threshold_enabled(threshold):
                data_peak = max(float(np.max(distances)), 1.0e-6)
                if threshold <= max(1.0, data_peak * 4.0):
                    axis.axhline(
                        threshold, color="tab:orange", linestyle="--", linewidth=1.2
                    )
                else:
                    axis.text(
                        0.99,
                        0.98,
                        f"gate={threshold:.2f}",
                        transform=axis.transAxes,
                        ha="right",
                        va="top",
                        fontsize=8,
                        color="tab:orange",
                    )

            axis.set_title(title, fontsize=10, loc="left")
            if source_topic:
                axis.set_title(source_topic, fontsize=8, color="0.35", loc="right")
            axis.text(
                0.01,
                0.02,
                component_text,
                transform=axis.transAxes,
                ha="left",
                va="bottom",
                fontsize=7.5,
                color="0.40",
            )
            axis.set_xlim(-self.history_seconds, 0.0)
            axis.grid(True, alpha=0.25)
            axis.set_xlabel("Time (s)")
            axis.set_ylabel("Mahalanobis")
            ymax = max(np.max(distances) * 1.2, 0.1)
            axis.set_ylim(0.0, ymax)

    def draw_selected_stream(self, latest_time) -> None:
        stream_names = self.all_stream_names()
        if self.selected_stream not in stream_names:
            if stream_names:
                self.selected_stream = stream_names[0]
            else:
                self.clear_axis_with_message(
                    self.measurement_ax, "Selected stream", "No stream selected yet.",
                    secondary_axes=[self.measurement_secondary_ax]
                )
                self.clear_axis_with_message(
                    self.innovation_ax, "Innovation", "Waiting for sensor updates.",
                    secondary_axes=[self.innovation_secondary_ax]
                )
                self.clear_axis_with_message(
                    self.sigma_ax,
                    "Rolling Sigma",
                    "Waiting for sensor updates.",
                    secondary_axes=[self.sigma_norm_ax],
                )
                self.clear_axis_with_message(
                    self.gate_metric_ax,
                    "Branch Gate Metric",
                    "Waiting for sensor updates.",
                    secondary_axes=[self.gate_metric_secondary_ax],
                )
                self.clear_axis_with_message(
                    self.selected_covariance_ax, "Selected Covariance", "Waiting for sensor updates."
                )
                return

        history = self.stream_histories.get(self.selected_stream)
        if history is None or not history.times:
            waiting_message = (
                "Configured, but no innovation diagnostics have arrived.\n"
                "Check whether this stream is actually being fused."
            )
            self.clear_axis_with_message(
                self.measurement_ax,
                "Measurement vs Predicted",
                waiting_message,
                secondary_axes=[self.measurement_secondary_ax],
            )
            self.clear_axis_with_message(
                self.innovation_ax,
                "Innovation",
                waiting_message,
                secondary_axes=[self.innovation_secondary_ax],
            )
            self.clear_axis_with_message(
                self.sigma_ax,
                "Innovation Sigma",
                waiting_message,
                secondary_axes=[self.sigma_norm_ax],
            )
            self.clear_axis_with_message(
                self.gate_metric_ax,
                "Branch Gate Metric",
                waiting_message,
                secondary_axes=[self.gate_metric_secondary_ax],
            )
            self.clear_axis_with_message(
                self.selected_covariance_ax,
                "Latest Covariance Diagonals",
                waiting_message,
            )
            return

        labels = self.component_labels_for_stream(self.selected_stream, history)
        if not labels:
            self.clear_axis_with_message(
                self.measurement_ax,
                "Measurement vs Predicted",
                "No component data yet.",
                secondary_axes=[self.measurement_secondary_ax],
            )
            self.clear_axis_with_message(
                self.innovation_ax,
                "Innovation",
                "No component data yet.",
                secondary_axes=[self.innovation_secondary_ax],
            )
            self.clear_axis_with_message(
                self.sigma_ax,
                "Innovation Sigma",
                "No component data yet.",
                secondary_axes=[self.sigma_norm_ax],
            )
            self.clear_axis_with_message(
                self.gate_metric_ax,
                "Branch Gate Metric",
                "No component data yet.",
                secondary_axes=[self.gate_metric_secondary_ax],
            )
            self.clear_axis_with_message(
                self.selected_covariance_ax, "Latest Covariance Diagonals", "No covariance data yet."
            )
            return

        relative_times = np.array(history.times, dtype=float) - latest_time
        absolute_times = np.array(history.times, dtype=float)
        component_count = len(labels)
        colors = plt.cm.tab10(np.linspace(0.0, 1.0, max(component_count, 1)))
        has_linear = any(label in LINEAR_STATE_LABELS for label in labels)
        has_angular = any(label in ANGULAR_STATE_LABELS for label in labels)

        self.measurement_ax.clear()
        self.measurement_secondary_ax.clear()
        self.measurement_secondary_ax.set_visible(False)
        self.innovation_ax.clear()
        self.innovation_secondary_ax.clear()
        self.innovation_secondary_ax.set_visible(False)
        self.sigma_ax.clear()
        self.sigma_norm_ax.clear()
        self.gate_metric_ax.clear()
        self.gate_metric_secondary_ax.clear()
        self.selected_covariance_ax.clear()

        for component_index, label in enumerate(labels):
            measurement_series = scale_series_for_display(
                label,
                self.component_series(
                history.measurements, history.state_indices, component_index, component_count
                ),
            )
            predicted_series = scale_series_for_display(
                label,
                self.component_series(
                history.predicted_measurements,
                history.state_indices,
                component_index,
                component_count,
                ),
            )
            innovation_series = scale_series_for_display(
                label,
                self.component_series(
                history.innovations, history.state_indices, component_index, component_count
                ),
            )
            innovation_covariance_series = scale_variance_for_display(
                label,
                self.component_series(
                history.innovation_covariances,
                history.state_indices,
                component_index,
                component_count,
                ),
            )
            predicted_sigma_samples = np.sqrt(
                np.clip(innovation_covariance_series, a_min=0.0, a_max=None)
            )
            observed_sigma_series = self.rolling_window_std(
                absolute_times,
                innovation_series,
                self.sigma_window_seconds,
            )
            predicted_sigma_series = self.rolling_window_mean(
                absolute_times,
                predicted_sigma_samples,
                self.sigma_window_seconds,
            )

            measurement_axis = self.detail_axis_for_component(
                self.measurement_ax,
                self.measurement_secondary_ax,
                label,
                has_linear,
                has_angular,
            )
            innovation_axis = self.detail_axis_for_component(
                self.innovation_ax,
                self.innovation_secondary_ax,
                label,
                has_linear,
                has_angular,
            )

            measurement_axis.plot(
                relative_times,
                measurement_series,
                color=colors[component_index],
                linewidth=1.4,
                alpha=0.85,
                label=label,
                zorder=2,
            )
            measurement_axis.plot(
                relative_times,
                predicted_series,
                color=colors[component_index],
                linestyle="--",
                linewidth=1.8,
                label="_nolegend_",
                zorder=4,
            )
            innovation_axis.plot(
                relative_times,
                innovation_series,
                color=colors[component_index],
                linewidth=1.5,
                label=label,
            )
            self.sigma_ax.plot(
                relative_times,
                observed_sigma_series,
                color=colors[component_index],
                linewidth=1.6,
                label=label,
            )
            self.sigma_ax.plot(
                relative_times,
                predicted_sigma_series,
                color=colors[component_index],
                linestyle="--",
                linewidth=1.7,
                label="_nolegend_",
            )
        legend_columns = min(3, max(1, int(math.ceil(component_count / 2.0))))

        self.measurement_ax.set_title("Measurement vs Predicted", fontsize=11, loc="left")
        self.measurement_ax.set_title(
            "solid = meas | dashed = pred",
            fontsize=8,
            color="0.35",
            loc="right",
        )
        self.measurement_ax.set_xlim(-self.history_seconds, 0.0)
        self.measurement_ax.grid(True, alpha=0.25)
        self.measurement_ax.set_xlabel("Time (s)")
        if has_linear and has_angular:
            self.measurement_ax.set_ylabel("Linear quantity")
            self.measurement_secondary_ax.set_visible(True)
            angular_labels = [label for label in labels if label in ANGULAR_STATE_LABELS]
            self.measurement_secondary_ax.set_ylabel(
                self.axis_label_for_component_family(angular_labels, "angular")
            )
        elif has_angular:
            self.measurement_ax.set_ylabel(
                self.axis_label_for_component_family(labels, "angular")
            )
        else:
            self.measurement_ax.set_ylabel(
                self.axis_label_for_component_family(labels, "linear")
            )
        self.apply_combined_legend(
            self.measurement_ax,
            self.measurement_secondary_ax,
            fontsize=8,
            ncol=legend_columns,
        )
        self.autoscale_axis_from_lines(self.measurement_ax)
        self.autoscale_axis_from_lines(self.measurement_secondary_ax)

        self.innovation_ax.set_title("Innovation", fontsize=11, loc="left")
        self.innovation_ax.set_xlim(-self.history_seconds, 0.0)
        self.innovation_ax.grid(True, alpha=0.25)
        self.innovation_ax.set_xlabel("Time (s)")
        if has_linear and has_angular:
            self.innovation_ax.set_ylabel("Linear innovation")
            self.innovation_secondary_ax.set_visible(True)
            angular_labels = [label for label in labels if label in ANGULAR_STATE_LABELS]
            angular_unit = self.display_unit_for_component_group(angular_labels)
            self.innovation_secondary_ax.set_ylabel(f"Angular innovation ({angular_unit})")
        elif has_angular:
            angular_unit = self.display_unit_for_component_group(labels)
            self.innovation_ax.set_ylabel(f"Innovation ({angular_unit})")
        else:
            linear_unit = self.display_unit_for_component_group(labels)
            if linear_unit == "mixed units":
                self.innovation_ax.set_ylabel("Innovation")
            else:
                self.innovation_ax.set_ylabel(f"Innovation ({linear_unit})")
        self.apply_combined_legend(
            self.innovation_ax,
            self.innovation_secondary_ax,
            fontsize=8,
            ncol=legend_columns,
        )
        self.autoscale_axis_from_lines(self.innovation_ax, include_zero=True)
        self.autoscale_axis_from_lines(self.innovation_secondary_ax, include_zero=True)

        self.sigma_ax.set_title("Innovation Sigma", fontsize=11, loc="left")
        self.sigma_ax.set_title(
            f"solid = obs | dashed = pred ({self.sigma_window_seconds:.1f}s)",
            fontsize=8,
            color="0.35",
            loc="right",
        )
        self.sigma_ax.set_xlim(-self.history_seconds, 0.0)
        self.sigma_ax.grid(True, alpha=0.25)
        self.sigma_ax.set_xlabel("Time (s)")
        sigma_unit = self.display_unit_for_component_group(labels)
        if sigma_unit != "mixed units":
            self.sigma_ax.set_ylabel(f"Sigma ({sigma_unit})")
        else:
            self.sigma_ax.set_ylabel("Sigma")
        self.sigma_norm_ax.set_visible(False)
        if has_linear and has_angular:
            self.sigma_ax.text(
                0.01,
                0.02,
                "Linear and angular sigmas share the left axis. Compare within family.",
                transform=self.sigma_ax.transAxes,
                ha="left",
                va="bottom",
                fontsize=8,
                color="0.35",
            )
        self.sigma_ax.legend(
            *self.sigma_ax.get_legend_handles_labels(),
            loc="upper left",
            fontsize=8,
            ncol=legend_columns,
        )
        self.configure_sigma_axis_scale()

        display_innovations = np.column_stack(
            [
                scale_series_for_display(
                    labels[component_index],
                    self.component_series(
                        history.innovations,
                        history.state_indices,
                        component_index,
                        component_count,
                    ),
                )
                for component_index in range(component_count)
            ]
        )
        innovation_norm_series = np.linalg.norm(display_innovations, axis=1)
        mahalanobis_series = np.array(history.mahalanobis, dtype=float)
        rejected_mask = np.logical_not(np.array(history.accepted, dtype=bool))

        self.gate_metric_ax.set_title(
            "Full Innovation / Normalized Innovation", fontsize=11, loc="left"
        )
        self.gate_metric_ax.set_title(
            "solid = normalized innovation | dashed = full innovation norm",
            fontsize=8,
            color="0.35",
            loc="right",
        )
        self.gate_metric_ax.plot(
            relative_times,
            mahalanobis_series,
            color="tab:red",
            linewidth=1.8,
            label="Normalized innovation",
        )
        self.gate_metric_secondary_ax.plot(
            relative_times,
            innovation_norm_series,
            color="tab:blue",
            linestyle="--",
            linewidth=1.6,
            label="Full innovation norm",
        )
        if threshold_enabled(history.latest_threshold()):
            normalized_peak = max(float(np.nanmax(mahalanobis_series)), 1.0e-6)
            threshold_value = history.latest_threshold()
            if threshold_value <= max(1.0, normalized_peak * 4.0):
                self.gate_metric_ax.axhline(
                    threshold_value,
                    color="tab:orange",
                    linestyle=":",
                    linewidth=1.2,
                    label="Threshold",
                )
            else:
                self.gate_metric_ax.text(
                    0.99,
                    0.98,
                    f"gate={threshold_value:.2f}",
                    transform=self.gate_metric_ax.transAxes,
                    ha="right",
                    va="top",
                    fontsize=8,
                    color="tab:orange",
                )
        if np.any(rejected_mask):
            self.gate_metric_ax.scatter(
                relative_times[rejected_mask],
                mahalanobis_series[rejected_mask],
                color="tab:red",
                marker="x",
                s=28,
                zorder=5,
            )
        self.gate_metric_ax.set_xlim(-self.history_seconds, 0.0)
        self.gate_metric_ax.grid(True, alpha=0.25)
        self.gate_metric_ax.set_xlabel("Time (s)")
        self.gate_metric_ax.set_ylabel("Normalized innovation / gate (n-sigma)")
        self.gate_metric_secondary_ax.set_ylabel(
            f"Full innovation norm ({self.display_unit_for_component_group(labels)})"
        )
        self.autoscale_axis_from_lines(self.gate_metric_ax, include_zero=True)
        self.autoscale_axis_from_lines(self.gate_metric_secondary_ax, include_zero=True)
        gate_handles, gate_labels = self.gate_metric_ax.get_legend_handles_labels()
        gate_metric_handles, gate_metric_labels = self.gate_metric_secondary_ax.get_legend_handles_labels()
        self.gate_metric_ax.legend(
            gate_handles + gate_metric_handles,
            gate_labels + gate_metric_labels,
            loc="upper left",
            fontsize=8,
            ncol=2,
        )

        latest_measurement_cov = history.measurement_covariances[-1]
        latest_innovation_cov = history.innovation_covariances[-1]
        latest_measurement_cov_display = np.array(
            [
                scale_variance_for_display(label, [latest_measurement_cov[index]])[0]
                for index, label in enumerate(labels)
            ],
            dtype=float,
        )
        latest_innovation_cov_display = np.array(
            [
                scale_variance_for_display(label, [latest_innovation_cov[index]])[0]
                for index, label in enumerate(labels)
            ],
            dtype=float,
        )
        positions = np.arange(component_count)
        width = 0.38
        self.selected_covariance_ax.bar(
            positions - width / 2.0,
            latest_measurement_cov_display,
            width=width,
            color="tab:green",
            label="Measurement cov diag",
        )
        self.selected_covariance_ax.bar(
            positions + width / 2.0,
            latest_innovation_cov_display,
            width=width,
            color="tab:purple",
            label="Innovation cov diag",
        )
        self.selected_covariance_ax.set_title("Latest Covariance Diagonals", fontsize=11, loc="left")
        self.selected_covariance_ax.set_xticks(positions)
        self.selected_covariance_ax.set_xticklabels(labels, rotation=30, ha="right")
        self.selected_covariance_ax.set_ylabel("Covariance diagonal (display units^2)")
        self.selected_covariance_ax.grid(True, axis="y", alpha=0.25)
        self.selected_covariance_ax.legend(loc="upper left", fontsize=8)

    def draw_filter_state_covariance(self) -> None:
        self.filter_covariance_ax.clear()
        if self.estimate_covariance_diagonal.size == 0:
            self.clear_axis_with_message(
                self.filter_covariance_ax,
                "Filter Covariance",
                "Waiting for periodic filter state diagnostics.",
            )
            return

        state_count = min(len(STATE_LABELS), self.estimate_covariance_diagonal.size)
        positions = np.arange(state_count)
        width = 0.38
        estimate_covariance_display = np.array(
            [
                scale_variance_for_display(STATE_LABELS[index], [self.estimate_covariance_diagonal[index]])[0]
                for index in range(state_count)
            ],
            dtype=float,
        )
        self.filter_covariance_ax.bar(
            positions - width / 2.0,
            estimate_covariance_display,
            width=width,
            color="tab:blue",
            label="Estimate cov diag",
        )
        if self.process_noise_diagonal.size >= state_count:
            process_noise_display = np.array(
                [
                    scale_variance_for_display(STATE_LABELS[index], [self.process_noise_diagonal[index]])[0]
                    for index in range(state_count)
                ],
                dtype=float,
            )
            self.filter_covariance_ax.bar(
                positions + width / 2.0,
                process_noise_display,
                width=width,
                color="tab:orange",
                label="Process noise diag",
            )
        self.filter_covariance_ax.set_title("Filter Covariance Snapshot", fontsize=11, loc="left")
        self.filter_covariance_ax.set_xticks(positions)
        self.filter_covariance_ax.set_xticklabels(STATE_LABELS[:state_count], rotation=35, ha="right")
        self.filter_covariance_ax.set_ylabel("Covariance diagonal (display units^2)")
        self.filter_covariance_ax.grid(True, axis="y", alpha=0.25)
        self.filter_covariance_ax.legend(loc="upper left", fontsize=8)

    def component_series(self, sample_deque, index_deque, component_index: int, expected_size: int):
        series = []
        for sample, indices in zip(sample_deque, index_deque):
            if len(indices) != expected_size or sample.size <= component_index:
                series.append(np.nan)
            else:
                series.append(sample[component_index])
        return np.array(series, dtype=float)

    def rolling_window_std(self, times, values, window_seconds: float):
        output = []
        values = np.array(values, dtype=float)
        finite_mask = np.isfinite(values)
        for current_time in times:
            window_mask = (
                (times >= current_time - window_seconds)
                & (times <= current_time)
                & finite_mask
            )
            window_values = values[window_mask]
            if window_values.size == 0:
                output.append(np.nan)
            else:
                output.append(float(np.std(window_values)))
        return np.array(output, dtype=float)

    def rolling_window_mean(self, times, values, window_seconds: float):
        output = []
        values = np.array(values, dtype=float)
        finite_mask = np.isfinite(values)
        for current_time in times:
            window_mask = (
                (times >= current_time - window_seconds)
                & (times <= current_time)
                & finite_mask
            )
            window_values = values[window_mask]
            if window_values.size == 0:
                output.append(np.nan)
            else:
                output.append(float(np.mean(window_values)))
        return np.array(output, dtype=float)

    def autoscale_axis_from_lines(
        self,
        axis,
        include_zero: bool = False,
        min_span: float = 1.0e-3,
        pad_ratio: float = 0.12,
    ) -> None:
        if axis is None or not axis.get_visible():
            return

        values = []
        for line in axis.get_lines():
            line_values = np.asarray(line.get_ydata(), dtype=float)
            if line_values.size > 0:
                values.append(line_values)

        if not values:
            return

        combined_values = np.concatenate(values)
        finite_values = combined_values[np.isfinite(combined_values)]
        if finite_values.size == 0:
            return

        y_min = float(np.min(finite_values))
        y_max = float(np.max(finite_values))
        if include_zero:
            y_min = min(y_min, 0.0)
            y_max = max(y_max, 0.0)

        if math.isclose(y_min, y_max):
            padding = max(abs(y_max) * 0.25, min_span)
        else:
            padding = max((y_max - y_min) * pad_ratio, min_span)

        axis.set_ylim(y_min - padding, y_max + padding)

    def configure_sigma_axis_scale(self) -> None:
        sigma_values = []
        predicted_sigma_values = []
        observed_sigma_values = []

        for line in self.sigma_ax.get_lines():
            values = np.asarray(line.get_ydata(), dtype=float)
            finite_values = values[np.isfinite(values)]
            if finite_values.size == 0:
                continue
            sigma_values.append(finite_values)
            if line.get_linestyle() == "--":
                predicted_sigma_values.append(finite_values)
            else:
                observed_sigma_values.append(finite_values)

        if not sigma_values:
            return

        sigma_values = np.concatenate(sigma_values)
        positive_sigma_values = sigma_values[sigma_values > 0.0]
        if positive_sigma_values.size == 0:
            self.sigma_ax.set_yscale("linear")
            self.sigma_ax.set_ylabel("Sigma")
            self.autoscale_axis_from_lines(self.sigma_ax, include_zero=True)
            return

        observed_positive = (
            np.concatenate(observed_sigma_values) if observed_sigma_values else np.array([])
        )
        observed_positive = observed_positive[observed_positive > 0.0]
        predicted_positive = (
            np.concatenate(predicted_sigma_values) if predicted_sigma_values else np.array([])
        )
        predicted_positive = predicted_positive[predicted_positive > 0.0]

        scale_ratio = 1.0
        if observed_positive.size > 0 and predicted_positive.size > 0:
            scale_ratio = np.max(predicted_positive) / max(np.max(observed_positive), 1.0e-12)

        if scale_ratio > 25.0:
            linthresh = max(np.min(positive_sigma_values), 1.0e-6)
            self.sigma_ax.set_yscale("symlog", linthresh=linthresh)
            upper = np.max(positive_sigma_values) * 1.25
            self.sigma_ax.set_ylim(-linthresh * 0.05, upper)
        else:
            self.sigma_ax.set_yscale("linear")
            self.autoscale_axis_from_lines(self.sigma_ax, include_zero=True)

    def detail_axis_for_component(
        self,
        primary_axis,
        secondary_axis,
        label: str,
        has_linear: bool,
        has_angular: bool,
    ):
        if has_linear and has_angular and label in ANGULAR_STATE_LABELS:
            secondary_axis.set_visible(True)
            return secondary_axis
        return primary_axis

    def apply_combined_legend(self, primary_axis, secondary_axis, fontsize: int, ncol: int) -> None:
        primary_handles, primary_labels = primary_axis.get_legend_handles_labels()
        secondary_handles = []
        secondary_labels = []
        if secondary_axis is not None and secondary_axis.get_visible():
            secondary_handles, secondary_labels = secondary_axis.get_legend_handles_labels()

        if primary_handles or secondary_handles:
            primary_axis.legend(
                primary_handles + secondary_handles,
                primary_labels + secondary_labels,
                loc="upper left",
                fontsize=fontsize,
                ncol=ncol,
            )

    def clear_axis_with_message(self, axis, title: str, message: str, secondary_axes=None) -> None:
        axis.clear()
        axis.set_title(title)
        axis.text(0.5, 0.5, message, ha="center", va="center", transform=axis.transAxes)
        axis.set_xticks([])
        axis.set_yticks([])

        if secondary_axes is None:
            return

        for secondary_axis in secondary_axes:
            if secondary_axis is None:
                continue
            secondary_axis.clear()
            secondary_axis.set_yticks([])
            secondary_axis.set_visible(False)


def main(args=None) -> None:
    rclpy.init(args=args)
    visualizer = EkfTuningVisualizer()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(visualizer)

    try:
        last_redraw = 0.0
        while rclpy.ok() and plt.fignum_exists(visualizer.figure.number):
            executor.spin_once(timeout_sec=0.1)
            now = time.monotonic()
            if now - last_redraw >= 0.5:
                visualizer.redraw()
                last_redraw = now
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(visualizer)
        visualizer.destroy_node()
        plt.close("all")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
