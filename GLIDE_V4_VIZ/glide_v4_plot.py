#!/usr/bin/env python3
"""Minimal standalone Plotly viewer for precomputed GLIDE V4 trajectory JSON."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import plotly.graph_objects as go
from PIL import Image


HERE = Path(__file__).resolve().parent
DEFAULT_INPUT = HERE / "sample_glide_v4_trajectory.json"
DEFAULT_OUTPUT = HERE / "glide_v4_visualization.html"
DEFAULT_EARTH_RADIUS_KM = 6371.0
DEFAULT_EARTH_TEXTURE = HERE / "earth_texture.jpg"
EARTH_LAT_STEPS = 181
EARTH_LON_STEPS = 361
MAX_ANIMATION_FRAMES = 300
ANIMATION_FRAME_DURATION_MS = 80
CORRIDOR_LABEL_OFFSET_KM = 525.0
PLOT_DIV_ID = "glide-v4-plot"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render a precomputed GLIDE V4 3D trajectory into a self-contained HTML file."
    )
    parser.add_argument(
        "input_json",
        nargs="?",
        default=str(DEFAULT_INPUT),
        help="Path to the local trajectory JSON file.",
    )
    parser.add_argument(
        "output_html",
        nargs="?",
        default=str(DEFAULT_OUTPUT),
        help="Path to the output HTML file.",
    )
    return parser.parse_args()


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def point_to_xyz(point: dict, earth_radius_km: float) -> list[float]:
    vector = point.get("xyz_km") or point.get("ecef_km")
    if vector is not None:
        if len(vector) != 3:
            raise ValueError(f"Expected 3 values for xyz/ecef point, got {vector!r}")
        return [float(value) for value in vector]

    if "lat_deg" not in point or "lon_deg" not in point:
        raise ValueError(
            f"Point must include either xyz_km/ecef_km or lat_deg/lon_deg: {point!r}"
        )

    lat = math.radians(float(point["lat_deg"]))
    lon = math.radians(float(point["lon_deg"]))
    alt = float(point.get("alt_km", 0.0))
    radius = earth_radius_km + alt

    x = radius * math.cos(lat) * math.cos(lon)
    y = radius * math.cos(lat) * math.sin(lon)
    z = radius * math.sin(lat)
    return [x, y, z]


def xyz_to_geodetic(xyz: list[float], earth_radius_km: float) -> tuple[float, float, float]:
    x, y, z = xyz
    radius = math.sqrt(x * x + y * y + z * z)
    if radius == 0.0:
        raise ValueError("Point at Earth center cannot be converted to latitude/longitude/altitude")

    lat_deg = math.degrees(math.asin(z / radius))
    lon_deg = math.degrees(math.atan2(y, x))
    alt_km = radius - earth_radius_km
    return lat_deg, lon_deg, alt_km


def normalize_point(point: dict, earth_radius_km: float) -> dict:
    xyz = point_to_xyz(point, earth_radius_km)
    lat_deg, lon_deg, alt_km = xyz_to_geodetic(xyz, earth_radius_km)
    return {
        "xyz_km": xyz,
        "lat_deg": lat_deg,
        "lon_deg": lon_deg,
        "alt_km": alt_km,
    }


def format_point_hover(name: str, normalized_point: dict) -> str:
    xyz = normalized_point["xyz_km"]
    return (
        f"{name}<br>"
        f"lat={normalized_point['lat_deg']:.2f} deg<br>"
        f"lon={normalized_point['lon_deg']:.2f} deg<br>"
        f"alt={normalized_point['alt_km']:.1f} km<br>"
        f"x={xyz[0]:.1f} km<br>"
        f"y={xyz[1]:.1f} km<br>"
        f"z={xyz[2]:.1f} km"
    )


def build_time_label(point: dict, point_index: int) -> str:
    for key in ("timestamp", "time", "time_label", "label"):
        value = point.get(key)
        if value is not None:
            return str(value)
    return str(point_index)


def build_payload_marker_customdata(point_index: int, time_label: str, normalized_point: dict) -> list[list[object]]:
    return [[
        point_index,
        time_label,
        normalized_point["lat_deg"],
        normalized_point["lon_deg"],
        normalized_point["alt_km"],
    ]]


def build_payload_marker_hovertemplate(name: str) -> str:
    return (
        f"{name}<br>"
        "frame=%{customdata[0]}<br>"
        "time=%{customdata[1]}<br>"
        "lat=%{customdata[2]:.2f} deg<br>"
        "lon=%{customdata[3]:.2f} deg<br>"
        "alt=%{customdata[4]:.1f} km"
        "<extra></extra>"
    )


def select_animation_indices(num_points: int, max_frames: int = MAX_ANIMATION_FRAMES) -> list[int]:
    if num_points <= max_frames:
        return list(range(num_points))
    if max_frames < 2:
        return [0, num_points - 1]

    scale = (num_points - 1) / (max_frames - 1)
    indices: list[int] = []
    seen: set[int] = set()

    for frame_index in range(max_frames):
        point_index = round(frame_index * scale)
        if point_index not in seen:
            indices.append(point_index)
            seen.add(point_index)

    if indices[-1] != num_points - 1:
        indices.append(num_points - 1)
    return indices


def vector_norm(vector: list[float]) -> float:
    return math.sqrt(sum(component * component for component in vector))


def normalize_vector(vector: list[float]) -> list[float]:
    magnitude = vector_norm(vector)
    if magnitude == 0.0:
        raise ValueError("Cannot normalize a zero-length vector")
    return [component / magnitude for component in vector]


def add_vectors(a: list[float], b: list[float]) -> list[float]:
    return [a[index] + b[index] for index in range(3)]


def scale_vector(vector: list[float], scale: float) -> list[float]:
    return [component * scale for component in vector]


def cross_product(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def build_corridor_label_position(anchor_xyz: list[float], marker_index: int) -> list[float]:
    radial = normalize_vector(anchor_xyz)
    candidate_axes = ([0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0])

    tangent_a: list[float] | None = None
    for axis in candidate_axes:
        candidate = cross_product(radial, list(axis))
        if vector_norm(candidate) > 1e-6:
            tangent_a = normalize_vector(candidate)
            break

    if tangent_a is None:
        tangent_a = [1.0, 0.0, 0.0]

    tangent_b = normalize_vector(cross_product(radial, tangent_a))
    angle = math.radians((marker_index * 137.5) % 360.0)
    tangent_component = add_vectors(
        scale_vector(tangent_a, math.cos(angle)),
        scale_vector(tangent_b, math.sin(angle)),
    )
    offset_direction = normalize_vector(
        add_vectors(scale_vector(tangent_component, 1.0), scale_vector(radial, 0.8))
    )
    return add_vectors(anchor_xyz, scale_vector(offset_direction, CORRIDOR_LABEL_OFFSET_KM))


def sample_texture_color(image: Image.Image, lat_deg: float, lon_deg: float) -> int | tuple[int, int, int]:
    width, height = image.size
    x = min(width - 1, max(0, int(round(((lon_deg + 180.0) / 360.0) * (width - 1)))))
    y = min(height - 1, max(0, int(round(((90.0 - lat_deg) / 180.0) * (height - 1)))))
    return image.getpixel((x, y))


def rgb_to_css(pixel: tuple[int, int, int]) -> str:
    return f"rgb({pixel[0]},{pixel[1]},{pixel[2]})"


def build_earth_surface(
    radius_km: float,
    texture_path: Path,
    lat_steps: int = EARTH_LAT_STEPS,
    lon_steps: int = EARTH_LON_STEPS,
) -> go.Mesh3d:
    texture = Image.open(texture_path).convert("RGB")

    latitudes = [-90.0 + 180.0 * index / (lat_steps - 1) for index in range(lat_steps)]
    longitudes = [-180.0 + 360.0 * index / (lon_steps - 1) for index in range(lon_steps)]

    x_values: list[float] = []
    y_values: list[float] = []
    z_values: list[float] = []

    for lat_deg in latitudes:
        lat = math.radians(lat_deg)
        cos_lat = math.cos(lat)
        sin_lat = math.sin(lat)
        for lon_deg in longitudes:
            lon = math.radians(lon_deg)
            x_values.append(radius_km * cos_lat * math.cos(lon))
            y_values.append(radius_km * cos_lat * math.sin(lon))
            z_values.append(radius_km * sin_lat)

    i_values: list[int] = []
    j_values: list[int] = []
    k_values: list[int] = []
    face_colors: list[str] = []

    def vertex_index(lat_index: int, lon_index: int) -> int:
        return lat_index * lon_steps + lon_index

    for lat_index in range(lat_steps - 1):
        lat0 = latitudes[lat_index]
        lat1 = latitudes[lat_index + 1]
        cell_lat = (lat0 + lat1) / 2.0
        for lon_index in range(lon_steps - 1):
            lon0 = longitudes[lon_index]
            lon1 = longitudes[lon_index + 1]
            cell_lon = (lon0 + lon1) / 2.0
            cell_color = rgb_to_css(sample_texture_color(texture, cell_lat, cell_lon))

            top_left = vertex_index(lat_index, lon_index)
            bottom_left = vertex_index(lat_index + 1, lon_index)
            top_right = vertex_index(lat_index, lon_index + 1)
            bottom_right = vertex_index(lat_index + 1, lon_index + 1)

            i_values.extend([top_left, top_right])
            j_values.extend([bottom_left, bottom_left])
            k_values.extend([top_right, bottom_right])
            face_colors.extend([cell_color, cell_color])

    return go.Mesh3d(
        x=x_values,
        y=y_values,
        z=z_values,
        i=i_values,
        j=j_values,
        k=k_values,
        facecolor=face_colors,
        flatshading=True,
        lighting=dict(ambient=1.0, diffuse=0.0, specular=0.0, roughness=1.0, fresnel=0.0),
        hoverinfo="skip",
        name="Earth",
    )


def build_point_trace(name: str, point: dict, earth_radius_km: float, color: str, size: int) -> go.Scatter3d:
    normalized = normalize_point(point, earth_radius_km)
    xyz = normalized["xyz_km"]
    return go.Scatter3d(
        x=[xyz[0]],
        y=[xyz[1]],
        z=[xyz[2]],
        mode="markers+text",
        text=[name],
        textposition="top center",
        hovertemplate=format_point_hover(name, normalized) + "<extra></extra>",
        marker=dict(size=size, color=color, line=dict(color="#f8fafc", width=2)),
        name=name,
    )


def build_payload_trace(name: str, normalized_points: list[dict]) -> go.Scatter3d:
    return go.Scatter3d(
        x=[point["xyz_km"][0] for point in normalized_points],
        y=[point["xyz_km"][1] for point in normalized_points],
        z=[point["xyz_km"][2] for point in normalized_points],
        mode="lines",
        line=dict(color="#f59e0b", width=6),
        hoverinfo="skip",
        name=f"{name} Trajectory",
    )


def build_payload_marker_trace(
    name: str,
    normalized_point: dict,
    point_index: int,
    time_label: str,
) -> go.Scatter3d:
    xyz = normalized_point["xyz_km"]

    return go.Scatter3d(
        x=[xyz[0]],
        y=[xyz[1]],
        z=[xyz[2]],
        mode="markers+text",
        text=[name],
        textposition="top center",
        customdata=build_payload_marker_customdata(point_index, time_label, normalized_point),
        hovertemplate=build_payload_marker_hovertemplate(name),
        marker=dict(size=10, color="#f59e0b", line=dict(color="#f8fafc", width=2)),
        name=name,
    )


def build_payload_frames(
    name: str,
    raw_points: list[dict],
    normalized_points: list[dict],
    payload_trace_index: int,
) -> tuple[list[go.Frame], list[dict]]:
    animation_indices = select_animation_indices(len(normalized_points))
    frames: list[go.Frame] = []
    slider_steps: list[dict] = []

    for point_index in animation_indices:
        normalized_point = normalized_points[point_index]
        time_label = build_time_label(raw_points[point_index], point_index)
        frame_name = f"payload_{point_index}"

        frames.append(
            go.Frame(
                name=frame_name,
                data=[
                    go.Scatter3d(
                        x=[normalized_point["xyz_km"][0]],
                        y=[normalized_point["xyz_km"][1]],
                        z=[normalized_point["xyz_km"][2]],
                        customdata=build_payload_marker_customdata(point_index, time_label, normalized_point),
                    )
                ],
                traces=[payload_trace_index],
            )
        )
        slider_steps.append(
            {
                "label": time_label,
                "method": "animate",
                "args": [
                    [frame_name],
                    {
                        "mode": "immediate",
                        "frame": {"duration": 0, "redraw": True},
                        "transition": {"duration": 0},
                    },
                ],
            }
        )

    return frames, slider_steps


def build_post_script(div_id: str) -> str:
    return f"""
const gd = document.getElementById("{div_id}");
if (gd && gd.parentNode) {{
  const controls = document.createElement("div");
  controls.style.display = "flex";
  controls.style.alignItems = "center";
  controls.style.gap = "10px";
  controls.style.padding = "8px 16px 0 16px";
  controls.style.color = "#e2e8f0";
  controls.style.fontFamily = "system-ui, sans-serif";

  const play = document.createElement("button");
  play.textContent = "Play";
  play.style.background = "#111827";
  play.style.color = "#e2e8f0";
  play.style.border = "1px solid #334155";
  play.style.borderRadius = "6px";
  play.style.padding = "6px 10px";
  play.style.cursor = "pointer";

  const pause = document.createElement("button");
  pause.textContent = "Pause";
  pause.style.background = "#111827";
  pause.style.color = "#e2e8f0";
  pause.style.border = "1px solid #334155";
  pause.style.borderRadius = "6px";
  pause.style.padding = "6px 10px";
  pause.style.cursor = "pointer";

  const getFrameNames = () => {{
    const frameState = gd._transitionData && gd._transitionData._frames;
    return frameState ? frameState.map((frame) => frame.name) : null;
  }};

  const getPayloadTraceIndex = () => {{
    const meta = gd.layout && gd.layout.meta;
    return meta && Number.isInteger(meta.payload_marker_trace_index)
      ? meta.payload_marker_trace_index
      : -1;
  }};

  const getCurrentFrameListIndex = () => {{
    const frameNames = getFrameNames();
    if (!frameNames || !frameNames.length) {{
      return 0;
    }}

    const traceIndex = getPayloadTraceIndex();
    if (traceIndex >= 0 && gd.data && gd.data[traceIndex]) {{
      const trace = gd.data[traceIndex];
      const customdata = trace.customdata;
      if (customdata && customdata[0] && customdata[0].length) {{
        const pointIndex = Number(customdata[0][0]);
        if (Number.isFinite(pointIndex)) {{
          const frameName = `payload_${{pointIndex}}`;
          const frameListIndex = frameNames.indexOf(frameName);
          if (frameListIndex >= 0) {{
            return frameListIndex;
          }}
        }}
      }}
    }}

    const sliderState = gd._fullLayout && gd._fullLayout.sliders && gd._fullLayout.sliders[0];
    if (sliderState && Number.isInteger(sliderState.active)) {{
      return sliderState.active;
    }}

    return 0;
  }};

  let timerId = null;

  const stopTimer = () => {{
    if (timerId !== null) {{
      window.clearTimeout(timerId);
      timerId = null;
    }}
  }};

  const showFrameByListIndex = (frameListIndex) => {{
    const frameNames = getFrameNames();
    if (!frameNames || frameListIndex < 0 || frameListIndex >= frameNames.length) {{
      return;
    }}

    Plotly.animate(gd, [frameNames[frameListIndex]], {{
      frame: {{duration: 0, redraw: true}},
      transition: {{duration: 0}},
      mode: "immediate",
    }});
  }};

  const scheduleNextFrame = (frameListIndex) => {{
    const frameNames = getFrameNames();
    if (!frameNames || !frameNames.length) {{
      stopTimer();
      return;
    }}

    if (frameListIndex >= frameNames.length) {{
      stopTimer();
      return;
    }}

    showFrameByListIndex(frameListIndex);

    if (frameListIndex >= frameNames.length - 1) {{
      stopTimer();
      return;
    }}

    timerId = window.setTimeout(() => {{
      scheduleNextFrame(frameListIndex + 1);
    }}, {ANIMATION_FRAME_DURATION_MS});
  }};

  const playAnimation = () => {{
    stopTimer();
    const frameNames = getFrameNames();
    if (!frameNames || !frameNames.length) {{
      return;
    }}

    const currentFrameListIndex = getCurrentFrameListIndex();
    const startFrameListIndex =
      currentFrameListIndex >= frameNames.length - 1 ? 0 : currentFrameListIndex + 1;
    scheduleNextFrame(startFrameListIndex);
  }};

  const pauseAnimation = () => {{
    stopTimer();
    Plotly.animate(gd, [null], {{
      frame: {{duration: 0, redraw: true}},
      transition: {{duration: 0}},
      mode: "immediate",
    }});
  }};

  play.addEventListener("click", playAnimation);
  pause.addEventListener("click", pauseAnimation);

  controls.appendChild(play);
  controls.appendChild(pause);
  gd.parentNode.insertBefore(controls, gd);
}}
"""


def build_corridor_traces(markers: list[dict], earth_radius_km: float) -> list[go.Scatter3d]:
    if not markers:
        return []

    normalized_points = [normalize_point(point, earth_radius_km) for point in markers]
    labels = [point.get("name", f"Corridor {index + 1}") for index, point in enumerate(markers)]
    hover_text = [format_point_hover(label, point) for label, point in zip(labels, normalized_points)]
    label_points = [
        build_corridor_label_position(point["xyz_km"], marker_index)
        for marker_index, point in enumerate(normalized_points)
    ]

    leader_x: list[float | None] = []
    leader_y: list[float | None] = []
    leader_z: list[float | None] = []

    for anchor_point, label_point in zip(normalized_points, label_points):
        anchor_xyz = anchor_point["xyz_km"]
        leader_x.extend([anchor_xyz[0], label_point[0], None])
        leader_y.extend([anchor_xyz[1], label_point[1], None])
        leader_z.extend([anchor_xyz[2], label_point[2], None])

    return [
        go.Scatter3d(
            x=leader_x,
            y=leader_y,
            z=leader_z,
            mode="lines",
            line=dict(color="rgba(103, 232, 249, 0.72)", width=4),
            hoverinfo="skip",
            showlegend=False,
            name="Corridor Leaders",
        ),
        go.Scatter3d(
            x=[point[0] for point in label_points],
            y=[point[1] for point in label_points],
            z=[point[2] for point in label_points],
            mode="markers+text",
            text=labels,
            textposition="top center",
            hovertext=hover_text,
            hovertemplate="%{hovertext}<extra></extra>",
            textfont=dict(color="#e2e8f0", size=11),
            marker=dict(
                size=7,
                color="#67e8f9",
                symbol="diamond",
                line=dict(color="#f8fafc", width=1),
            ),
            name="Corridor Markers",
        ),
    ]


def build_figure(data: dict) -> go.Figure:
    earth = data.get("earth", {})
    earth_radius_km = float(earth.get("radius_km", DEFAULT_EARTH_RADIUS_KM))
    texture_path = DEFAULT_EARTH_TEXTURE
    if not texture_path.exists():
        raise FileNotFoundError(f"Earth texture not found: {texture_path}")

    required_keys = ("node_a", "node_b", "payload")
    missing = [key for key in required_keys if key not in data]
    if missing:
        raise ValueError(f"Missing required top-level keys: {', '.join(missing)}")

    payload = data["payload"]
    payload_name = payload.get("name", "Payload / Tag")
    payload_points = payload["trajectory"]
    normalized_payload_points = [normalize_point(point, earth_radius_km) for point in payload_points]

    figure = go.Figure()
    figure.add_trace(build_earth_surface(earth_radius_km, texture_path))
    figure.add_trace(
        build_point_trace(
            data["node_a"].get("name", "Node A"),
            data["node_a"],
            earth_radius_km,
            color="#ef4444",
            size=10,
        )
    )
    figure.add_trace(
        build_point_trace(
            data["node_b"].get("name", "Node B"),
            data["node_b"],
            earth_radius_km,
            color="#10b981",
            size=10,
        )
    )
    figure.add_trace(build_payload_trace(payload_name, normalized_payload_points))
    for corridor_trace in build_corridor_traces(data.get("corridor_markers", []), earth_radius_km):
        figure.add_trace(corridor_trace)
    payload_trace_index = len(figure.data)
    figure.add_trace(
        build_payload_marker_trace(
            payload_name,
            normalized_payload_points[0],
            point_index=0,
            time_label=build_time_label(payload_points[0], 0),
        )
    )

    if len(normalized_payload_points) > 1:
        frames, slider_steps = build_payload_frames(
            payload_name,
            payload_points,
            normalized_payload_points,
            payload_trace_index,
        )
        figure.frames = frames
        figure.update_layout(
            sliders=[
                dict(
                    active=0,
                    x=0.1,
                    y=0,
                    len=0.85,
                    currentvalue=dict(prefix="Time Index: "),
                    pad=dict(t=58, b=8),
                    steps=slider_steps,
                )
            ],
        )

    title = data.get("title", "GLIDE V4 3D Visualization")
    figure.update_layout(
        title=title,
        meta=dict(payload_marker_trace_index=payload_trace_index),
        paper_bgcolor="#020617",
        plot_bgcolor="#020617",
        font=dict(color="#e2e8f0"),
        legend=dict(bgcolor="rgba(2, 6, 23, 0.65)"),
        margin=dict(l=0, r=0, t=96, b=0),
        uirevision="glide_v4_animation",
        scene=dict(
            xaxis=dict(visible=False),
            yaxis=dict(visible=False),
            zaxis=dict(visible=False),
            bgcolor="#020617",
            aspectmode="data",
            camera=dict(eye=dict(x=1.55, y=1.35, z=0.85)),
        ),
    )
    return figure


def main() -> None:
    args = parse_args()
    input_path = Path(args.input_json).resolve()
    output_path = Path(args.output_html).resolve()

    data = load_json(input_path)
    figure = build_figure(data)
    figure.write_html(
        output_path,
        include_plotlyjs=True,
        full_html=True,
        auto_play=False,
        div_id=PLOT_DIV_ID,
        post_script=build_post_script(PLOT_DIV_ID),
    )
    print(f"Wrote {output_path}")


if __name__ == "__main__":
    main()
