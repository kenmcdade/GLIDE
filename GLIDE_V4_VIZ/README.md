# GLIDE V4 Plotly 3D Viewer

Standalone Python-based 3D visualization for precomputed GLIDE V4 trajectory data.

This viewer is intentionally simple and local:
- input is a local JSON file with precomputed state/trajectory data
- output is a self-contained HTML file that opens in a normal browser
- no physics or simulation is performed inside the viewer

## Included Files

- `glide_v4_plot.py`: standalone Plotly renderer
- `sample_glide_v4_trajectory.json`: sample input data
- `glide_v4_visualization.html`: generated self-contained HTML demo
- `earth_texture.jpg`: Earth texture used for the globe
- `requirements.txt`: Python dependencies

## Features

- textured Earth rendering
- Node A and Node B rendered as orbital objects
- static full-path payload trajectory
- animated `Payload / Tag` marker moving along the trajectory
- corridor markers offset from the path with leader lines back to their capture points
- browser playback controls with play, pause, and frame scrubbing

## Requirements

- Python 3.10+

Install dependencies:

```powershell
py -3 -m pip install -r requirements.txt
```

## Run

Use the sample data:

```powershell
py -3 glide_v4_plot.py
```

Use a custom input/output path:

```powershell
py -3 glide_v4_plot.py path\\to\\trajectory.json path\\to\\output.html
```

The default output file is `glide_v4_visualization.html`.

## Input Data Format

The viewer expects a JSON document with this structure:

```json
{
  "title": "GLIDE V4 Sample 3D Trajectory",
  "earth": {
    "radius_km": 6371.0
  },
  "node_a": {
    "name": "Node A",
    "xyz_km": [-2704.395, -5033.124, 3861.182]
  },
  "node_b": {
    "name": "Node B",
    "xyz_km": [-4280.77, 3631.642, 4030.894]
  },
  "payload": {
    "name": "Payload / Tag",
    "trajectory": [
      { "xyz_km": [-2704.395, -5033.124, 3861.182] },
      { "xyz_km": [-3605.891, -4374.296, 3998.999] }
    ]
  },
  "corridor_markers": [
    {
      "name": "Corridor 1",
      "xyz_km": [-4562.242, -3228.414, 4262.813]
    }
  ]
}
```

## Coordinate Handling

- Orbital objects are interpreted from Cartesian state vectors.
- `altitude = norm(position) - Earth radius`
- If needed, points may also be provided as `lat_deg`, `lon_deg`, and optional `alt_km`, but the current sample data uses `xyz_km`.

## Notes

- The payload animation uses existing trajectory points as ordered frames.
- The full trajectory line remains visible during playback.
- `glide_v4_visualization.html` is intentionally included in this directory as a ready-to-open artifact.

## Earth Texture Attribution

Earth texture source: NASA Blue Marble Next Generation.

- https://science.nasa.gov/earth/earth-observatory/blue-marble-next-generation/base-topography-bathymetry/
- https://assets.science.nasa.gov/content/dam/science/esd/eo/images/bmng/bmng-topography-bathymetry/june/world.topo.bathy.200406.3x5400x2700.jpg
