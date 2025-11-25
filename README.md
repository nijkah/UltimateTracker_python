# UltimateTracker_python

[![Python 3.6+](https://img.shields.io/badge/python-3.6+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Platform: Windows](https://img.shields.io/badge/platform-Windows-lightgrey.svg)](https://www.microsoft.com/windows)

Read orientation and position data from the **VIVE Ultimate Tracker** using Python — no VR headset required. Windows only.

![6DoF Visualization](https://via.placeholder.com/800x400?text=6DoF+Tracker+Visualization)

## ✨ Features

- **Direct Tracker Access** — Read position and orientation data without a VR headset
- **CSV Logging** — Record timestamped pose data with quaternion orientation
- **Real-time Visualization** — Multiple plot modes:
  - 6DoF 3D view with tracker model and orientation axes
  - Color-coded trajectory with orientation history ghosts
  - X/Y/Z position time series
  - Legacy 3D trajectory plot
- **Configurable** — Full command-line control over sampling rate, visualization, and output

## 📋 Prerequisites

### 1. SteamVR Setup
1. Install [SteamVR](https://store.steampowered.com/app/250820/SteamVR/)
2. Enable the null driver for virtual headset using [SteamVRNoHeadset](https://github.com/username223/SteamVRNoHeadset)

### 2. VIVE Streaming Hub
1. Download [VIVE Streaming Hub](https://www.vive.com/us/vive-hub/download/)
2. Activate PC Streaming Beta with code: `VIVEUTRCPreview` *(valid as of 08/2024)*
3. Follow the in-app instructions to create a tracking map
   - Skip the final step requiring a SteamVR headset connection

> **Note:** As of 21.08.2024, launching SteamVR is no longer necessary, but installation and enabling the null HMD driver is still required.

### 3. Python Dependencies
```bash
pip install -r requirements.txt
```

Or install manually:
```bash
pip install openvr numpy matplotlib win-precise-time
```

## 🚀 Quick Start

```bash
# Run with default 6DoF visualization
python vive_tracker_recorder.py

# List connected trackers
python vive_tracker_recorder.py --list-devices

# High-performance logging (no visualization)
python vive_tracker_recorder.py --no-plot --no-print -o tracker_data.csv
```

## 📖 Usage

```bash
python vive_tracker_recorder.py [OPTIONS]
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `-r, --rate HZ` | Sampling rate in Hz | 120 |
| `-o, --output FILE` | Output CSV filename | `ultimate_tracker_data.csv` |
| `--plot-6dof` / `--no-plot-6dof` | Toggle 6DoF visualization | Enabled |
| `--plot-3d` | Enable 3D trajectory plot | Disabled |
| `--plot-xyz` | Enable X/Y/Z time series | Disabled |
| `--no-plot` | Disable all visualizations | — |
| `--log` / `--no-log` | Toggle CSV logging | Enabled |
| `--print` / `--no-print`, `-q` | Toggle console output | Enabled |
| `--max-history N` | Trajectory history points | 100 |
| `--ghost-interval N` | Orientation ghost spacing | 10 |
| `--axis-length M` | Orientation axis length (m) | 0.12 |
| `--colormap NAME` | Trajectory colormap | `viridis` |
| `--list-devices` | List VR devices and exit | — |

### Examples

```bash
# Custom sampling rate and output
python vive_tracker_recorder.py -r 240 -o experiment_001.csv

# Enable all visualization modes
python vive_tracker_recorder.py --plot-6dof --plot-3d --plot-xyz

# Customize visualization appearance
python vive_tracker_recorder.py --max-history 200 --ghost-interval 5 --colormap plasma

# Quiet mode with logging only
python vive_tracker_recorder.py --no-plot -q
```

## 📁 Output Format

The CSV file contains the following columns:

| Column | Description |
|--------|-------------|
| `TrackerIndex` | Device index |
| `Time` | Unix timestamp |
| `PositionX/Y/Z` | Position in meters |
| `RotationW/X/Y/Z` | Orientation quaternion |

## 🏗️ Project Structure

```
UltimateTracker_python/
├── vive_tracker_recorder.py  # Main script with 6DoF visualization
├── requirements.txt          # Python dependencies
├── README.md
└── LICENSE
```

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgements

This project is a fork of the original [UltimateTracker_python](https://github.com/jkulozik/UltimateTracker_python) by Julian Kulozik. We thank the original author for creating the foundation for direct VIVE tracker access in Python.

## 📬 Contact

For questions or issues regarding this fork, please open an issue on GitHub.

For the original project, contact: **kulozik[at]isir.upmc.fr**

## 📚 Citation

If you use this code in your research, please cite:

```bibtex
@software{nijkah2025vivetracker,
  author = {nijkah},
  title = {UltimateTracker_python: VIVE Tracker Data Collection and 6DoF Visualization},
  year = {2025},
  url = {https://github.com/nijkah/UltimateTracker_python}
}
```

This project is based on the original work by Julian Kulozik:

```bibtex
@software{kulozik2024vivetracker,
  author = {Kulozik, Julian},
  title = {UltimateTracker_python: VIVE Tracker DirectRead},
  year = {2024},
  url = {https://github.com/jkulozik/UltimateTracker_python}
}
```