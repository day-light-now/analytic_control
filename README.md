# PICO × AgileX Nero Teleop

PICO 4 Ultra controller teleoperation for AgileX Nero seven-axis arms. The host converts controller poses into joint targets and provides a browser UI for the 3D model, real-arm feedback, connection control, and mounting settings. Single-arm and dual-arm setups are supported.

## Quick Start

Requirements:

- Linux host, Ubuntu 24.04 recommended, with SocketCAN support.
- Python 3.12.
- PICO 4 Ultra and controllers with USB debugging enabled.
- AgileX Nero arm and a compatible USB-to-CAN adapter.

### Install

```bash
cd <project-dir>
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m pip install -e ./third_party/pyAgxArm
python setup.py build_ext --inplace
```

The last command builds the IK extension. Rebuild it if you change Python or the platform.

### Prepare PICO and CAN

Connect the headset and the USB-CAN adapter directly to the Linux host, then run:

```bash
adb devices
adb install -r apk/generated_apk/pico_head_relative_pose.apk
ip -details link show can0
```

Accept the USB-debugging prompt in the headset. If `can0` is missing or down:

```bash
sudo modprobe gs_usb
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

If `adb` reports a permission error, install the udev rules and re-login:

```bash
sudo apt install android-sdk-platform-tools-common
sudo usermod -aG plugdev "$USER"
```

### Run

```bash
env -u PYTHONPATH python run.py --no-browser --port 8095
```

Open the URL printed in the terminal. Use `--demo` instead for a UI-only check; demo mode does not enable an arm.

## GUI

The screenshots below are from pure simulation.

![Main window](docs/screenshots/ui_01_arm_simulation_annotated.png)

1 Connect Arms · 2 ROS 2 Output · 3 STOP · 4 pages · 5 status · 6 legend · 7 System Log.

- **Connect Arms** opens the channel dialog.
- **Connect / Disconnect** in that dialog connects and enables, or releases, the selected side.
- **Start** is available in STOPPED once settings are saved. It moves connected arms to the Startup Pose before following; using Start again after STOP repeats this sequence.
- **STOP** stops advancing new targets and holds the arm. It is not a physical emergency stop.
- **ROS 2 Output** is an optional switch; leave it off unless you need ROS topics.
- The status bar shows Pico input, controller tracking, arm state, ROS state, and joint-limit margin.
- Closing the browser does not stop the program. Press **Ctrl+C** in the terminal to exit.

![Connect Arms dialog](docs/screenshots/ui_04_connect_dialog.png)

- **Channel** is the SocketCAN interface for that side, for example `can0`.
- **Connect** connects and enables the arm; **Disconnect** releases it; **Close** closes the dialog without changing the connection.

The three pages are **Arm Simulation** for the 3D target and feedback model, **Controllers** for controller poses and buttons, and **Settings** for Startup Pose and mounting configuration.

![Controllers page](docs/screenshots/ui_02_controllers.png)

![Settings page](docs/screenshots/ui_03_settings.png)

- **Apply** validates, applies, and saves the settings. It is disabled while a transaction is in progress and rejected while a real arm is following; press STOP first.
- **Restore Defaults** removes the saved settings and restores defaults.
- **Cancel** and **Start** in the Start dialog cancel or begin the startup and alignment sequence.

![Start dialog](docs/screenshots/ui_05_start_dialog.png)

### Settings

- **Power Port Facing**: choose **Front** when the power port faces the horizon-forward direction, or **Down** when it faces gravity. The magenta marker shows the selected direction.
- **Startup Pose**: keep the defaults for a first installation; edit the seven joint angles only after checking the real arm.
- Keep the other Base Frames values at their defaults unless the arm has been measured.
- The shipped defaults are the original model's Front installation. Use **Custom** only with values measured from the actual arm.

![Mounting direction reference](docs/screenshots/orientation_diagram.png)

## For developers

See [dev/README.md](dev/README.md).
