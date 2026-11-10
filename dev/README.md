# Developer Documentation

Technical notes for this project. Installation and normal GUI usage are in the top-level [README.md](../README.md).

## Mounting presets and custom angles

### 5.1 How to Change When the Physical Port Faces "Back" or "Up"

The current UI only has Front / Down / Custom built in; **Back and Up are not implemented as presets**. The direction is determined by the matrix, not guessed by flipping a sign in an RPY value: with horizon forward `f=(0,0,-1)`, down `d=(0,-1,0)`, back `b=(0,0,1)`, up `u=(0,1,0)`, and the port local vector `v=(-1,0,0)`, `R_front·v=f`. To keep the same mounting baseline, simply left-multiply a rotation about +X in the horizon parent frame:

| Target orientation | Left-multiplied rotation | Port direction | Left-arm RPY/° (example) | Right-arm RPY/° (example) |
|---|---|---|---|---|
| Front | `Rx(0°)` | `(0,0,-1)` | `(-90,-90,90)` | `(0,-90,180)` |
| Down | `Rx(-90°)` | `(0,-1,0)` | `(-90,0,90)` | `(90,0,90)` |
| Up | `Rx(+90°)` | `(0,1,0)` | `(90,0,-90)` | `(-90,0,-90)` |
| Back | `Rx(180°)` | `(0,0,1)` | `(180,90,0)` | `(0,90,0)` |

How to use (choose one as needed):

1. **Try it on one device only**: set Power Port Facing to **Custom**, enter the left/right RPY values from the table for the corresponding side (using the left column as the left-arm example), and after Apply verify against the physical arm and the magenta reference marker; the two bases are independent, so do not copy the left-arm values to the other side.
2. **Make it a proper preset**: add entries for `Rx(±90°/180°)·R_front` to the preset rotation table in `mounting.py`, and register them in the UI's `Power Port Facing` options/labels; derive the RPY from the matrix with `transforms.rpy_deg_from_rotation_matrix()`, **treating the matrix as authoritative** — do not hand-calculate Euler angles and look only at the numbers. Again: this changes only the mounting frame; it does not change joint zero positions, feedback, limits, or translation.

**Key points:**

- `Rx(-90°)` produces "down", `Rx(+90°)` produces "up", and `Rx(180°)` produces "back"; the angle sign decides up vs. down, and 180° decides front vs. back — it is not just adding a minus sign to one RPY number.
- The RPY values in the table are derived from the matrices using this repository's ZYX/URDF convention and have gimbal-equivalent encodings; after any change, verify with the matrix/reference marker, not just the RPY string.
- If verification shows that the **reference vector itself is reversed** (the physical direction is opposite to the model's local −X), change the sign of `mounting.POWER_PORT_LOCAL_VECTOR` (`(-1,0,0)` ↔ `(1,0,0)`); this flips Front/Back and Down/Up as a whole. Re-verify with the matrix and the magenta marker, and do not compensate by changing joints or limits.

Settings rules:

- `config.py` is the source of defaults; at runtime a `settings.json` is created **at the project root** to store overrides, and `config.py` is not modified. The repository as delivered usually has no such file.
- If you keep multiple source copies/environments, each stores its own `settings.json` and logs and they do not sync automatically; changing one does not affect another running process.
- Apply order: local validation → acquire global settings-transaction ownership → control-thread validation/acceptance (with revision) → 3D sync → write to disk. On rejection, nothing is written and the view is not changed.
- While a transaction is in flight, panel input, Apply, and Restore are disabled; a submission from another open page is explicitly rejected (`another page is applying settings`), and file writes are not interleaved.
- Restore Defaults: applies the `ControlConfig()` defaults and deletes the override file; the Startup Pose also returns to default, but the real arm returns to the new startup pose only on the next Start.
- For an old settings file with no `mounts` section and only manual RPY, both sides adopt the original RPY as `custom`; if the `mounts` label and the RPY matrix disagree, the file is treated as invalid and ignored.
- Open pages sync by revision; unsaved edits are replaced by the new valid configuration during sync, and "Settings changed in another page" is shown.
- If writing/deleting fails, the file snapshot is restored first, then a controller rollback is requested; only when "rollback confirmed **and** file restore succeeded **and** the state was already consistent before the transaction began (`was_converged_before_begin`)" do you see the rolled-back state and is Start allowed. When unconfirmed, the actually effective configuration is shown, and START stays disabled until a healthy Apply/Restore succeeds.

## Settings transactions and controller state

- Every settings change goes through the controller and carries the last known `settings_revision`; a stale page is rejected instead of overwriting a newer value.
- A settings transaction serializes the controller commit, `settings.json` write, and failure recovery across pages. While it is active, other pages cannot commit.
- `Apply` updates `config` and the arm-base frames, resets IK/filter/planner anchors, increments the revision, and replies with the result. A write failure restores the file snapshot and requests a controller rollback; `Start` remains blocked until settings converge.
- The UI only edits supported fields. The matrices and preset tables live in `mounting.py`; `frames.py` is the single consumer used by control, scene, and ghost models.

## 6. How Teleoperation Works

Data flow: Pico APK → ADB reverse → pico_bridge → IPC → input filtering/limiting → shoulder/base/flange transforms → analytical IK → joint filtering/planning → pyAgxArm → SocketCAN → arm feedback.

### 6.1 Input and filtering

- The APK sends one 19-field CSV line per sample. Poses are in the horizon frame (head origin, yaw only); PC receive time decides validity (0.25 s).
- One Euro filtering is applied to both controller poses and IK joint targets. The quaternion is filtered per component (not SLERP); SLERP is used only for the rotation-step limit.
- Input steps are capped at 0.04 m / 0.16 rad per step and 2 m/s / 4 rad/s. These limit the controller target, not the real arm speed.

### 6.2 Transform and IK

- The controller target is transformed into the arm-base frame. Only the shoulder-relative translation is scaled by 1.45, then the flange tool offset is applied.
- The analytical seven-axis IK clamps reach and searches locally around the previous arm angle; if no solution exists, the last target is held.
- Joint targets are bounded by joint limits, 0.4/0.5 rad/s, 1 rad/s², and 0.03 rad per tick. These are target-side limits.

### 6.3 Arm SDK and state machine

- Connect enables the arm. Startup uses `move_j`; servo takeover then uses `move_js` at 50 Hz.
- States: STARTUP → ALIGNING (with real arms) → TELEOP; STOP or Connect latches STOPPED; FAULT requires a restart.
- STOP stops advancing new targets and holds the joints; it does not cut torque.
- ROS 2 output is optional and requires the ROS libraries in the same Python environment as the GUI.

## 7. Parameter and Command Quick Reference

| Parameter | Default and meaning |
|---|---|
| Web UI | `127.0.0.1:8080`; this tutorial usually uses `--port 8095`, and if occupied it increments within 50 ports |
| Pico APK → headset local | TCP `127.0.0.1:29761` |
| ADB reverse → PC receive | headset `29761` → PC `127.0.0.1:9100` (`PicoConfig.vr_port`) |
| IPC | `127.0.0.1:9101`, only one client allowed (do not connect around it) |
| Control / GUI / scene | 50 Hz / 30 Hz (configured) / 10 fps |
| Pose valid / input-connected display | 0.25 s / 0.5 s |
| Real-arm feedback timeout | 0.25 s |
| SDK speed_percent | 20 (SDK-side ratio before sending targets); separate from host-side target rate limiting |

The following parameters come from `run.py::parse_args`; with `PORT=0` (the default) the `config.py` default is used, and a free port is sought starting from that port within 50 ports:

```text
--no-pico       Do not start pico_bridge, but still wait for IPC input (no bridge means no pose frames); not the same as --demo
--demo          Synthetic input; implies --no-pico and uses DemoPoseSource instead
--headless      Do not start NiceGUI
--duration SEC  Run seconds for headless only; 0 means run until Ctrl+C
--port PORT     Preferred web port (default 0 → 8080)
--vr-port PORT  PC Pico receive port (default 0 → 9100)
--ipc-port PORT Inter-process port (default 0 → 9101)
--no-browser    Do not open the browser automatically
```

Common combinations (run **at the project root**):

```bash
python run.py --demo --no-browser --port 8095              # Offline UI check, no real arm
python run.py --no-browser --port 8095                     # Real Pico + connectable real arm
python run.py --demo --headless --duration 2               # Synthetic input self-test without UI, exits after 2 s
```

Use Settings for mounting orientation/startup pose; filtering, IK, rate limits, and tool offset are mainly in `config.py`. After changing source code you must end the old process and restart; refreshing the browser does not reload Python. `--headless --duration` can be used for an offline startup check without opening the web page.

## 8. File Structure

```text
analytic_control/
├── run.py                 # Main entry, port selection, child-process supervision and cleanup
├── config.py              # Immutable config classes and default parameters
├── controller_state.py    # Snapshots, commands, settings transactions, and global ownership
├── control_loop.py        # Startup/alignment/teleop/stop state machine
├── hardware.py            # SDK loading, firmware, CAN, j/js, and feedback
├── joint_feedback.py      # Joint and motor feedback reads, format checks
├── ipc.py                 # TCP/NDJSON and Pico text-frame parsing
├── pose_source.py         # Real-Pico/demo input sources and freshness
├── filters.py             # One Euro, quaternion handling, input limits
├── frames.py              # Controller/shoulder/base/flange transform chain
├── mounting.py            # Single source of truth for mounting presets
├── transforms.py          # Rigid transforms, quaternions, RPY
├── ik.py                  # NeroIK wrapper, flange offset, reach range
├── ik_solver.pyx          # Cython analytical IK/FK and continuous branches
├── setup.py               # Builds ik_solver
├── motion.py              # Limits, velocity/acceleration limits, startup trajectory
├── settings_store.py      # settings.json validation/migration/saving/restore
├── logging_utils.py       # UI, console, and rotating file logs
├── ros_output.py          # Optional Nero state topics
├── read_joints.py         # Standalone read tool; do not mix casually with real-arm control
├── pico_bridge/
│   ├── __main__.py        # Child-process CLI and parent-exit monitoring
│   ├── bridge.py          # ADB/TCP/IPC, state, and focus
│   └── ros_output.py      # Optional /read_pose topics
├── ui/
│   ├── app.py             # Layout, buttons, status refresh
│   ├── scene.py           # URDF/STL, target and feedback ghosts
│   ├── controllers.py     # Controller page
│   └── settings.py        # Form, mounting orientation, commit/restore
├── apk/
│   ├── src/               # Rust/OpenXR and TCP sending
│   ├── java/              # Android Activity
│   ├── generated_apk/     # Installable APK
│   ├── runtime_libs/      # OpenXR loader
│   └── build.sh           # Optional APK build/signing
├── nero_urdf/             # Nero model and meshes
├── third_party/pyAgxArm/  # AgileX SDK submodule
├── scripts/can0-persist/  # Optional CAN systemd/permission installer
├── test/                  # Offline algorithms, fake hardware, IPC, GUI regression
├── requirements.txt       # Python dependencies
├── settings.json          # Generated at runtime, local overrides
└── logs/                  # control.log and rotated logs
```

`.so`, `ik_solver.c`, `build/`, and `__pycache__/` are build/runtime artifacts, not a second algorithm. Historical acceptance records from the upstream development docs do not represent acceptance on the current real arm.

## 9. Troubleshooting

### 9.1 Find the Entry Point by Symptom

| Symptom | Check first |
|---|---|
| Web page unreachable | Whether the program terminal is running, any exceptions, the actually printed port |
| Orientation control missing | Whether this project is running; end the old process, restart, then refresh |
| ik_solver import failure | Rebuild with build_ext in the same Python/platform |
| Pico WAITING/0Hz | adb devices, headset authorization, APK, USB connection |
| Pico CONNECTED but controller LOST | Headset worn, tracking, Controllers, app focus warnings |
| can0 does not exist | Check `lsusb`, the `gs_usb` module, the USB cable/port, and `ip -details link show type can` |
| ERROR-PASSIVE/BUS-OFF | After parking/supporting the arm and exiting the program, check power, wiring, bitrate, and termination resistor; if can0-persist is installed you can `sudo systemctl restart can0.service`, otherwise use the standalone §3.6 commands to down, set bitrate, and up in order |
| CAN stays down after replugging | Exit the program first, then `sudo systemctl restart can0.service` (auto bounce); without the script, reconfigure manually; do not run while the real arm is moving |
| IK found no solution | Target/pose/limits or local search has no solution; move back into the reachable region |
| another page applying | Wait for the other page to finish, then retry |
| not persisted/Start blocked | Disk write or rollback did not converge; fix file access, then Apply/Restore |
| ROS UNAVAILABLE | Normal teleop can leave it off; if needed, check the ROS libraries for the same interpreter |

### 9.2 The Real Arm Suddenly Stops Moving

1. STOP first, note the time, and do not repeatedly Start or blindly reconnect.
2. Check the Pico rate and TRACKED; invalid input holds the previous target.
3. Check IK holding / target recovered; an unreachable target pauses advancement.
4. Check joint-feedback warnings, `re-requested CAN push`, and CAN state.
5. After keeping the logs and stopping, you can passively check with `timeout 2s candump can0`; if frames exist, tell the types apart, and a lack of frames does not directly prove motor damage.

If the SDK socket is actually closed, that path marks ERROR and locks STOP. On abnormal feedback, have the operator stop first and check power, emergency stop, wiring, and driver state before deciding to resume.

### 9.3 Capability Boundaries

- No self-collision or environment-collision detection, and not every controller target is guaranteed reachable.
- No host-side torque closed loop, bilateral force feedback, or measured gravity compensation.
- The physical mounting axis must be verified on the real arm; mathematical verification does not equal passed calibration.
- Closing/freezing the web page in the background is not a verified hardware-stop trigger.
- Target sending, SDK feedback, and freshly measured arrival are different facts; the ghost and TELEOP cannot substitute for accuracy acceptance.
- The USB and browser links are not hard-real-time; real GPU frame rate and end-to-end latency have not been fully accepted.

## 10. Verification

### 10.1 Offline Verification

In the same environment and at the project root, with no real robot arm connected:

```bash
# Project root, and do not connect a real robot arm
python -m pytest -q -p no:cacheprovider
python run.py --demo --no-browser --headless --duration 2
# Expected: tests pass; the headless run exits after 2 seconds
```

Offline regression on the delivered source: **147 passed** (2026-11-10, pure simulation). This does not represent real-arm load, safety, or accuracy. Default logs are `logs/control.log` and rotated files; when troubleshooting, keep the logs from before and after the incident plus the current configuration, and redact irrelevant information such as headset serial numbers when sharing.

Third-party SDKs and models follow their own licenses; this README does not re-grant licenses. When reporting an issue, provide the source version, OS, Python, startup command, key logs, and redacted configuration — not just a "connected" screenshot.
