# The link to the drive ESP32: USB serial (`main`)

On this branch the rover laptop reaches the drive ESP32 through the ESP32's USB port, which also powers it. The
packets (README, "Protocol") travel as one byte stream: COBS-encoded, each ending in `0x00`, 115200 8N1. The other
branch, `ethernet-udp`, carries the same packets over a W5500 Ethernet module instead (README, "Branches").

## On this branch

- Link: USB serial, `/dev/rover-drive` (udev rule) at 115200 baud
- Firmware: fw 2.0.2. Flash it: `cd firmware && pio run -e esp32doit-devkit-v1 -t upload`
- Rover: `ros2 launch rover_pilot_remastered rover.launch.py` (add `port:=/dev/ttyUSB0` without the udev rule)

## Rover laptop

Give the ESP32 a stable name (once, with only the drive ESP32 plugged in):

```bash
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}'      # copy the value into udev/99-rover-drive.rules
sudo cp ~/rover_pilot_remastered/udev/99-rover-drive.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger && ls -l /dev/rover-drive
sudo usermod -aG dialout $USER                          # then log out and in
```

If the board uses a CH340 (no unique serial), use the commented `KERNELS==` line in the rule instead: it matches the
physical USB port (`udevadm info -a -n /dev/ttyUSB0 | grep KERNELS`). Until the rule is installed, pass
`port:=/dev/ttyUSB0` to the launch file.

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `port:=` | `/dev/rover-drive` | the ESP32's serial port |

`baud` (115200) is in `config/drive.yaml` and must equal `SERIAL_BAUD` in `firmware/include/config.h`.

The bench tools take the port as their first argument: `python3 tools/wheel_test.py /dev/ttyUSB0`,
`python3 tools/profile.py /dev/ttyUSB0` (default `/dev/rover-drive`).

## Firmware

`firmware/platformio.ini` pins the platform to pioarduino 55.03.311 (Arduino-ESP32 core 3.3.11), so every computer
builds the same firmware. The code also compiles on core 2.x, but there the loop watchdog is 5 s instead of 1 s, so
leave the pin alone. The serial line carries only binary packets, so a normal serial monitor shows
garbage: use `tools/wheel_test.py` or the drive node's logs instead. Opening the port with DTR/RTS asserted (most
serial monitors) resets a DevKit; the drive node and the tools open it with both de-asserted.

## Simulator

`fake_esp` speaks the protocol on a pseudo-terminal:

```bash
ros2 run rover_pilot_remastered fake_esp --link /tmp/rover-drive
ros2 launch rover_pilot_remastered rover.launch.py port:=/tmp/rover-drive
```

## Files

The link-specific code on this branch; the other branch has its own versions of these, everything else is shared.

| File | What it does here |
|---|---|
| `firmware/src/main.cpp` | reads and writes the packets on `Serial` |
| `firmware/include/config.h` | `SERIAL_BAUD` |
| `ros2_ws/.../drive_node.py` | opens the serial port (`open_port`), reconnects every second |
| `ros2_ws/.../fake_esp.py` | the simulator on a pseudo-terminal (`Simulator` for the tests and the rehearsal) |
| `ros2_ws/.../launch/rover.launch.py`, `config/drive.yaml` | `port:=`, `baud` |
| `tools/wheel_test.py`, `tools/profile.py` | open the serial port (pyserial) |
| `ros2_ws/.../test/test_end_to_end.py`, `test_ros_integration.py` | run over a pseudo-terminal |
| `udev/99-rover-drive.rules` | the `/dev/rover-drive` name |
