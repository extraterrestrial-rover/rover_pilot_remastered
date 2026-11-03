# The link to the drive ESP32: Ethernet through a W5500 (`ethernet-udp`)

On this branch the drive ESP32 has a W5500 Ethernet module on its SPI bus, and one Ethernet cable goes to a port of
the rover laptop that is used for nothing else. The packets (README, "Protocol") travel in UDP datagrams, one or more
complete packets per datagram, each ending in `0x00`. The ESP32 stays powered from the laptop's USB, which is also how
it is flashed. The other branch, `main`, carries the same packets over that USB port instead (README, "Branches").

The addresses and ports are those of the rover's future CAN master (`telemetry_dashboard`, `protocol/ETHERNET_LINK.md`),
so the laptop's network setup serves both, and the CAN master can later take the drive ESP32's place on the same cable.

## On this branch

- Link: Ethernet (W5500, UDP): ESP32 192.168.77.2:47700, rover laptop 192.168.77.1:47701 on its own port
- Firmware: fw 3.0.0 (Arduino-ESP32 core 3.x). Flash it over USB: `cd firmware && pio run -e esp32doit-devkit-v1 -t upload`
- Rover: `ros2 launch rover_pilot_remastered rover.launch.py interface:=<the ESP32's port, e.g. enx001122334455>`

## Wiring

A generic W5500 board (RJ45 jack with magnetics, a 2×5 or 2×6 header) to the DOIT DevKit V1. The ESP32's usual SPI
pins drive motors, so the bus goes to free GPIOs (`config.h`, `W5500_*`):

| W5500 board | ESP32 | Notes |
|---|---|---|
| 3.3V (or V, 3V3) | 3V3 | About 130 mA peak, from the DevKit's regulator (USB-powered). Leave a 5V pin unconnected |
| GND | GND | |
| SCLK (SCK) | GPIO 16 | 16 MHz SPI (`W5500_SPI_MHZ`); keep the wires short (under about 10 cm) |
| MOSI (MO) | GPIO 17 | |
| MISO (MI) | GPIO 34 | an input-only GPIO, fine for MISO |
| SCS (CS, SS) | GPIO 5 | a strapping pin, but its boot level (high) is the idle level of a chip select: no pull-down on it |
| INT | GPIO 35 | input-only. Without this wire, set `W5500_INT_PIN -1`: the driver then polls every 10 ms |
| RST | GPIO 4 | the driver resets the W5500 at start-up |

The motor pins are unchanged. GPIO 21 stays free for the profiling build's loop marker.

**Power, and the kill switch.** As on `main`, the ESP32 runs from the onboard computer's USB: with the laptop it stays
on when the kill switch cuts the battery (centre the sticks, or press Circle, before switching back on); with a Pi or
Jetson it goes off with everything else. The Ethernet magnetics keep the Ethernet cable from adding a ground path.

## Rover laptop

The ESP32's cable needs a laptop port of its own: a built-in port, or a USB-Ethernet adapter where the built-in one
goes to the radio. Find its name: plug the ESP32 in (powered) and run `ip -br link`; the port that just came `UP` is
it, for example `enx001122334455` (USB adapters are named after their MAC address, so the name is stable).

Give it the static address 192.168.77.1/24, with no gateway, once:

```bash
sudo nmcli connection add type ethernet ifname <port> con-name rover-master ipv4.method manual \
  ipv4.addresses 192.168.77.1/24 ipv4.never-default yes ipv6.method disabled connection.autoconnect yes
sudo nmcli connection up rover-master
ping -c 3 192.168.77.2          # the ESP32 answers once its firmware is running
```

This is the same connection the CAN bridge's installer creates (`telemetry_dashboard/bridge/deploy/install.sh
--master-interface <port>`): if that has run, this step is done. Without NetworkManager (Ubuntu Server), the
installer's `--print-network-config` shows the netplan file instead.

**One program at a time.** `drive_node`, `tools/wheel_test.py`, `tools/profile.py` and the CAN bridge all use UDP
port 47701; the second one to start gets "port in use". That is the README's "only one of them may follow `/cmd_vel`"
rule, now enforced by the laptop.

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `master:=` | `192.168.77.2` | the ESP32's address |
| `interface:=` | (none) | the laptop port the ESP32 is plugged into. Set it on the rover: the drive node then sends only through that port (Linux `IP_UNICAST_IF`), so with the cable out the commands cannot leave through the radio, and `/diagnostics` shows the port's carrier |

`master_port` (47700) and `local_port` (47701) are in `config/drive.yaml` and must equal `ESP_UDP_PORT` and
`PC_UDP_PORT` in `firmware/include/config.h`.

The bench tools take the same settings as options: `python3 tools/wheel_test.py --interface <port>` (also `--master`,
`--port`, `--local-port`), and likewise `tools/profile.py`. They need nothing beyond the Python standard library.

## Firmware

- Arduino-ESP32 core **3.x** only (pioarduino): the W5500 goes through its `ETH` library (ESP-IDF's W5500 driver and
  lwIP). Core 2.x has no SPI Ethernet; `main` keeps building on it.
- After a reset, the Ethernet link takes 1–3 s to come up; the motors stay off until a neutral command anyway.
- The network never blocks the control loop. With the cable out, no commands arrive and the 300 ms timeout stops the
  motors; when it is back, traffic resumes without a reset, and the ESP32 waits for centred sticks as after any stop.
- The ESP32 takes commands only from 192.168.77.1. Datagrams from any other address, and a packet cut off at the end
  of a datagram, are dropped and counted in `esp_rx_bad`.
- The USB port is not part of the link. It prints a few lines at 115200 baud: the firmware version and whether the
  W5500 started at boot, then "Ethernet link up/down". `pio device monitor` shows them, but opening the port resets a
  DevKit: only with the wheels off the ground.

## Bench checks (wheels off the ground)

1. **Link:** after flashing, the jack's LEDs light, the USB text says `W5500 started`, and `ping -c 3 192.168.77.2`
   answers.
2. **Wheels:** `python3 tools/wheel_test.py --interface <port>` (README, "First test on the bench"): every line says
   **fw 3.0.0**.
3. **Cable out:** while driving, pull the Ethernet cable. The wheels stop within 0.3 s; the rover log says
   `ESP32 lost ... (the PC port has no carrier: Ethernet cable out?)`. Plug it back with a stick pushed: nothing moves
   until the sticks are centred.
4. **`/diagnostics`:** `link_send_errors` and `esp_rx_bad` stay at 0 while the cable is in; `interface` says `up`.

## Simulator

`fake_esp` speaks the protocol over UDP on the loopback, at the same ports:

```bash
ros2 run rover_pilot_remastered fake_esp
ros2 launch rover_pilot_remastered rover.launch.py master:=127.0.0.1
```

## Troubleshooting

| What you see | Usual cause |
|---|---|
| `UDP port 47701 is in use` | Another `drive_node`, a bench tool or the CAN bridge is running: stop it |
| `nothing heard from the ESP32` / `no status from the ESP32` | The cable or the ESP32's power; the laptop port has no 192.168.77.1/24 (`ip -br addr`); the ESP32 still runs the USB firmware (fw 2.x): flash this branch's |
| USB text: `W5500 NOT answering` | SPI wiring (SCLK/MOSI/MISO/CS swapped or loose), the W5500's 3.3 V or GND, or RST. Try `W5500_SPI_MHZ 8` with long wires |
| Link LEDs dark | Cable, or the W5500 has no power |
| Ping works, nothing heard | `local_port`/`master_port` differ from the firmware's ports, or a firewall drops UDP 47701 (`sudo ufw status`) |
| `interface: ... NOT pinned` in `/diagnostics` | The `interface:=` name is wrong: compare with `ip -br link` |
| `esp_rx_bad` rising | Datagrams from another device on that port, or corrupted packets |
| After `git switch main`, nothing is heard over USB | The ESP32 still runs fw 3.x: flash `main`'s firmware (2.x) |

## Files

The link-specific code on this branch; `main` has its own versions of these, everything else is shared.

| File | What it does here |
|---|---|
| `firmware/src/net_link.cpp/.h` | the W5500 (ETH), the static address, the UDP socket |
| `firmware/lib/udp_frames/` | datagram rules and the source check (unit-tested: `test/test_udp/`) |
| `firmware/src/main.cpp`, `firmware/include/config.h`, `firmware/platformio.ini` | the link glue, the W5500 pins and the addresses, core 3.x |
| `ros2_ws/.../udp_link.py` | the PC side: datagrams, source check, interface pin, carrier |
| `ros2_ws/.../drive_node.py` | uses `udp_link`; "ESP32 lost/back" and the link's `/diagnostics` |
| `ros2_ws/.../fake_esp.py` | the simulator on UDP (`Simulator` for the tests and the rehearsal) |
| `ros2_ws/.../launch/rover.launch.py`, `config/drive.yaml`, `package.xml`, `setup.py` | `master:=`, `interface:=`, the ports; version 3.0.0 |
| `tools/wheel_test.py`, `tools/profile.py` | talk UDP (`udp_link`) |
| `ros2_ws/.../test/test_udp_link.py`, `test_end_to_end.py`, `test_ros_integration.py` | run over UDP |
| `udev/99-rover-drive.rules` | not used here; kept, so an installed rule keeps working after `git switch main` |
