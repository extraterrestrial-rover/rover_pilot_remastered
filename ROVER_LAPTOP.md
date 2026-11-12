# A new rover laptop, from a blank Ubuntu 22.04 (`visual-odom-serial`)

This guide turns a fresh Ubuntu 22.04 laptop with an NVIDIA GPU into the rover's onboard computer. The finished
laptop:
- runs the ZED 2i, the autonomy stack and `drive_node`;
- drives the ESP32 over USB;
- is reached by the base station over the MikroTik radio (wired) at **192.168.1.68**;
- is started over SSH in tmux.

Do the setup at a desk with a screen, a keyboard and internet access (Wi-Fi is fine), then put the laptop in the
rover. It takes one to two hours, mostly downloads and the ZED build. Run each block in a terminal, in order.

## 0. Check what you have

```bash
lsb_release -ds                     # Ubuntu 22.04.x LTS
ls /opt/ros                         # humble (if empty, step 4 installs it)
lspci | grep -i -E 'vga|3d'         # must list an NVIDIA GPU
df -h ~                             # at least 40 GB free: CUDA, the ZED SDK and its AI models are large
mokutil --sb-state                  # "SecureBoot enabled" matters in step 2
```

## 1. Updates, tools, a name

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl tmux openssh-server htop
sudo apt remove -y brltty           # it grabs USB-serial chips (CH340) and the ESP32's /dev/ttyUSB0 vanishes
sudo hostnamectl set-hostname rover
sudo reboot
```

If `apt update` fails on the ROS repository (`EXPKEYSIG`, `NO_PUBKEY`, or "Conflicting values set for option
Signed-By"), the old ROS key has expired. Install the current ROS apt source package, then run `sudo apt update` again:

```bash
sudo rm -f /etc/apt/sources.list.d/ros2.list
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

## 2. NVIDIA driver

```bash
ubuntu-drivers devices              # shows the recommended driver
sudo ubuntu-drivers install         # installs it
sudo reboot
nvidia-smi                          # the GPU, and "CUDA Version: 12.x" or newer at the top right
```

**With Secure Boot on**, the installation asks you to choose a password. At the next boot a blue "MOK management"
screen appears: choose *Enroll MOK* → *Continue* → *Yes*, and type that password. If you miss it, the driver doesn't
load (`nvidia-smi` fails): run `sudo update-secureboot-policy --enroll-key`, reboot, and enroll again.

## 3. ZED SDK

Download the latest **ZED SDK for Ubuntu 22.04** (`.run` file, version 5.2 or newer) from
<https://www.stereolabs.com/developers/release>. The `cudaXX.X` in its file name is the CUDA version it uses (12.x and
13.x both run on a driver whose `nvidia-smi` shows that version or newer). Then:

```bash
cd ~/Downloads
chmod +x ZED_SDK_Ubuntu22_*.run
./ZED_SDK_Ubuntu22_*.run            # not with sudo: it asks for your password itself
```

Answers: accept the licence; **yes** to installing CUDA if it offers; **no** to the static version; **yes** to
optimising the AI models (it takes a while now instead of at the first launch); the Python API and the samples are
not needed. Reboot when it finishes.

**Check CUDA.** The "CUDA Version" that `nvidia-smi` shows is only the newest CUDA the driver supports. The SDK needs
the CUDA toolkit itself. The installer may report that it can't find CUDA and then install it. Check that it's there
and that the SDK's libraries resolve:

```bash
ls /usr/local | grep -i cuda                                  # a cuda-12.x or cuda-13.x folder
ldd /usr/local/zed/lib/libsl_zed.so | grep "not found"        # must print nothing
ls -l /usr/local/cuda && /usr/local/cuda/bin/nvcc --version   # the ZED wrapper's build (step 5) looks here
```

- **A `cuda-13.0` folder but no `/usr/local/cuda`**: link it, `sudo ln -s /usr/local/cuda-13.0 /usr/local/cuda`
  (with your folder's name).
- **No CUDA folder, or `ldd` lists missing libraries**: install the toolkit that the SDK's file name asks for, then
  run the SDK installer again. For example, `…_cuda12.8_…` needs `cuda-toolkit-12-8`. Install only the toolkit: the
  `cuda` and `cuda-drivers` packages would replace the working driver.
  ```bash
  wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
  sudo dpkg -i cuda-keyring_1.1-1_all.deb && sudo apt update
  sudo apt install -y cuda-toolkit-12-8
  ```

Plug the ZED 2i **straight into a USB 3 port** (blue, or marked SS), not through a hub. Then check that it shows up
and that you get a picture and depth:

```bash
lsusb | grep -i 2b03                        # Stereolabs: the camera and its sensors
/usr/local/zed/tools/ZED_Explorer           # live picture (needs the screen)
/usr/local/zed/tools/ZED_Depth_Viewer       # depth and point cloud: the real CUDA test
```

ZED_Explorer can show a picture even without CUDA; depth needs it. If the installer couldn't optimise the AI models
(CUDA was missing at that point), the depth viewer's first start pauses for several minutes to do it. Let it finish
once.

## 4. ROS 2 packages

If `ls /opt/ros` showed nothing, `ros-humble-ros-base` below installs ROS itself. If `apt` can't find the
`ros-humble-*` packages, add the ROS apt source (the block in step 1) first.

```bash
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions python3-rosdep python3-pytest \
    python3-serial python3-yaml ros-humble-rmw-cyclonedds-cpp ros-humble-diagnostic-msgs \
    ros-humble-navigation2 ros-humble-robot-localization ros-humble-rtabmap-ros \
    ros-humble-xacro ros-humble-robot-state-publisher
sudo rosdep init                    # "already initialized" is fine
rosdep update
```

## 5. ZED ROS 2 wrapper

```bash
mkdir -p ~/zed_ws/src && cd ~/zed_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ~/zed_ws && source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --packages-skip zed_debug
```

Build it only once the depth viewer works (step 3): it needs CUDA at `/usr/local/cuda`. The build takes 10 to 30
minutes. If it stops with the compiler killed (out of memory), rerun it with
`--parallel-workers 1` added. The wrapper's `master` branch needs ZED SDK 5.2 or newer. With an older SDK, check out
the wrapper release that matches it (its GitHub releases page) before building.

## 6. The rover software

```bash
cd ~ && git clone https://github.com/extraterrestrial-rover/rover_pilot_remastered.git
cd ~/rover_pilot_remastered && git switch visual-odom-serial
cd ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
cd ~/rover_pilot_remastered && tools/install_hooks.sh       # rebuilds by itself after every pull and branch switch
```

Every terminal needs the same environment. Append it to `~/.bashrc`, with **your team's domain ID, the same as the
base station's**:

```bash
cat >> ~/.bashrc <<'EOF'
source /opt/ros/humble/setup.bash
source ~/zed_ws/install/local_setup.bash
source ~/rover_pilot_remastered/ros2_ws/install/local_setup.bash
export PATH=/usr/local/cuda/bin:$PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0        # CHANGE: the base station's number
EOF
source ~/.bashrc
```

## 7. The drive ESP32 (USB)

```bash
sudo usermod -aG dialout $USER      # serial ports without sudo; takes effect after logging out and in (or a reboot)
```

Give the ESP32 the fixed name `/dev/rover-drive`. If it's the same ESP32 board as on the old laptop and
`udev/99-rover-drive.rules` already holds its serial number, only copy the rule. Otherwise, with **only the drive
ESP32** plugged in, read its serial number first and put it in the rule (LINK.md, "Rover laptop", covers CH340 boards
without one):

```bash
udevadm info -a -n /dev/ttyUSB0 | grep '{serial}'      # copy the value into udev/99-rover-drive.rules
sudo cp ~/rover_pilot_remastered/udev/99-rover-drive.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger && ls -l /dev/rover-drive
```

## 8. Network: the MikroTik link and SSH

Connect the laptop's Ethernet port to the rover's MikroTik and power the radio. Give the port the rover's address:

```bash
nmcli device status                 # the Ethernet device's name, for example enp3s0 or eno1
sudo nmcli connection add type ethernet ifname enp3s0 con-name rover-lan \
    ipv4.method manual ipv4.addresses 192.168.1.68/24 ipv4.never-default yes ipv6.method disabled
sudo nmcli connection up rover-lan
ping -c 3 192.168.1.<base station>  # the base station answers over the radio
sudo ufw status                     # "inactive" (Ubuntu's default); if active: sudo ufw allow from 192.168.1.0/24
```

Use your device's name instead of `enp3s0`. `never-default` keeps the internet on Wi-Fi during setup.

From the **base station**, once, so it logs in without a password:

```bash
ssh-copy-id <user>@192.168.1.68     # <user>: your login name on the rover laptop
```

**Turn Wi-Fi off in the field** (`nmcli radio wifi off`; `on` again at home for updates). A Wi-Fi network that also
uses 192.168.1.x confuses routing to the base station, and ROS 2 nodes would announce themselves on Wi-Fi as well.

**Power the radio before you launch.** Without a powered radio the port has no carrier, and NetworkManager takes
the 192.168.1.68 address off it. ROS 2 nodes started then find only the laptop itself and never see the base
station, not even when the radio comes up later. If you launched too early: Ctrl-C and launch again.

## 9. Living closed inside the rover

The laptop must keep running with its lid shut, never sleep, and never start updates on its own:

```bash
sudo sed -i -E 's/^#?(HandleLidSwitch|HandleLidSwitchExternalPower|HandleLidSwitchDocked)=.*/\1=ignore/' /etc/systemd/logind.conf
grep HandleLidSwitch /etc/systemd/logind.conf                       # all three: ignore
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
sudo systemctl disable --now unattended-upgrades
sudo reboot
```

After the reboot, close the lid and check from the base station that `ssh <user>@192.168.1.68` still works a minute
later.

- **Battery.** It runs on its own battery: the kill switch doesn't turn it off, and the ZED, RTAB-Map and Nav2 load
  the GPU and CPU. Before relying on it, time a full run on battery (`upower -i $(upower -e | grep BAT)` shows the
  charge). For the best performance on battery: `powerprofilesctl set performance`.
- **Heat.** Keep the vents clear in the rover and watch the temperatures: `nvidia-smi` for the GPU, `sensors` for the
  CPU (`sudo apt install lm-sensors`).
- **Shutdown.** `sudo poweroff` over SSH, before you store the rover.

## 10. Check the installation

No hardware is needed for these. Both rehearsals run the real launch files against the simulated ESP32:

```bash
cd ~/rover_pilot_remastered/ros2_ws/src/rover_pilot_remastered && python3 -m pytest -q -p no:launch_testing -p no:launch_ros test
cd ../rover_autonomy && python3 -m pytest -q -p no:launch_testing -p no:launch_ros test
cd ~/rover_pilot_remastered
python3 tools/rehearsal.py                            # the bench test, about 1 minute
python3 tools/rehearsal_auto.py                       # AUTO with the real Nav2, about 6 minutes
```

The tests and both rehearsals release the e-stop and drive at full speed, so none of them uses your shell's
`ROS_DOMAIN_ID`: they run in ROS domains of their own (tests 87, rehearsals 63 and 61/62), keep their ROS traffic on
the laptop, and refuse to start when another node is already in that domain. A rover that is switched on cannot hear
them. Each run ends with its PASS/FAIL count; exit code 1 on any failure.

Then the camera through ROS. The first start can take minutes while the SDK optimises its depth model:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
# in a second terminal:
ros2 topic list | grep zed_node                       # copy the names into autonomy.yaml (zed_*_topic)
ros2 topic hz /zed/zed_node/odom                      # about 15 to 30 Hz
```

## 11. Before the first drive

Measure and fill in `ros2_ws/src/rover_autonomy/config/autonomy.yaml` (AUTONOMY.md, "Before the first run"):
- the rover's size;
- the ZED's mount;
- the ZED topic names from step 10;
- the deadband, with `tools/wheel_test.py`, wheels on the ground. It ships as 0.0 (not measured): AUTO then works,
  but the rover may not move at Nav2's slow commands, and AUTO stops itself after 3 s without motion.

Rebuild afterwards (`cd ~/rover_pilot_remastered/ros2_ws && colcon build --symlink-install`). The base station must
be on `visual-odom-serial` too, for the Triangle button: `git switch visual-odom-serial` there.

## Every session

Base station: `ros2 launch rover_autonomy base_station.launch.py` (the PS5 and RViz).

Rover, from the base station:

```bash
ssh <user>@192.168.1.68
tmux new -A -s rover                                   # opens the "rover" session, or attaches to it if it runs
ros2 launch rover_autonomy rover_autonomy.launch.py    # or: ros2 launch rover_pilot_remastered rover.launch.py (driving only)
```

**Always launch inside tmux.** If the radio drops, the SSH session dies, but tmux keeps the launch running:
- the rover stops by itself (the base station's heartbeats are missing);
- once the radio is back, you can drive again, and a goal in AUTO resumes.

A launch started outside tmux dies with the SSH session.

| In tmux | Keys |
|---|---|
| leave it running and go back to the shell (detach) | `Ctrl-b`, then `d` |
| get back to it, also after a radio drop | `ssh …` then `tmux attach -t rover` |
| a second window (for `ros2 topic …`) | `Ctrl-b`, then `c`; switch with `Ctrl-b`, then `n` |
| stop the rover software | attach, then `Ctrl-C` |

## When something is wrong

| Symptom | Cause, fix |
|---|---|
| `/dev/ttyUSB0` appears and vanishes a second later | `brltty` claims the chip: `sudo apt remove brltty`, replug |
| `cannot open /dev/rover-drive: Permission denied` | not in `dialout` yet: log out and in, or reboot |
| `nvidia-smi`: "couldn't communicate with the NVIDIA driver" | Secure Boot key not enrolled (step 2), or no reboot since the install |
| ZED SDK installer: "CUDA not found" | step 3, "Check CUDA": it may have installed CUDA itself; check with `ldd` |
| ZED wrapper build: CUDA not found, or `Specify CUDA_TOOLKIT_ROOT_DIR` | no `/usr/local/cuda` link: step 3, "Check CUDA" |
| ZED not found, or `CAMERA STREAM FAILED TO START` | not on a USB 3 port, a hub in between, or the cable; `lsusb \| grep -i 2b03` |
| The base station sees no rover topics | same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` on both; Wi-Fi off; `ping` works; neither sets `ROS_LOCALHOST_ONLY=1` |
| The launch ended when SSH dropped | it ran outside tmux: always `tmux new -A -s rover` first |
| The laptop went to sleep in the rover | step 9: check `grep HandleLidSwitch /etc/systemd/logind.conf` and that the masks are in place |
| ZED wrapper build killed | not enough memory: `colcon build … --parallel-workers 1` |
