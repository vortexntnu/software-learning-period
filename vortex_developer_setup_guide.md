# Ubuntu 22.04 x64 Setup Guide

This guide describes how to set up a fresh Ubuntu 22.04 x64 machine for ROS 2 Humble development with the Vortex NTNU ROS 2 workspace.

The guide intentionally links to official installation pages where longer installation procedures are required. Follow those pages directly instead of copying ROS 2, GitHub, or Foxglove installation commands from this guide.

---

## Assumptions

- Operating system: Ubuntu 22.04 x64
- ROS distribution: ROS 2 Humble
- Workspace name: `ros2_ws`
- Example username/path used in some configuration: `/home/vortex/ros2_ws`

> If your username is not `vortex`, adjust paths such as `/home/vortex/ros2_ws/build` to match your own home directory.

---

## 1. Install Visual Studio Code

Go to the VS Code download page:

<https://code.visualstudio.com/download>

Choose the correct **`.deb x64`** download for Ubuntu.

After downloading, go to your `Downloads` folder and install the package:

```bash
sudo apt install ./code**
```

> Replace `code**` with the actual downloaded filename, or use shell tab-completion after typing `./code`.

---

## 2. Install ROS 2 Humble

Follow the official ROS 2 Humble Ubuntu Debian installation guide:

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

> Use the commands from the ROS documentation page directly. This guide does not duplicate the ROS install commands, because the official page is the source of truth.

---

## 3. Install Terminator

Terminator is a terminal multiplexer that makes it easier to work with several terminals at once.

```bash
sudo apt install terminator
```

Useful shortcuts:

| Shortcut | Action |
|---|---|
| `Ctrl + Alt + T` | Open a new terminal window |
| `Ctrl + Shift + T` | Open a new terminal tab |
| `Ctrl + Shift + O` | Split terminal horizontally |
| `Ctrl + Shift + E` | Split terminal vertically |

---

## 4. Install VS Code Extensions

Install the following VS Code extensions:

- `clangd`
- `Remote - SSH`

### clangd ROS 2 setup

For clangd to understand the ROS 2 workspace properly, the workspace should be built with compile commands exported.

Create a `.vscode/settings.json` file under `ros2_ws`.

Add the following content:

```json
{
  "clangd.arguments": [
    "--completion-style=detailed",
    "--compile-commands-dir=/home/vortex/ros2_ws/build",
    "--header-insertion=never"
  ]
}
```

> Adjust `/home/vortex/ros2_ws/build` to match your own workspace path.
>
> Do not put comments inside `settings.json`, because standard JSON does not support comments.

---

## 5. Create the ROS 2 Workspace

Create the ROS 2 workspace and source directory:

```bash
mkdir ros2_ws
mkdir ros2_ws/src
```

The expected workspace structure is:

```text
ros2_ws/
└── src/
```

---

## 6. Set Up SSH Keys for GitHub

Follow the GitHub documentation for generating a new SSH key and adding it to the SSH agent:

<https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent>

Follow the sections:

- **Generating a new SSH key**
- **Adding your SSH key to the ssh-agent**

Then follow the GitHub documentation for adding the SSH key to your GitHub account:

<https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account>

Add the key on GitHub as an **authentication key**.

> This is required before cloning the private Vortex repositories using `git@github.com:...` SSH URLs.

---

## 7. Install GCC 13

GCC 13 is required to compile C++20 code, for example code used in `vortex-vkf`.

This is included in the `vortex-auv` install script, but it can be useful to do manually so that you are aware of the dependency.

```bash
sudo apt-get update -qq
sudo apt-get install -y --no-install-recommends software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt-get update -qq

sudo apt-get install -y --no-install-recommends gcc-13 g++-13 lcov
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100
sudo update-alternatives --install /usr/bin/gcov gcov /usr/bin/gcov-13 100
```

> After this, `gcc`, `g++`, and `gcov` should point to the GCC 13 versions through `update-alternatives`.

---

## 8. Clone the ROS 2 Repositories

Go into `ros2_ws/src` and clone the required repositories:

```bash
git clone git@github.com:vortexntnu/vortex-auv.git
git clone git@github.com:vortexntnu/vortex-msgs.git
git clone git@github.com:vortexntnu/vortex-utils.git
git clone git@github.com:vortexntnu/vortex-vkf.git
git clone git@github.com:vortexntnu/vortex-cv.git
```

> These commands use SSH, so GitHub SSH authentication must be set up first.

---

## 9. Install Dependencies from `vortex-auv`

Run the install script in `vortex-auv` to install dependencies required by packages in the workspace:

```bash
sudo apt update
sudo apt install -y python3-pip python3-pip-whl
./src/vortex-auv/scripts/ci_install_dependencies.sh
```

> Run this from the root of `ros2_ws`, not from inside `src`.

---

## 10. Set Up ROS 2 CLI Tools

### Source ROS 2 in new terminal sessions

Add ROS 2 Humble sourcing to `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

> This makes ROS 2 commands available in new terminal sessions.

### Set up colcon argcomplete

Install colcon extensions:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

Add colcon argcomplete sourcing to `.bashrc`:

```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

### Add a build helper function

Add an alias-like shell function for building packages with compile commands exported:

```bash
cat >> ~/.bashrc <<'EOF'
cbu() {
  colcon build --packages-up-to "$@" --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
}

_cbu_completion() {
  local cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "$(colcon list --names-only 2>/dev/null)" -- "$cur") )
}
complete -F _cbu_completion cbu
EOF
source ~/.bashrc
```

> The `cbu` function builds the selected package and all packages it depends on.
>
> The `-DCMAKE_EXPORT_COMPILE_COMMANDS=1` option is important for clangd support.

---

## 11. Install rosdeps for Workspace Packages

Install and run rosdep for packages in the workspace:

```bash
sudo apt install python3-rosdep2
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> Run the `rosdep install` command from the root of `ros2_ws`.

---

## 12. Set Up the Stonefish Simulator

Go into `ros2_ws/src` and clone the simulator repositories:

```bash
git clone git@github.com:vortexntnu/vortex-stonefish-sim.git
git clone git@github.com:vortexntnu/stonefish_ros2.git
```

Run the install script to install Stonefish dependencies:

```bash
./ros2_ws/src/vortex-stonefish-sim/scripts/ci_install_dependencies.sh
```

> Check your current directory before running this command. The path shown above assumes you are outside `ros2_ws`.

Then build the packages:

```text
stonefish_sim
stonefish_ros2
stonefish_sim_interface
```

> Build these packages using the workspace build workflow.

---

## 13. Set Up Foxglove

Download Foxglove from:

<https://foxglove.dev/download>

Install it from the `Downloads` folder:

```bash
sudo apt install ./foxglove-studio-*.deb
```

Install the Foxglove ROS bridge:

```bash
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
```

Launch the Foxglove bridge:

```bash
ros2 launch foxglove_bridge foxglove_bridge.launch.xml
```

> Foxglove can connect to the running bridge to visualize ROS 2 topics.
