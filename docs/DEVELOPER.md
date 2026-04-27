# RepRapFirmware Developer Setup Guide

This guide explains how to set up a development environment for RepRapFirmware from scratch. No prior knowledge of VS Code or Docker is assumed.

---

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Install Required Software](#install-required-software)
4. [Clone the Repository](#clone-the-repository)
5. [Open the Project in VS Code](#open-the-project-in-vs-code)
6. [Open in Dev Container](#open-in-dev-container)
7. [Initialize Submodules](#initialize-submodules)
8. [Build the Firmware](#build-the-firmware)
9. [Build Targets](#build-targets)
10. [Debug vs Release Builds](#debug-vs-release-builds)
11. [Git Tools: Git Graph and GitLens](#git-tools-git-graph-and-gitlens)
12. [Troubleshooting](#troubleshooting)

---

## Overview

RepRapFirmware is firmware for 3D printer control boards based on ARM Cortex-M processors. It targets several Duet hardware variants:

| Board | Processor |
|---|---|
| Duet 3 MB6HC | SAME70 |
| Duet 3 MB6XD | SAME70 |
| Duet 3 Mini 5+ | SAME51 (SAME5x) |

The project uses a **dev container** — a self-contained Linux build environment defined in code. When you open the project in VS Code, VS Code will offer to build and start this container automatically. Inside the container, all required tools (ARM GCC cross-compiler, Eclipse IDE, make) are pre-installed. You do not need to install them manually on your computer.

---

## Prerequisites

You need three things installed on your computer before starting:

- **Git** — for cloning the repository
- **Docker Desktop** — provides the containerised build environment
- **Visual Studio Code (VS Code)** — the editor

### What is Docker?

Docker runs isolated environments called **containers**. Each container is like a lightweight virtual machine with its own operating system, compilers, and tools. The dev container for this project includes the ARM GCC cross-compiler and all other build dependencies. You do not need Linux — Docker runs on Windows, macOS, and Linux.

### What is VS Code?

VS Code is a free code editor. The **Dev Containers** extension for VS Code can automatically build and start a Docker container, then connect your editor to it so you can write, build, and debug code inside the container as if it were running locally.

---

## Install Required Software

### 1. Git

- **Windows / macOS**: download from https://git-scm.com/downloads
- **Ubuntu/Debian Linux**: `sudo apt install git`

Verify the installation by opening a terminal and running:

```
git --version
```

### 2. Docker Desktop

Download and install Docker Desktop from https://www.docker.com/products/docker-desktop/

After installation:
- **Windows**: start Docker Desktop from the Start menu. Wait for the whale icon in the system tray to stop animating — Docker is ready when the icon is still.
- **macOS**: start Docker Desktop from Applications. Wait for the menu bar icon to stop animating.
- **Linux**: follow the post-installation steps at https://docs.docker.com/engine/install/linux-postinstall/ to allow your user to run Docker without `sudo`.

Verify Docker is running:

```
docker --version
```

### 3. Visual Studio Code

Download and install VS Code from https://code.visualstudio.com/

After installing VS Code, install the **Dev Containers** extension:

1. Open VS Code.
2. Click the **Extensions** icon in the left sidebar (it looks like four squares).
3. Search for `Dev Containers`.
4. Click **Install** on the extension published by Microsoft.

---

## Clone the Repository

Open a terminal (on Windows use Git Bash or PowerShell, on macOS/Linux use Terminal) and run:

```sh
git clone https://github.com/Duet3D/RepRapFirmware.git
cd RepRapFirmware
```

> [!NOTE] You can optionally initialise submodules now by appending `--recurse-submodules` to the `git clone` command, or you can do it later using `make init-submodules` as described in [Initialize Submodules](#initialize-submodules).

---

## Open the Project in VS Code

1. Open VS Code.
2. Choose **File → Open Folder…** and select the `RepRapFirmware` folder you just cloned.

Alternatively, from the terminal inside the cloned folder:

```sh
code .
```

---

## Open in Dev Container

When VS Code opens the folder it will detect the `.devcontainer/` configuration and show a notification in the bottom-right corner:

> **"Folder contains a Dev Container configuration file. Reopen in Container?"**

Click **Reopen in Container**.

If you miss the notification:
1. Press `F1` (or `Ctrl+Shift+P` / `Cmd+Shift+P` on macOS) to open the Command Palette.
2. Type `Dev Containers: Reopen in Container` and press `Enter`.

VS Code will now:
1. Build the Docker container image (this downloads the ARM GCC toolchain and installs Eclipse — **this takes several minutes on the first run**).
2. Start the container.
3. Reconnect the editor so it runs inside the container.

You will see a green **"Dev Container: C++"** badge in the bottom-left corner of VS Code when the container is ready.

> All subsequent terminal sessions opened inside VS Code (`Terminal → New Terminal`) will run inside the container where the ARM GCC toolchain is available.

---

## Initialize Submodules

The libraries (`CANlib`, `CoreN2G`, `FreeRTOS`, `RRFLibraries`, etc.) are Git submodules — separate repositories pinned to specific versions. You must initialise them before building.

Inside VS Code, open a terminal (`Terminal → New Terminal`) and run:

```sh
make init-submodules
```

This clones all required library submodules at the correct pinned versions and builds their static library artifacts.

> You only need to run this once after cloning. Run it again if you switch branches and the pinned submodule versions change.

### Automatically update submodules on branch switch

You can configure Git to automatically check out submodules to the correct pinned version whenever you run `git checkout` or `git pull`. Run this once, anywhere on your machine:

```sh
git config --global submodule.recurse true
```

With this set, switching branches or pulling updates will automatically move each submodule to the commit recorded in the new branch. You will still need to re-run `make init-submodules` if a submodule is newly added (i.e. the submodule directory does not yet exist on disk), but existing submodules will track branch changes automatically.

---

## Build the Firmware

With the submodules initialised, build the firmware for a specific board:

```sh
make Duet3_MB6HC
```

The compiled firmware binary will be placed in the `Duet3_MB6HC/` output directory.

### Building from the VS Code Task Runner

VS Code tasks are pre-configured for every board. To use them:

1. Press `Ctrl+Shift+B` (or `Cmd+Shift+B` on macOS) to open the Build task list.
2. Select a target such as **Build Duet3_MB6HC - Release**.

---

## Build Targets

| `make` target | Board |
|---|---|
| `Duet3_MB6HC` | Duet 3 MB6HC |
| `Duet3_MB6XD` | Duet 3 MB6XD |
| `Duet3Mini5plus` | Duet 3 Mini 5+ |
| `Duet3_CAN0` | Duet 3 CAN expansion |
| `Duet3_MB6HC_no_SD` | Duet 3 MB6HC (no SD card) |
| `FMDC_V03` | FMDC v0.3 |
| `all` | All of the above |

### Other useful targets

| `make` target | Description |
|---|---|
| `init-submodules` | Initialise/update library submodules |
| `clean` | Remove build outputs for all targets |
| `clean-<config>` | Remove build outputs for one target, e.g. `clean-Duet3_MB6HC` |
| `clean-all` | Remove build outputs and all rebuilt libraries |
| `test-toolchain` | Verify the ARM GCC toolchain is accessible |
| `help` | Print all available targets |

---

## Debug vs Release Builds

By default, builds are **release** builds (optimised, no debug symbols).

To build with debug symbols and reduced optimisation:

```sh
make DEBUG=1 Duet3_MB6HC
```

Or use the VS Code task **Build Duet3_MB6HC - Debug**.

### Verbose output

To see every compiler command as it runs:

```sh
make V=1 Duet3_MB6HC
```

---

## Git Tools: Git Graph and GitLens

Two VS Code extensions make it much easier to work with branches, merges, and history across the RepRapFirmware repository and its submodules.

### Installing the extensions

1. Click the **Extensions** icon in the VS Code left sidebar.
2. Search for and install each of the following:
   - **Git Graph** (by mhutchie)
   - **GitLens — Git supercharged** (by GitKraken)

Both extensions work inside the dev container automatically — no extra configuration is needed.

### Git Graph

Git Graph displays a visual commit graph for any repository open in VS Code.

- **Open it**: click the **Git Graph** button that appears in the Source Control toolbar, or run `Git Graph: View Git Graph` from the Command Palette (`F1`).
- **Compare branches**: right-click any branch or commit in the graph and choose **Compare References…** to see a diff of changed files between two points in history.
- **Merges**: right-click a branch and choose **Merge into current branch** to perform a merge with a visual preview of what will change.
- **Submodules**: use the repository picker at the top of the Git Graph panel to switch between the main repository and any initialised submodule.

### GitLens

GitLens adds rich inline and side-panel Git information throughout the editor.

- **Commit graph**: open the **GitLens: Commit Graph** panel from the Source Control view for a full-featured visual history with branch lanes, authors, and tags.
- **Branch comparison**: in the GitLens side panel, open **Commits** and use the **Compare** button to diff any two branches or tags — useful when checking what a submodule update actually changes.
- **Inline blame**: hovering over any line of code shows the commit that last changed it, including the message and author.
- **File history**: right-click a file in the Explorer and choose **Open File History** (GitLens) to see every commit that touched that file.
- **Merge / rebase assistance**: during a merge or rebase conflict, GitLens adds inline decorations showing the incoming vs current change and links directly to the originating commits.

### Recommended workflow for switching RRF branches

1. Use Git Graph or the Source Control panel to check out the target branch.
2. If `submodule.recurse true` is set (see [Automatically update submodules on branch switch](#automatically-update-submodules-on-branch-switch)), submodules update automatically.
3. Run `make init-submodules` if any submodule directory is new on this branch.
4. Use **Compare References** in Git Graph to review differences from your previous branch before building.

---

## Troubleshooting

### Docker is not running

If VS Code cannot start the container, ensure Docker Desktop is running. Look for the Docker icon in your system tray (Windows) or menu bar (macOS).

### "Toolchain not found" error

Run `make test-toolchain` in the VS Code terminal to check whether the ARM GCC compiler is on the `PATH`. Inside the dev container this should always succeed. If you are building outside the container, install the ARM GNU toolchain from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads and set `CROSS_COMPILE` to point to it.

### Submodule directories are empty

Run `make init-submodules`. If that fails, run:

```sh
git submodule update --init --recursive
```

### First container build is very slow

The first time VS Code builds the container it downloads the ARM GCC toolchain (~1 GB) and Eclipse. This is normal. Subsequent starts reuse the cached container image and are fast.

### VS Code does not offer "Reopen in Container"

Ensure the **Dev Containers** extension is installed (see [Install Required Software](#install-required-software)). Also confirm that Docker Desktop is running.
