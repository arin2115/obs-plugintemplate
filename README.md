# UDP MJPEG Preview Streamer for OBS

## Introduction

This OBS Studio plugin streams the preview/program output over UDP as MJPEG-compressed video frames. It's designed for ultra-low latency streaming at high frame rates.

### Features

* **Low latency UDP streaming**: Minimal delay for real-time applications
* **MJPEG compression**: Built-in JPEG encoder (no external dependencies)
* **Configurable output**: 700x700 resolution at 144fps by default
* **Hotkey support**: Toggle streaming on/off with a customizable hotkey
* **Cross-platform**: Works on Windows, macOS, and Linux

### Default Configuration

| Setting | Default Value |
|---------|---------------|
| Resolution | 700x700 |
| Frame Rate | 144 fps |
| JPEG Quality | 80 |
| Destination Host | 127.0.0.1 |
| Destination Port | 5000 |

## Usage

1. Install the plugin into OBS Studio
2. The plugin automatically creates a UDP MJPEG output
3. Set up the hotkey "Toggle UDP MJPEG Streaming" in OBS Settings → Hotkeys
4. Press the hotkey to start/stop streaming
5. Receive the MJPEG stream on the destination using a UDP receiver

### Receiving the Stream

You can receive the stream using tools like FFmpeg or Python. Example using FFmpeg:

```bash
ffmpeg -f mjpeg -i udp://127.0.0.1:5000 -f sdl2 "Preview"
```

Or with Python:

```python
import socket
import cv2
import numpy as np

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 5000))

while True:
    data, addr = sock.recvfrom(65535)
    # For single-packet frames
    if data[:2] == b'\xff\xd8':  # JPEG SOI marker
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            cv2.imshow('Preview', img)
            if cv2.waitKey(1) == 27:
                break
```

## Building on Windows

### Prerequisites

1. **Visual Studio 2022** (Community edition or higher) with C++ desktop development workload
2. **CMake 3.28 or higher** (https://cmake.org/download/)
3. **Git** (https://git-scm.com/download/win)

### Build Steps

1. **Clone the repository:**
   ```cmd
   git clone https://github.com/your-repo/obs-udp-mjpeg-streamer.git
   cd obs-udp-mjpeg-streamer
   ```

2. **Download OBS Studio dependencies:**
   
   The build system will automatically download OBS dependencies. Alternatively, set `OBS_SOURCE_DIR` to your OBS Studio source directory.

3. **Configure with CMake:**
   ```cmd
   cmake --preset windows-x64
   ```

4. **Build the project:**
   ```cmd
   cmake --build build_x64 --config RelWithDebInfo
   ```

5. **Find the output:**
   The compiled plugin DLL will be in:
   ```
   build_x64\RelWithDebInfo\obs-udp-mjpeg-streamer.dll
   ```

### Installation

1. Copy the compiled DLL to your OBS plugins folder:
   ```
   C:\Program Files\obs-studio\obs-plugins\64bit\
   ```

2. Copy any locale files from `data\locale\` to:
   ```
   C:\Program Files\obs-studio\data\obs-plugins\obs-udp-mjpeg-streamer\locale\
   ```

3. Restart OBS Studio

### Building with Visual Studio GUI

1. Open Visual Studio 2022
2. Select "Open a local folder" and navigate to the plugin directory
3. Visual Studio will detect the CMake project
4. Select the "windows-x64" preset from the configuration dropdown
5. Build → Build All

### Troubleshooting

**CMake configuration fails:**
- Ensure CMake 3.28+ is installed and in your PATH
- Make sure Visual Studio 2022 is installed with C++ workload

**Missing OBS headers:**
- The build system downloads OBS dependencies automatically
- If issues persist, manually download OBS source and set `OBS_SOURCE_DIR`

**Plugin not loading:**
- Check OBS logs for error messages
- Ensure the DLL is in the correct plugins folder
- Make sure you're using the 64-bit version of OBS

## Supported Build Environments

| Platform  | Tool   |
|-----------|--------|
| Windows   | Visual Studio 17 2022 |
| macOS     | XCode 16.0 |
| Windows, macOS  | CMake 3.30.5 |
| Ubuntu 24.04 | CMake 3.28.3 |
| Ubuntu 24.04 | `ninja-build` |
| Ubuntu 24.04 | `pkg-config`
| Ubuntu 24.04 | `build-essential` |

## Documentation

For more information on OBS plugin development:

* [OBS Plugin Template Wiki](https://github.com/obsproject/obs-plugintemplate/wiki)
* [OBS Plugin Development Documentation](https://obsproject.com/docs/plugins.html)

## GitHub Actions & CI

Default GitHub Actions workflows are available for the following repository actions:

* `push`: Run for commits or tags pushed to `master` or `main` branches.
* `pr-pull`: Run when a Pull Request has been pushed or synchronized.
* `dispatch`: Run when triggered by the workflow dispatch in GitHub's user interface.
* `build-project`: Builds the actual project and is triggered by other workflows.
* `check-format`: Checks CMake and plugin source code formatting and is triggered by other workflows.

### Retrieving build artifacts

Successful builds on GitHub Actions will produce build artifacts that can be downloaded for testing. These artifacts are commonly simple archives and will not contain package installers or installation programs.

### Building a Release

To create a release, an appropriately named tag needs to be pushed to the `main`/`master` branch using semantic versioning (e.g., `12.3.4`, `23.4.5-beta2`). A draft release will be created on the associated repository with generated installer packages or installation programs attached as release artifacts.

## License

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
