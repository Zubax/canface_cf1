# Production Testing Software

This directory contains production testing application for Zubax Babel.

This application is based on DrWatson - a software framework written in Python for automating hardware testing tasks
in production.

## Installation

This application requires an Ubuntu-based GNU/Linux distribution and an old version of Python ca. 3.6.

A working GNU ARM toolchain version 7 or newer must be installed.
Please read this for installation instructions: <https://kb.zubax.com/x/NoEh>.

The OS must be configured to allow access to virtual serial ports to regular users without superuser privileges.
Read this tutorial for setup instructions: <https://kb.zubax.com/x/N4Ah>.

After checking out this repository and all of its submodules (`git clone --recursive <URL of the repository>`),
execute `./setup.sh`, and you're ready to get started:

```bash
./drwatson_babel.py --help
```

## Other documentation

Refer to <https://kb.zubax.com/> to find more documentation about anything.
