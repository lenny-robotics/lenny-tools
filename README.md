# lenny-tools
Basic tools for lenny repositories

## Update cmake version in Ubuntu
To update the cmake version to 3.22 (required), run the following commands in the terminal:

- `sudo wget https://github.com/Kitware/CMake/releases/download/v3.22.6/cmake-3.22.6-linux-x86_64.sh -q -O /tmp/cmake-install.sh`
- `sudo chmod u+x /tmp/cmake-install.sh`
- `sudo mkdir /tmp/cmake && sudo /tmp/cmake-install.sh --skip-license --prefix=/tmp/cmake`
- `sudo rm /tmp/cmake-install.sh && sudo rm /usr/bin/cmake`
- `sudo ln -s /tmp/cmake/bin/cmake /usr/bin/`

These commands download the required installation file, executes it, removes the old version, and creates a symbolic link to the new version.
