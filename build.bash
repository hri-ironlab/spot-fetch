ARCH=$(uname -m)
if [ -z "$1" ]; then
	:
else
	ARCH="$1"
fi
if [ "$ARCH" = "aarch64" ]; then # Jetson
	echo "Building for aarch64 (Jetson or similar)"
elif [ "$ARCH" = "x86_64" ]; then # non-Jetson
	echo "Building for x86_64 (likely not running on robot)"
else
	read -p "What is your architecture (x86_64 or aarch64 probably)? " ARCH
fi
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/$ARCH-linux-gnu/libpython3.6m.so
