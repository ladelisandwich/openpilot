# The comma: builds whatever openpilot checkout you point it at for x86 and runs it against the simulated car.
# System packages only; the build, its venv and its model live in the `comma-work` volume (see prepare_build.py).
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1

# openpilot's native build (tools/install_ubuntu_dependencies.sh, minus Qt, which only cabana wants) + libx264
# for the encoder targets + the screen: Xvfb, x11vnc and noVNC to show the comma's UI in a browser
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates clang build-essential curl git git-lfs locales rsync xz-utils \
      libssl-dev libcurl4-openssl-dev \
      gcc-arm-none-eabi libnewlib-arm-none-eabi capnproto libcapnp-dev \
      ffmpeg libavformat-dev libavcodec-dev libavdevice-dev libavutil-dev libavfilter-dev libx264-dev \
      libbz2-dev libeigen3-dev libffi-dev libgles2-mesa-dev libglfw3-dev libglib2.0-0 libjpeg-dev \
      libncurses-dev libusb-1.0-0-dev libzmq3-dev libzstd-dev libsqlite3-dev portaudio19-dev gettext \
      opencl-headers ocl-icd-libopencl1 ocl-icd-opencl-dev pocl-opencl-icd intel-opencl-icd \
      python3 python3-dev python3-venv \
      xvfb x11vnc novnc websockify libgl1-mesa-dri libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/* \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && locale-gen \
    && mkdir -p /etc/OpenCL/vendors && echo libnvidia-opencl.so.1 > /etc/OpenCL/vendors/nvidia.icd

ENV LANG=en_US.UTF-8 LANGUAGE=en_US:en LC_ALL=en_US.UTF-8

# uv, which the build's uv.lock is resolved with
RUN python3 -m venv /opt/uv && /opt/uv/bin/pip install --no-cache-dir "uv>=0.8,<1" && ln -s /opt/uv/bin/uv /usr/local/bin/uv

# NVIDIA GPUs (compose.nvidia.yml): driver libraries for OpenCL + GL come in through the container runtime
ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

RUN git config --global --add safe.directory '*' && git config --global init.defaultBranch master \
    && git config --global user.email sim@localhost && git config --global user.name "mazda sim"

WORKDIR /work
EXPOSE 6080 8082
ENTRYPOINT ["bash", "/sim/mazda_sim/docker/comma_entrypoint.sh"]
