# The Mazda: the CX-9's ECUs, the TI, the road and the physics. Optionally renders the world with MetaDrive.
# The simulator's own code is mounted from the repo at run time (compose.yml), so editing it needs no rebuild.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates python3 python3-venv \
      xvfb libgl1 libglx-mesa0 libgl1-mesa-dri libegl1 libglib2.0-0 libsm6 libxext6 libxrender1 \
    && rm -rf /var/lib/apt/lists/*

# numpy..pycryptodome: opendbc (the car's DBC). aiohttp: the dashboard. MetaDrive: the rendered world (same
# minimal wheel openpilot's tools use)
RUN python3 -m venv /opt/venv && /opt/venv/bin/pip install --no-cache-dir \
      "numpy>=2.0" crcmod tqdm "pycapnp==2.1.0" pycryptodome aiohttp opencv-python-headless \
      "metadrive-simulator @ https://github.com/commaai/metadrive/releases/download/MetaDrive-minimal-0.4.2.4/metadrive_simulator-0.4.2.4-py3-none-any.whl"
ENV PATH=/opt/venv/bin:$PATH

ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=graphics,utility

WORKDIR /sim
EXPOSE 8770
ENTRYPOINT ["bash", "/sim/mazda_sim/docker/mazda_entrypoint.sh"]
