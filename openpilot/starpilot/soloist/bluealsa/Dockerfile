FROM ubuntu:24.04@sha256:33ceb71981b602c1a7443a53469e4dba065f7503eab3078a2d7a57a2ab987517
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential autoconf automake libtool pkg-config git ca-certificates \
    libasound2-dev libbluetooth-dev libdbus-1-dev libglib2.0-dev libsbc-dev \
    libldacbt-enc-dev libldacbt-abr-dev libbsd-dev check dbus alsa-utils libsamplerate0-dev \
    && rm -rf /var/lib/apt/lists/*
