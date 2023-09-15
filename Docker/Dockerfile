FROM nvcr.io/nvidia/isaac-sim:2022.2.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    locales \
    xterm \
    dbus-x11 \
    terminator \
    sudo \
    unzip \
    lsb-release \
    curl \
    net-tools \
    software-properties-common \
    subversion \
    libssl-dev \
    htop \
    gedit \
    gdb \
    valgrind \
    build-essential \
    bash-completion \
    python3-pip \
    vim \
    cmake \
    sl \
    wget && \
    apt-get clean && rm -rf /var/lib/apt/list*

ENV ISAACSIM_PATH="${HOME}/isaac-sim"
ENV ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"

RUN cd .. && \
	cd isaac-sim/ && \
	echo "alias PYTHON_PATH=/isaac-sim/python.sh" >> ${HOME}/.bashrc
    #git clone https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs.git && \

#COPY ./entrypoint.sh /root/entrypoint.sh

SHELL ["/bin/bash", "-c"]

#ENTRYPOINT ["/root/entrypoint.sh"]
ENTRYPOINT ["/bin/bash"]
