FROM ubuntu:22.04 as rustmap3d

ARG UID=1000
ARG GID=100
ARG USER=rust
RUN echo "Building docker with uid:${UID} - gid:${GID} - user:${USER}"

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update -y && \
    apt-get install -yq --no-install-recommends \
        make \
        build-essential \
        libssl-dev \
        zlib1g-dev \
        libbz2-dev \
        libreadline-dev \
        libsqlite3-dev \
        wget \
        curl \
        llvm \
        libncurses5-dev \
        xz-utils \
        tk-dev \
        libxml2-dev \
        libxmlsec1-dev \
        git \
        ca-certificates \
        libffi-dev \
        mingw-w64 \
    && apt-get clean autoclean \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/* \
    && rm -f /var/cache/apt/archives/*.deb

RUN useradd --no-log-init --create-home --shell /bin/bash -G ${GID} -u ${UID} ${USER}

USER ${USER}
WORKDIR /home/${USER}

# https://github.com/dhermes/python-multi/blob/main/src/Dockerfile
RUN git clone https://github.com/pyenv/pyenv /home/${USER}/.pyenv/

# Install the desired versions of Python.
RUN for PYTHON_VERSION in 3.8.18 3.9.18 3.10.13 3.11.8 3.12.2; do \
  set -ex \
    && /home/${USER}/.pyenv/bin/pyenv install ${PYTHON_VERSION} \
    && /home/${USER}/.pyenv/versions/${PYTHON_VERSION}/bin/python -m pip install --upgrade pip \
  ; done

# Add to PATH, in order of lowest precedence to highest.
ENV PATH=/home/${USER}/.pyenv/versions/3.8.18/bin:${PATH}
ENV PATH=/home/${USER}/.pyenv/versions/3.9.18/bin:${PATH}
ENV PATH=/home/${USER}/.pyenv/versions/3.10.13/bin:${PATH}
ENV PATH=/home/${USER}/.pyenv/versions/3.11.8/bin:${PATH}
ENV PATH=/home/${USER}/.pyenv/versions/3.12.2/bin:${PATH}
ENV PATH=/home/rust/.cargo/bin:${PATH}
ENV VENV=/home/${USER}/.venv

RUN curl https://sh.rustup.rs -sSf | bash -s -- -y && \
    python -m venv ${VENV} && \
    echo "source ${VENV}/bin/activate" >> /home/rust/.bashrc

WORKDIR /home/${USER}/rustmap3d

COPY --chown=${USER} . .
RUN ${VENV}/bin/python -m pip install -r python/requirements.txt