ARG PYTHON_VERSION=3.10
FROM python:${PYTHON_VERSION}-slim as rustmap3d

ARG UID=1000
ARG GID=100
ARG USER=rust
RUN echo "Building docker with uid:${UID} - gid:${GID} - user:${USER}"

RUN apt-get update -y && \
    apt-get install -y \
        build-essential \
        curl \
        gdb \
        git
ENV PATH="/home/rust/.cargo/bin:${PATH}"
ENV VENV=/home/${USER}/.venv

RUN useradd --no-log-init --create-home --shell /bin/bash -G ${GID} -u ${UID} ${USER}

USER ${USER}
WORKDIR /home/${USER}
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y && \
    python -m venv ${VENV} && \
    ${VENV}/bin/python -m pip install maturin && \
    echo "source ${VENV}/bin/activate" >> /home/rust/.bashrc

WORKDIR /home/${USER}/rustmap3d

COPY --chown=${USER} . .
RUN ${VENV}/bin/python -m pip install pip --upgrade && \
    ${VENV}/bin/python -m pip install -r python/requirements.txt