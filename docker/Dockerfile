# ------------------------------------------------------------------------------
# Base Image
# ------------------------------------------------------------------------------

ARG BASE_IMAGE=ros:humble
FROM ${BASE_IMAGE}

# ------------------------------------------------------------------------------
# Runtime Configuration
# ------------------------------------------------------------------------------

USER root
SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

ARG USER_ID=1000
ARG GROUP_ID=1000

# Create a group/user and home directory for the developer user. This user will
# be the default user for running the container. Using `|| true` makes the
# commands idempotent in case the group/user already exists in a derived image.
RUN groupadd -g ${GROUP_ID} devgroup || true \
 && useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash devuser || true \
 && mkdir -p /home/devuser/.ros \
 && chown -R ${USER_ID}:${GROUP_ID} /home/devuser

ENV HOME=/home/devuser


# ------------------------------------------------------------------------------
# Workspace Setup
# ------------------------------------------------------------------------------

ENV WORKSPACE=/ros2_ws
WORKDIR ${WORKSPACE}

# ------------------------------------------------------------------------------
# Copy Workspace Files
# ------------------------------------------------------------------------------

COPY . .

# ------------------------------------------------------------------------------
# Default Startup Command
# ------------------------------------------------------------------------------

CMD ["bash"]
