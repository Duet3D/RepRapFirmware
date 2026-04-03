FROM mcr.microsoft.com/devcontainers/cpp:2-ubuntu24.04

ENV ARM_GNU_TOOLCHAIN_VERSION="15.2.rel1"
ENV ARM_GNU_TOOLCHAIN_DIR="/opt/arm-gnu-toolchain-${ARM_GNU_TOOLCHAIN_VERSION}"
ENV ECLIPSE_RELEASE="2026-03"
ENV ECLIPSE_PACKAGE="eclipse-cpp-${ECLIPSE_RELEASE}-R"
ENV ECLIPSE_DIR="/opt/eclipse"
ENV PATH="${ECLIPSE_DIR}:${ARM_GNU_TOOLCHAIN_DIR}/bin:${PATH}"

COPY ./install-eclipse.sh /tmp/
COPY ./install-arm-gcc.sh /tmp/

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends ca-certificates curl default-jre tar xz-utils \
    && rm -rf /var/lib/apt/lists/*

RUN echo "Installing Eclipse ${ECLIPSE_RELEASE}..." \
    && chmod +x /tmp/install-eclipse.sh \
    && /tmp/install-eclipse.sh \
    && rm -f /tmp/install-eclipse.sh

RUN echo "Installing ARM GNU Toolchain version ${ARM_GNU_TOOLCHAIN_VERSION}..." \
    && chmod +x /tmp/install-arm-gcc.sh \
    && /tmp/install-arm-gcc.sh \
    && rm -f /tmp/install-arm-gcc.sh

# [Optional] Uncomment this section to install additional vcpkg ports.
# RUN su vscode -c "${VCPKG_ROOT}/vcpkg install <your-port-name-here>"

# [Optional] Uncomment this section to install additional packages.
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends <your-package-list-here>
