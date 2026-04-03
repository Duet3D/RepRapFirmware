FROM mcr.microsoft.com/devcontainers/cpp:2-ubuntu24.04

ARG REINSTALL_CMAKE_VERSION_FROM_SOURCE="none"
ENV ARM_GNU_TOOLCHAIN_VERSION="15.2.rel1"
ENV ARM_GNU_TOOLCHAIN_DIR="/opt/arm-gnu-toolchain-${ARM_GNU_TOOLCHAIN_VERSION}"
ENV PATH="${ARM_GNU_TOOLCHAIN_DIR}/bin:${PATH}"

# Optionally install the cmake for vcpkg
COPY ./reinstall-cmake.sh /tmp/

RUN if [ "${REINSTALL_CMAKE_VERSION_FROM_SOURCE}" != "none" ]; then \
        chmod +x /tmp/reinstall-cmake.sh && /tmp/reinstall-cmake.sh ${REINSTALL_CMAKE_VERSION_FROM_SOURCE}; \
    fi \
    && rm -f /tmp/reinstall-cmake.sh

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends ca-certificates curl xz-utils \
    && case "$(dpkg --print-architecture)" in \
        amd64) arm_gnu_toolchain_host_arch="x86_64" ;; \
        arm64) arm_gnu_toolchain_host_arch="aarch64" ;; \
        *) echo "Unsupported architecture: $(dpkg --print-architecture)" >&2; exit 1 ;; \
    esac \
    && arm_gnu_toolchain_archive="arm-gnu-toolchain-${ARM_GNU_TOOLCHAIN_VERSION}-${arm_gnu_toolchain_host_arch}-arm-none-eabi" \
    && arm_gnu_toolchain_url="https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_GNU_TOOLCHAIN_VERSION}/binrel/${arm_gnu_toolchain_archive}.tar.xz" \
    && curl -L "${arm_gnu_toolchain_url}" -o "/tmp/${arm_gnu_toolchain_archive}.tar.xz" \
    && tar -xJf "/tmp/${arm_gnu_toolchain_archive}.tar.xz" -C /opt \
    && ln -s "/opt/${arm_gnu_toolchain_archive}" "${ARM_GNU_TOOLCHAIN_DIR}" \
    && mkdir -p /workspaces \
    && ln -s "/opt/${arm_gnu_toolchain_archive}" "/workspaces/${arm_gnu_toolchain_archive}" \
    && rm -f "/tmp/${arm_gnu_toolchain_archive}.tar.xz" \
    && rm -rf /var/lib/apt/lists/*

# [Optional] Uncomment this section to install additional vcpkg ports.
# RUN su vscode -c "${VCPKG_ROOT}/vcpkg install <your-port-name-here>"

# [Optional] Uncomment this section to install additional packages.
# RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
#     && apt-get -y install --no-install-recommends <your-package-list-here>
