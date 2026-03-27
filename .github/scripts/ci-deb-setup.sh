#!/usr/bin/env bash
# Prepare APT and install a CMake >= 3.15 inside CI containers (Debian/Ubuntu).
set -euo pipefail

OLD_RELEASE="${1:-0}"

export DEBIAN_FRONTEND=noninteractive

if [[ -f /etc/os-release ]]; then
  # shellcheck source=/dev/null
  . /etc/os-release
  if [[ "${ID:-}" == "debian" && "${VERSION_ID:-}" == "10" ]]; then
    echo "deb http://archive.debian.org/debian buster main" >/etc/apt/sources.list
    echo "deb http://archive.debian.org/debian-security buster/updates main" >>/etc/apt/sources.list
    printf 'Acquire::Check-Valid-Until "false";\n' >/etc/apt/apt.conf.d/99no-check-valid
  fi
fi

if [[ "$OLD_RELEASE" == "1" ]]; then
  sed -i 's|archive.ubuntu.com|old-releases.ubuntu.com|g' /etc/apt/sources.list || true
  sed -i 's|security.ubuntu.com|old-releases.ubuntu.com|g' /etc/apt/sources.list || true
  mkdir -p /etc/apt/apt.conf.d
  printf 'Acquire::Check-Valid-Until "false";\n' >/etc/apt/apt.conf.d/99no-check-valid
fi

apt-get update -qq
apt-get install -y -qq --no-install-recommends \
  ca-certificates \
  curl \
  build-essential \
  cmake \
  dpkg-dev \
  file \
  make \
  pkg-config

need_bootstrap_cmake() {
  if ! command -v cmake >/dev/null 2>&1; then
    return 0
  fi
  ver="$(cmake --version | head -1 | awk '{print $3}')"
  major="${ver%%.*}"
  minor="${ver#*.}"
  minor="${minor%%.*}"
  if [[ "$major" -lt 3 ]]; then return 0; fi
  if [[ "$major" -eq 3 && "$minor" -lt 15 ]]; then return 0; fi
  return 1
}

if need_bootstrap_cmake; then
  CMAKE_VER="3.28.6"
  case "$(uname -m)" in
    x86_64) CMAKE_SUFFIX="linux-x86_64" ;;
    aarch64) CMAKE_SUFFIX="linux-aarch64" ;;
    armv7l | armv6l) CMAKE_SUFFIX="linux-armv7l" ;;
    *)
      echo "unsupported machine $(uname -m) for Kitware CMake bootstrap"
      exit 1
      ;;
  esac
  CMAKE_DIR="/opt/cmake-${CMAKE_VER}-${CMAKE_SUFFIX}"
  echo "Installing CMake ${CMAKE_VER} (${CMAKE_SUFFIX}) to ${CMAKE_DIR}"
  curl -fsSL "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VER}/cmake-${CMAKE_VER}-${CMAKE_SUFFIX}.tar.gz" \
    | tar xz -C /opt
  mkdir -p /usr/local/bin
  for _tool in cmake ctest cpack; do
    ln -sf "${CMAKE_DIR}/bin/${_tool}" "/usr/local/bin/${_tool}"
  done
fi

cmake --version
