#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BOOST_VERSION="1.84.0"
BOOST_UNDERSCORE="${BOOST_VERSION//./_}"
BOOST_DIR="$ROOT_DIR/third_party/boost"
SRC_DIR="$BOOST_DIR/src"
INSTALL_DIR="$BOOST_DIR/install"
TMP_INSTALL="/tmp/boost_install_$$"

mkdir -p "$BOOST_DIR"
cd "$BOOST_DIR"

if [[ ! -d "$SRC_DIR" ]]; then
  echo "Downloading Boost ${BOOST_VERSION}..."
  mkdir -p "$SRC_DIR"

  ARCHIVE="$BOOST_DIR/boost.tar.gz"
  URLS=(
    "https://boostorg.jfrog.io/artifactory/main/release/${BOOST_VERSION}/source/boost_${BOOST_UNDERSCORE}.tar.gz"
    "https://sourceforge.net/projects/boost/files/boost/${BOOST_VERSION}/boost_${BOOST_UNDERSCORE}.tar.gz/download"
  )

  for URL in "${URLS[@]}"; do
    echo "Trying $URL"
    curl -L "$URL" -o "$ARCHIVE"
    if tar -tzf "$ARCHIVE" >/dev/null 2>&1; then
      tar -xzf "$ARCHIVE" -C "$SRC_DIR" --strip-components=1
      rm -f "$ARCHIVE"
      break
    else
      echo "Download failed or invalid archive, trying next URL..."
    fi
  done

  if [[ ! -f "$SRC_DIR/bootstrap.sh" ]]; then
    echo "Failed to download a valid Boost archive." >&2
    exit 1
  fi
fi

# If a previous run created an empty/invalid src dir, recover
if [[ ! -f "$SRC_DIR/bootstrap.sh" ]]; then
  echo "Boost source missing or incomplete. Re-downloading..."
  rm -rf "$SRC_DIR"
  "$0"
  exit 0
fi

cd "$SRC_DIR"

if [[ ! -x "./b2" ]]; then
  ./bootstrap.sh --with-libraries=system,filesystem,serialization,program_options
fi

cat > project-config.jam <<EOF
# B2 Configuration
import option ;
import feature ;

if ! clang in [ feature.values <toolset> ]
{
    using clang ;
}

project : default-build <toolset>clang ;
libraries =  --with-system --with-filesystem --with-serialization --with-program_options ;
option.set prefix : ${TMP_INSTALL} ;
option.set exec-prefix : ${TMP_INSTALL} ;
option.set libdir : ${TMP_INSTALL}/lib ;
option.set includedir : ${TMP_INSTALL}/include ;
option.set keep-going : false ;
EOF

./b2 install \
  link=static \
  threading=multi \
  runtime-link=static \
  cxxflags=-fPIC

rm -rf "$INSTALL_DIR"
mkdir -p "$INSTALL_DIR"
cp -a "$TMP_INSTALL/." "$INSTALL_DIR/"
rm -rf "$TMP_INSTALL"

echo "Boost installed to $INSTALL_DIR"
