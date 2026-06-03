#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$SCRIPT_DIR/deb-package"
OUTPUT="$SCRIPT_DIR/d1-follow-control_1.0.deb"

chmod 755 "$PKG_DIR/DEBIAN"
chmod 755 "$PKG_DIR/DEBIAN/postinst"
chmod 755 "$PKG_DIR/DEBIAN/prerm"
chmod 755 "$PKG_DIR/DEBIAN/postrm"

dpkg-deb --build "$PKG_DIR" "$OUTPUT"
sudo dpkg -i "$OUTPUT"
