#!/bin/bash
# post_install.sh — configuration post-installation Debian

echo "[post-install] Configuration RawCaptureWithConfig..."

CONF_DIR="/etc/rawcapture"
CONF_SRC="/usr/share/rawcapture/configParam.ini"

if [ ! -d "$CONF_DIR" ]; then
    mkdir -p "$CONF_DIR"
    echo "Création du dossier $CONF_DIR"
fi

if [ -f "$CONF_SRC" ]; then
    cp "$CONF_SRC" "$CONF_DIR/"
    echo "Config copiée vers $CONF_DIR/"
else
    echo "⚠️ Aucun fichier de configuration trouvé à $CONF_SRC"
fi
