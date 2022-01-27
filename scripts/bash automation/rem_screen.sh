#!/usr/bin/env bash
gnome-terminal --working-directory="/usr/share/X11/xorg.conf.d/" -- sudo mv 10-headless.conf.bak 10-headless.conf &
sync
