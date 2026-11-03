#!/usr/bin/env bash
# Once per clone (rover laptop, base station, your computer): git then runs tools/git-hooks/ after every
# `git switch <branch>` and `git pull`, which rebuilds ros2_ws and prints what the branch needs (link, firmware).
#   tools/install_hooks.sh            install, rebuild now and show the summary
#   git config --unset core.hooksPath # uninstall
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
chmod +x tools/git-hooks/*
git config core.hooksPath tools/git-hooks
echo "git hooks installed (core.hooksPath = tools/git-hooks)"
tools/git-hooks/after-update.sh
