#!/bin/sh
set -e

REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
cd "$REPO_ROOT"

# Configure commit template and hooks path
git config commit.template .gitmessage
git config core.hooksPath .githooks

# Ensure hooks are executable
if [ -d .githooks ]; then
  chmod +x .githooks/* 2>/dev/null || true
fi

echo "Configured commit.template and core.hooksPath. Hooks are executable."
