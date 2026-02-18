#setup.sh 
#make a venv and install requirements from requirements.txt

#!/usr/bin/env bash
set -euo pipefail

# Usage: ./setup.sh [venv_dir] [--check]
VENV_DIR="${1:-venv}"
REQ_FILE="requirements.txt"

# allow --check as first/second arg
for a in "$@"; do
  if [ "$a" = "--check" ]; then CHECK_ONLY=true; fi
done
CHECK_ONLY=${CHECK_ONLY:-false}

PY=python3
if ! command -v "$PY" >/dev/null 2>&1; then
  echo "python3 not found"
  exit 1
fi

if [ ! -d "$VENV_DIR" ]; then
  echo "Creating venv at ./$VENV_DIR"
  "$PY" -m venv "$VENV_DIR"
fi

# activate venv for this script
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"

if [ ! -f "$REQ_FILE" ]; then
  echo "No $REQ_FILE found in $(pwd)"
  exit 0
fi

if [ "$CHECK_ONLY" = true ]; then
  echo "Checking installed packages against $REQ_FILE..."
  MISSING=0
  while IFS= read -r line || [ -n "$line" ]; do
    # skip comments and empty lines
    line="$(echo "$line" | sed 's/#.*//; s/^[ \t]*//; s/[ \t]*$//')"
    [ -z "$line" ] && continue
    pkg="$(echo "$line" | sed 's/[<=>!].*$//' | sed 's/\[.*\]//')"
    if ! pip show "$pkg" >/dev/null 2>&1; then
      echo "MISSING: $line"
      MISSING=1
    fi
  done < "$REQ_FILE"
  if [ "$MISSING" -eq 0 ]; then
    echo "All packages from $REQ_FILE are installed (by name)."
  else
    echo "Some packages are missing; run './setup.sh $VENV_DIR' to install them."
    exit 2
  fi
else
  echo "Installing requirements from $REQ_FILE into venv..."
  python -m pip install --upgrade pip
  pip install -r "$REQ_FILE"
  echo "Done."
fi
