#!/bin/bash

set -e

python3.12 -m venv .venv
echo "venv created"

source .venv/bin/activate
echo "sourced"

pip install -r requirements.txt
echo "installed"

deactivate
echo "deactivated"

