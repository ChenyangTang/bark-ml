#!/bin/bash
virtualenv -p python3.5 ./python/venv
source ./python/venv/bin/activate && pip install -r utils/requirements.txt
