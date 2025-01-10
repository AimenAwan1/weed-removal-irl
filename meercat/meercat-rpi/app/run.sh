#!/bin/bash
set -e

source ${VENV_NAME}/bin/activate

# ensures that the proto files can find each other
export PYTHONPATH=${APP_NAME}/src/proto:${PYTHONPATH}

# starts the application using cfg default entry point
python3 ${APP_NAME}/${ENTRY}
