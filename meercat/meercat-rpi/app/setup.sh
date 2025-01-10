#!/bin/bash
set -e

GREEN="\e[32m"
RED="\e[31m"
RESET="\e[0m"

if [ ! -d ${VENV_NAME} ]; then
    # missing virtual environment
    echo "Creating virtual environment..."
    python3 -m venv ${VENV_NAME}
    echo -e "${GREEN}Done!${RESET}"
fi

echo -e "Activating virtual environment..."
source ${VENV_NAME}/bin/activate
echo -e "${GREEN}Done!${RESET}"

echo "Installing pip dependencies..."
pip3 install -r ${APP_NAME}/requirements.txt
echo -e "${GREEN}Done!${RESET}"

echo "${GREEN}Setup complete!${RESET}"
