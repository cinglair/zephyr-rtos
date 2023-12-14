#!/bin/bash

cd ~/zephyrproject
source .venv/bin/activate
west flash -d ~/zephyrproject/zephyr/zephyr-rtos/build/

