#!/bin/bash

cd ~/zephyrproject
source .venv/bin/activate
west build -p always ~/zephyrproject/zephyr/zephyr-rtos -d ~/zephyrproject/zephyr/zephyr-rtos/build 

