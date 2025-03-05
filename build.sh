#!/bin/bash
cmake -S . -B build
cmake --build build
chmod +x run_sender.sh
chmod +x run_receiver.sh
