#!/bin/bash
set -e

if [[ "$SIM_TTY_1" && "$SIM_TTY_1_ADDRESS" && "$SIM_TTY_1_PORT" ]];
then
  {
    socat pty,link="$SIM_TTY_1" tcp:"$SIM_TTY_1_ADDRESS":"$SIM_TTY_1_PORT"
  }&
fi

exec "$@"
