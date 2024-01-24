#!/bin/bash
set -e

if [[ $EMU_SIM_TTY_1 && $EMU_SIM_TTY_1_PORT ]];
then
  {
    socat tcp-listen:"$EMU_SIM_TTY_1_PORT" pty,link="$EMU_SIM_TTY_1"
  }&
fi

exec "$@"
