FROM ubuntu:20.04

ENV SIM_TTY_1 ""
ENV SIM_TTY_1_ADDRESS ""
ENV SIM_TTY_1_PORT ""

WORKDIR /home/emulator/simple_emulator

RUN apt-get update \
    && apt-get install -y socat python3-pip

COPY . .
RUN chmod +x ./docker/entrypoint.sh

RUN pip3 install .

ENTRYPOINT ["./docker/entrypoint.sh"]

