FROM ubuntu:20.04

ENV EMU_SIM_TTY_1 ""
ENV EMU_SIM_TTY_1_PORT ""

WORKDIR /home/driver/simple_driver

RUN apt-get update \
    && apt-get install -y socat python3-pip

COPY . .
RUN chmod +x ./docker/entrypoint.sh

RUN pip3 install .

ENTRYPOINT ["./docker/entrypoint.sh"]

