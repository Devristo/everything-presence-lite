FROM esphome/esphome:latest

RUN apt update && apt-get install -y build-essential nano \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*