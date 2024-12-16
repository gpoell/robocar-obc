FROM python:3.12-slim-bullseye
# Install required packages for gpiozero to work
RUN apt-get update && \
    apt-get install -y python3-gpiozero && \
    apt-get install -y gcc && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . .
RUN pip install -r requirements.txt --no-cache-dir