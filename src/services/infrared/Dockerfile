# Use Bullseye image compatable with RPI and gcc
FROM python:3.12-slim-bullseye

# Install required packages for gpiozero to work
RUN apt-get update && \
    apt-get install -y python3-gpiozero && \
    apt-get install -y gcc && \
    rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app

# Docker-compose sets context at root level
# The infrared sensor depends on parts, client, and its main source files
COPY ./src/services/infrared/ .
COPY ./src/client/ ./client
COPY ./src/parts/ ./parts

# Install dependencies
RUN pip install -r requirements.txt --no-cache-dir