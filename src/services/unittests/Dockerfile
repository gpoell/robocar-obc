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
# Copy parts, client, and tests
COPY ./src/tests .
COPY ./src/client ./client
COPY ./src/parts/ ./parts
COPY ./src/services/unittests/requirements.txt .

# Install dependencies
RUN pip install -r ./requirements.txt --no-cache-dir