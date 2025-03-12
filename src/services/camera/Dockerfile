# Use Bullseye image compatable with RPI and gcc
FROM python:3.12-slim-bullseye

# Install dependencies, including graphics libraries
RUN apt-get update && \
    apt-get install -y libgl1-mesa-glx && \
    apt-get install -y libglib2.0-0 && \
    pip install opencv-python && \
    rm -rf /var/lib/apt/lists/*

# Create a working directory in the container
WORKDIR /app

# Copy the camera script into the container
COPY ./src/services/camera/ .
COPY ./src/client/ ./client
COPY ./src/parts/camera.py .


# Install dependencies
RUN pip install -r requirements.txt --no-cache-dir
