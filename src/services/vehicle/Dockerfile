# Use a slim python image
FROM python:3.12-alpine

# Set the working directory
WORKDIR /app

# Docker-compose sets context at root level
# The vehicle depends on parts, client, and its main source files
COPY ./src/services/vehicle .
COPY ./src/client ./client
COPY ./src/parts/ ./parts
COPY ./src/pipelines/ ./pipelines

# Install dependencies
RUN pip install -r requirements.txt --no-cache-dir