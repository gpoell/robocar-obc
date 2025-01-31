# Use lightweight alpine image
FROM python:3.12-alpine

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