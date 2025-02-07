# Use lightweight alpine image
FROM python:3.12-alpine

# Copy files to working directory and install dependencies
WORKDIR /app
COPY . .
RUN pip install -r requirements.txt --no-cache-dir
