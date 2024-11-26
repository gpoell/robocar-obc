FROM python:3.12-alpine
# Install required packages for gpiozero to work
WORKDIR /app
COPY . .
RUN pip install -r requirements.txt --no-cache-dir