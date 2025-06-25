# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Set the maintainer label
LABEL maintainer="your-email@example.com"

# Update package lists and install necessary build tools and dependencies
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    libpthread-stubs0-dev \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /app

# Copy the project files to the working directory
COPY . /app

# Build the project
RUN mkdir -p build && \
    cd build && \
    cmake .. && \
    make

# Command to run the project (adjust as necessary)
CMD ["./build/eu_motor"]
