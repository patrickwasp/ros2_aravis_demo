# ROS2 Aravis Camera Demo

This project demonstrates the integration of ROS2 with Foxglove Studio for visualization and Aravis for camera support.

## Prerequisites

Before you begin, ensure you have the following installed:
- Docker and Docker Compose (Follow the [official Docker installation guide](https://docs.docker.com/get-docker/))
- Foxglove Studio ([Download Foxglove Studio](https://foxglove.dev/download))

## Hardware Requirements

- An Aravis-compatible USB camera.

## Setting Up the Aravis Camera

1. **Connect your USB camera** to your computer.

2. **Modify UDEV Rules**: If necessary, update the `drivers/udev/90-aravis.rules` file to match your camera's vendor and product ID. This step ensures your Docker container can access the camera.

## Running the Project

1. **Build and Run Docker Containers**:

    Navigate to the project's root directory and run the following command:

    ```bash
    docker-compose up --build
    ```

    This command builds the Docker images and starts the containers defined in `docker-compose.yaml`.

2. **Viewing the Camera Feed in Foxglove Studio**:

    - Open Foxglove Studio.
    - Connect to the ROS2 data bridge by selecting "Open Connection" and entering the URL `ws://localhost:8765`.

## Additional Notes

- **ROS_DOMAIN_ID**: Both services in `docker-compose.yaml` are configured to use `ROS_DOMAIN_ID=7`. Ensure this matches your ROS2 network configuration if you're integrating with existing ROS2 systems.
- **Privileged Mode**: Containers are run in privileged mode to ensure access to USB devices. 


