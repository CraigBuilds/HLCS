#!/bin/bash
# Script to run tests locally using the same Docker container as CI
# This replicates the GitHub Actions CI environment

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Docker image from CI
DOCKER_IMAGE="ros:humble"

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${GREEN}=== Running HLCS Tests Locally ===${NC}"
echo "Using Docker image: ${DOCKER_IMAGE}"
echo "Repository directory: ${SCRIPT_DIR}"
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed or not in PATH${NC}"
    echo "Please install Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

# Check if Docker daemon is running
if ! docker info &> /dev/null; then
    echo -e "${RED}Error: Docker daemon is not running${NC}"
    echo "Please start Docker and try again"
    exit 1
fi

echo -e "${YELLOW}Pulling Docker image if needed...${NC}"
docker pull ${DOCKER_IMAGE}

echo ""
echo -e "${YELLOW}Running tests in Docker container...${NC}"
echo ""

# Run Docker container with the same setup as CI
docker run --rm \
    -v "${SCRIPT_DIR}:/workspace" \
    -w /workspace \
    ${DOCKER_IMAGE} \
    bash -c "
        set -e
        echo '=== Installing dependencies ===' && \
        apt-get update -qq && \
        apt-get install -y -qq python3-pip && \
        pip3 install -q -r requirements.txt && \
        echo '' && \
        echo '=== Building package ===' && \
        source /opt/ros/humble/setup.bash && \
        colcon build --packages-select hlcs && \
        echo '' && \
        echo '=== Running tests ===' && \
        source install/setup.bash && \
        python3 -m pytest test/ -v
    "

# Capture the exit code
EXIT_CODE=$?

echo ""
if [ ${EXIT_CODE} -eq 0 ]; then
    echo -e "${GREEN}=== Tests Passed! ===${NC}"
else
    echo -e "${RED}=== Tests Failed! ===${NC}"
fi

exit ${EXIT_CODE}
