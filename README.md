# iRobot: Mock ROS in Pure Python

## Overview
iRobot is a lightweight, pure-Python “mock ROS” framework that implements a simple peer-to-peer pub/sub system, backed by SQLite for subscription registry and message history. It exposes a Publisher and Subscriber API, a thread-safe DBManager for CRUD operations, and a CLI entry point (iRobot run …) for launching nodes.

## Features
- Pub/Sub over TCP
- SQLite registry of subscribers and message history
- CRUD operations for subscriptions and messages
- Thread-safe singleton database manager
- CLI wrapper: iRobot run <module> <node> [--args …]
- Example nodes: talker (publisher) and listener (subscriber)
- Comprehensive pytest test suite
- Easy installation via pip install -e .

## Prerequisites
- Python 3.8 or newer

- (Optional) A virtual environment tool (venv, conda, etc.)

- (On macOS/Linux) pip

## Installation
```bash
# Clone your repo locally
git clone https://github.com/trieut415/iRobot.git
cd mock_ros

# Create and activate a virtualenv (optional but recommended)
python3 -m venv .venv
source .venv/bin/activate

# Install the package in editable mode
pip install --upgrade pip
pip install -e .
```
Note: Installing in editable mode (-e) lets you iterate on the code without reinstalling.

## Usage
You can invoke the framework via the console script iRobot:

```bash
# Run the talker (publisher) node
iRobot run nodes_py talker --topic chatter --message "Hello, ROS!" --rate 2.0

# In another terminal, run the listener (subscriber) node
iRobot run nodes_py listener --topic chatter
```
If you prefer not to install the console script, you can run the CLI module directly:

```bash
python -m irobot.cli run nodes_py talker --topic chatter --message "Hi there" --rate 1.0
python -m irobot.cli run nodes_py listener --topic chatter
```
### Node Arguments

- **talker**
  - `--topic`   (default: `chatter`)
  - `--message` (default: `"Hello, ROS!"`)
  - `--rate`    (Hz, default: `1.0`)

- **listener**
  - `--topic`   (default: `chatter`)

Project Structure
```markdown
mock_ros/
├── setup.cfg
├── setup.py
├── requirements.txt
├── README.md
├── irobot/
│   ├── __init__.py
│   ├── cli.py
│   ├── db_manager.py
│   ├── network.py
│   └── nodes_py/
│       ├── __init__.py
│       ├── talker.py
│       └── listener.py
└── tests/
    ├── conftest.py
    ├── test_cli.py
    ├── test_db_manager.py
    └── test_network.py
```
## Testing
Run the pytest suite:

```bash
# from the project root
pip install pytest
pytest -q
```
You should see all tests pass, verifying DB operations, networking, and CLI error handling.

