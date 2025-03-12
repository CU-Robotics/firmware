# Waggle Interceptor

This is a tool for allowing you to send information from teensy to the waggle dashboard. 

## Installation 

- Install [Waggle](https://github.com/CU-Robotics/waggle) somewhere on your machine

- Install [Rust](https://www.rust-lang.org/tools/install)
  ```sh
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
  ```

- MAC ONLY:
  - Install [Homebrew](https://brew.sh) if you haven't already 
    ```sh
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

  - Install unbuffer
    ```sh
    brew install expect
    ```

- LINUX ONLY:

  - Install unbuffer
    ```sh
    sudo apt install expect
    ```

## Usage

- Run waggle with `./waggle` 

- Anywhere you want to add a datapoint to the dashboard, print on a newline with the format `waggle graph GRAPH_NAME GRAPH_VALUE`. For instance, if I wanted to graph that `torque_fr` is currently 3.14, I would print `waggle graph torque_fr 3.14`

- Instead of running make upload, run `./tools/waggle-interceptor/make-upload.sh` **Run this in your standard terminal, not the VS Code built-in terminal, unless you want to have no memory remaining on your machine**

That's it :)
