# Waggle Interceptor

This is a tool for allowing you to send information from teensy to the waggle dashboard. 

## Usage

Install and run [Waggle](https://github.com/CU-Robotics/waggle) on your machine

Anywhere you want to add a datapoint to the dashboard, print on a newline with the format `waggle graph GRAPH_NAME GRAPH_VALUE`. For instance, if I wanted to graph that `torque_fr` is currently 3.14, I would print `waggle graph torque_fr 3.14`

Instead of running make upload, run `./tools/waggle-interceptor/make-upload.sh`

That's it :)
