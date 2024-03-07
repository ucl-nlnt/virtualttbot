# NLNT Data Annotator Studio

This is a Pygame-based user interface to easily annotate data for use in NLNT. It is to be run on a client PC with a connection to a Turtlebot v3 Burger via TCP and a Webcam

## Requirements
- Client PC (Usually laptop) with display output
- Webcam connected
- Client PC and Turtlebot on the same Network connection (can ping each other)

## Setup Guide
0. Clone this repository 
```bash
git clone https://github.com/ucl-nlnt/virtualttbot.git
```
1. It is recommended to setup a venv via miniconda 
2. Install requirements 
```bash
pip install -r annotator-reqs.txt
```
3. In some instances, the webcam might not be detected properly, please configure this portion in `annotator.py`

## Parameters