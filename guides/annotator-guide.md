> WARNING: out of date!

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

## Gab's Todo:
- integrate sensor data in the bot via `data_text` element. You may use the `get_text()` and `set_text()` methods to just append the data as string. You can read more from the official [docs](https://pygame-gui.readthedocs.io/en/v_069/pygame_gui.elements.html#module-pygame_gui.elements.ui_text_entry_line)
- let me know how you want to handle image saving. currently, it is easy to save the image file via the `image.save()` method [docs](https://www.pygame.org/docs/ref/image.html?highlight=image#module-pygame.image) 
- idk how to make this as a class WHILE also integrating the data receiving part, but I imagine this portion should be easy
- the idea for now is that the input sensor data is a single line of text, but we can also store the recorded values in a global dictionary list or something
- please update `schema/annotated_datapoint.json` so that the output json format (which would be eventually fed for training (?)) is up to your requirements