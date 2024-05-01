# Level 3 Prompt Randomizer

import csv
import random
import os

class l3_prompt_randomizer:
  def __init__ (self, smp_obj=0, desc_obj=0, stand_on_x=0, ffw=0, imp=0):
    
    self.inc = []

    if smp_obj:
      # include prompts referring to simple objects
      # example: approach the pencil
      self.inc.append("l3_smp_obj")
    elif desc_obj:
      # include prompts referring to objects with additional description
      # example: approach the blue ball
      self.inc.append("l3_desc_obj")
    elif stand_on_x:
      # include prompts about standing on a shape or color
      # example: stand on the blue square
      self.inc.append("l3_stand_on_x")
    elif ffw:
      # include prompts about 
      self.inc.append("l3_ffw")
    elif imp:
      # include prompts about
      self.inc.append("l3_imp")
    else:
      self.inc = ["l3_smp_obj", "l3_desc_obj", "l3_stand_on_x", "l3_ffw", "l3_imp"]

  def new_prompt(self): 
    pick_csv = random.choice(self.inc)                          # randomize from which csv files we're getting the prompts
    return l3_prompt_randomizer.randomize_simple(pick_csv)             # randomize the prompt from the csv

  def randomize(filename):
    this_path = os.path.abspath(os.path.dirname(__file__))
    path = os.path.join(this_path, f"Prompts/{filename}.csv")

    with open(path, newline='') as f:
        read_file = csv.reader(f)
        all_prompts = [list(row) for row in read_file]

    return random.choice(all_prompts)
  
  def randomize_simple(filename):
    this_path = os.path.abspath(os.path.dirname(__file__))
    path = os.path.join(this_path, f"Prompts/{filename}.csv")
    
    prompt_list = []

    with open(path, 'r') as file:
        prompt_list = file.read().split('\n')

    return random.choice(prompt_list)



# HOW TO USE THE LEVEL 3 PROMPT RANDOMIZER
#
# (1) Use the correct file names for the csv files and add them to the Prompts folder
# (2) When you first run the file, use l3_prompt_randomizer(<input args here>) and include the csv files you want to run for this session
# (3) Every time you need a new prompt, use prompt = l3_prompt_randomizer.new_prompt()

if __name__ == "__main__":
  l3_prompt = l3_prompt_randomizer(stand_on_x=1)
  print(l3_prompt.new_prompt())
  print(l3_prompt.new_prompt())
  print(l3_prompt.new_prompt())
  print(l3_prompt.new_prompt())