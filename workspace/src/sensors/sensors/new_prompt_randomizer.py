import numpy
import random
import csv
import ast
import math
import os

# Prompt Generator

class prompt_generator:
  def __init__(self):

    self.rephrase_move = ["move", "go", "advance", "coast", "glide", "get yourself", "move yourself", "proceed"]
    self.prepositions = ["", "by", "a distance of", "for", "for a total distance of", "equal to", "by a measure of", "about", "by about", "about", "around"]
    self.prepositions2 = ["", "by", "a distance of", "for", "for a total distance of", "equal to", "by a measure of", "about", "by about", "about", "around"]
    self.rephrase_forward = ["forward", "onwards", "in front of you", "straight ahead"]
    self.rephrase_rotate = ["turn", "shift", "rotate", "spin", "turn yourself", "shift yourself", "rotate yourself", "spin"]
    self.rephrase_lrot = ["to the left", "to your left", "counterclockwise", "CCW", "leftward"]
    self.rephrase_rrot = ["to your right", "to the right", "clockwise", "CW", "rightward"]
    self.rephrase_lside = ["to your left", "to the left", "leftwards", "to your relative west"]
    self.rephrase_rside = ["to your right", "to the right", "rightwards", "to your relative east"]
    self.rephrase_back1 = ["behind you", "backwards", "rearward", "to your rear", "to the spot behind you"]
    self.rephrase_back2 = ["behind you", "backwards", "in reverse", "rearward", "to your rear"]
    self.rephrase_wait = ["wait @ seconds", "pause for @ seconds", "pause for a duration of @ seconds", "hold on for @ seconds", "give it @ seconds", "wait patiently for @ seconds", "pause for a count of @ seconds", "take a brief @-second pause", "allow @ seconds to pass", "hold off for just @ seconds", "wait out the @-second interval", "take a momentary break for @ seconds", "temporarily halt for @ seconds"]      # gpt-rephrased
    self.rephrase_map = ["map out your surroundings", "explore the area around you", "chart the landscape nearby", "familiarize yourself with your environment", "navigate through your surroundings", "plot the territory surrounding you", "understand the layout of your vicinity", "survey the area surrounding you", "determine the geography of your surroundings", "take stock of the space around you", "establish a mental picture of your immediate environment"]         # gpt-rephrased
    self.rephrase_start = ["sketch", "create", "outline", "render", "draft", "make", "sketch out", "form", "illustrate", "construct", "design"]
    self.shaper = ["triangle", "triangular", "3-sided", "three-sided", "four-sided", "4-sided", "@four-sided", "@4-sided", "square", "quadrilateral with equal sides and right angles", "rectangle", "rectangular", "quadrilateral with equal opposite sides and right angles", "five-sided", "5-sided", "pentagon", "six-sided", "6-sided", "hexagon", "seven-sided", "7-sided", "heptagon", "eight-sided", "8-sided", "octagon", "nine-sided", "9-sided", "nonagon", "10-sided", "ten-sided", "decagon"]
    self.shape_sides = [3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10]
    self.rephrase_side2side = ["side to side", "from one side to another", "from one side to the other", "horizontally"]
    self.rephrase_move_vague_doubles = ["move", "oscillate", "sway", "shuffle", "veer", "weave", "traverse", "meander"]
    self.rephrase_move_vague_doubles.extend(self.rephrase_move)
    self.rephrase_backnforth = ["back and forth", "alternate directions", "shift vertically", "to and fro"]
    self.rephrase_sideways = ["sideways", "to the side", "laterally", "horizontally"]
    self.rephrase_lrotate = ["rotate slightly to your left", "rotate a little to your left", "rotate a little bit to your left", "turn a little to your left", "turn just a bit towards your left", "shift slightly to your left side", "angle yourself a tad to the left", "adjust slightly in the direction of your left", "tilt slightly leftwards", "nudge yourself a bit leftward", "slide gently to your left", "swivel a touch to your left", "slightly veer towards your left", "lean a bit to the left", "scoot over a bit to your left", "tip slightly towards your left side", "ease yourself a tad to the left", "inch a bit towards your left", "shift your position a tad to the left"] # gpt-rephrased
    self.rephrase_rrotate = ["rotate slightly to your right", "rotate a bit to your right", "turn just a bit towards your right", "shift slightly to your right side", "angle yourself a tad to the right", "adjust slightly in the direction of your right", "tilt slightly rightwards", "nudge yourself a bit rightward", "slide gently to your right", "swivel a touch to your right", "slightly veer towards your right", "lean a bit to the right", "scoot over a bit to your right", "tip slightly towards your right side", "ease yourself a tad to the right", "inch a bit towards your right", "shift your position a tad to the right"] # gpt-rephrased
    self.rephrase_vrotate = ["rotate slightly", "rotate a little bit", "turn just a bit", "shift slightly", "angle yourself a tad", "adjust slightly", "tilt slightly", "swivel a touch", "veer slightly", "tip slightly", "shift your position a tad"] # gpt-rephrased
    self.rephrase_sforward = ["take a teeny step forward", "move a little forward", "scoot a little forward", "move a little bit", "advance a bit","take a small step forward","move forward slightly","proceed a little","step forward a bit","advance a tad","take a small forward movement","shift forward a bit","progress a little","move ahead slightly","take a slight step forward","move just a bit forward","nudge forward a little","move forward a touch","shift ahead a bit", "take a tiny step forward", "slide a little forward", "gently shift forward", "ease forward slightly", "move forward a tad", "slip forward just a bit", "glide forward a touch", "nudge forward a little", "proceed forward a smidge", "shift ahead slightly", "advance a bit forward", "scoot forward just a tad", "progress forward slightly", "take a small step forward", "move ahead a little", "gently inch forward", "slide forward a touch"] # gpt-rephrased
    self.rephrase_aveforward = ["proceed a normal amount forward", "advance an average distance forward","move forward a standard amount", "proceed forward an ordinary distance", "continue forward a typical distance", "move ahead a regular amount", "advance forward a customary distance", "progress forward a common distance", "move forward a usual amount", "proceed forward a typical distance", "continue ahead an average distance", "move forward an average distance", "advance forward a standard amount", "proceed ahead a normal distance", "move forward a standard distance", "progress forward an ordinary amount"] # gpt-rephrased
    self.rephrase_bforward = ["coast a large distance forward", "glide a significant distance forward", "sail a considerable way forward", "drift a long distance ahead", "glide a vast distance forward", "skate a substantial distance forward", "sail a great distance ahead", "glide a considerable distance forward", "drift a substantial distance ahead", "sail a significant distance forward",  "skate a long distance ahead", "glide a substantial distance forward", "drift a considerable distance ahead", "sail a large distance forward", "skate a considerable distance ahead", "glide a significant distance forward"] # gpt-rephrased
    self.rephrase_drawstar = ["draw a star", "sketch a star", "make a star", "create a star", "outline a star", "depict a star", "design a star", "render a star", "draft a star", "portray a star", "illustrate a star", "trace a star", "craft a star", "form a star", "produce a star"] # gpt-rephrased

  def generate_inst(self):
    inst_types = ["FWD", "LROT", "RROT", "LSIDE", "RSIDE", "BACK",
                  "DIAGONAL LEFT FORWARD", "DIAGONAL RIGHT FORWARD",
                  "X METERS AT ANGLE Y LEFT", "X METERS AT ANGLE Y RIGHT",
                  "DRAW SHAPE", "B&F", "S2S", "DRAW STAR", "vSIDEWAYS", 
                  "vsFORWARD", "vaFORWARD", "vbFORWARD", "vsROTATE"]

    #inst_types = ["DRAW SHAPE", "B&F", "S2S", "vSIDEWAYS", "DRAW STAR", "vsFORWARD", "vaFORWARD", "vbFORWARD", "vsROTATE"]

    randtype = numpy.random.choice(inst_types)

    #randtype = numpy.random.choice(inst_types, p=[0.10, 0.10, 0.10, 0.10, 0.15, 0.15, 0.10, 0.10, 0.10])

    if randtype == "FWD":
      return self.move_forward()
    elif randtype == "LROT":
      return self.rotate_left()
    elif randtype == "RROT":
      return self.rotate_right()
    elif randtype == "LSIDE":
      return self.move_left()
    elif randtype == "RSIDE":
      return self.move_right()
    elif randtype == "BACK":
      return self.move_back()
    elif randtype == "DIAGONAL LEFT FORWARD":
      return self.move_diag_left()
    elif randtype == "DIAGONAL RIGHT FORWARD":
      return self.move_diag_right()
    elif randtype == "X METERS AT ANGLE Y LEFT":
      return self.xmeters_angley_left()
    elif randtype == "X METERS AT ANGLE Y RIGHT":
      return self.xmeters_angley_right()
    elif randtype == "DRAW SHAPE":
      return self.draw_shape()
    elif randtype == "WAIT":
      return self.wait()
    elif randtype == "MAP":
      return self.make_map()
    elif randtype == "B&F":
      return self.backnforth()
    elif randtype == "S2S":
      return self.side2side()
    elif randtype == "vSIDEWAYS":
      return self.vague_sideways()
    elif randtype == "DRAW STAR":
      return self.draw_star()
    elif randtype == "vsFORWARD":
      return self.vague_small_forward()
    elif randtype == "vaFORWARD":
      return self.vague_ave_forward()
    elif randtype == "vbFORWARD":
      return self.vague_big_forward()
    elif randtype == "vsROTATE":
      return self.vague_small_rotate()
    #elif randtype == "vZIGZAG":
    #  return prompt_generator.vague_zigzag()

  def extra_times(self):
    chs1 = numpy.random.choice([True, False], p=[0.25, 0.75])

    if chs1:
      pick_rep = random.randint(0, 2)
      rep = ["once", "twice", "thrice"]
      add_to_prompt = f" {rep[pick_rep]}"
      number_of_times = pick_rep + 1
    else:
      number_of_times = random.randint(1, 15)
      rephrase_total = [" ", " a total of ", " for a total of "]
      rephrase_intro = [" for ", " "]

      if number_of_times == 1:
        chs2 = random.choice([True, False])

        if chs2:
          number_val = ["1", "one", "a single"]
          rephrase_times = [" time", " instance", " iteration", " round", " cycle", " turn"]
          add_to_prompt = f"{random.choice(rephrase_intro)}{random.choice(number_val)}{random.choice(rephrase_times)}"
        else:
          add_to_prompt = random.choice(["", " once"])
      else:
        rephrase_times = [" times", " instances", " repetitions", " iterations", " rounds", " cycles", " turns", " repeats", " occurrences"]
        chs = random.choice([True, False])

        if chs:
          number_val = [None, None, "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen"]
        else:
          number_val = [number_of_times]
          rephrase_times.append("x")

        add_to_prompt = f"{random.choice(rephrase_total)} {str(random.choice(number_val))}{random.choice(rephrase_times)}"

    return (add_to_prompt, number_of_times)

  def draw_star(self):
    dist = random_value.dist()
    prompt = f"{random.choice(self.rephrase_drawstar)} using {dist[0]} long lines"
    equiv = [("MOVE", dist[1])]
    for i in range(4):
      equiv.extend([("RGHT", 36), ("MOVE", dist[1])])

    return (prompt, equiv)
  
  def vague_small_forward(self):
    prompt = random.choice(self.rephrase_sforward)
    #equiv = round(float(random.randint(5, 50))/100, 2)
    equiv =  [("MOVE", 0.20)]

    return (prompt, equiv)

  def vague_ave_forward(self):
    prompt = random.choice(self.rephrase_aveforward)
    #equiv = round(float(random.randint(20, 80))/100, 2)
    equiv = [("MOVE", 0.80)]

    return (prompt, equiv)

  def vague_big_forward(self):
    prompt = random.choice(self.rephrase_bforward)
    #equiv = round(float(random.randint(100, 300))/100, 2)
    equiv = [("MOVE", 1.50)]

    return (prompt, equiv)

  def vague_small_rotate(self):
    rotate_dir = random.choice(["LEFT", "RGHT", "EITHER"])
    
    if rotate_dir == "LEFT":
      dir = "LEFT"
      prompt = random.choice(self.rephrase_lrotate)

    elif rotate_dir == "RGHT":
      dir = "RGHT"
      prompt = random.choice(self.rephrase_rrotate)
    
    else:
      dir = random.choice(["LEFT", "RGHT"])
      prompt = random.choice(self.rephrase_vrotate)
    
    #v_rot = random.randint(5, 45)
    v_rot = 30
    equiv = [(dir, v_rot)]

    return (prompt, equiv)

  def backnforth(self):
    # example: "move 30in back and forth 5 times"

    dist = random_value.dist()
    dir = random.choice(["LEFT", "RGHT"])
    extra = self.extra_times()

    prompt = f"{random.choice(self.rephrase_move_vague_doubles)} {random.choice(self.prepositions)} {dist[0]} {random.choice(self.rephrase_backnforth)}{extra[0]}"
    equiv = []

    for i in range(extra[1]+1):
      equiv.extend([("MOVE", dist[1]), (dir, 180), ("MOVE", dist[1]), (dir, 180)])

    return (prompt, equiv)

  def side2side(self):
    # example: "move 1 meter side to side 10 times"

    dist = random_value.dist()
    dir = random.choice(["LEFT", "RGHT"])
    chs = random.choice([True, False])
    extra = self.extra_times()

    if chs:
      prompt = f"{random.choice(self.rephrase_move_vague_doubles)} {random.choice(self.prepositions)} {dist[0]} {random.choice(self.rephrase_side2side)}{extra[0]}"
    else:
      prompt = f"{random.choice(self.rephrase_move_vague_doubles)} {random.choice(self.rephrase_side2side)} {random.choice(self.prepositions2)} {dist[0]}{extra[0]}"

    equiv = [(dir, 90)]
    for i in range(extra[1]+1):
      equiv.extend([("MOVE", dist[1]), (dir, 180), ("MOVE", dist[1]), (dir, 180)])

    return (prompt, equiv)

  def vague_sideways(self):
    # example: "move 5 m sideways"

    dist = random_value.dist()
    dir = random.choice(["LEFT", "RGHT"])
    chs = random.choice([True, False])

    if chs:
      prompt = f"{random.choice(self.rephrase_move)} {random.choice(self.prepositions)} {dist[0]} {random.choice(self.rephrase_sideways)}"
    else:
      prompt = f"{random.choice(self.rephrase_move)} {random.choice(self.rephrase_sideways)} {random.choice(self.prepositions)} {dist[0]}"

    equiv = [(dir, 90), ("MOVE", dist[1])]

    return (prompt, equiv)

  def move_forward(self):
    dist = random_value.dist()
    chs = random.choice([True, False])

    if chs:
      prompt = f"{random.choice(self.rephrase_move)} {random.choice(self.prepositions)} {dist[0]} {random.choice(self.rephrase_forward)}"
    else:
      prompt = f"{random.choice(self.rephrase_move)} {random.choice(self.rephrase_forward)} {random.choice(self.prepositions)} {dist[0]}"

    equiv = [("MOVE", dist[1])]

    return (prompt, equiv)

  def rotate_left(self):
    rot = random_value.rot()
    chs = random.choice([True, False])

    if chs:
      prompt = f"{random.choice(self.rephrase_rotate)} {rot[0]} {random.choice(self.rephrase_lrot)}"
    else:
      prompt = f"{random.choice(self.rephrase_rotate)} {random.choice(self.rephrase_lrot)} {rot[0]}"

    equiv = [("LEFT", rot[1])]

    return (prompt, equiv)

  def rotate_right(self):
    rot = random_value.rot()
    chs = random.choice([True, False])

    if chs:
      prompt = f"{random.choice(self.rephrase_rotate)} {rot[0]} {random.choice(self.rephrase_rrot)}"
    else:
      prompt = f"{random.choice(self.rephrase_rotate)} {random.choice(self.rephrase_rrot)} {rot[0]}"

    equiv = [("RGHT", rot[1])]

    return (prompt, equiv)

  def move_left(self):
    dist = random_value.dist()
    chs = random.choice([True, False])

    if chs:
      prompt = f"{random.choice(self.rephrase_move)} {random.choice(self.prepositions)} {dist[0]} {random.choice(self.rephrase_lside)}"
    else:
      prompt = f"{random.choice(self.rephrase_move)} {random.choice(self.rephrase_lside)} {random.choice(self.prepositions2)} {dist[0]}"

    equiv = [("LEFT", 90), ("MOVE", dist[1])]

    return (prompt, equiv)

  def move_right(self):
    dist = random_value.dist()
    chs = random.choice([True, False])

    if chs:
      prompt = f"{random.choice(self.rephrase_move)} {random.choice(self.prepositions)} {dist[0]} {random.choice(self.rephrase_rside)}"
    else:
      prompt = f"{random.choice(self.rephrase_move)} {random.choice(self.rephrase_rside)} {random.choice(self.prepositions2)} {dist[0]}"

    equiv = [("RGHT", 90), ("MOVE", dist[1])]

    return (prompt, equiv)

  def move_back(self):
    dist = random_value.dist()
    chs = random.choice([True, False])

    if chs:
      prompt = f"{random.choice(self.rephrase_move)} {dist[0]} {random.choice(self.rephrase_back1)}"
    else:
      prompt = f"{random.choice(self.rephrase_move)} {random.choice(self.rephrase_back2)} {random.choice(self.prepositions)} {dist[0]}"

    equiv = [("RGHT", 180), ("MOVE", dist[1])]

    return (prompt, equiv)

  def move_diag_left(self):
    dist = random_value.dist()
    prompt = f"{random.choice(self.rephrase_move)} {dist[0]} diagonally to your left"
    equiv = [("LEFT", 45), ("MOVE", dist[1])]

    return (prompt, equiv)

  def move_diag_right(self):
    dist = random_value.dist()
    prompt = f"{random.choice(self.rephrase_move)} {dist[0]} diagonally to your right"
    equiv = [("RGHT", 45), ("MOVE", dist[1])]

    return (prompt, equiv)

  def xmeters_angley_left (self):
    dist = random_value.dist()
    rot = random_value.rot()
    prompt = f"{random.choice(self.rephrase_move)} {dist[0]} at {rot[0]} {random.choice(self.rephrase_lrot)}"
    equiv = [("LEFT", rot[1]), ("MOVE", dist[1])]

    return (prompt, equiv)

  def xmeters_angley_right (self):
    dist = random_value.dist()
    rot = random_value.rot()
    prompt = f"{random.choice(self.rephrase_move)} {dist[0]} at {rot[0]} {random.choice(self.rephrase_rrot)}"
    equiv = [("RGHT", rot[1]), ("MOVE", dist[1])]

    return (prompt, equiv)

  def wait(self):
    wait_time = random_value.time()
    prompt = random.choice(self.rephrase_wait).replace("@", wait_time[0])
    equiv = [("WAIT", wait_time[1])]

    return (prompt, equiv)

  def make_map(self):
    prompt = random.choice(self.rephrase_map)
    equiv = [("MAP", None)]

    return (prompt, equiv)

  def draw_shape(self):
    shape_int = random.randint(0, len(self.shaper)-1)
    shape = self.shaper[shape_int]

    if shape == "rectangle" or shape == "rectangular" or shape == "quadrilateral with equal opposite sides and right angles":
      dist1 = random_value.dist()
      dist2 = random_value.dist()

      if shape == "rectangular":
          extra = [" shape", " form", " figure"]
          shape += random.choice(extra)

      chs = random.choice([True, False])

      if chs:
        # ex: sketch a 5x6 rectangular shape
        rephrase_sides = ["x", " by ", " x "]
        prompt = f"{random.choice(self.rephrase_start)} a {dist1[0]}{random.choice(rephrase_sides)}{dist2[0]} {shape}"
      else:
        rephrase_next = ["with dimensions", "measuring", "with sides of", "with dimensions of", "that measures"]
        rephrase_sides = ["@1x@2", "@1 x @2", "@1 by @2", "@1 wide and @2 tall", "@1 across and @2 high"]

        prompt = f"{random.choice(self.rephrase_start)} a {shape} {random.choice(rephrase_next)} {((random.choice(rephrase_sides)).replace('@1', dist1[0])).replace('@2', dist2[0])}"
    
      equiv = [("MOVE", round(dist1[1],2)), ("LEFT", 90), ("MOVE", round(dist2[1],2)), ("LEFT", 90), ("MOVE", round(dist1[1],2)), ("LEFT", 90), ("MOVE", round(dist2[1],2))]

      return (prompt, equiv)

    elif shape == "square" or shape == "quadrilateral with equal sides and right angles" or shape == "@four-sided" or shape == "@4-sided":
      if shape == "square" or shape == "@four-sided" or shape == "@4-sided":
        shape = shape.replace("@", "")
        extra = [" shape", " figure", ""]
        shape += random.choice(extra)

      dist =  random_value.dist()

      chosen = random.randint(0, 1)
      if chosen:
        # ex: sketch a 5x5 square shape
        rephrase_sides = ["x", " by ", " x "]
        prompt = f"{random.choice(self.rephrase_start)} a {dist[0]}{random.choice(rephrase_sides)}{dist[0]} {shape}"
      else:
        rephrase_next = ["with dimensions", "measuring", "with sides of", "with dimensions of", "that measures", "with side lengths equal to"]
        rephrase_sides = ["@x@", "@ by @", "@", "@ x @"]
        prompt = f"{random.choice(self.rephrase_start)} a {shape} {random.choice(rephrase_next)} {(random.choice(rephrase_sides)).replace('@', str(dist[0]))}"

      equiv = [("MOVE", round(dist[1], 2)), ("LEFT", 90), ("MOVE", round(dist[1], 2)), ("LEFT", 90), ("MOVE", round(dist[1], 2)), ("LEFT", 90), ("MOVE", round(dist[1], 2))]

      return (prompt, equiv)

    else:
      dist =  random_value.dist()
      angle = round(360/self.shape_sides[shape_int], 2)
      equiv =[]

      if shape == "triangular" or ("sided" in shape):
        extra = [" shape", " form", " figure"]
        shape += random.choice(extra)

      rephrase_next = ["with @-sides", "with sides measuring @", "with sides equal to @", "with side lengths equal to @", "with dimensions equal to @", "with @ dimensions", "measuring @", "that measures @ per side", "with length @ per side"]

      if shape == "octagon":
        preposition = "an"
      else:
        preposition = "a"

      prompt = f"{random.choice(self.rephrase_start)} {preposition} {shape} {(random.choice(rephrase_next)).replace('@', str(dist[0]))}"

      for i in range(self.shape_sides[shape_int]-1):
        equiv.append(("MOVE", round(dist[1], 2)))
        equiv.append(("LEFT", round(angle, 2)))

      equiv.append(("MOVE", round(dist[1], 2)))

      return (prompt, equiv)

########################################################################################################################################################
# Randomize Values and the ways they are phrased like Distance, Time, and Rotation

class random_value:
  def dist():
    meas = ["mm", " millimeters", "cm", " centimeters", "m", " meters", '"', "in", " inches", "'", "ft", " feet", "yds", " yards"]
    dis_type = random.choice(meas)

    written_num = ["actual number", "quarters", "spelled out"]
    selected_num_type = random.choice(written_num)

    if selected_num_type == "actual number":
      if dis_type == "millimeters" or dis_type == "mm":
        rand_num = float(random.randint (1000, 300000))/100
        in_meters = rand_num / 1000
      elif dis_type == "cm" or dis_type == "centimeters":
        rand_num = float(random.randint (100, 3000))/100
        in_meters = rand_num / 100
      elif dis_type == "m" or dis_type == "meters":
        rand_num = float(random.randint (1, 300))/100
        in_meters = rand_num
      elif dis_type == '"' or dis_type == "in" or dis_type == "inches":
        rand_num = float(random.randint (39, 11811))/100
        in_meters = round(rand_num/39.37, 2)
      elif dis_type == "'" or dis_type == "ft" or dis_type == "feet":
        rand_num = float(random.randint (3, 984))/100
        in_meters = round(rand_num/3.281, 2)
      elif dis_type == "yds" or dis_type == "yards":
        rand_num = float(random.randint (1, 328))/100
        in_meters = round(rand_num/1.094, 2)
      else:
        rand_num = 1

      if rand_num == 1:
        meas = ["m", " meter", "ft", " foot", " ruler", " yard", " yardstick"]
        mets = [1, 1, 0.31, 0.31, 0.31, 0.91, 0.91]
        num = random.randint(0, len(meas)-1)
        dis_type = meas[num]
        in_meters = mets[num]

      phrasing = f"{str(rand_num)}{dis_type}"

    elif selected_num_type == "quarters":
      rand_quarter = ["a single", "a quarter", "a third", "a fourth", "a quarter", "a fifth", "a half", "three quarters", "an eigth", "two fifths", "three eighths", "five eights"]
      writ = [1, 0.25, 0.30, 0.25, 0.25, 0.20, 0.5, 0.75, 0.125, 0.40, 0.375, 0.625]
      measure = ["meter", "foot", "ruler", "yardstick"]
      num = random.randint(0, len(rand_quarter)-1)

      meas = random.randint(0, len(measure)-1)

      if meas == 0:
        in_meters = writ[num]
      elif meas == 1 or meas == 2:
        in_meters = round(writ[num]/3.281, 2)
      elif meas == 3:
        in_meters = round(writ[num]/1.094, 2)

      if num == 1:
        phrasing = f"{rand_quarter[num]} {measure[meas]}"
      else:
        phrasing = f"{rand_quarter[num]} of a {measure[meas]}"

    elif selected_num_type == "spelled out":
      prepositions_b = ["by", "a distance of", "for", "for a total distance of"]
      numbers = ["a", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven","twelve","thirteen","fourteen","fifteen","sixteen", "seventeen", "eighteen", "nineteen", "twenty"]
      writ = [1,1,2, 3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
      added = ["", "full ", "whole "]

      meas = random.randint(0, 4)

      if meas == 0:
        num = random.randint(0, len(numbers)-1)
        in_meters = writ[num] / 100
      elif meas == 1:
        num = random.randint(0, 3)
        in_meters = writ[num]
      elif meas == 2 or meas == 3:
        num = random.randint(0, 9)
        in_meters = round(writ[num]/3.281, 2)
      elif meas == 4:
        num = random.randint(0, 3)
        in_meters = round(writ[num]/1.094, 2)

      if num == 0 or num == 1:
        measure = ["centimeter", "meter", "foot", "ruler", "yardstick"]
      else:
        measure = ["centimeters", "meters", "feet", "rulers", "yardsticks"]

      phrasing = f"{numbers[num]} {random.choice(added)}{measure[meas]}"

    return (phrasing, round(in_meters, 2))

  def rot():
    angle_type = ["degrees", "radians", "rads", "degs", "%", "fraction"]
    rot_type = random.choice(angle_type)

    if rot_type == "degrees" or rot_type == "degs":
      deg = random.randint(0, 360)

      phrasing = f"{str(deg)} {rot_type}"
      in_degrees = deg

    elif rot_type == "radians" or rot_type == "rads":
      rad_type = ["numerical", "pi"]
      a = random.choice(rad_type)

      if a == "numerical":
        rad = float(random.randint(0, 628))/100
        deg = round(rad * 57.296, 2)

        phrasing = f"{str(rad)} {rot_type}"
        in_degrees = deg

      elif a == "pi":
        pi_rot = ["pi/6", "pi/4", "pi/3", "pi/2", "pi", "2pi/3", "3pi/4", "5pi/6", "7pi/6", "5pi/4", "4pi/3", "3pi/2", "5pi/3", "7pi/4", "11pi/6", "2 pi"]
        deg_rot = [30, 45, 60, 90, 180, 120, 135, 150, 210, 225, 240, 270, 300, 315, 330, 360]
        rads = [" radians", " rads", ""]
        deg_choice = random.randint(0, len(pi_rot)-1)

        phrasing = f"{pi_rot[deg_choice]}{random.choice(rads)}"
        in_degrees = deg_rot[deg_choice]

    elif rot_type == "%":
      percentage = random.randint(0, 100)
      deg = (float(percentage) / 100.00) * 360.00

      phrasing = f"{str(percentage)}%"
      in_degrees = deg

    elif rot_type == "fraction":
      frac_rot = ["1/2", "1/4", "1/3", "1/8", "3/4", "half", "quarter", "three quarters", "3 quarters", "a third", "an eighth"]
      deg_rot = [180, 90, 120, 45, 270, 180, 90, 270, 270, 120, 45]
      deg_choice = random.randint(0, len(frac_rot) -1)

      phrasing = f"{str(frac_rot[deg_choice])} of a circle"
      in_degrees = deg_rot[deg_choice]

    return (phrasing, round(in_degrees, 2))

  def time():
    choices = ["seconds", "minute fractions"]
    choice = random.choice(choices, [70, 30])

    if choice == "seconds":
      add = [" seconds", "s", " secs", "secs"]
      #wait_time = round(float(random.randint(5, 150))/10,2)
      wait_time = round(float(random.randint(5, 600))/10,2)
      in_seconds = wait_time
      phrasing = f"{str(wait_time)}{random.choice(add)}"

    elif choice == "minute fractions":
      frac = ["a quarter of a minute", "1/4 minute", "0.25 of a minute", "half of a minute", "0.5 of a minute", "1/2 of a minute", "three-quarters of a minute", "3/4s of a minute", "0.75 of a minute", "a full minute", "a whole minute", "one full minute", "one whole minute"]
      equiv_sec = [15, 15, 15, 30, 30, 30, 45, 45, 45, 60, 60, 60, 60]
      index = random.randint(0, len(frac)-1)
      phrasing = frac[index]
      in_seconds = equiv_sec[index]

    return (phrasing, round(in_seconds, 2))

  #def speed():
  #  return (speed, round(equiv, 2))


####################################################################################################################################################
# CSV Prompt Randomizer
# for levels 1 and 2

class l12_prompt_randomizer:
    def randomize(filename):
      this_path = os.path.abspath(os.path.dirname(__file__))
      path = os.path.join(this_path, f"Prompts/{filename}.csv")

      with open(path, newline='') as f:
          read_file = csv.reader(f)
          all_prompts = [list(row) for row in read_file]

      return random.choice(all_prompts)

    def csv():
        prompt = l12_prompt_randomizer.randomize("combined")

        prompt[0] = prompt[0].replace('"', "")

        prompt[1] = prompt[1].replace('"', "")
        prompt[1] = prompt[1].replace("|", ",")

        prompt[1] = prompt[1].replace('RGHT', '\"RGHT\"')
        prompt[1] = prompt[1].replace('MOVE', '\"MOVE\"')
        prompt[1] = prompt[1].replace('LEFT', '\"LEFT\"')

        # Convert string to list of tuples
        prompt[1] = ast.literal_eval(prompt[1])

        for i, (action, value) in enumerate(prompt[1]):
            if (action == 'RGHT') or (action == 'LEFT'):
                # Convert the angle value to degrees
                prompt[1][i] = (action, round(math.degrees(value), 2))
            elif action == 'MOVE':
                continue

        str_equiv = str(prompt[1]).replace("[", "").replace("]", "")
        prompt = (prompt[0], prompt[1])

        return prompt

    def impossible():
      return self.randomize("impossible")

    def bogus():
      return self.randomize("bogus")
    
#####################################################################################################################################################
# TODO
# Prompt Maker

import random

class prompt_maker:
  def __init__(self):
    self.prompt = ""
    self.equiv = []
    self.ground_truth = []
    self.cumulative = []                      # previously computed_move
    self.impossible = False
    self.bogus = False
    self.vague = False                        # ask for clarification

  def flags():
    # Flags are created here

    flags = ["@STOP_I", "@STOP_W", "@GO_AROUND_I", "@GO_AROUND_W"]

    fl = random.choice(flags)

    # Defaults
    #distance = 0                    # distance from obstacle
    wait = 0                        # amount of time to wait until obstacle is removed from path or until robot tries to move around the obstacle
    stop = False                    # stops at obstacle
    go_around = False               # try to move around the object, else stop

    if fl == "@STOP_I":
      # stop immediately once obstacle is detected
      possible_prompts = ["Stop immediately once the obstacle is detected.", "Cease promptly upon detection of the obstacle.", "Once the obstacle is detected, halt immediately.", "Upon detecting the obstacle, stop right away.", "Cease movement as soon as the obstacle is detected.", "Stop instantly upon detection of the obstacle.", "Once the obstacle is spotted, come to a halt immediately.", "Halt without delay once the obstacle is detected.", "Upon detection of the obstacle, stop abruptly.", "Cease motion immediately upon detecting the obstacle.", "Stop at once when the obstacle is detected.", "Come to a stop instantly upon detecting the obstacle.", "Stop before you hit anything.", "Stop before you hit something."]
      prompt_addition = random.choice(possible_prompts)

      stop = True

    elif fl == "@STOP_W":
      # stop after X seconds
      possible_prompts = ["Wait up to @ seconds for obstacle removal once the obstacle is detected. If the obstacle is still there after @ seconds, stop.", "Pause for up to @ seconds upon detecting the obstacle and resume when it's gone. If the obstacle persists after @ seconds, come to a halt.", "Delay for a maximum of @ seconds upon detecting the obstacle. If the obstacle remains after @ seconds, cease movement.", "Pause for @ seconds after detecting the obstacle. If it persists, stop.", "After detecting the obstacle, wait for @ seconds. If it remains, halt.", "Wait for @ seconds after detecting the obstacle. If it's still in your way, stop.", "Delay for @ seconds upon detecting the obstacle. If it remains, halt operations.", "After spotting the obstacle, wait for @ seconds before continuing. If it lingers, halt.", "Upon detecting the obstacle, wait for @ seconds before resuming. If it stays there, come to a stop."]
      prompt_addition = random.choice(possible_prompts)

      stop = True
      wait = random.randint(1, max_time)

      # replace X
      prompt_addition = prompt_addition.replace("@", str(wait))
      if wait == 1:
        prompt_addition = prompt_addition.replace("1 seconds", "1 second")


    elif fl == "@GO_AROUND_I":
      # move around obstacle immediately once obstacle is detected. if not possible, stop
      possible_prompts = ["Try to go around it. If you can't go around it, stop.", "Immediately try to go around it. If you can't go around it, stop.", "Attempt to maneuver around it. If you're unable to, come to a halt.",  "Make an effort to navigate around it. If circling isn't possible, halt.", "Try to bypass it. If bypassing isn't feasible, cease movement.", "Attempt to maneuver past it. If maneuvering isn't feasible, come to a stop.", "Make an effort to avoid it. If avoidance isn't possible, halt."]
      prompt_addition = random.choice(possible_prompts)

      go_around = True

    elif fl == "@GO_AROUND_W":
      # move around obstacle after <wait> seconds. if not possible, stop
      possible_prompts = ["Wait up to @ seconds once the obstacle is detected. If the obstacle is still there after @ seconds, try to go around it. If you can't go around it, stop.", "Wait up to @ seconds for obstacle removal. If it persists, attempt avoidance; if unsuccessful, stop.", "Wait until @ seconds for barrier clearance. If it persists, try to circumvent; if not possible, stop.", "Wait @ seconds maximum for obstruction removal. If it remains, attempt avoidance; if unsuccessful, stop.", "Allow up to @ seconds for hurdle elimination. If it lingers, try to navigate around; if not feasible, stop.", "Give it @ seconds max for barrier eradication. If it continues, attempt avoidance; if unachievable, stop.", "Wait until @ seconds for impediment removal. If it persists, try to sidestep; if impossible, stop."]
      prompt_addition = random.choice(possible_prompts)

      go_around = True
      wait = random.randint(1, max_time)

      # replace X
      prompt_addition = prompt_addition.replace("@", str(wait))
      if wait == 1:
        prompt_addition = prompt_addition.replace("1 seconds", "1 second")

    elif fl == "@ROTATE_AND_CONTINUE":
      # rotate to face an open space and continue counting from then
      

    #t_or_f = [True, False]
    #announce = random.choice(t_or_f)            # announce that an obstacle was detected in status

    json_flags = {
      "@WAIT": wait,
      "@STOP": stop,
      "@GO_AROUND": go_around,
      }

    #flags = json.loads(str(json_flags))

    self.prompt += prompt_addition

    return json_flags

  def rand_multiples(max_left):
    multiples = ["", "twice", "two times", "2 times", "2x", "thrice", "three times", "3x", "3 times", "4 times", "4x", "four times", "five times", "5x", "5 times"]
    times = [0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4]

    if max_left == 0:
      time_int = 0
    elif max_left == 1:
      time_int = random.randint(0, 4)
    elif max_left == 2:
      time_int = random.randint(0, 8)
    elif max_left == 3:
      time_int = random.randint(0, 11)
    else:
      time_int = random.randint(0, len(multiples)-1)

    return (multiples[time_int], times[time_int])

  def computed_move(self, equivalent):
    start = [0,0]
    self.cumulative = [(0,0)]

    for y in equivalent:
      if y[0] == "MOVE":
        start[0] = start[0] + y[1]
        start[0] = round(start[0], 2)

      elif y[0]  == "LEFT":
        start[1] = start[1] - y[1]
        start[1] = round(start[1], 2)

      elif y[0]  == "RGHT":
        start[1] = start[1] + y[1]
        start[1] = round(start[1], 2)

      self.cumulative.append((start[0], start[1]))

    return

  def maker(self, max_per_prompt=5, coords=[0,0,0], add_flags=False):
    # Randomize whether it will make use of the Prompt Generator, CSV Randomizer (GPT-Rephrased Prompts), Impossible Prompt Randomizer, Bogus Prompt Randomizer, or Vague Prompt Randomizer
    #rnd_prompt = ["generator", "csv", "impossible", "bogus", "vague"]
    #chs_prompt = random.choice(rnd_prompt, weights=(60, 10, 10, 10, 10))

    chs_prompt = "generator"

    if chs_prompt == "csv":
      # Use CSV Randomizer
      self.prompt, self.equiv = l12_prompt_randomizer.csv()

      # ground truth
      # cumulative

    elif chs_prompt == "impossible":
      # Use Impossible Prompt Randomizer
      self.prompt = l12_prompt_randomizer.impossible()
      self.impossible = True

    elif chs_prompt == "bogus":
      # Use Bogus Prompt Randomizer
      self.prompt = l12_prompt_randomizer.bogus()
      self.bogus = True

    else:
      # Use Prompt Generator
      gen = prompt_generator()
      prompt_seq = gen.generate_inst()

      self.prompt = prompt_seq[0]
      self.equiv = prompt_seq[1]

    # Flags
    if add_flags:
      flags = self.flags()
    else:
      flags = None

    # compute cumulative
    self.computed_move(self.equiv)

    json_fl = {
      "nl_prompt": self.prompt,
      "instructions": self.equiv,
      "ground_truth_coordinates": self.ground_truth,
      "cumulative": self.cumulative,
      "impossible": self.impossible,
      "bogus": self.bogus,
      "vague": self.vague,
      "flags": flags
    }

    coords = [0, 0, 0]        # reset coords

    return (self.prompt, self.equiv, str(self.cumulative), json_fl)
  
####################################################################################################################################################

if __name__ == "__main__":
  #testing

  maker = prompt_maker()
  prompt1 = maker.maker()
  print("Prompt: ", prompt1[0])
  print("Single: ", prompt1[1])
  print("Cumulative: ", prompt1[2])
  #print("Flag: ", prompt1[3])

  print("\n")

  maker = prompt_maker()
  prompt2 = maker.maker()
  print("Prompt: ", prompt2[0])
  print("Single: ", prompt2[1])
  print("Cumulative: ", prompt2[2])
  #print("Flag: ", prompt2[3])

  print("\n")

  maker = prompt_maker()
  prompt3 = maker.maker()
  print("Prompt: ", prompt3[0])
  print("Single: ", prompt3[1])
  print("Cumulative: ", prompt3[2])
  #print("Flag: ", prompt3[3])