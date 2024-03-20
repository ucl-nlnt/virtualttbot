import random

class prompt_randomizer:
  def rand_dist ():
    meas = ["mm", "millimeters", "cm", "centimeters", "m", "meters", '"', "in", "inches", "'", "ft", "feet", "yds", "yards"]
    dis_type = random.choice(meas)
    #dis_type = meas[1]


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
    
    space = ['', ' ']

    return (str(rand_num) + random.choice(space) + dis_type, in_meters)

  def rand_multiples():
    multiples = ["", "twice", "two times", "2 times", "2x", "thrice", "three times", "3x", "3 times", "4 times", "4x", "four times", "five times", "5x", "5 times"]
    return random.choice(multiples)

  def rand_insts():
    no_of_instructions = random.randint(1, 5)
    return no_of_instructions

  def rand_rot():
    angle_type = ["degrees", "radians", "rads", "degs", "%", "fraction"]
    rot_type = random.choice(angle_type)

    if rot_type == "degrees" or rot_type == "degs":
      return str(random.randint(0, 360)) + " " + rot_type
        
    elif rot_type == "radians" or rot_type == "rads":
      rad_type = ["numerical", "pi"]
      a = random.choice(rad_type)

      if a == "numerical":
        return str(float(random.randint(0, 628))/100) + rot_type

      elif a == "pi":
        pi_rot = ["pi/6", "pi/4", "pi/3", "pi/2", "pi", "2pi/3", "3pi/4", "5pi/6", "7pi/6", "5pi/4", "4pi/3", "3pi/2", "5pi/3", "7pi/4", "11pi/6", "2 pi"]
        return random.choice(pi_rot)

    elif rot_type == "%":
      return str(random.randint(0, 100)) + "%"
    
    elif rot_type == "fraction":
      frac_rot = ["1/2", "1/4", "1/3", "1/8", "3/4", "half", "quarter", "three quarters", "3 quarters", "a third"]

      return str(random.choice(frac_rot)) + " of a circle"

  # Simple Randomizer
  def rand_inst():
    inst_types = ["FWD", "ROT_LEFT", "ROT_RIGHT", "LEFT", "RIGHT", "BACK", "DIAGONAL LEFT FORWARD", "DIAGONAL RIGHT FORWARD", "X METERS AT ANGLE Y"]

    randtype = random.choice(inst_types)

    if randtype == "FWD":
      dist =  prompt_randomizer.rand_dist()
      return "move " + dist[0] + " forward"
    elif randtype == "ROT_LEFT":
      rot = prompt_randomizer.rand_rot()
      return "rotate " + rot + " counterclockwise"
    elif randtype == "ROT_RIGHT":
        rot = prompt_randomizer.rand_rot()
        return "rotate " + rot + " clockwise"
    elif randtype == "LEFT":
        dist =  prompt_randomizer.rand_dist()
        return "go " + dist[0] + " to your left"
    elif randtype == "RIGHT":
      dist =  prompt_randomizer.rand_dist()[0]
      return "go " + dist[0] + " to your right"
    elif randtype == "BACK":
      dist =  prompt_randomizer.rand_dist()
      return "go to the spot " + dist[0] + " behind you"
    elif randtype == "DIAGONAL LEFT FORWARD":
      dist =  prompt_randomizer.rand_dist()
      return "move " + dist[0] + " diagonally to your left"
    elif randtype == "DIAGONAL RIGHT FORWARD":
      dist =  prompt_randomizer.rand_dist()
      return "move " + dist[0] + " diagonally to your right"
    elif randtype == "X METERS AT ANGLE Y":
      dist =  prompt_randomizer.rand_dist()
      rot = prompt_randomizer.rand_rot()
      return "move " + dist[0] + " at " + rot

  def prompt_maker():
    prompt = ""
    no_of_insts = prompt_randomizer.rand_insts()
    for x in range(no_of_insts):
      if x > 0 and not x == no_of_insts:
        add = [', ', ', then ']
        prompt += random.choice(add)
      elif (x == 0) and (x == no_of_insts):
        addition = [', and ', ', then ']
        prompt += random.choice(addition)
      
      new_inst = prompt_randomizer.rand_inst()
      # print(new_inst)
      prompt += new_inst

      # of times
      y = random.randint(0, 10)
      if y > 8:
        prompt += " " + prompt_randomizer.rand_multiples()
    return prompt

# print(prompt_randomizer.prompt_maker())
