import random

class prompt_randomizer:
  def rand_dist ():
    meas = ["mm", " millimeters", "cm", " centimeters", "m", " meters", '"', "in", " inches", "'", "ft", " feet", "yds", " yards"]
    dis_type = random.choice(meas)
    #dis_type = meas[1]

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
        dis_type = random.choice(["mm", " millimeter", "cm", " centimeter", "m", " meter", "ft", " foot", " ruler", " yard", " yardstick"])

      #space = ['', ' ']
      #return (str(rand_num) + random.choice(space) + dis_type, in_meters)
      return str(rand_num) + dis_type

    elif selected_num_type == "quarters":
      rand_quarter = ["a single", "a quarter", "a third", "a fourth", "a quarter", "a fifth", "a half", "three quarters", "an eigth", "two fifths", "three eighths", "five eights"]
      measure = [" millimeter", " centimeter", " meter", " foot", " ruler", " yardstick"]

      return random.choice(rand_quarter) + " of a" + random.choice(measure)

    elif selected_num_type == "spelled out":
      prepositions_b = ["by", "a distance of", "for", "for a total distance of"]
      numbers = ["a", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "eleven","twelve","thirteen","fourteen","fifteen","sixteen",
             "seventeen", "eighteen", "nineteen"]
      added = ["", "full", "whole"]

      num = random.choice(numbers)

      if num == "a" or num == "one":
        measure = [" millimeter", " centimeter", " meter", " foot", " ruler", " yardstick"]
      else:
        measure = [" millimeters", " centimeters", " meters", " foots", " rulers", " yardsticks"]

      return num + " " + random.choice(added) + random.choice(measure)

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
        return str(float(random.randint(0, 628))/100) + " " + rot_type

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
    inst_types = ["FWD1", "ROT1", "SIDE1", "BACK", "DIAGONAL LEFT FORWARD", "DIAGONAL RIGHT FORWARD", "X METERS AT ANGLE Y"]
    rephrase_move = ["move", "go", "advance", "coast", "glide", "get yourself", "move yourself", "proceed"]
    prepositions = ["", "by", "a distance of", "for", "for a total distance of", "equal to", "by a measure of", "about", "by about", "about", "around"]
    rephrase_forward = ["forward", "onwards", "in front of you", "straight ahead"]
    rephrase_rotate = ["turn", "shift", "rotate", "spin", "turn yourself", "shift yourself", "rotate yourself", "spin"]
    rephrase_rot = ["to the left", "to your left", "to your right", "to the right", "clockwise", "counterclockwise", "CW", "CCW", "leftward", "rightward"]
    rephrase_side = ["to your left", "to your right", "to the left", "to the right", "leftwards", "rightwards", "to your relative east", "to your relative west"]

    randtype = random.choice(inst_types)

    if randtype == "FWD1":
      dist =  prompt_randomizer.rand_dist()
      return random.choice(rephrase_move) + " " +  random.choice(prepositions) +" " +  dist + " " +  random.choice(rephrase_forward)
    elif randtype == "FWD2":
      dist =  prompt_randomizer.rand_dist()
      return random.choice(rephrase_move) +" " +   random.choice(rephrase_forward) + " " + random.choice(prepositions) +" " +  dist
    elif randtype == "ROT1":
      rot = prompt_randomizer.rand_rot()
      return random.choice(rephrase_rotate)  +" " +  rot +" " +  random.choice(rephrase_rot)
    elif randtype == "ROT2":
        rot = prompt_randomizer.rand_rot()
        return random.choice(rephrase_rotate) +" " +  random.choice(rephrase_rot) + " " + rot
    elif randtype == "SIDE1":
        dist =  prompt_randomizer.rand_dist()
        return random.choice(rephrase_move) +" " +  random.choice(prepositions) +" " +  dist +" " +  random.choice(rephrase_side)
    elif randtype == "SIDE2":
      dist =  prompt_randomizer.rand_dist()[0]
      return random.choice(rephrase_move) +" " +  random.choice(rephrase_side) + " " + random.choice(prepositions) +" " +  dist
    elif randtype == "BACK":
      dist =  prompt_randomizer.rand_dist()
      return random.choice(rephrase_move) +" " +  dist + " behind you"
    elif randtype == "DIAGONAL LEFT FORWARD":
      dist =  prompt_randomizer.rand_dist()
      return random.choice(rephrase_move) + " " + dist + " diagonally to your left"
    elif randtype == "DIAGONAL RIGHT FORWARD":
      dist =  prompt_randomizer.rand_dist()
      return random.choice(rephrase_move) +" " +  dist + " diagonally to your right"
    elif randtype == "X METERS AT ANGLE Y":
      dist =  prompt_randomizer.rand_dist()
      rot = prompt_randomizer.rand_rot()
      return random.choice(rephrase_move) + " " +  dist + " at " + rot + random.choice(rephrase_rot)

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

print(prompt_randomizer.prompt_maker())
