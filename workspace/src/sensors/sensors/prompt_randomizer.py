import random
import json
import math
from csv_randomizer import random_csv

class prompt_randomizer:
  #def __init__(self):
  #  self.init = [0.0, 0.0]

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
        meas = ["m", " meter", "ft", " foot", " ruler", " yard", " yardstick"]
        mets = [1, 1, 0.31, 0.31, 0.31, 0.91, 0.91]
        num = random.randint(0, len(meas)-1)
        dis_type = meas[num]
        in_meters = mets[num]

      #space = ['', ' ']
      #return (str(rand_num) + random.choice(space) + dis_type, in_meters)
      return (str(rand_num) + dis_type, in_meters)

    elif selected_num_type == "quarters":
      rand_quarter = ["a single", "a quarter", "a third", "a fourth", "a quarter", "a fifth", "a half", "three quarters", "an eigth", "two fifths", "three eighths", "five eights"]
      writ = [1, 0.25, 0.30, 0.25, 0.25, 0.20, 0.5, 0.75, 0.125, 0.40, 0.375, 0.625]
      measure = [" meter", " foot", " ruler", " yardstick"]
      num = random.randint(0, len(rand_quarter)-1)

      meas = random.randint(0, len(measure)-1)

      if meas == 0:
        in_meters = writ[num]
      elif meas == 1 or meas == 2:
        in_meters = round(writ[num]/3.281, 2)
      elif meas == 3:
        in_meters = round(writ[num]/1.094, 2)

      if num == 1:
        return (rand_quarter[num] + " " + measure[meas], in_meters)
      else:
        return (rand_quarter[num] + " of a" + measure[meas], in_meters)

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

      return (numbers[num] + " " + random.choice(added) + measure[meas], in_meters)

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
  
  #def rand_insts(max_per_prompt):
  #  no_of_instructions = random.randint(1, max_per_prompt)
  #  return no_of_instructions

  def rand_rot():
    angle_type = ["degrees", "radians", "rads", "degs", "%", "fraction"]
    rot_type = random.choice(angle_type)

    if rot_type == "degrees" or rot_type == "degs":
      deg = random.randint(0, 360)
      return (str(deg) + " " + rot_type, deg)

    elif rot_type == "radians" or rot_type == "rads":
      rad_type = ["numerical", "pi"]
      a = random.choice(rad_type)

      if a == "numerical":
        rad = float(random.randint(0, 628))/100
        deg = round(rad * 57.296, 2)
        return ( str(rad) + " " + rot_type, deg)

      elif a == "pi":
        pi_rot = ["pi/6", "pi/4", "pi/3", "pi/2", "pi", "2pi/3", "3pi/4", "5pi/6", "7pi/6", "5pi/4", "4pi/3", "3pi/2", "5pi/3", "7pi/4", "11pi/6", "2 pi"]
        deg_rot = [30, 45, 60, 90, 180, 120, 135, 150, 210, 225, 240, 270, 300, 315, 330, 360]
        rads = [" radians", " rads", ""]
        deg_choice = random.randint(0, len(pi_rot)-1)
        return (pi_rot[deg_choice] + random.choice(rads), deg_rot[deg_choice])

    elif rot_type == "%":
      percentage = random.randint(0, 100)
      deg = (float(percentage) / 100.00) * 360.00
      return (str(percentage) + "%", deg)

    elif rot_type == "fraction":
      frac_rot = ["1/2", "1/4", "1/3", "1/8", "3/4", "half", "quarter", "three quarters", "3 quarters", "a third", "an eighth"]
      deg_rot = [180, 90, 120, 45, 270, 180, 90, 270, 270, 120, 45]
      deg_choice = random.randint(0, len(frac_rot) -1)
      return (str(frac_rot[deg_choice]) + " of a circle", deg_rot[deg_choice])

  # Simple Randomizer
  def rand_inst():
    inst_types = ["FWD1", "FWD2",  "LROT1", "LROT2", "RROT1", "RROT2", "LSIDE1", "LSIDE2", "RSIDE1", "RSIDE2", "BACK1", "BACK2", "DIAGONAL LEFT FORWARD", "DIAGONAL RIGHT FORWARD", "X METERS AT ANGLE Y LEFT", "X METERS AT ANGLE Y RIGHT", "DRAW SHAPE", "DRAW SHAPE"] #, "WAIT"]
    rephrase_move = ["move", "go", "advance", "coast", "glide", "get yourself", "move yourself", "proceed"]
    prepositions = ["", "by", "a distance of", "for", "for a total distance of", "equal to", "by a measure of", "about", "by about", "about", "around"]
    prepositions2 = ["", "by", "a distance of", "for", "for a total distance of", "equal to", "by a measure of", "about", "by about", "about", "around"]
    rephrase_forward = ["forward", "onwards", "in front of you", "straight ahead"]
    rephrase_rotate = ["turn", "shift", "rotate", "spin", "turn yourself", "shift yourself", "rotate yourself", "spin"]
    rephrase_lrot = ["to the left", "to your left", "counterclockwise", "CCW", "leftward"]
    rephrase_rrot = ["to your right", "to the right", "clockwise", "CW", "rightward"]
    rephrase_lside = ["to your left", "to the left", "leftwards", "to your relative west"]
    rephrase_rside = ["to your right", "to the right", "rightwards", "to your relative east"]
    rephrase_back1 = ["behind you", "backwards", "rearward", "to your rear", "to the spot behind you"]
    rephrase_back2 = ["behind you", "backwards", "in reverse", "rearward", "to your rear"]

    randtype = random.choice(inst_types)

    #randtype = "WAIT"

    if randtype == "FWD1":
      dist =  prompt_randomizer.rand_dist()
      return (random.choice(rephrase_move) + " " +  random.choice(prepositions) +" " +  dist[0] + " " +  random.choice(rephrase_forward), "(MOVE, " + str(round(dist[1],2)) + ")", [("MOVE", round(dist[1],2))])
    elif randtype == "FWD2":
      dist =  prompt_randomizer.rand_dist()
      return (random.choice(rephrase_move) + " " +   random.choice(rephrase_forward) + " " + random.choice(prepositions) +" " +  dist[0], "(MOVE, " + str(round(dist[1],2)) + ")", [("MOVE", round(dist[1],2))])
    elif randtype == "LROT1":
      rot = prompt_randomizer.rand_rot()
      return (random.choice(rephrase_rotate)  + " " +  rot[0] + " " +  random.choice(rephrase_lrot), "(LEFT, " + str(round(rot[1], 2)) + ")", [("LEFT", round(rot[1], 2))])
    elif randtype == "LROT2":
        rot = prompt_randomizer.rand_rot()
        return (random.choice(rephrase_rotate) + " " +  random.choice(rephrase_lrot) + " " + rot[0], "(LEFT, " + str(round(rot[1], 2)) + ")", [("LEFT", round(rot[1], 2))])
    elif randtype == "RROT1":
      rot = prompt_randomizer.rand_rot()
      return (random.choice(rephrase_rotate)  + " " +  rot[0] + " " +  random.choice(rephrase_rrot), "(RGHT, " + str(round(rot[1], 2)) + ")", [("RGHT", round(rot[1], 2))])
    elif randtype == "RROT2":
        rot = prompt_randomizer.rand_rot()
        return (random.choice(rephrase_rotate) + " " +  random.choice(rephrase_rrot) + " " + rot[0], "(RGHT, " + str(round(rot[1], 2)) + ")", [("RGHT", round(rot[1], 2))])
    elif randtype == "LSIDE1":
        dist =  prompt_randomizer.rand_dist()
        return (random.choice(rephrase_move) + " " +  random.choice(prepositions) + " " +  dist[0] + " " +  random.choice(rephrase_lside), "(LEFT, 90), (MOVE, " + str(round(dist[1],2)) +")", [("LEFT", 90), ("MOVE", round(dist[1],2))])
    elif randtype == "LSIDE2":
      dist =  prompt_randomizer.rand_dist()
      return (random.choice(rephrase_move) + " " +  random.choice(rephrase_lside) + " " + random.choice(prepositions2) + " " +  dist[0], "(LEFT, 90), (MOVE, " + str(round(dist[1],2)) +")", [("LEFT", 90), ("MOVE", round(dist[1],2))])
    elif randtype == "RSIDE1":
      dist =  prompt_randomizer.rand_dist()
      return (random.choice(rephrase_move) + " " +  random.choice(prepositions) + " " +  dist[0] + " " +  random.choice(rephrase_rside), "(RGHT, 90), (MOVE, " + str(round(dist[1],2)) +")", [("RGHT", 90), ("MOVE", round(dist[1],2))])
    elif randtype == "RSIDE2":
      dist =  prompt_randomizer.rand_dist()
      return (random.choice(rephrase_move) + " " +  random.choice(rephrase_rside) + " " + random.choice(prepositions2) + " " +  dist[0], "(RGHT, 90), (MOVE, " + str(round(dist[1],2)) +")", [("RGHT", 90), ("MOVE", round(dist[1],2))])
    elif randtype == "BACK1":
      dist =  prompt_randomizer.rand_dist()
      return (random.choice(rephrase_move) + " " +  dist[0] + " " + random.choice(rephrase_back1), "(RGHT, 180), (MOVE, " + str(round(dist[1],2)) +")", [("RGHT", 180), ("MOVE", round(dist[1],2))])
    elif randtype == "BACK2":
      dist =  prompt_randomizer.rand_dist()
      return (random.choice(rephrase_move) + " " + random.choice(rephrase_back2)+ " " +  random.choice(prepositions) + " " + dist[0], "(RGHT, 180), (MOVE, " + str(round(dist[1],2)) +")", [("RGHT", 180), ("MOVE", round(dist[1],2))])
    elif randtype == "DIAGONAL LEFT FORWARD":
      dist =  prompt_randomizer.rand_dist()
      return (random.choice(rephrase_move) + " " + dist[0] + " diagonally to your left", "(LEFT, 45), (MOVE, " + str(round(dist[1],2)) +")", [("LEFT", 45), ("MOVE", round(dist[1],2))])
    elif randtype == "DIAGONAL RIGHT FORWARD":
      dist =  prompt_randomizer.rand_dist()
      return (random.choice(rephrase_move) + " " +  dist[0] + " diagonally to your right", "(RGHT, 45), (MOVE, " + str(round(dist[1],2)) +")", [("RGHT", 45), ("MOVE", round(dist[1],2))])
    elif randtype == "X METERS AT ANGLE Y LEFT":
      dist =  prompt_randomizer.rand_dist()
      rot = prompt_randomizer.rand_rot()
      return (random.choice(rephrase_move) + " " +  dist[0] + " at " + rot[0] + " " + random.choice(rephrase_lrot) , "(LEFT, " + str(round(rot[1], 2)) + "), (MOVE, " + str(round(dist[1],2)) +")", [("LEFT", round(rot[1], 2)), ("MOVE", round(dist[1],2))])
    elif randtype == "X METERS AT ANGLE Y RIGHT":
      dist =  prompt_randomizer.rand_dist()
      rot = prompt_randomizer.rand_rot()
      return (random.choice(rephrase_move) + " " +  dist[0] + " at " + rot[0] + " " + random.choice(rephrase_rrot) , "(RGHT, " + str(round(rot[1], 2)) + "), (MOVE, " + str(round(dist[1],2)) +")", [("RGHT", round(rot[1], 2)), ("MOVE", round(dist[1],2))])
    elif randtype == "WAIT":
      rephrase_wait = ["wait @ seconds", "pause for @ seconds", "pause for a duration of @ seconds", "hold on for @ seconds", "give it @ seconds", "wait patiently for @ seconds", "pause for a count of @ seconds", "take a brief @-second pause", "allow @ seconds to pass", "hold off for just @ seconds", "wait out the @-second interval", "take a momentary break for @ seconds", "temporarily halt for @ seconds"]
      wait_time = round(float(random.randint(5, 150))/10,2)
      phrasing = random.choice(rephrase_wait).replace("@", str(wait_time))
      gen_equiv = [("WAIT", wait_time)]
      str_equiv = "(WAIT, " + str(wait_time) + ")"
      return (phrasing, str_equiv, gen_equiv)
    elif randtype == "DRAW SHAPE":
      rephrase_start = ["sketch", "create", "outline", "render", "draft", "make", "sketch out", "form", "illustrate", "construct", "design"]
      shaper = ["triangle", "triangular", "3-sided", "three-sided", "four-sided", "4-sided", "@four-sided", "@4-sided", "square", "quadrilateral with equal sides and right angles", "rectangle", "rectangular", "quadrilateral with equal opposite sides and right angles", "five-sided", "5-sided", "pentagon", "six-sided", "6-sided", "hexagon", "seven-sided", "7-sided", "heptagon", "eight-sided", "8-sided", "octagon", "nine-sided", "9-sided", "nonagon", "10-sided", "ten-sided", "decagon"]
      shape_sides = [3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10, 10, 10]

      shape_int = random.randint(0, len(shaper)-1)
      shape = shaper[shape_int]

      if shape == "rectangle" or shape == "rectangular" or shape == "quadrilateral with equal opposite sides and right angles":
        dist1 = prompt_randomizer.rand_dist()
        dist2 = prompt_randomizer.rand_dist()
        if shape == "rectangular":
          extra = [" shape", " form", " figure"]
          shape += random.choice(extra)
        choose = [True, False]
        chosen = random.choice(choose)
        if chosen:
          # ex: sketch a 5x6 rectangular shape
          rephrase_sides = ["x", " by ", " x "]
          phrasing = random.choice(rephrase_start) + " a " + dist1[0] + random.choice(rephrase_sides) + dist2[0] + " " + shape
        else:
          rephrase_next = ["with dimensions", "measuring", "with sides of", "with dimensions of", "that measures"]
          rephrase_sides = ["@1x@2", "@1 x @2", "@1 by @2", "@1 wide and @2 tall", "@1 across and @2 high"]
          phrasing = random.choice(rephrase_start) + " a " + shape + " " + random.choice(rephrase_next) + " " + (random.choice(rephrase_sides)).replace("@1", str(dist1[0])).replace("@2", str(dist2[0]))

        #str_equiv = '[("MOVE", ' + dist1 + ')], [("LEFT", ' + 90 ')], [("MOVE", ' + dist2 + ')], [("LEFT", ' + 90 ')], [("MOVE", ' + dist1 + ')], [("LEFT", ' + 90 ')], [("MOVE", ' + dist2 + ')], [("LEFT", ' + 90 ')]'
        gen_equiv = [("MOVE", round(dist1[1], 2)), ("LEFT", 90), ("MOVE", round(dist2[1], 2)), ("LEFT", 90), ("MOVE", round(dist1[1], 2)), ("LEFT", 90), ("MOVE", round(dist2[1], 2))]
        str_equiv = str(gen_equiv).replace("[", "").replace("]", "")
        return (phrasing, str_equiv, gen_equiv)

      elif shape == "square" or shape == "quadrilateral with equal sides and right angles" or shape == "@four-sided" or shape == "@4-sided":
        if shape == "square" or shape == "@four-sided" or shape == "@4-sided":
          shape = shape.replace("@", "")
          extra = [" shape", " figure", ""]
          shape += random.choice(extra)

        dist =  prompt_randomizer.rand_dist()

        chosen = random.randint(0, 1)
        if chosen:
          # ex: sketch a 5x5 square shape
          rephrase_sides = ["x", " by ", " x "]
          phrasing = random.choice(rephrase_start) + " a " + dist[0] + random.choice(rephrase_sides) + dist[0] + " " + shape
        else:
          rephrase_next = ["with dimensions", "measuring", "with sides of", "with dimensions of", "that measures", "with side lengths equal to"]
          rephrase_sides = ["@x@", "@ by @", "@", "@ x @"]
          phrasing = random.choice(rephrase_start) + " a " + shape + " " + random.choice(rephrase_next) + " " + (random.choice(rephrase_sides)).replace("@", str(dist[0]))
      
        gen_equiv = [("MOVE", round(dist[1], 2)), ("LEFT", 90), ("MOVE", round(dist[1], 2)), ("LEFT", 90), ("MOVE", round(dist[1], 2)), ("LEFT", 90), ("MOVE", round(dist[1], 2))]
        str_equiv = str(gen_equiv).replace("[", "").replace("]", "")

        return (phrasing, str_equiv, gen_equiv)

      else: 
        dist =  prompt_randomizer.rand_dist()
        angle = round(360/shape_sides[shape_int], 2)

        gen_equiv =[]

        if shape == "triangular" or ("sided" in shape):
          extra = [" shape", " form", " figure"]
          shape += random.choice(extra)

        rephrase_next = ["with @-sides", "with sides measuring @", "with sides equal to @", "with side lengths equal to @", "with dimensions equal to @", "with @ dimensions", "measuring @", "that measures @ per side", "with length @ per side"]
        
        if shape == "octagon":
          preposition = " an "
        else:
          preposition = " a "

        phrasing = random.choice(rephrase_start) + preposition + shape + " " + (random.choice(rephrase_next)).replace("@", str(dist[0]))
        
        gen_equiv = []

        for i in range(shape_sides[shape_int]-1):
          gen_equiv.append(("MOVE", round(dist[1], 2)))
          gen_equiv.append(("LEFT", round(angle, 2)))

        gen_equiv.append(("MOVE", round(dist[1], 2)))
        str_equiv = str(gen_equiv).replace("[", "").replace("]", "")

        return (phrasing, str_equiv, gen_equiv)

      # "create a <shape> with sides with length equal to <dist>"
      # "draw a <shape> "
      # sketch a @x@ or @by@ <rectangle/square>
      # sketch a <shape> with @ sides
      # sketch a <shape> with sides equal to @

  def flag(max_time=10):

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
      wait = random.randint(0, max_time)

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
      possible_prompts = ["Wait up to @ seconds once the obstacle is detected. If the obstacle is still there after @ seconds, try to go around it. If you can't go around it, stop.", "Wait up to X seconds for obstacle removal. If it persists, attempt avoidance; if unsuccessful, stop.", "Wait until X seconds for barrier clearance. If it persists, try to circumvent; if not possible, stop.", "Wait X seconds maximum for obstruction removal. If it remains, attempt avoidance; if unsuccessful, stop.", "Allow up to X seconds for hurdle elimination. If it lingers, try to navigate around; if not feasible, stop.", "Give it X seconds max for barrier eradication. If it continues, attempt avoidance; if unachievable, stop.", "Wait until X seconds for impediment removal. If it persists, try to sidestep; if impossible, stop."]
      prompt_addition = random.choice(possible_prompts)

      go_around = True
      wait = random.randint(0, max_time)

      # replace X
      prompt_addition = prompt_addition.replace("@", str(wait))
      if wait == 1:
        prompt_addition = prompt_addition.replace("1 seconds", "1 second")

    #t_or_f = [True, False]
    #announce = random.choice(t_or_f)            # announce that an obstacle was detected in status

    json_flags = {
      "@WAIT": wait,
      "@STOP": stop,
      "@GO_AROUND": go_around,
      }

    #flags = json.loads(str(json_flags))
    return (prompt_addition, json_flags)


  def compute_total(x, init):
      #print(str(x[0]))
      #print(str(x[1]))
      if x[0] == "MOVE":
        init[0] = init[0] + x[1]
        init[0] = round(init[0], 2)
      	#init[0] = init[0] + 1

      elif x[0]  == "LEFT":
        init[1] = init[1] - x[1]
        init[1] = round(init[1], 2)
      	#init[1] = init[1] - 1

      elif x[0]  == "RGHT":
        init[1] = init[1] + x[1]
        init[1] = round(init[1], 2)
      	#init[1] = init[1] + 1

      return init

  def ground_truth_zero(inst, s_coords):
    n_coords = s_coords

    if inst[0] == 'MOVE':
      # double_check
      n_coords[0] += inst[1] * math.sin(math.radians(s_coords[2]))           # x
      n_coords[1] += inst[1] * math.cos(math.radians(s_coords[2]))           # y
    elif inst[0] == 'LEFT':
      n_coords[2] -= inst[1]      # z
    elif inst[0] == 'RGHT':
      n_coords[2] += inst[1]      # z
    #else:
      # do nothing
      
    n_coords[0] = round(n_coords[0], 2)
    n_coords[1] = round(n_coords[1], 2)
    n_coords[2] = round(n_coords[2], 2)
    return n_coords

  def prompt_maker(max_per_prompt = 3, coords = [0, 0, 0]):
    simple_no = random.randint(0, 10)
    if simple_no < 1:
      return prompt_randomizer.prompt_maker_csv(coords)
    else:
      return prompt_randomizer.prompt_maker_here(max_per_prompt, coords)
  
  def prompt_maker_csv(coords):
    ground_truth = [coords[:]]
    computed_move =[]
    init = [0.0, 0.0]
    csv_prompt = random_csv.csv_randomizer()
    print(csv_prompt)
    prompt = csv_prompt[0]
    simple_move = csv_prompt[1]
    equiv = csv_prompt[2][:]

    for y in range(len(equiv)):
      init = prompt_randomizer.compute_total(equiv[y], init)
      computed_move.append(init[:])
      equiv.append(y)
      gt_next = prompt_randomizer.ground_truth_zero(equiv[y], coords)
      #print(gt_next)
      coords = gt_next
      ground_truth.append(coords[:])

    flag = prompt_randomizer.flag()

    equiv = csv_prompt[2].append(("STOP"))

    add_flags = prompt_randomizer.flag()
    prompt = prompt + " " + add_flags[0]
    
    #computed_move = 

    json_fl = {
      "nl_prompt": prompt,
      "instructions": equiv,
      "ground_truth_coordinates": ground_truth,
      "cumulative": computed_move,
      "flags": add_flags[1]
    }

    coords = [0, 0, 0]        # reset coords

    return (prompt, simple_move, computed_move, json_fl)

  def prompt_maker_here(max_per_prompt, coords):
    # maximum number of steps per prompt ; does not count in repeated steps (ex. 2x)
    prompt = ""
    no_of_insts = random.randint(1, max_per_prompt)

    #no_of_insts = prompt_randomizer.rand_insts(max_per_prompt)
    init = [0.0, 0.0]  # initialize randomizer
    computed_move = []
    #no_of_insts = 5
    simple_move = []
    equiv = []
    ground_truth = [coords[:]]       # [x, y, phi]
    #print(ground_truth)
    x = 0
    #print("no of insts: ", no_of_insts)

    polite = ('', '', 'Please ', '', 'Show me how you can ', '', '')
    prompt += random.choice(polite)

    #for x in range(no_of_insts):
    while x < no_of_insts:
      #print(x)
      if (no_of_insts) == 2 and (x != 0):
        add = [' and ', ' then ', '. Please also ', ', and ', ', then ', '. Then, ']
        to_add = random.choice(add)

        if to_add == '. Please also ' or to_add == '. Then, ' or random.randint(0, 10) >= 7:
          prompt = prompt.capitalize()

        prompt += to_add

      elif no_of_insts > 2:
        if (x < no_of_insts - 1) and (x != 0):
          add = [', ', ', then ']
          prompt += random.choice(add)
        elif (x == no_of_insts - 1) and (x != 0):
          addition = [', and ', ', then ', ', finally, ']
          prompt += random.choice(addition)

      new_inst = prompt_randomizer.rand_inst()
      prompt += new_inst[0]
      simple_move.append(new_inst[1])

      if x == 0:
        prompt = prompt.capitalize()

      for y in new_inst[2]:
        init = prompt_randomizer.compute_total(y, init)
        computed_move.append(init[:])
        equiv.append(y)
        gt_next = prompt_randomizer.ground_truth_zero(y, coords)
        coords = gt_next
        #print(y)
        #print(coords[:])
        ground_truth.append(coords[:])

      #max_per_prompt -= 1
      x += 1

      # of times
      a = random.randint(0, 10)
      if a > 8:
        multi = prompt_randomizer.rand_multiples(no_of_insts - x)
        prompt += " " + multi[0]
        for b in range(multi[1]):
          simple_move.append(new_inst[1])

          for c in new_inst[2]:
            init = prompt_randomizer.compute_total(c, init)
            computed_move.append(init[:])
            equiv.append(c)
            gt_next = prompt_randomizer.ground_truth_zero(c, coords)
            #print(c)
            #print(coords)
            coords = gt_next
            ground_truth.append(coords[:])

          x += 1

    flag = prompt_randomizer.flag()



    #if random.randint(0, 10) >= 7:
    #	prompt = prompt.capitalize()

    #if random.randint(1, 2) == 2:
    #prompt += "."

    equiv.append(("STOP"))

    add_flags = prompt_randomizer.flag()
    prompt = prompt + ". " + add_flags[0]

    prompt = prompt.replace(" a a ", " a ")


    #prompt = prompt.capitalize()

    json_fl = {
      "nl_prompt": prompt,
      "instructions": equiv,
      "ground_truth_coordinates": ground_truth,
      "cumulative": computed_move,
      "flags": add_flags[1]

    }

    coords = [0, 0, 0]        # reset coords

    return (prompt, simple_move, computed_move, json_fl)
    #return json_fl

#TEST
#print(prompt_randomizer.flag()[0])

'''
if __name__ == "__main__":
  #testing
  prompt1 = prompt_randomizer.prompt_maker(5)
  print("Prompt: ", prompt1[0])
  print("Single: ", prompt1[1])
  print("Cumulative: ", prompt1[2])
  print("Flag: ", prompt1[3])

  print("\n")

  prompt2 = prompt_randomizer.prompt_maker(8, [1, 2, 3])
  print("Prompt: ", prompt2[0])
  print("Single: ", prompt2[1])
  print("Cumulative: ", prompt2[2])
  print("Flag: ", prompt2[3])

  print("\n")

  prompt3 = prompt_randomizer.prompt_maker(1)
  print("Prompt: ", prompt3[0])
  print("Single: ", prompt3[1])
  print("Cumulative: ", prompt3[2])
  print("Flag: ", prompt3[3])
'''