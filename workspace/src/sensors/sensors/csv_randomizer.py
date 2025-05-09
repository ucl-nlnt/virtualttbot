# for levels 1 and 2

import csv
import random
import ast
import math
import os

class random_csv:
    def csv_randomizer():
        this_path = os.path.abspath(os.path.dirname(__file__))
        path = os.path.join(this_path, "Prompts/combined.csv")

        with open(path, newline='') as f:
            read_file = csv.reader(f)
            all_prompts = [list(row) for row in read_file]

        prompt = random.choice(all_prompts)

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
        prompt = (prompt[0], str_equiv, prompt[1])

        return prompt

#print(random_csv.csv_randomizer())