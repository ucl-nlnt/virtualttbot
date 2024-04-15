# for levels 1 and 2

import csv
import random
import ast
import math

class random_csv:
    def csv_randomizer():
        with open('Prompts/combined.csv', newline='') as f:
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
                prompt[1][i] = (action, round(math.degrees(value)))
            elif action == 'MOVE':
                continue

        prompt = tuple(prompt)

        return prompt

