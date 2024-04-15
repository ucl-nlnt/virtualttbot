# for levels 1 and 2

import csv
import random

class random_csv:
    def csv_randomizer():
        with open('/home/marae/Desktop/virtualttbot/workspace/src/sensors/sensors/Prompts/combined.csv', newline='') as f:
            read_file = csv.reader(f)
            all_prompts = [list(row) for row in read_file]

        #print(len(all_prompts))
        #prompt_no = random.randint(0, len(all_prompts)-1)
        prompt = random.choice(all_prompts)

        #print(prompt)
        prompt[0] = prompt[0].replace('"', "")
        prompt[1] = prompt[1].replace('"', "")

        prompt[1] = prompt[1].replace("|", ",")

        return prompt

        #print("Prompt: ", prompt[0])
        #print("Equivalent: ", prompt[1])
        #print(prompt)