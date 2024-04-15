from prompt_randomizer import prompt_randomizer

x = prompt_randomizer.prompt_maker()
for i in x[3].keys():
    print(x[3][i])