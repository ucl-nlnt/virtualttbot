from prompt_randomizer import prompt_randomizer
import ast


def parse_instruction(instruction):
    # Removing parentheses and splitting by comma
    parts = instruction.replace('(', '').replace(')', '').split(', ')
    # Casting the second part to float if it contains a decimal point, else to int
    action = parts[0].strip()
    value = float(parts[1]) if '.' in parts[1] else int(parts[1])
    return action, value

        
print()
y = prompt_randomizer.prompt_maker()
x = y[1]
print(y[0])
print(y[2])
for i in x:
    print('=========================')
    q = i.split(', ')
    print(q)
    print(len(q))
    print('elements:')
    s = '['
    for j in range(0,len(q),2):
        s +=  q[j]+', '+q[j+1]
        s += ', '
    s += ']'
    print(s)
    print(ast.literal_eval(s))