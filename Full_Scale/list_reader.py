def parse_file(filename='Ideal_Flight_Profile.csv'):
    with open(filename) as f:
        lines = f.readlines()

    for i, line in enumerate(lines):
        line = line.replace('\n','').split(',')
        line = [float(i) for i in line]
        lines[i] = line

    return lines