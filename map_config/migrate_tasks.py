import sys
import yaml

with open(sys.argv[1], 'r') as stream:
    contents = yaml.load(stream)

for i in range(len(contents)):
    for j in range(len(contents[i]["path"])):
        contents[i]["path"][j] += int(sys.argv[2])

with open("out.yaml", 'w') as stream:                                                                             
    yaml.dump(contents, stream, default_flow_style=False)
