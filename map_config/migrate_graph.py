import yaml

with open("graph_copy.yaml", 'r') as stream:
    contents = yaml.load(stream)

for vtx in contents:
    vtx["id"] += 49
    for i in range(len(vtx["edges"])):
        vtx["edges"][i] += 49
    vtx["z"] = 15.0

with open("graph_copy2.yaml", 'w') as stream:                                                                             
        yaml.dump(contents, stream, default_flow_style=False)
