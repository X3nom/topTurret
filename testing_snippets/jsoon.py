import json

with open("test.json", "r") as f:
    team_config_json = json.load(f)

for t in team_config_json["teams"]:
    print(t)
    print(t["name"])

team_config_json["teams"][0]["name"]

with open("test.json", "w") as f:
    json.dump(team_config_json, f, indent=4)