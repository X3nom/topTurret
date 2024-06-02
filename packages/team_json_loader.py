import json

class TeamConfig():
    def __init__(self, fPath = "config/teams.json") -> None:
        self.fPath = fPath
        with open(self.fPath, "r") as f:
            self.teams_dict = json.load(f)
    
    def get_teams(self):
        return self.teams_dict["teams"]

    def get_friendly(self):
        friendly = []
        for name in self.teams_dict["friendly"]:
            friendly.append(*list(filter(lambda team: team['name'] == name, self.teams_dict["teams"])))
        return friendly
    
    def get_enemy(self):
        enemy = []
        for name in self.teams_dict["enemy"]:
            enemy.append(*list(filter(lambda team: team['name'] == name, self.teams_dict["teams"])))
        return enemy


# tc = TeamConfig()