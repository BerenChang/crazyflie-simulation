import csv

class trajectory():
    def __init__(self, path, time_scale=1.0):
        # read trajectory from csv file
        with open(path) as csvfile:
            self.waypoints = list(csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC))
        # accumulate time
        time = 0.0
        for i in range(len(self.waypoints)):
            time += time_scale * self.waypoints[i][0]
            self.waypoints[i][0] = time
        self.waypoints.reverse()

    def get_waypoint(self, time):
        if len(self.waypoints) == 1 or time <= self.waypoints[-1][0]:
            return self.waypoints[-1]
        else:
            self.waypoints.pop()
            return self.waypoints[-1]
