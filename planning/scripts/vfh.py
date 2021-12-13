import math

""" Python implementation of VFH+ algorithm
    Next step is implementing VFH*

    Current pitfalls/todos:
    Edge conditions - not sure how to handle, currently having robot not consider adjacent out
                        of bounds cells. Maybe should move checking for edges into polar_histogram
    Getting stuck in alcoves/long straight obstacles
        - should be fixed in VFH*
    Constants need configuring - changing constants give some wonky results
    Converting between gps and histogramgrid coordinates
        - should probably add a class specifically for localization of gps coordinates
"""


def wrap(index, l):
    """ Helper function to get valid index of list
    """
    if index >= len(l):
        return len(l) - 1
    elif index < 0:
        return 0
    else:
        return index


class HistogramGrid:
    """ Class HistogramGrid defines a nested array ("grid") of certainty values
        Coordinate points start from 0
    """

    MAX_CERTAINTY = 15

    def __init__(self, nrows, ncols):
        self.grid = [([0] * ncols)] * nrows

    def out_of_bounds(self, x, y):
        """ Returns whether the cell is out of the grid. Used for edge conditions """
        return 0 > y or y >= len(self.grid) or 0 > x or x >= len(self.grid[0])

    def get_valid_x(self, index):
        return wrap(index, self.grid[0])

    def get_valid_y(self, index):
        return wrap(index, self.grid)

    def get_certainty(self, x, y):
        return self.grid[y][x]

    def add_certainty(self, x, y):
        """ Increments cell certainty by one, capped at MAX_CERTAINTY.
            Maybe change increment value to parameter in future iterations """
        if self.grid[y][x] < self.MAX_CERTAINTY:
            self.grid[y][x] += 1

    def print_hg(self, robot_locations, start, end, current):
        """ For testing purposes """
        string = ""
        for y in range(len(self.grid)):
            for x in range(len(self.grid[0])):

                if self.get_certainty(x, y) == 1:
                    string += "1 "
                elif (x, y) == start:
                    string += "S "
                elif (x, y) == end:
                    string += "E "
                elif (x, y) == current:
                    string += "C "
                elif (x, y) in robot_locations:
                    string += "X "
                else:
                    string += "0 "
            string += "\n"
        string += "0/1 - Free/Occupied (Certainty values)\nX - Robot locations\nS - Start Position (%d, %d)\nE - End Target (%d, %d)\nC - Current" % (
            start[0], start[1], end[0], end[1])
        print (string)


class PolarHistogram:
    """ Class PolarHistogram defines an array of sectors each with a certainty value
        IMPORTANT! Each sector angle is at the start of the sector. The sector
                    starts at each angle then goes clockwise
    """

    def __init__(self, nsectors):
        if 360 % nsectors != 0:
            raise ValueError("nsectors should be a factor of 360")

        self.nsectors = nsectors
        self.sector_angle = 360/nsectors
        self.polar_histogram = [0] * self.nsectors

    def get(self, index):
        return self.polar_histogram[wrap(index, self.polar_histogram)]

    def smooth_polar_histogram(self, l):
        """Smoothing function that smooths the values of the histogram using a moving average."""
        smoothed_histogram = [0] * self.nsectors

        for k_i in range(self.nsectors):
            smoothed_histogram[k_i] = sum(
                [(l - abs(k_i-l_i)) * self.get(l_i) for l_i in range(k_i-l+1, k_i+l)]) / (2*l+1)

        self.polar_histogram = smoothed_histogram

    def __str__(self):
        """ Testing purposes """
        string = ""
        for tup in enumerate(self.polar_histogram):
            if tup[1] != 0:
                string += "{:<3} {}\n".format(tup[0]
                                              * self.sector_angle, tup[1])
        return string


def small_angle_diff(a1, a2):
    """ Helper function for getting smallest angle difference between two angles """
    return abs((a1 - a2 + 180) % 360 - 180)


def wrap_angle(angle):
    """ Helper function to keep angle under 360 """
    if angle < 0:
        return 360 + angle
    elif angle >= 360:
        return angle - 360
    else:
        return angle


class VFH:
    """ Class VFH contains class methods for VFH+ algorithm
    """

    @classmethod
    def map_active_hg_to_ph(cls, hg, ph, vcp, w_s, a=1, b=1, l=5):
        """ Function map_active_hg_to_ph takes a HistogramGrid and populates an empty PolarHistogram

            Keyword Arguments:
            hg - HistogramGrid object
            ph - empty PolarHistogram object
            vcp - Vehicle Center Position defined as a tuple (x, y)
            w_s - Window Size defines the width of the active region
                    Thus, the radius of the active region is (w_s - 1)/2
                    to account for the vcp
            a, b - Positive constants for calculating cell magnitude
            l - Positive constant for PolarHistogram smoothing method
        """
        print ("\nMAP_ACTIVE_HG_TO_PH")
        print ("VCP {}   W_S {}".format(vcp, w_s))

        robot_x, robot_y = vcp
        window_radius = (w_s - 1)/2

        if a - b * math.sqrt(2) * window_radius != 0:
            print ("Not optimum values for positive a b constants. a - b * sqrt(2) * window_radius should be 0.")
            a = b * math.sqrt(2) * window_radius
            print ("Setting a to %d" % a)

        print ("Active Region -- X marks the robot")
        ar_string = ""
        for y in range(hg.get_valid_y(int(robot_y - window_radius)), hg.get_valid_y(int(robot_y + window_radius + 1))):
            for x in range(hg.get_valid_x(int(robot_x - window_radius)), hg.get_valid_x(int(robot_x + window_radius + 1))):
                ar_string += "{} ".format(hg.get_certainty(x, y)
                                          ) if (x, y) != vcp else "X "
            ar_string += "\n"
        print (ar_string[:-1])

        # Loop through all cells in active region
        for x in range(int(robot_x - window_radius), int(robot_x + window_radius + 1)):
            for y in range(int(robot_y - window_radius), int(robot_y + window_radius + 1)):

                dy = robot_y - y
                dx = robot_x - x
                # The cell layer in the active region of the current cell
                layer = float(max(abs(dy), abs(dx)))
                # E.g. if the vcp is (0, 0), then (1, 1) would be layer 1, (2, 0) layer 2....
                #print "(%d, %d) layer: %d" % (x, y, layer)

                # Just a quick effort to reduce compute time
                # Skips cells that are out of bounds (if they are not layer 1), are free, or are the vcp
                # Layer 1 out of bounds cells are used to generate the "border" around the edge of the HistogramGrid
                if not (layer == 1 and hg.out_of_bounds(x, y)) and (hg.out_of_bounds(x, y) or hg.get_certainty(x, y) == 0 or (x, y) == vcp):
                    #print "    OU?T early"
                    continue

                # Angle between cell and vcp. 0 is E. wrap_angle changes range to [0, 359]
                cell_angle = wrap_angle(math.degrees(math.atan2(dx, dy)))
                # Distance between cell and vcp
                cell_distance = math.hypot(dx, dy)
                cell_certainty = hg.get_certainty(x, y) if not hg.out_of_bounds(
                    x, y) else HistogramGrid.MAX_CERTAINTY
                # MAX_CERTAINTY for edge of histogramgrid

                # Grabbed from paper
                cell_magnitude = (cell_certainty ** 2) * (
                    a - b * cell_distance) if not hg.out_of_bounds(x, y) else HistogramGrid.MAX_CERTAINTY

                # Angle between center of each cell in current layer
                layer_angle_interval = 360.0 / (layer * 8)

                print ("({0:<2}, {1:<2}) {5:>3}/{6:<3} = {2:6.1f} deg (layer {8:4.1f})-- Distance: {3:5.1f} Certainty: {7} Magnitude: {4:.1f}".format(
                    x, y, cell_angle, cell_distance, cell_magnitude, dy, dx, cell_certainty, layer))

                # Loops through all the ph sectors that pass through current cell
                # Is more important at closer layers
                for sector_index in range(int(math.floor((cell_angle - layer_angle_interval / 2) / ph.sector_angle)),
                                           int(math.ceil((cell_angle + layer_angle_interval / 2) / ph.sector_angle))):
                    if sector_index < len(ph.polar_histogram):
                        #print "   sector:", sector_index * ph.sector_angle
                        if layer == 1 and hg.out_of_bounds(x, y):
                            # MAX_CERTAINTY for edge of histogramgrid
                            ph.polar_histogram[sector_index] += HistogramGrid.MAX_CERTAINTY
                        else:
                            ph.polar_histogram[sector_index] += cell_magnitude

                # Deprecated
                # sector_index = int(math.floor(cell_angle/ph.sector_angle) if cell_angle >= 0 else math.ceil(cell_angle/ph.sector_angle) - 1)
                # ph.polar_histogram[sector_index] += cell_magnitude

        # Helps smooth steering, just kidding idk what it does but is important
        ph.smooth_polar_histogram(l)
        print ("Smoothed Polar Histogram")
        print (ph)

        return ph

    @classmethod
    def get_best_direction(cls, ph, target_direction, current_direction, previous_direction, t=10, smax=5, a=5, b=1, c=1):
        """ Function get_best_direction uses PolarHistogram to determine the best direction
            IMPORTANT! Angle orientation/notation shown below
                N
                0
            E 90 270 W
               180
                S

            Keyword Arguments:
            ph - polar_histogram array NOT THE OBJECT
            target_direction - angle to target
            current_direction - current robot angle (heading)
            previous_direction - previous robot angle
        """
        print ("\nGETTING BEST DIRECTION")

        # Filter out sectors that are above the threshold
        free_sectors = list(filter(lambda s: s[1] < t, enumerate(ph)))
        sector_angle = 360 / len(ph)
        # Sector target is in
        target_sector = int(target_direction / sector_angle)

        print ("Free sectors: %d" % len(free_sectors))
        for tup in free_sectors:
            print ("Angle: {0:3} deg    Certainty: {1}".format(
                tup[0] * sector_angle, tup[1]))
        print ("Target sector:", target_sector * sector_angle)

        print ("Time to get the free valleys")
        free_valleys = []
        start_angle = free_sectors[0][0] * \
            sector_angle  # Start with first free sector

        for i in range(len(free_sectors)):
            #print "s[%d] a: %d c: %d" % (i, free_sectors[i][0] * sector_angle, free_sectors[i][1])
            if i + 1 == len(free_sectors):  # If this is the last free sector
                if len(free_valleys) == 0:  # Only one valley
                    print ("Only one valley")
                    free_valleys.append(
                        [start_angle, (free_sectors[i][0] + 1) * sector_angle])
                #elif free_sectors[i][0] * sector_angle == 360 - sector_angle and free_valleys[0][0] == 0:
                    # If the first valley begins <360 and ends >0 e.g. 320 -> 20
                    #print ("Wrapping first valley")
                    #free_valleys[0] = (start_angle, free_valleys[0][1] + 360)
                else:
                    print ("Putting in last valley")
                    free_valleys.append(
                        [start_angle, (free_sectors[i][0] + 1) * sector_angle])

            elif free_sectors[i + 1][0] > free_sectors[i][0] + 1:
                print ("Gap starting new valley")
                free_valleys.append(
                    [start_angle, (free_sectors[i][0] + 1) * sector_angle])
                start_angle = free_sectors[i + 1][0] * sector_angle
        print ("Candidate valleys:", free_valleys)

        print ("Generating candidate angles")
        candidate_angles = []

        # Deprecated
        # for v in free_valleys:
        #     if v[0] < target_direction < v[1] or v[0] < target_direction + 360 < v[1]: # If target angle is in a free valley, add it to candidate angles
        #         print "Target in btwn (%d, %d)" % (v[0], v[1])
        #         candidate_angles.append(target_direction)

        #     if v[1] - v[0] > smax * sector_angle: # wide valley
        #         print "Wide Valley"
        #         candidate_angles.append(v[0]) # Add start and end to candidate angles
        #         candidate_angles.append(v[1])

        #     candidate_angles.append((v[0] + v[1]) / 2) # Add middle angle to candidate angles

        # Finds valley nearest to target sector and gets an angle spaced smax sectors away from edge of valley
        target_in_valley = False
        for v in free_valleys:
            if v[1] - v[0] > smax * sector_angle:  # wide valley
                # Add start and end to candidate angles
                candidate_angles.append(wrap_angle(
                    v[0] + smax * sector_angle / 2))
                candidate_angles.append(wrap_angle(
                    v[1] - sector_angle * sector_angle / 2))
                print ("Wide Valley (%d, %d)... Adding %d and %d" % (
                    v[0], v[1], candidate_angles[-2], candidate_angles[-1]))

                # If target angle is in a free valley, add it to candidate angles
                if v[0] < target_direction < v[1] or v[0] < target_direction + 360 < v[1]:
                    print ("Target in btwn (%d, %d)... Adding %d" % (
                        v[0], v[1], target_direction))
                    candidate_angles.append(target_direction)

                # Deprecated
                # # Start from side of the valley closest to the target and head smax sectors toward the other side of the valley
                # # Creates space when near obstacles
                # if not target_in_valley:
                #     # Sorted with nearest side first and far second second
                #     sorted_valley = sorted(v, key=lambda a: small_angle_diff(a, target_direction))
                #     print "Nearest side:", sorted_valley[0]
                #     side = sorted_valley[1] - sorted_valley[0]
                #     print "side", side

                #     if side < 0:
                #         candidate_angles.append(wrap_angle(sorted_valley[0] - smax * sector_angle / 2))
                #         print "Going counter-clockwise... Adding", candidate_angles[-1]
                #     else:
                #         candidate_angles.append(wrap_angle(sorted_valley[0] + smax * sector_angle / 2))
                #         print "Going clockwise... Adding", candidate_angles[-1]
            else:
                # Add middle of valley to candidat angles
                candidate_angles.append((v[0] + v[1]) / 2)
                print ("Narrow Valley (%d, %d)... Adding %d" % (
                    v[0], v[1], candidate_angles[-1]))

        # Early exit
        if len(candidate_angles) == 1:
            return candidate_angles[0]

        # Grabbed from paper
        if a <= b + c:
            print ("Not best values for a b c constants, should satisfy a > b + c")
            a = b + c + 1
            print ("Setting a to", a)

        def delta(a1, a2):
            return min(abs(a1 - a2), abs(a1 - a2 - sector_angle), abs(a1 - a2 + sector_angle))

        print ("Calculating costs!")
        costs = []
        for ca in candidate_angles:  # Calculate cost for each candidate angle
            # Positive constants:
            # a - goal oriented steering
            # b, c - smooth steering
            costs.append(a * delta(ca, target_direction) + b * delta(ca,
                                                                     current_direction) + c * delta(ca, previous_direction))
            print ("Candidate angle: {0:5.1f}   Cost: {1}".format(ca, costs[-1]))

        # Target angle is angle with lowest cost
        target_angle = min(
            candidate_angles, key=lambda a: costs[candidate_angles.index(a)])
        print

        return target_angle

        # Deprecated
        # target_valley = None
        # diff = 360
        # for v in free_valleys:
        #     if v[0] < target_direction < v[1]:
        #         print "Target in btwn (%d, %d) so returning target" % (v[0], v[1])
        #         return target_direction

        #     print "valley: (%d, %d) - dif: %d - diffs: (%d, %d)" % (v[0], v[1], diff,  small_angle_diff(v[0], target_direction), small_angle_diff(v[1], target_direction))
        #     if small_angle_diff(v[0], target_direction) < diff:
        #         diff = small_angle_diff(v[0], target_direction)
        #         target_valley = v
        #     if small_angle_diff(v[1], target_direction) < diff:
        #         diff = small_angle_diff(v[1], target_direction)
        #         target_valley = v
        # print "Target Valley: (%d, %d)" % (target_valley[0], target_valley[1])

        # nearest_side = min(target_valley, key=lambda a: small_angle_diff(a, target_direction))
        # print "Nearest side:", nearest_side
        # side = (target_direction - nearest_side + 180) % 360 - 180
        # print "side", side

        # if target_valley[1] - target_valley[0] > smax * sector_angle: # wide valley
        #     print "Wide Valley"
        #     if side < 0:
        #         print "Going counter-clockwise"
        #         return wrap_angle(nearest_side - smax * sector_angle / 2)
        #     else:
        #         print "Going clockwise"
        #         return wrap_angle(nearest_side + smax * sector_angle / 2)
        # else:
        #     print "Narrow Valley"
        #     return (target_valley[0] + target_valley[1]) / 2


""" TO BE IMPLEMENTED """


# class Sensor:
#     """ "Abstract Class" for sensors (lidar, sonar...) to implement
#     """

#     def get_readings(self):
#         """ Should return dictionary of polar coordinates of readings
#         """
#         pass


# class Bot:
#     """ Unsure of what/how to put/use this class
#         May move most of localization funcationality to new class
#     """

#     def __init__(self, cell_resolution):
#         self.cell_resolution = cell_resolution

#     def add_sensor(self, sensor):
#         self.sensors.append(sensor)

#     def update_hg(self):
#         readings = list(map(Sensor.get_readings, self.sensors))
#         localized_readings = localize_readings(readings)
#         for r in localized_readings:
#             self.hg.add_certainty(r[0], r[1])

#     def localize(self, polar):
#         angle, distance = polar
#         local_x = int((distance * math.cos(math.radians(angle))) / self.cell_resolution)
#         local_y = int((distance * math.sin(math.radians(angle))) / self.cell_resolution)
#         return (local_x, local_y)

#     def localize_readings(self, readings):
#         localized_readings = list(map(self.localize, readings))
#         return localized_readings

#     def update_global_location(glob_loc):
#         self.global_location = glob_loc
#         self.localized_location = self.localize_global_location(glob_loc)

#     def localize_global_location(glob_loc):
#         pass
