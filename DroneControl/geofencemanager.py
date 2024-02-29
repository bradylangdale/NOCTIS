import math 
import json
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np
from pathfinder import Pathfinder


# TODO: remove hardcoded paths only here for debugging purposes
class GeofenceManager:

    def __init__(self):
        self.geofence = None
        self.navmesh = None
        self.mesh_size = None
        self.angular_res = 111139
        self.pathfinder = None
        self.kia = []

        try:
            with open('../DroneControl/data/geofence.json', 'r') as f:
                self.geofence = json.load(f)
            
            print('Successfully loaded geofence.json')

            try:
                self.navmesh = list(np.loadtxt('../DroneControl/data/navmesh.data'))
                print('Successfully loaded navmesh.data')

                self.find_bounds()

                self.pathfinder = Pathfinder(self.navmesh)
                print('Successfully initialized the pathfinder.')
            except:
                print('No navmesh found but geofence.json is loaded, generating navmesh.data.')

                self.find_bounds()
                self.generate_navmesh()

                np.savetxt('../DroneControl/data/navmesh.data', self.navmesh)

                print('Navmesh generating and written.')
                
                self.pathfinder = Pathfinder(self.navmesh)
                print('Successfully initialized the pathfinder.')
        except:
            if not self.geofence:
                print('No geofence.json found.')
            else:
                print('Error generating and saving navmesh.')

    def update_geo_nav_data(self, geofence):
        self.geofence = json.loads(geofence)
        with open('../DroneControl/data/geofence.json', 'w') as f:
            json.dump(self.geofence, f)
 
        self.find_bounds()
        self.generate_navmesh()

        np.savetxt('../DroneControl/data/navmesh.data', self.navmesh)

        print('Navmesh generating and rewritten.')
        
        self.pathfinder = Pathfinder(self.navmesh)
        print('Successfully reinitialized the pathfinder.')

    def find_bounds(self):
        initial_point = self.geofence[0]['data'][0]

        self.x_bounds = [initial_point[0], initial_point[0]]
        self.y_bounds = [initial_point[1], initial_point[1]]

        for shape in self.geofence:
            if shape['type'] != 'KIA':
                continue

            for point in shape['data']:
                if point[0] < self.x_bounds[0]:
                    self.x_bounds[0] = point[0]

                if point[0] > self.x_bounds[1]:
                    self.x_bounds[1] = point[0]

                if point[1] < self.y_bounds[0]:
                    self.y_bounds[0] = point[1]

                if point[1] > self.y_bounds[1]:
                    self.y_bounds[1] = point[1]

        self.mesh_size = [abs(self.x_bounds[1] - self.x_bounds[0]) * self.angular_res, abs(self.y_bounds[1] - self.y_bounds[0]) * self.angular_res]
        self.mesh_size = [int(math.ceil(self.mesh_size[0])), int(math.ceil(self.mesh_size[1]))]

    def latlng_to_index(self, coord):
        return [
            int((self.mesh_size[0] - 1) * ((coord[0] - self.x_bounds[0]) / (self.x_bounds[1] - self.x_bounds[0]))),
            int((self.mesh_size[1] - 1) * ((coord[1] - self.y_bounds[0]) / (self.y_bounds[1] - self.y_bounds[0])))
        ]

    def index_to_latlng(self, index):
        delta = [
            self.x_bounds[1] - self.x_bounds[0],
            self.y_bounds[1] - self.y_bounds[0]
        ]

        return [
            delta[0] * (index[0] / (self.mesh_size[0] - 1)) + self.x_bounds[0],
            delta[1] * (index[1] / (self.mesh_size[1] - 1)) + self.y_bounds[0]
        ]
    
    def apply_shape_to_mesh(self, shape, fill, is_kia=False, kia_only=False):
        if not kia_only:
            for x in range(self.mesh_size[0]):
                for y in range(self.mesh_size[1]):
                    if shape.contains(Point(self.index_to_latlng([x, y]))):
                        self.navmesh[x][y] = fill

                        if is_kia:
                            self.kia.append([x, y])
        else:
            for p in self.kia:
                if shape.contains(Point(self.index_to_latlng([p[0], p[1]]))):
                        self.navmesh[p[0]][p[1]] = fill

    def generate_navmesh(self):
        self.navmesh = [[-10 for y in range(self.mesh_size[1])] for x in range(self.mesh_size[0])]

        for shape in self.geofence:
            if shape['type'] != 'KIA':
                continue
            
            self.kia = []
            self.apply_shape_to_mesh(Polygon(shape['data']), -1, is_kia=True)

        for shape in self.geofence:
            if shape['type'] != 'EA':
                continue
            
            self.apply_shape_to_mesh(Polygon(shape['data']), -1, kia_only=True)
                                     
        for shape in self.geofence:
            if shape['type'] != 'KOA':
                continue
            
            self.apply_shape_to_mesh(Polygon(shape['data']), -10, kia_only=True)

    def move_from_edge(self, point, radius=3):
        green_count = 0
        gx = 0
        gy = 0

        red_count = 0
        rx = 0
        ry = 0
        for x in range(radius * 2):
            for y in range(radius * 2):
                px = (x + point[0]) - radius
                py =(y + point[1]) - radius

                if self.navmesh[px][py] == -10:
                    rx += px
                    ry += py
                    red_count += 1
                elif self.navmesh[px][py] == -1:
                    gx += px
                    gy += py
                    green_count += 1

        if red_count > 1:
            rx /= red_count
            ry /= red_count

            gx /= green_count
            gy /= green_count

            length = math.sqrt(math.pow(gx - rx, 2) + math.pow(gy - ry, 2))

            vx = (radius * ((gx - rx) / length)) + gx
            vy = (radius * ((gy - ry) / length)) + gy

            return [vx, vy]
        else:
            return point

    def move_by_radius(self, path, radius=3):
        new_path = []
        new_path.append(path[0])

        i = 1
        while i < len(path) - 1:
           new_path.append(self.move_from_edge(path[i], radius=radius))
           i += 1

        new_path.append(path[-1])

        return new_path  

    def line_of_sight(self, start, end):
        length = int(math.sqrt(math.pow(end[0] - start[0], 2) + math.pow(end[1] - start[1], 2)))

        if length < 10:
            length = 10

        for i in range(length - 1):
            t = i / length

            x = int(t * (end[0] - start[0]) + start[0])
            y = int(t * (end[1] - start[1]) + start[1])

            if self.navmesh[x][y] == -10:
                return False
            
        return True
    
    def remove_by_los(self, path, start_point):
        new_path = []
        new_path.append(start_point)
        i = 0
        # line of site optimization
        while i < len(path) - 1:
            j = 1

            collision = False
            while (i + j) < len(path):
                if self.line_of_sight(path[i], path[i + j]):
                    j += 1
                elif j > 1:
                    collision = True
                    new_path.append(path[i + j - 1])
                    i += j - 1
                    break
                else:
                    collision = True
                    new_path.append(path[i + 1])
                    i += 1
                    break

            if not collision:
                new_path.append(path[-1])
                break

        return new_path
    
    def check_radius(self, start, end, radius=3):
        length = int(math.sqrt(math.pow(end[0] - start[0], 2) + math.pow(end[1] - start[1], 2)))

        if length < 20:
            length = 20

        for i in range(5, length - 5):
            t = i / length

            x = int(t * (end[0] - start[0]) + start[0])
            y = int(t * (end[1] - start[1]) + start[1])

            new_point = self.move_from_edge([x, y], radius=radius)

            if new_point != [x, y]:
                return False, new_point
            
        return True, None
    
    def ensure_radius(self, path, radius=3):
        new_path = []
        new_path.append(path[0])

        i = 1
        while i < len(path):
            los, new_point = self.check_radius(path[i - 1], path[i], radius=radius)

            if not los:
                new_path.append(new_point)
                new_path.append(path[i])
            else:
                new_path.append(path[i])

            i += 1

        return new_path

    def get_path(self, start, end):
        start_index = self.latlng_to_index(start)
        end_index = self.latlng_to_index(end)

        path = self.pathfinder.getPath(start_index, end_index)

        if not path:
            path = self.pathfinder.getPath(end_index, start_index)

            if not path:
                return []
            else:
                path.reverse()
        
        path = self.move_by_radius(path, radius=4)
        path = self.remove_by_los(path, start_index)
        
        # do three checks for good measure
        path = self.ensure_radius(path, radius=3)
        path = self.ensure_radius(path, radius=3)
        path = self.ensure_radius(path, radius=3)

        coords = []
        for p in path:
            coords.append(self.index_to_latlng(p))

        return coords
    
    def get_survey_path(self, start):
        
