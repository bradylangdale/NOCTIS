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

        try:
            with open('/home/brady/Projects/NOCTIS/DroneControl/data/geofence.json', 'r') as f:
                self.geofence = json.load(f)
            
            print('Successfully loaded geofence.json')

            try:
                self.navmesh = list(np.loadtxt('/home/brady/Projects/NOCTIS/DroneControl/data/navmesh.data'))
                print('Successfully loaded navmesh.data')

                self.find_bounds()

                self.pathfinder = Pathfinder(self.navmesh)
                print('Successfully initialized the pathfinder.')
            except:
                print('No navmesh found but geofence.json is loaded, generating navmesh.data.')

                self.find_bounds()
                self.generate_navmesh()

                np.savetxt('/home/brady/Projects/NOCTIS/DroneControl/data/navmesh.data', self.navmesh)

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
        with open('/home/brady/Projects/NOCTIS/DroneControl/data/geofence.json', 'w') as f:
            json.dump(self.geofence, f)
 
        self.find_bounds()
        self.generate_navmesh()

        np.savetxt('/home/brady/Projects/NOCTIS/DroneControl/data/navmesh.data', self.navmesh)

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
    
    def apply_shape_to_mesh(self, shape, fill):
        for x in range(self.mesh_size[0]):
            for y in range(self.mesh_size[1]):
                if shape.contains(Point(self.index_to_latlng([x, y]))):
                    self.navmesh[x][y] = fill

    def generate_navmesh(self):
        self.navmesh = [[-10 for y in range(self.mesh_size[1])] for x in range(self.mesh_size[0])]

        for shape in self.geofence:
            if shape['type'] != 'KIA':
                continue
            
            self.apply_shape_to_mesh(Polygon(shape['data']), -1)

        for shape in self.geofence:
            if shape['type'] != 'EA':
                continue
            
            self.apply_shape_to_mesh(Polygon(shape['data']), -1)
                                     
        for shape in self.geofence:
            if shape['type'] != 'KOA':
                continue
            
            self.apply_shape_to_mesh(Polygon(shape['data']), -10)


    def get_path(self, start, end):
        start_index = self.latlng_to_index(start)
        end_index = self.latlng_to_index(end)

        path = self.pathfinder.getPath(start_index, end_index)

        if not path:
            path = self.pathfinder.getPath(end_index, start_index)

            if not path:
                return []

        coords = []
        coords.append(start);
        for p in path:
            coords.append(self.index_to_latlng(p))
        coords.append(end);

        return coords
