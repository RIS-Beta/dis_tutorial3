class Cluster:

    #global id counter for clusters
    id_counter = 0

    def __init__(self, type, position, normal):
        self.id = self.id_counter
        Cluster.id_counter += 1
        self.type = type # "people" or "ring"
        self.center_position = position # [y,x,z] 
        self.status = "NOT_INTERACTED" # "NOT_INTERACTED", "READY", "INTERACTED"
        self.normal = normal # normal vector for calcluating the pose for robot postion to interact
        self.count = 1 #how many markers are in the cluster for better clustering

    def update(self, new_marker, new_normal):
        #getting new center
        self.center_position = [(self.center_position[0]*self.count + new_marker.pose.position.y)/(self.count+1),
                                (self.center_position[1]*self.count + new_marker.pose.position.x)/(self.count+1),
                                (self.center_position[2]*self.count + new_marker.pose.position.z)/(self.count+1)]
        
        #normal vector for calculating the pose for robot postion to interact
        sum_normal = [self.normal[0]*self.count + new_normal[0],
                       self.normal[1]*self.count + new_normal[1],
                       self.normal[2]*self.count + new_normal[2]]
        norm = np.linalg.norm(sum_normal)
        if norm != 0:
            self.normal = [sum_normal[0]/norm, sum_normal[1]/norm, sum_normal[2]/norm]
        
        self.count += 1