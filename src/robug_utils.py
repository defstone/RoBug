import copy

class v3:

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return '[ x: {:.3f}, y: {:.3f}, z: {:.3f} ]'.format(self.x, self.y, self.z)

    def add(self, A):
        self.x += A.x
        self.y += A.y
        self.z += A.z

    def sub(self, A):
        self.x -= A.x
        self.y -= A.y
        self.z -= A.z
        
    def mult(self, k):
        self.x = self.x * k
        self.y = self.y * k
        self.z = self.z * k 

    def set(self, A):
        self.x = A.x
        self.y = A.y
        self.z = A.z
        
    def set_from_list(self, L):
        self.x = L[0]
        self.y = L[1]
        self.z = L[2]       
    
    def to_list(self):
        return [self.x, self.y, self.z]
    
    # micropython says: not deepcopy-able
    # works in cpython -> investigate
#     def copy(self):
#         return copy.deepcopy(self)    