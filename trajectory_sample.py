class TrajectorySample:
    
    def __init__(self):
        
        self.time = []

        self.velocities = {
        	"v_x" : [],
        	"v_y" : [],
        	"v_z" : []
        }

        self.accelerations = {
        	"a_x" : [],
        	"a_y" : [],
        	"a_z" : []
        }

        self.positions = {
        	"x" : [],
        	"y" : [],
        	"z" : []
        }

        self.jerk = {
        	"j_x" : [],
        	"j_y" : [],
        	"j_z" : []
        }

        self.snap = {
        	"s_x" : [],
        	"s_y" : [],
        	"s_z" : []
        }