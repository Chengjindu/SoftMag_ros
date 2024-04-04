# coding: utf8
import Sofa.Core
import rclpy

def precb(data):
    d = data.tolist()
    d = float(d[0])
    print(f'the received pressure is {d}') 
    return d
    
def poscb(data):
    d = data.tolist()
    d = float(d[0])
    print(f'the received position is {d}') 
    return d
    
def moveRestPos(rest_pos, dx, dy, dz):
    out = []
    for i in range(0,len(rest_pos)) :
        out += [[rest_pos[i][0]+dx, rest_pos[i][1]+dy, rest_pos[i][2]+dz]]
    return out

class RosSender(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # to reduce the latency in TCP, we can disable Nagle's algo with tcp_nodelay=False in ROS1
        # (https://en.wikipedia.org/wiki/Nagle%27s_algorithm)
        # Todo: find the equivalent in ROS2
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = "RosSender"

        # Args
        self.node = args[0]
        rosname = args[1]
        self.datafield = args[2]
        msgtype = args[3]
        self.sendingcb = args[4]

        # Create or connect to the topic rosname as a publisher
        self.pub = self.node.create_publisher(msgtype, rosname, 10)

    def onAnimateEndEvent(self, event):
        data = self.sendingcb(self.datafield)
        self.pub.publish(data)


class RosReceiver(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.snode = kwargs["node"]
        self.constraints = []
        #self.dofs = []
        for i in range(1,3):
            #self.dofs.append(self.snode.getChild('finger' + str(i)).tetras)
            self.constraints.append(self.snode.getChild('finger'+str(i)).cavity.SurfacePressureConstraint)
        
        self.name = "RosReceiver"

        self.node = args[0]
        msgtype = args[1]
        
        topic_pressure = "/animation/receiver/pressure"
        topic_position = "/animation/receiver/position"


        # Create or connect to the topic rosname as a subscription
        self.sub = self.node.create_subscription(msgtype, topic_pressure, self.pressureCB, 10)
        self.sub = self.node.create_subscription(msgtype, topic_position, self.positionCB, 10)

        self.predata = None
        self.posdata = None

    def pressureCB(self, data):
        self.predata = data.data
        
    def positionCB(self, data):
        self.posdata = data.data

    def onAnimateBeginEvent(self, event):
        rclpy.spin_once(self.node, timeout_sec=0.001)  # Something must be hidden in this spin_once(), without it the callback() is never called
        if self.predata is not None:
            d = precb(self.predata)
            for i in range(2):
            	self.constraints[i].value[0] = d*self.snode.dt.value
            print (f'Pressure Value = {d} kPa')
            self.predata = None
            
        if self.posdata is not None:
            d = poscb(self.posdata)
            self.dofs[1].rest_position.value = d*self.snode.dt.value
            print (f'Position Value = {d} kPa')
            self.posdata = None


def init(nodeName="Sofa"):
    rclpy.init()
    node = rclpy.create_node(nodeName)
    node.get_logger().info('Created node')
    return node
