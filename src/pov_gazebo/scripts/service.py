from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

# <plugin filename="gz-sim-user-commands-system"
#     name="gz::sim::systems::UserCommands">
# </plugin>

def main():
    node = Node()
    request = Pose()
    request.name = "simple_box"
    request.position.x = 0
    request.position.y = 5
    request.position.z = 1
    req = node.request("/world/default/set_pose",
                       request ,
                    Pose,
                    Boolean,
                    timeout=300)
    
    print(req)

if __name__ == "__main__":
    main()