import pygame
import sys
import math
from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from tf_transformations import quaternion_from_euler


def spherical_to_cartesian(r, theta, phi):
    x = r * math.sin(phi) * math.cos(theta)
    y = r * math.sin(phi) * math.sin(theta)
    z = r * math.cos(phi)
    return x, y, z

def send_pose_request(x, y, z, phi, theta):
    request = Pose()
    request.name = "simple_box"
    request.position.x = x
    request.position.y = y
    request.position.z = z
    q = request.orientation
    
    q.x, q.y, q.z, q.w = quaternion_from_euler(0, -phi, math.pi + theta)
    

    res = node.request("/world/default/set_pose",
                       request ,
                    Pose,
                    Boolean,
                    timeout=300)
    

def main():
    # Initialize Pygame
    pygame.init()

    # Create a window
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Keyboard Event Listener")
    theta = math.radians(0)  # 45 degrees azimuthal angle
    phi = math.radians(0) 
    R = 10
    # Main loop
    running = True
    while running:
        calc = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # Detect key press
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    phi-=0.1
                    calc = True
                elif event.key == pygame.K_DOWN:
                    phi+=0.1
                    calc = True
                elif event.key == pygame.K_LEFT:
                    theta-=0.1
                    calc = True
                elif event.key == pygame.K_RIGHT:
                    theta+=0.1
                    calc = True
                elif event.key == pygame.K_PLUS:
                    R+=0.1
                    calc = True
                elif event.key == pygame.K_MINUS:
                    R-=0.1
                    calc = True
                elif event.key == pygame.K_q:
                    running = False

            if calc:
                x, y, z = spherical_to_cartesian(R, theta, phi)
                print(f"{x=}, {y=}, {z=}, R=0, P={math.degrees(phi)}, Y={math.degrees(theta)}")
                send_pose_request(x, y, z, phi, theta)
            # Detect key release
            # elif event.type == pygame.KEYUP:
            #     print(f"Key released: {pygame.key.name(event.key)}")

    # Quit Pygame
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    node = Node()
    main()
