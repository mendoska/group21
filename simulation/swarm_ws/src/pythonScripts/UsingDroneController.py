import rclpy
from rclpy.node import Node
from CLIFunctions.cliFunctions import addDroneToSimulation
from Models.DroneController import DroneController
from Models.Drone import Drone
from icecream import ic
import asyncio


async def spin_node(node):
    while node.is_at_target_position() is False:
        ic("here")
        rclpy.spin()
        await asyncio.sleep(0.01) 




async def main(args=None) -> None:
    # Ros2 Command Line Init Function to "open" a connection to the simunlation

   
    droneDirectory = {}
    summonNumberOfDrones = 5

    for x in range(summonNumberOfDrones):
       addDroneToSimulation(droneID=x)
       droneDirectory[x]=(Drone(droneID=x, droneController=DroneController(droneID=x)))
       
    ic(droneDirectory)

    try:
        spinList=[]
        # Start the ROS 2 event loop for each DroneController
        for droneID in droneDirectory:
            ic(droneDirectory[droneID].droneID)
            iteratedDroneController=droneDirectory[droneID].droneController
            # iteratedDroneController.move_to_target_position()
            spinList.append(spin_node(node=iteratedDroneController))

    

    #     # Wait for user input to start all drones simultaneously
        # input("Press Enter to start all drones simultaneously...")

    #     # Update the target positions or perform any other actions before starting the drones
    #     # ...

    #     # Start all drones simultaneously by calling move_to_target_position
        for droneID in droneDirectory:
            ic(droneDirectory[droneID].droneID)
            
            iteratedDroneController=droneDirectory[droneID].droneController
            iteratedDroneController.timerCallback()
            
        await asyncio.gather(*spinList)
    #     # Continue spinning the event loop to listen for callbacks
            # rclpy.spin(node=iteratedDroneController)

    # except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        for droneID in droneDirectory:
            ic(droneDirectory[droneID].droneController)
            droneDirectory[droneID].droneController.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    
    asyncio.run(main())
    
    rclpy.shutdown()
    
    
    