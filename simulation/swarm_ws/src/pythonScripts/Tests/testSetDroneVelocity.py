from CLIFunctions.cliFunctions import setDroneVelocity, getDroneLocation


speedMultiplier=.01
location=getDroneLocation(droneID=1)
setDroneVelocity(droneID=1, xVelocity=-location["x"]*speedMultiplier,yVelocity=-location["y"]*speedMultiplier,zVelocity=location["z"])
# setDroneVelocity(droneID=0, xVelocity=40,yVelocity=-10)
