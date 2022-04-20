import argparse

def main():

    argParser = argparse.ArgumentParser(description='Saves wifi map to png file')
    
    argParser.add_argument("-r", "--resolution", metavar='n', type=int, nargs=2,
                            default = [800, 600],
                            help="Specifies the resolution of the png in pixels")
    
    argParser.add_argument("-o", "--origin", metavar='n', type=float, nargs=3,
                           default = [0, 0, 0],
                           help="Center of the map in meters")

    argParser.add_argument("-s", "--pixelSize", metavar='n', type=float, nargs=1,
                            default = [.01],
                            help="Size of each pixel in meters")

    argParser.add_argument("-p", "--path", metavar='filePath', type=str, nargs=1,
                            default = ["wifiMap.png"],
                            help="Path of exported png file")

    argParser.add_argument("-w", "--wifiSamples", metavar='b', type=int, nargs=1,
                            default = [1],
                            help="Plot WifiSample points if 'b' is non-zero")


    args = argParser.parse_args()
    print(f"Saving png with args: {args}")

    from system.RobotSystem import RobotSystem
    from world.world2d import world2d

    world = world2d()
    
    robot_system = RobotSystem(world)

    robot_system.system.wifiMap.RenderMap(
        pngPath = args.path[0],
        resolution = args.resolution, 
        pixelSize = args.pixelSize[0], 
        origin = args.origin,            
        plotSamples = (args.wifiSamples[0] != 0)       
    )

if __name__ == '__main__':
    main()