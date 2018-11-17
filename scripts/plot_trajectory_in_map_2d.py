#!/usr/bin/python2
import argparse
import sys
import yaml
import matplotlib.pyplot as plt
import os

def main(args):
    error = False
    if args.map is None:
        print("Parameter '--map' is not set.")
        error = True

    assert(args.resolution != 0.0)

    if args.trajectory is None:
        print("Parameter '--trajectory' is not set.")
        error = True

    if args.output is None:
        print("Parameter '--output' is not set.")
        error = True


    if error:   
        return

    print('[map]:        ' + args.map)
    print('[trajectory]: ' + args.trajectory)
    print('[output]:     ' + args.output)

    # map in ros yaml format
    origin      = [0., 0., 0.]
    image       = ""
    resolution  = 0.0
    root        = os.path.dirname(args.map)

    with open(args.map, 'r') as yaml_document:
        try:
            data        = yaml.load(yaml_document)
            origin      = data["origin"]
            resolution  = data["resolution"]
            image       = root + '/' + data["image"]

            print("origin    : " + str(origin))
            print("Image     : " + str(image))
            print("Resolution: " + str(resolution))

        except yaml.YAMLError as exc:
            print(exc)

    scale = 1.0 / resolution
    xs = []
    ys = []
    img = plt.imread(image)

    with open(args.trajectory, 'r') as f:
        for l in f:
            entries = [x.strip() for x in l.split(',')]
            x = float(entries[1]) - origin[0]
            y = float(entries[2]) - origin[1]

            xs.append(x * scale)
            ys.append(img.shape[0] - y * scale)

    fig, ax = plt.subplots()

    ax.imshow(img, cmap='gray', origin='upper')

    ax.plot(xs, ys, '-', linewidth=1, color='blue')
    plt.scatter(xs, ys, s=1, facecolors='none', edgecolors='blue',linewidth='1')



    fig = plt.gcf()
    fig.savefig(args.output, format='eps', dpi=1000)
    plt.show()


    return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Render an occupancy map with the trajectory it was recorded with.')
    parser.add_argument('--map',        help='Saved occupancy grid map in compatible image format.',     type=str)
    parser.add_argument('--resolution', help='Occupancy grid resolution.',                               type=float)
    parser.add_argument('--trajectory', help='Trajectory in csv format in with nodes in pixel format.', type=str)
    parser.add_argument('--output',     help='The path the result should be saved to.',                  type=str)

    args = parser.parse_args()
    main(args)