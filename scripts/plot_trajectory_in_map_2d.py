import argparse
import sys
import yaml
import matplotlib.pyplot as plt


def main(args):
    error = False
    if args.map is None:
        print("Parameter '--map' is not set.")
        error = True

    if args.resolution is None:
        print("Parameter '--resolution' is not set.")
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
    print('[resolution]: ' + str(args.resolution))
    print('[trajectory]: ' + args.trajectory)
    print('[output]:     ' + args.output)


    scale = 1.0 / args.resolution
    xs = []
    ys = []
    with open(args.trajectory, 'r') as f:
        for l in f:
            entries = [x.strip() for x in l.split(',')]
            xs.append(float(entries[1]) * scale)
            ys.append(-float(entries[2]) * scale)

    img = plt.imread(args.map)
    fig, ax = plt.subplots()

    ax.imshow(img, cmap='gray', origin='upper')

    ax.plot(xs, ys, '-', linewidth=1, color='blue')
    # plt.scatter(xs, ys, s=2, facecolors='none', edgecolors='blue')



    fig = plt.gcf()
    fig.savefig(args.output, dpi=300)
    plt.show()


    return


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Render an occupancy map with the trajectory it was recorded with.')
    parser.add_argument('--map',        help='Saved occupancy grid map in compatible image format.',     type=str)
    parser.add_argument('--resolution', help='Occupancy grid resolution.',                               type=float)
    parser.add_argument('--trajectory', help='Trajectory in yaml format in with nodes in pixel format.', type=str)
    parser.add_argument('--output',     help='The path the result should be saved to.',                  type=str)

    args = parser.parse_args()
    main(args)