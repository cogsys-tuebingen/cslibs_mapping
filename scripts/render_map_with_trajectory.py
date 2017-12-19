import argparse
import sys
import yaml
import matplotlib.pyplot as plt

def main(argv):
    parser = argparse.ArgumentParser(description='Render an occupancy map with the trajectory it was recorded with.')
    parser.add_argument('--map', help='Saved occupancy grid map in compatible image format.', type=str)
    parser.add_argument('--trajectory', help='Trajectory in yaml format in with nodes in pixel format.', type=str)
    parser.add_argument('--output', help='The path the result should be saved to.', type=str)

    args = parser.parse_args()
    error = False
    if args.map is None:
        print("Parameter '--map' is not set.")
        error = True

    if args.trajectory is None:
        print("Parameter '--trajectory' is not set.")
        error = True

    if args.output is None:
        print("Parameter '--output' is not set.")
        error = True
    if error:
        return

    print('[map ]:       ' + args.map)
    print('[trajectory]: ' + args.trajectory)
    print('[output]:     ' + args.output)

    trajectory = None
    with open(args.trajectory) as f:
        trajectory = yaml.safe_load(f)

    xs = []
    ys = []
    for p in trajectory:
        xs.append(p[0])
        ys.append(p[1])

    img = plt.imread(args.map)
    fig, ax = plt.subplots()


    ax.plot(xs, ys, '--', linewidth=5, color='firebrick')

    ax.imshow(img, cmap='gray', origin='lower')
    plt.show()

    return


if __name__ == "__main__":
    main(sys.argv)