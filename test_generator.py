import random
import argparse
from argparse import ArgumentParser as ap
import sys

'''
Layout: is_beacon, x_pos, y_pos
'''

if __name__ == "__main__":
	# Parse args
	parser = ap()
	parser.add_argument('nodecount', type=int)
	parser.add_argument('beaconcount', type=int)
	parser.add_argument('--min_x', type=float, default=-50.0)
	parser.add_argument('--min_y', type=float, default=-50.0)
	parser.add_argument('--max_x', type=float, default=50.0)
	parser.add_argument('--max_y', type=float, default=50.0)
	parser.add_argument('--seed', type=int)

	parser.add_argument('file', nargs='?', type=argparse.FileType('w'), default=sys.stdout)

	args = parser.parse_args()

	if args.seed != None:
		random.seed(args.seed)


	# generate nodes
	for _ in range(args.nodecount):
		x_pos = random.uniform(args.min_x, args.max_x)
		y_pos = random.uniform(args.min_y, args.max_y)
		args.file.write(f"0 {x_pos} {y_pos}\n")

	# generate beacons
	for _ in range(args.beaconcount):
		x_pos = random.uniform(args.min_x, args.max_x)
		y_pos = random.uniform(args.min_y, args.max_y)
		args.file.write(f"1 {x_pos} {y_pos}\n")
