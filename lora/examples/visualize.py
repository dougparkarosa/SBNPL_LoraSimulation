
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import glob
import csv
import vis_support as vs
import argparse


def main():
    parser = argparse.ArgumentParser(description='Generate Experiment Plots')
    parser.add_argument('--image_type', default='pdf',
                        help='specify image file type (anything supported by matplotlib)')
    parser.add_argument("--combine", action='store_true',
                        help="combine all images in a single pdf")
    args = parser.parse_args()
    vs.visualize(args.image_type, args.combine, True)


if __name__ == "__main__":
    main()
