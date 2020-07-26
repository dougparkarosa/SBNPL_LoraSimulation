# Visualize across various experiments.

import os
import vis_support as vs
import pprint
import argparse
import sys
import subprocess
import time
from multiprocessing import Pool

def summarize_exp(exp_path):
    print(exp_path)
    with os.scandir(exp_path) as entries:
        iter_length = len(list(entries))
        print(f"{iter_length}: items")
        sum_path = os.path.join(exp_path, 'Exp2Summary.csv')
        if(os.path.isfile(sum_path)):
            sum = vs.getSummary(sum_path)
            pprint.pprint(sum)


def summarize(root_path):
    with os.scandir(root_path) as entries:
        for entry in entries:
            if(entry.name in vs.exclude):
                continue

            if entry.is_dir():
                subdir = os.path.join(root_path, entry)
                with vs.cwd(subdir):
                    file = 'Exp2Summary.csv'
                    try:
                        sum = vs.getSummary(file)
                        pprint.pprint(sum)
                    except FileNotFoundError:
                        print(f"{file} Not found in {subdir}, skipping.") 

def check(root_path):
    expected_files = ['Exp2Summary.csv']
    entries= vs.sorted_subdirs(root_path)
    for entry in entries:
        print(entry.name)
        if(entry.name in vs.exclude):
            print(f"skipping {entry.name}")
            continue

        if entry.is_dir():
            subdir = os.path.join(root_path, entry)
            with vs.cwd(subdir):
                for file in expected_files:
                    if not os.path.exists(file):
                        print(f"{file} Not found in {subdir}, skipping.")


def recreate_args(argv):
    args_pass = ""
    if(len(argv) > 0):
        for i in range(1, len(argv)):
            args_pass += argv[i] + " "
    return args_pass

def do_vis_in_dir(subdir, args, run3DOnly=False ):
    with vs.cwd(subdir):
        if(run3DOnly):
            vs.visualize3D(args.image_type, 'nodes', args.combine)
        else:
            vs.visualize(args.image_type, args.combine)
    return(subdir)

def run_visualize(root_path, args, run3DOnly=False):
    force=True
    jobs = []
    with Pool(16) as p:

        entries = vs.sorted_subdirs(root_path)
        count = 0
        num_entries = len(entries)

        for entry in entries:
            if(entry.name in vs.exclude):
                continue

            count += 1
            #if(count % 10 == 0):
            vs.progress(f"starting {entry.name}", round(count/num_entries*100,2))


            if entry.is_dir():
                subdir = os.path.join(root_path, entry)
                if(force or vs.should_gen_plots(args.image_type)):
                    job = p.apply_async(do_vis_in_dir, (subdir, args, run3DOnly))
                    jobs.append(job)
        for job in jobs:
            print(f"Finished: {job.get()}")

def run_pickle(root_path):
    force = True
    entries = vs.sorted_subdirs(root_path)

    for entry in entries:
        if(entry.name in vs.exclude):
            continue

        if entry.is_dir():
            subdir = os.path.join(root_path, entry)
            with vs.cwd(subdir):
                if force or vs.should_pickle():
                    print(f"Pickle {subdir}")
                    vs.pickle_stats()
    vs.cross_pickle(root_path)


def main():
    parser = argparse.ArgumentParser(description='Generate Experiment Plots')
    parser.add_argument('--image_type', default='pgf',
                        help='specify image file type (anything supported by matplotlib)')
    parser.add_argument("--combine", action='store_true',
                        help="combine all images in a single pdf")
    subparsers = parser.add_subparsers(
        help='sub-command help', dest="subparser_name")
    subparsers.add_parser(
        'vis_dir', help='run per directory visualization')
    subparsers.add_parser(
        'vis_dir_3d', help='run per directory 3D visualization')

    subparsers.add_parser('summary', help='Summarize experiments')

    subparsers.add_parser(
        'cross_dir', help='run cross directory visualization')
    subparsers.add_parser(
        'pickle', help='Pickle the csv files')
    subparsers.add_parser(
        'makelinks', help='Generate folder with links to simulations that are somewhat sensible.')
    subparsers.add_parser(
        'check', help='Sanity check of experiment')
    cross_plot_parser = subparsers.add_parser('plot', help='Generate a cross experiment plot')
    cross_plot_parser.add_argument('--type', default='grid_nodes', help='Plot type to generate')
    subparsers.add_parser(
        'analyze', help='Analyze node locations.')

    args = parser.parse_args()
    print(args)
    print(f"subparser: {args.subparser_name}")
    #sub_args = cross_plot_parser.parse_args()
    #print(sub_args)

    #arg_pass = recreate_args(sys.argv)
    #print(arg_pass)

    start = time.perf_counter()

    if(args.subparser_name == 'cross_dir'):
        arg_pass = arg_pass.replace('cross_dir', '', 1)
        print(f"run_cross_dir({os.getcwd()}, {arg_pass})")
        print("cross_dir not implemented!")
    elif (args.subparser_name == 'vis_dir'):
        run_visualize(os.getcwd(), args)
    elif (args.subparser_name == 'vis_dir_3d'):
        run_visualize(os.getcwd(), args, True)
    elif (args.subparser_name == 'summary'):
        summarize(os.getcwd())
    elif(args.subparser_name == 'pickle'):
        run_pickle(os.getcwd())
    elif(args.subparser_name == 'check'):
        check(os.getcwd())
    elif(args.subparser_name == 'plot'):
        print('plot')
        vs.cross_plot(os.getcwd(), args.type)
    elif(args.subparser_name == 'makelinks'):
        vs.make_lookup_links(os.getcwd())
    elif(args.subparser_name == 'analyze'):
        vs.analyz_node_locs_all_sizes(os.getcwd())

    end = time.perf_counter()
    print(f"Elapsed time: {round(end-start,1)} seconds")

if __name__ == "__main__":
    main()
