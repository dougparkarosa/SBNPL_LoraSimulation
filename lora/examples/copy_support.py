# Copy support py files to experiment dir tree
import argparse
import os
import shutil


def copy_root_files(source, target):
    files = {'vis_cross.py', 'vis_support.py', 'visualize.py'}
    for file in files:
        f = os.path.join(source, file)
        shutil.copy(f, target)
        print(f"copy to: {target}/{file}")

def main():
    parser = argparse.ArgumentParser(description='Copy support files')
    parser.add_argument('source',
                        help='Location of support files')
    parser.add_argument('target',
                        help='Root of experiment directory')
    args = parser.parse_args()
    print(args)
    source = args.source
    target = args.target

    copy_root_files(source, target)


if __name__ == "__main__":
    main()