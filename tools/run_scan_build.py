#!/usr/bin/python3
# Copyright 2020 Toyota Research Institute

import os
import sys

EXTRA_EXCLUDE_ARGS = " --exclude src/pybind11 --exclude src/proj4 --exclude /usr/include/eigen3 "

# Returns a set of dependencies by name of all the `packages-up-to` argument. When None is provided, all the packages are returned.
def get_package_dependencies_names(packages_up_to = None):
  stream = os.popen('colcon graph') if not packages_up_to else os.popen('colcon graph --packages-up-to ' + packages_up_to)

  colcon_graph = stream.read().splitlines()
  result = set()
  for line in colcon_graph:
    result.add(line[:line.find(' ')])
  return result

# Get the paths located in the `filepath` and save them in the `store` set.
def get_exclusion_paths_from_file(filepath, store):
  if(os.path.exists(filepath)):
    with open(filepath) as fp:
      for line in fp.readlines():
          store.add(line.replace('\n', ' '))

# Convert a set of paths in a `--exclude`s arguments of scan-build command.
def convert_in_exclude_argument(paths):
  final_string = ''
  for path in paths:
    final_string = final_string + " --exclude " + str(path)
  return final_string

def get_effective_key(args):
  class Key:
    def __init__(self, name):
      self.name = name
      self.counter = 0
      self.index = 0

  keys = [Key("--packages-select"), Key("--packages-up-to")]
  for i, potential_key in enumerate(keys):
    keys[i].counter = args.count(potential_key.name)
    if (keys[i].counter > 0):
      keys[i].index = args.index(potential_key.name)

  if (keys[0].counter == 0 and keys[1].counter == 0):
    return ''
  elif (keys[0].counter == keys[1].counter):
    #  When there are two keys present, it just cares about the first one as colcon does.
    return keys[0].name if keys[0].index < keys[1].index else keys[1].name
  else:
    return keys[0].name if keys[0].counter else keys[1].name

def is_colcon_option(arg):
  return True if (arg[0:2] == '--') else False

def get_packages_from_args(args):
  key = get_effective_key(args)
  if not key:
    return set()
  index_of_key = args.index(key)

  packages = set()
  indexer = 1
  while (len(args) > (index_of_key + indexer)) and (not is_colcon_option(args[index_of_key + indexer])):
    packages.add(args[index_of_key + indexer])
    indexer += 1
  return packages

def get_packages_to_get_excludes_file_of(packages):
  packages_to_get_exclude_file_of = set()
  if not packages:
    packages_to_get_exclude_file_of = packages_to_get_exclude_file_of.union(get_package_dependencies_names())
  else:
    for package_from_arg in packages:
      packages_to_get_exclude_file_of = packages_to_get_exclude_file_of.union(get_package_dependencies_names(package_from_arg))
  return packages_to_get_exclude_file_of


def generate_exclude_args(argv):
  packages_from_arg = get_packages_from_args(argv)
  packages_to_get_exclude_file_of = get_packages_to_get_excludes_file_of(packages_from_arg)

  exclusion_paths = set()
  for package in packages_to_get_exclude_file_of:
    package_path = os.popen('colcon list --packages-select ' + package + ' --paths-only')
    command = 'find ' + package_path.read().splitlines()[0] + ' -name scan_build.supp'
    find = os.popen(command)
    supp_filepaths = find.read().splitlines()
    for filepath in supp_filepaths:
      get_exclusion_paths_from_file(filepath, exclusion_paths)
  return convert_in_exclude_argument(exclusion_paths)

def main(argv):
  excludes_args = generate_exclude_args(argv)
  excludes_args += EXTRA_EXCLUDE_ARGS

  colcon_extra_build_args = " ".join(argv[1:]) if(len(argv) > 1) else ""
  cmd = "scan-build-8 --status-bugs --use-cc=clang --use-c++=clang++ {} colcon build {} ".format(excludes_args, colcon_extra_build_args)
  print("scan-build command...\n--> " + cmd)
  return_code = os.system(cmd)
  if return_code > 255 :
    sys.exit(1)
  else:
    sys.exit(0)

if __name__ == "__main__":
  main(sys.argv)
