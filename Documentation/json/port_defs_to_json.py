#!/usr/bin/env python3

"""Parse C files for lego-port class structures and generate json output"""

from __future__ import print_function

import argparse
import glob
import json
import os.path
import re
import subprocess
import sys

def error(message):
    print("Error: {0}".format(message), file=sys.stderr)
    exit(1)

def parse_structured_comment(lines, i, result):
    found = False
    # beginning of documentation comment - line starts with '/**'
    if re.match(r'\s*/\*\*', lines[i]):
        found = True
        i += 1
        # read all lines until '*/' - end of documentation comment
        while not re.match(r'\s*\*/', lines[i]):
            # look for lines that define a value - '@key: value'
            match = re.match(r'\s*\*\s*@(\w+):\s*(.*)', lines[i])
            if match:
                result[match.group(1)] = match.group(2)
                # TODO: handle multi-line definitions
            else:
                # everything else is considered 'notes'
                match = re.match(r'\s*\*\s?(.*)', lines[i])
                if match:
                    if not 'notes' in result:
                        result['notes'] = []
                    result['notes'].append(match.group(1))
            i += 1
        i += 1
    return found, i

def parse_port(lines, i):
    result = {}
    # first line of a port def should look like "[<index>] = {"
    match = re.match(r'\s*\[(\w+)\]\s*=\s*\{', lines[i])
    if not match:
        raise Exception('Missing port id assignment at line {0}'.format(i))
    i += 1
    mode_name = match.group(1)
    result[mode_name] = {}
    # parse the lines until we find a closing bracket
    while not re.match(r'\s*\},?', lines[i]):
        found, i = parse_structured_comment(lines, i, result[mode_name])
        if found:
            continue
        # ignore regular comments "/* ... */"
        if re.match(r'\s*/\*', lines[i]):
            while not re.match(r'.*\*/', lines[i]):
                i += 1
            i += 1
            continue
        match = re.match(r'\s*\.([\w\.]+)\s*=\s*(".*?"|\'.*?\'|-?\d+|\w+|\{),?', lines[i])
        if not match:
            raise Exception('missing closing "}}" at line {0}'.format(i))
        i += 1
        result[mode_name][match.group(1)] = match.group(2).strip('"')
        # opening bracket means we have a array definition. Array defs are
        # structured the same as port defs, so we just call this function
        # recursively
        if match.group(2) == '{':
            result[mode_name][match.group(1)] = []
            while not re.match(r'\s*\},?', lines[i]):
                port, i = parse_port(lines, i)
                for key in port:
                    port[key]['id'] = key
                    result[mode_name][match.group(1)].append(port[key])
            i += 1
    i += 1
    return result, i

def parse_file(source_dir, file_name):
    with open(os.path.join(source_dir, file_name)) as file:
        lines = file.read().split('\n')
    mode_list = []
    port = {}
    i = 0
    # search for the port definitions. Looks like:
    #    const struct <some_type> <some_name>_mode_info[] = {
    while i < len(lines):
        match = re.match(r'(?:\w+\s)+(\w+_mode_info)\[\w*\]\s=\s\{', lines[i])
        i += 1
        source_line = i
        if match:
            port['struct_name'] = match.group(1)
            while not re.match(r'\s*\};', lines[i]):
                try:
                    found, i = parse_structured_comment(lines, i, port)
                    if found:
                        continue
                    mode, i = parse_port(lines, i)
                    for key in mode:
                        mode[key]['id'] = key
                        mode_list.append(mode[key])
                except Exception as ex:
                    error(str(ex) + ' in file  "{0}"'.format(file_name))
            port['mode_info'] = mode_list
            port['num_modes'] = len(mode_list)
            # if the driver name was not specified, guess it from the struct name
            if 'name' not in port:
                port['name'] = port['struct_name'].replace('_mode_info', '').replace('_', '-')
            # if the module name was not specified, guess it from the file name
            if 'module' not in port:
                port['module'] = file_name.split('/')[-1].replace('_', '-').replace('.c', '')
            port['source_file'] = file_name.replace('../', '')
            port['source_line'] = source_line
    if not len(mode_list):
        error("did not find *_mode_info struct in {0}".format(file_name))
    return port

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_dir', type=str, help='source directory')
    parser.add_argument('out_file', type=str, help='output file')
    parser.add_argument('file_names', metavar='file', type=str, nargs='+',
            help='port definition file names')
    args = parser.parse_args()
    all_file_names = []
    for arg_file_name in args.file_names:
        file_name = os.path.join(args.source_dir, arg_file_name)
        file_name = os.path.abspath(file_name)
        if not os.path.isfile(file_name):
            error('File {0} does not exist.'.format(file_name))
        all_file_names.append(arg_file_name)
    ports = {}
    for file_name in all_file_names:
        port = parse_file(args.source_dir, file_name)
        ports[port['struct_name']] = port

    with open(args.out_file, 'w') as out_file:
        print(json.dumps(ports, sort_keys=True, indent=4), file=out_file)

if __name__ == '__main__':
    main()
