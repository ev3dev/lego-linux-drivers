#!/usr/bin/env python3

"""Parse ev3dev *_motor_defs.c files and convert to markdown for online documentation."""

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
    if re.match(r'\s*/\*\*', lines[i]):
        found = True
        i += 1
        while not re.match(r'\s*\*/', lines[i]):
            match = re.match(r'\s*\*\s*@(\w+):\s*(.*)', lines[i])
            if match:
                result[match.group(1)] = match.group(2)
            else:
                match = re.match(r'\s*\*\s?(.*)', lines[i])
                if match:
                    if not 'notes' in result:
                        result['notes'] = []
                    result['notes'].append(match.group(1))
            i += 1
        i += 1
    return found, i

def parse_motor(lines, i, name_constants):
    result = {}
    # first line of a motor def should look like "[<index>] = {"
    match = re.match(r'\s*\[(\w+)\]\s*=\s*\{', lines[i])
    if not match:
        raise Exception('Missing motor id assignment at line {0}'.format(i))
    i += 1
    motor_name = match.group(1)
    result[motor_name] = {}
    # parse the lines until we find a closing bracket
    while not re.match(r'\s*\},?', lines[i]):
        found, i = parse_structured_comment(lines, i, result[motor_name])
        if found:
            continue
        # ignore regular comments "/* ... */"
        if re.match(r'\s*/\*', lines[i]):
            while not re.match('.*\*/', lines[i]):
                i += 1
            i += 1
            continue
        match = re.match(r'\s*\.([\w\.]+)\s*=\s*(".*?"|\'.*?\'|-?\d+|\w+|\{),?', lines[i])
        if not match:
            raise Exception('missing closing "}}" at line {0}'.format(i))
        i += 1
        result[motor_name][match.group(1)] = (name_constants.get(match.group(2)) or match.group(2)).strip('"')
        # opening bracket means we have a struct definition. Ignoring these for now.
        if match.group(2) == '{':
            result[motor_name][match.group(1)] = []
            while not re.match(r'\s*\},?', lines[i]):
                i += 1
            i += 1
    i += 1
    return result, i

def get_url_name(sensor):
    url_name = sensor['vendor_part_name']
    if 'vendor_name' in sensor:
        url_name = '{0} {1}'.format(sensor['vendor_name'], url_name)
    return re.sub(r'[^\w\.]+', '-', url_name.lower()).replace('--', '-').strip('-')

def parse_header(file_name, dict):
    """Parse the header file for #defines that have a string value and save the
    result in dict."""
    with open(file_name) as file:
        lines = file.read().split('\n')
    i = 0
    while i < len(lines):
        match = re.match(r'#define\s+(\w+)\s+"([\w-]+)"', lines[i])
        i += 1
        if match:
            dict[match.group(1)] = match.group(2)

def parse_file(kdir, file_name, name_constants):
    with open(file_name) as file:
        lines = file.read().split('\n')
    motor_list = []
    i = 0
    # search for the motor definitions. Looks like:
    #    const struct <some_type> <some_name>_motor_defs[] = {
    while i < len(lines):
        match = re.match(r'(?:\w+\s)+(\w+)_motor_defs\[\w*\]\s=\s\{', lines[i])
        i += 1
        source_line = i
        if match:
            motor_type = match.group(1).replace('_', '-')
            while not re.match(r'\s*\};', lines[i]):
                try:
                    motor, i = parse_motor(lines, i, name_constants)
                    for key in motor:
                        motor[key]['id'] = key
                        motor[key]['motor_type'] = motor_type
                        motor[key]['url_name'] = get_url_name(motor[key])
                        motor[key]['source_file'] = file_name.replace(kdir, '')
                        if motor[key]['source_file'][0] == '/':
                            motor[key]['source_file'] = motor[key]['source_file'][1:]
                        motor[key]['source_line'] = source_line
                        motor_list.append(motor[key])
                except Exception as ex:
                    error(str(ex) + ' in file  "{0}"'.format(file_name))
    if not len(motor_list):
        error("did not find *_motor_defs struct in {0}".format(file_name))
    return motor_list

def get_motor_sort_key(motor):
    """Returns vendor_name concatenated with vendor_part_number"""
    key = ""
    key += 'vendor_name' in motor and motor['vendor_name'] or ""
    key += 'vendor_part_number' in motor and motor['vendor_part_number'] or ""
    return key

def get_motor_page_title(motor):
    """Generates a page title out of the vendor information of a motor"""
    return ('vendor_name' in motor and '{0} '.format(motor['vendor_name']) \
            or "") + motor['vendor_part_name'] + ('vendor_part_number' in \
            motor and ' ({0})'.format(motor['vendor_part_number']) or "")

def main():
    # setup command line parameters
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=str, help='The source base directory')
    parser.add_argument('--out-file', type=str, help='The output file')
    parser.add_argument('--header-files', metavar='header', type=str, nargs='+',
            help='header file names')
    parser.add_argument('--source-files', metavar='file', type=str, nargs='+',
            help='motor definition file names')
    args = parser.parse_args()

    # expand list of files in --header-files arg using glob
    all_header_file_names = []
    for arg_file_name in args.header_files:
        arg_file_names = glob.glob(os.path.join(args.source_dir, arg_file_name))
        for file_name in arg_file_names:
            if not os.path.isfile(file_name):
                error('File {0} does not exist.'.format(file_name))
            all_header_file_names.append(file_name)

    # expand list of files in --source-files arg using glob
    source_file_names = []
    for arg_file_name in args.source_files:
        arg_file_names = glob.glob(os.path.join(args.source_dir, arg_file_name))
        for file_name in arg_file_names:
            if not os.path.isfile(file_name):
                error('File {0} does not exist.'.format(file_name))
            source_file_names.append(file_name)
            all_header_file_names.append(file_name)

    # expand list of files in --source-files arg using glob
    source_file_names = []
    for arg_file_name in args.source_files:
        arg_file_names = glob.glob(os.path.join(args.source_dir, arg_file_name))
        for file_name in arg_file_names:
            if not os.path.isfile(file_name):
                error('File {0} does not exist.'.format(file_name))
            source_file_names.append(file_name)

    # name constants will be used to substitute constant names for real string values
    name_constants = {}

    # parse header files to get all other name constants
    for file_name in all_header_file_names:
        parse_header(file_name, name_constants)

    # parse source file to get motor definitions
    motor_list = []
    for file_name in source_file_names:
        motor_list += parse_file(args.source_dir, file_name, name_constants)

    # sort by vendor_name, then by vendor_part_number
    motor_list.sort(key=lambda motor: get_motor_sort_key(motor))

    # generate json data file
    with open(args.out_file, 'w') as out_file:
        print(json.dumps(motor_list, sort_keys=True, indent=4), file=out_file)

if __name__ == '__main__':
    main()
