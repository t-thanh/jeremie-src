#!/usr/bin/env python

import sys
import os

def generate_sdf(template_path, output_path, trajectory_file):
    with open(template_path, 'r') as template_file:
        sdf_content = template_file.read()

    sdf_content = sdf_content.replace('{trajectory_file}', trajectory_file)

    with open(output_path, 'w') as output_file:
        output_file.write(sdf_content)

if __name__ == "__main__":

    template_path = sys.argv[1]
    output_path = sys.argv[2]
    trajectory_file = sys.argv[3]

    generate_sdf(template_path, output_path, trajectory_file)

