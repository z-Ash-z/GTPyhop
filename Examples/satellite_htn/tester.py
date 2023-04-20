"""
The tester file to read all the problems and produces results in a file.
--Aneesh Chodisetty <aneeshch@umd.edu>, April 20, 2023
"""

import gtpyhop
import time
import os

the_domain = gtpyhop.Domain(__package__)

from .actions import *
from .methods_updated import *

def main():

    print(f'\n---------------------------------------------------------')
    print(f'Running the {the_domain.__name__} domain')

    file_count = 0

    for x in os.listdir(os.path.join('.', 'problems/')):
        file_count += 1

    print(f'\n---------------------------')
    print(f'Tested {file_count} problems')