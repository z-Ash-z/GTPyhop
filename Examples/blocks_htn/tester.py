"""
The tester scipt for all the block world problems
--Aneesh Chodisetty <aneeshch@umd.edu>, April 24, 2023 
"""

import gtpyhop
import time
import os

the_domain = gtpyhop.Domain(__package__)

from .methods import *
from .actions import *

def run_pddl(lines):
    init = False
    goal = False

    # Setting the first run flag in methods for this run.
    setFirstRun()

    # Creating the gtpyhop initial and goal states.
    initial_state = gtpyhop.State('initial_state')
    goal_state = gtpyhop.Multigoal('goal_state')

    states = [initial_state, goal_state]

    it = 0 # The iterator for tracking the states.

    for line in lines:
        read_words = line.replace(':', '').replace('(', '').replace(')', '').replace('\n', '').replace('\t', '').split(' ')

        # print(f'{read_words}\n')
        if 'init' in read_words:
            init = True
            continue
        elif 'goal' in read_words:
            goal = True
            it += 1
            continue
        elif 'metric' in read_words:
            break

        # Setting the states for both initial and goal.
        if init and not goal or goal:
            
            if 'on-table' in read_words:
                if 'pos' not in states[it].__dict__.keys():
                    states[it].pos = {}
                states[it].pos[read_words[-1]] = 'table'
            
            elif 'on' in read_words:
                if 'pos' not in states[it].__dict__.keys():
                    states[it].pos = {}
                states[it].pos[read_words[-2]] = read_words[-1]
            
            elif 'clear' in read_words:
                if 'clear' not in states[it].__dict__.keys():
                    states[it].clear = {}
                states[it].clear[read_words[-1]] = True

            elif 'handempty' in read_words:
                states[it].holding = {'hand':False}
        
        else:
            continue

    for key in initial_state.pos.keys():
        if key not in initial_state.clear.keys():
            initial_state.clear[key] = False

    
    # initial_state.display('The initial state')
    # goal_state.display('The Goal state')

    # Running the HTN planner.
    gtpyhop.verbose = 1

    start_time = time.time()
    plan = gtpyhop.find_plan(initial_state, [('achieve', goal_state)])
    end_time = time.time()

    node_count = getNodeCount()
    
    # Resetting the states for the next run.
    del initial_state
    del goal_state

    return plan, (end_time - start_time), node_count

def multi_test():
    """
    Running all the test cases for the BWD.
    """
    print(f'\n---------------------------------------------------------')
    print(f'Running the {the_domain.__name__} domain')

    gtpyhop.current_domain = the_domain

    gtpyhop.print_domain()

    # Getting the current file path.
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Creating the report file.
    report_file = open(f'{current_dir}/report.txt', 'w')

    # The path where the pddl files are places.
    home_dir = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
    problems_path = os.path.join(home_dir, 'Domains/BWD/problem')

    # Keeping track of the count of the problems.
    file_count = 0
    failed_count = 0

    # Saving the header file
    report_file.write(f'File Name, Plan Length, CPU Time, Nodes Expanded\n')

    # Getting the problem list
    problem_list = os.listdir(problems_path)
    problem_list.sort()

    for x in problem_list:
        if 'problem' in x:

            print(f'\n---------------------------------------------------------')
            print(f'Running {x}\n')
            file_count += 1

            # The pddl file name.
            problem_file_name = x.split('.')[0]
            
            # Reading the pddl file.
            pddl_file = open(problems_path + '/' + x, 'r')
            lines = pddl_file.readlines()

            # Run the read pddl file.
            plan, duration, node_count = run_pddl(lines)

            # Printing to the report file.
            if plan:
                report_file.write(f'{problem_file_name},\t{len(plan)},\t{duration}sec,\t{node_count}\n')
            else:
                report_file.write(f'{problem_file_name},\tFAILED,\t{duration}sec,\t{node_count}\n')
                failed_count += 1

            # Closing actions.
            print(f'Done running {x}')   
            pddl_file.close()
    
    report_file.close()
    print(f'\n---------------------------')
    print(f'Tested {file_count} problems')
    print(f'\n{failed_count} plans failed out of {file_count}')


