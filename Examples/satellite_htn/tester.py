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

def run_pddl(lines):
    init = False
    goal = False

    # Setting the first run flag in methods for this run
    setFirstRun()

    # Creating the gtpyhop initial and goal states.
    initial_state = gtpyhop.State('initial_state')
    goal_state = gtpyhop.Multigoal('goal_state')

    states = [initial_state, goal_state]

    it = 0 # The iterator for tracking the states

    for line in lines:
        read_words = line.replace(':', '').replace('(', '').replace(')', '').replace('\n', '').replace('\t', '').split(' ')
        
        if 'init' in read_words:
            init = True
            continue
        elif 'goal' in read_words:
            # initial_state.display('The initial state')
            goal = True
            it += 1
            continue
        elif 'metric' in read_words:
            # goal_state.display('The Goal state')
            break

        # Setting the states for both initial and goal.
        if init and not goal or goal:

            # Setting the support predicate.
            if 'supports' in read_words:
                if 'supports' not in states[it].__dict__.keys():
                    states[it].supports = {}
                if read_words[1] in states[it].supports.keys():
                    states[it].supports[read_words[1]].append(read_words[2])
                else:
                    states[it].supports[read_words[1]] = [read_words[2]]

            # Setting the calibration_target predicate.
            elif 'calibration_target' in read_words:
                if 'calibration_target' not in states[it].__dict__.keys():
                    states[it].calibration_target = {}
                if read_words[1] in states[it].calibration_target.keys():
                    states[it].calibration_target[read_words[1]].append(read_words[2])
                else:
                    states[it].calibration_target[read_words[1]] = [read_words[2]]
            
            # Setting the on_board predicate.
            elif 'on_board' in read_words:
                if 'on_board' not in states[it].__dict__.keys():
                    states[it].on_board = {}
                states[it].on_board[read_words[1]] = read_words[2]

            # Setting the power_avail predicate.
            elif 'power_avail' in read_words:
                if 'power_avail' not in states[it].__dict__.keys():
                    states[it].power_avail = {}
                states[it].power_avail[read_words[1]] = True
            
            # Setting the pointing predicate.
            elif 'pointing' in read_words:
                if 'pointing' not in states[it].__dict__.keys():
                    states[it].pointing = {}
                states[it].pointing[read_words[1]] = read_words[2]

            # Setting the data_capacity function.
            elif 'data_capacity' in read_words:
                if 'data_capacity' not in states[it].__dict__.keys():
                    states[it].data_capacity = {}
                states[it].data_capacity[read_words[2]] = float(read_words[3])
            
            # Setting the fuel function.
            elif 'fuel' in read_words:
                if 'fuel' not in states[it].__dict__.keys():
                    states[it].fuel = {}
                states[it].fuel[read_words[2]] = float(read_words[3])
            
            # Setting the data function.
            elif 'data' in read_words:
                if 'data' not in states[it].__dict__.keys():
                    states[it].data = {}
                states[it].data[(read_words[2], read_words[3])] = float(read_words[4])

            # Setting the slew_time function.
            elif 'slew_time' in read_words:
                if 'slew_time' not in states[it].__dict__.keys():
                    states[it].slew_time ={}
                states[it].slew_time[(read_words[2], read_words[3])] = float(read_words[4])

            # Setting the data-stored function.
            elif 'data-stored' in read_words:
                states[it].data_stored = 0.0
            
            # Setting the fuel-used function.
            elif 'fuel-used' in read_words:
                states[it].fuel_used = 0.0

            # Setting the have_image predicate.
            elif 'have_image' in read_words:
                if 'have_image' not in states[it].__dict__.keys():
                    states[it].have_image = {}
                states[it].have_image[(read_words[1], read_words[2])] = True

        else:
            # Lines we are not interested in.
            continue

    # Running the HTN planner.
    gtpyhop.verbose = 1

    start_time = time.time()
    plan = gtpyhop.find_plan(initial_state, [('achieve_goal', goal_state)])
    end_time = time.time()

    node_count = getNodeCount()
    
    # Resetting the states for the next run.
    del initial_state
    del goal_state

    return plan, (end_time - start_time), node_count
    


def multi_test():
    """
    Running all the test cases related to the Satellite domain.
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
    problems_path = os.path.join(home_dir, 'Domains/Satellite_Domain/problem')

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
    print(f'\nTested {file_count} problems')
    print(f'\n{failed_count} plans failed out of {file_count}')