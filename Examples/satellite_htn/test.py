"""
The test cases for the satellite domain.
-- Aneesh Chodisetty <aneeshch@umd.edu>, April 11, 2023
"""

import gtpyhop
import time

the_domain = gtpyhop.Domain(__package__)

from .actions import *
from .methods_updated import *


def main():
    """
    Running all the test cases related to the Satellite domain.
    """
    print(f'\n---------------------------------------------------------')
    print(f'Running the {the_domain.__name__} domain')

    gtpyhop.current_domain = the_domain

    gtpyhop.print_domain()

    print(f'\nTesting the first test case')

    state1 = gtpyhop.State('state1')
    state1.supports = {'instrument0' : ['image1'],
                       'instrument1' : ['image1', 'thermograph0'],
                       'instrument2' : ['thermograph0', 'image1'],
                       'instrument3' : ['thermograph0', 'image1'],
                       'instrument4' : ['thermograph0', 'image1'],
                       'instrument5' : ['image1', 'thermograph0'],
                       'instrument6' : ['thermograph0', 'image1'],
                       'instrument7' : ['image1', 'thermograph0'],
                       }
    
    state1.calibration_target = {'instrument0' : ['Star2'],
                                 'instrument1' : ['Star1'],
                                 'instrument2' : ['GroundStation0'],
                                 'instrument3' : ['Star1'],
                                 'instrument4' : ['GroundStation0'],
                                 'instrument5' : ['GroundStation0'],
                                 'instrument6' : ['GroundStation0'],
                                 'instrument7' : ['GroundStation0'],
                                 }
    
    state1.on_board = {'instrument0' : 'satellite0',
                       'instrument1' : 'satellite0',
                       'instrument2' : 'satellite0',
                       'instrument3' : 'satellite1',
                       'instrument4' : 'satellite1',
                       'instrument5' : 'satellite1',
                       'instrument6' : 'satellite2',
                       'instrument7' : 'satellite2',
                       }
    
    state1.power_avail = {'satellite0' : True,
                          'satellite1' : True,
                          'satellite2' : True,
                          }
    
    state1.pointing = {'satellite0' : 'Planet3',
                       'satellite1' : 'Planet4',
                       'satellite2' : 'GroundStation0',
                       }
    
    state1.data_capacity = {'satellite0' : 1000.0,
                            'satellite1' : 1000.0,
                            'satellite2' : 1000.0,
                            }
    
    state1.fuel = {'satellite0' : 140.0,
                   'satellite1' : 138.0,
                   'satellite2' : 159.0,
                   }
    
    state1.data = {('Planet3', 'thermograph0') : 73.0,
                   ('Planet4', 'thermograph0') : 158.0,
                   ('Planet3', 'image1') : 42.0,
                   ('Planet4', 'image1') : 149.0,
                   }

    state1.slew_time = {('Star2', 'GroundStation0') : 43.3,
                        ('GroundStation0', 'Star2') : 43.3,
                        ('Star2', 'Star1') : 27.62,
                        ('Star1', 'Star2') : 27.62,
                        ('Star1', 'GroundStation0') : 11.32,
                        ('GroundStation0', 'Star1') : 11.32,
                        ('Planet3', 'GroundStation0') : 26.41,
                        ('GroundStation0', 'Planet3') : 26.41,
                        ('Planet3', 'Star1') : 43.88,
                        ('Star1', 'Planet3') : 43.88,
                        ('Planet3', 'Star2') : 28.05,
                        ('Star2', 'Planet3') : 28.05,
                        ('Planet4', 'GroundStation0') : 64.75,
                        ('GroundStation0', 'Planet4') : 64.75,
                        ('Planet4', 'Star1') : 27.12,
                        ('Star1', 'Planet4') : 27.12,
                        ('Planet4', 'Star2') : 89.01,
                        ('Star2', 'Planet4') : 89.01,
                        ('Planet4', 'Planet3') : 29.47,
                        ('Planet3', 'Planet4') : 29.47,
                        }
    
    state1.data_stored = 0.0

    state1.fuel_used = 0.0

    state1.display('The initial State')

    goal1 = gtpyhop.Multigoal('goal1')

    goal1.pointing = {'satellite2' : 'Planet3'}
    goal1.have_image = {('Planet3', 'thermograph0') : True,
                        ('Planet4', 'thermograph0') : True,
                        }
    goal1.display('The goal state')

    gtpyhop.verbose = 1
    start_time = time.time()
    plan = gtpyhop.find_plan(state1, [('achieve_goal', goal1)])
    end_time = time.time()

    if plan:
        print(f'Found plan : {plan}\n\nwith plan length {len(plan)} in {end_time - start_time} seconds')

    else:
        print(f'PLAN FAILED')