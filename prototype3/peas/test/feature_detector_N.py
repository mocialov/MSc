#!/usr/local/bin/python

# Python experiment setup for peas HyperNEAT implementation
# This setup is used by Webots to perform evolution of a detector for gesture recognition task

# Gesture recognition algorithm spawns this process, which is then used to evolve a detector that is evaluated by the gesture recognition algorithm
# The interaction between gesture recognition algorithm and peas implementation is done via fifo-queues.
# peas implementation is only provided with the fitness of each generated controller - peas does no evaluation itself

# Fitness function is defined in gesture recognition algorithm environment

# Note: all occurrences of 'N' should be changed to reflect the ID of the evolved detector, where every instance of this script will be evolving a different detector (in the report N = [1..10])

### IMPORTS ###
import sys, os
from functools import partial
from itertools import product

# Libs
import numpy as np
np.seterr(invalid='raise')

# Local
sys.path.append(os.path.join(os.path.split(__file__)[0],'..','..')) 
from peas.methods.neat import NEATPopulation, NEATGenotype
from peas.methods.evolution import SimplePopulation
from peas.methods.wavelets import WaveletGenotype, WaveletDeveloper
from peas.methods.hyperneat import HyperNEATDeveloper, Substrate
from peas.tasks.detector import DetectorTask

import time
import signal

# class that converts from genotype to phenotype: CPPN -> neural network
developer = None

# stats contains fitness evaluation and number of existing nodes of a genotype (CPPN). 'speed' and 'dist' are not used (these were used in the original example from peas framework)
stats = {'fitness':0.0, 'dist':0, 'speed':0, 'nodes':0};

# maximum number of nodes to be used in a resulting ANN
MAXIMUM_NODES = ...;

# minimum fitness that is allowed for this detector
MINIMUM_ALLOWED_FITNESS = ...;

def pass_ann(node_types, ann):
    """writes genotype to a fifo-queue (genes_N, where 'N' is the ID of the detector). node_types - list with a type for every node; ann - phenotype (ANN)"""
    fifo = open(os.path.join(os.path.dirname(__file__), '../../genes_N'), 'wb')
    fifo.write(' '.join(map(str, node_types)) + ' '+' '.join(map(str, ann)))  #str(len(ann)) + ' ' + ' '.join(map(str, ann)) -> len(ann) to include the size of the genotype (if dynamic)
    fifo.close()

def get_stats():
    """retrieves and returns fitness from fifo-queue that is then deleted"""
    # genes_fitness_N, where 'N' is the ID of the detector.
    while not os.path.exists(os.path.join(os.path.dirname(__file__), '../../genes_fitness_N')):
	    time.sleep(1)
	    continue
    fitness = open(os.path.join(os.path.dirname(__file__), '../../genes_fitness_N'))
    fitness_data = fitness.read()
    fitness.close()
    time.sleep(5)
    os.remove(os.path.join(os.path.dirname(__file__), '../../genes_fitness_N'))
    return fitness_data

def evaluate(individual, task, developer):
    """"evaluate function is called for every individual in the population to evaluate individual's fitness"""
    
    # convert CPPN to ANN
    phenotype = developer.convert(individual)
    
    # list of types of generated nodes
    nodes_types_indexes = list()
    
    # CPPN does not include all the nodes that are used in the resulting ANN. Therefore, the algorithm must know the 'MAXIMUM_NODES' number of nodes that are needed
    rest = MAXIMUM_NODES - len((individual.get_network_data()[1])[1:]) # max nodes - nodes in CPPN
    rest_array = np.linspace(4., 4., rest) #assign all the rest nodes value 4 - sigmoid activation
    
    # populate 'nodes_types_indexes' list with the types of nodes, where all the rest nodes will be of sigmoid type 
    for idx, node_type in enumerate((individual.get_network_data()[1])[1:]):
        if (idx == len((individual.get_network_data()[1])[1:]) - 2):
        	nodes_types_indexes.extend(rest_array)
        nodes_types_indexes.append(float(['sin', 'bound', 'linear', 'gauss', 'sigmoid', 'abs'].index(node_type)))
    
    # write resulting ANN and node types to fifo-queue
    pass_ann(nodes_types_indexes, phenotype.get_connectivity_matrix()) #phenotype.get_connectivity_matrix()
    
    # get back the fitness of the genotype
    fitness = get_stats()
    
    # store fitness
    stats = {'fitness':float(fitness), 'dist':0, 'speed':0}  #dist and speed are ignored - they do not affect the algorithm
    if isinstance(individual, NEATGenotype):
        stats['nodes'] = len(individual.node_genes)
    elif isinstance(individual, WaveletGenotype):
        stats['nodes'] = sum(len(w) for w in individual.wavelets)
    
    sys.stdout.flush()
    
    return stats
    
def solve(individual, task, developer):
    """function decides whether individual is a solution or not based on the minimum allowed fitness. Individuals that are solutions will persist in the population over generations"""
    return individual.stats['fitness'] > MINIMUM_ALLOWED_FITNESS

def run(method, setup, generations=15, popsize=10):
    """main function that drives the evolution"""
    
    # the following 'task_kwds' and 'task' variables should be ignored as they come from an example from the framework - I had no time to remove them yet.
    task_kwds = dict(field='eight',
                         observation='eight_striped',
                         max_steps=3000,
                         friction_scale=0.1,
                         damping=0.9,
                         motor_torque=3,
                         check_coverage=True,
                         flush_each_step=False,
                         initial_pos=(17, 256, np.pi*0.5)) #TODO: remove
    task = DetectorTask(**task_kwds) #TODO: remove
	
    # Detector has a specific topology: input 21x21; output 1x0
    substrate = Substrate()
    substrate.add_nodes([(r, theta) for r in np.linspace(-10,10,21) for theta in np.linspace(-10, 10, 21)], 'input', is_input=True)
    substrate.add_nodes([(r, theta) for r in np.linspace(0,0,1) for theta in np.linspace(0, 0, 1)], 'output')
    
    substrate.add_connections('input', 'output', -1)

    # evolutionary parameters
    geno_kwds = dict(feedforward=True, 
                     inputs=441,
                     outputs=1,
                     max_depth=3,
                     max_nodes=MAXIMUM_NODES,
                     weight_range=(-3.0, 3.0), 
                     prob_add_conn=0.3, 
                     prob_add_node=0.1,
                     bias_as_node=False,
                     types=['sigmoid'])

    geno = lambda: NEATGenotype(**geno_kwds)

    pop = NEATPopulation(geno, popsize=popsize, target_species=8)

    # sigmoid activation is only used for the generated ANN, since 'hnn' package can has only sigmoid activations
    developer = HyperNEATDeveloper(substrate=substrate, 
                                   add_deltas=False,
                                   sandwich=False,
                                   node_type='sigmoid')

    results = pop.epoch(generations=generations,
                        evaluator=partial(evaluate, task=task, developer=developer),
                        solution=partial(solve, task=task, developer=developer), 
                        )

    # output best solutions from every generation into a file 'best_solution_N', where 'N' is the ID of the detector
    fitnesses = list()

    fifo = open(os.path.join(os.path.dirname(__file__), '../../best_solution_N'), 'a+')
    for champion in results['champions']:
        fitnesses.append(champion.stats['fitness'])
        phenotype = developer.convert(champion)
        
        # option to visualise the detector ANN. Use some sort of counter so that the images won't be overwritten
        #dir = os.path.dirname('../../visual_N.png')
        #if not os.path.exists(dir):
        #   os.makedirs(dir)
        #phenotype.visualize('../../visual_N.png', inputs=441, outputs=1)
        
        rest = MAXIMUM_NODES - len((champion.get_network_data()[1])[1:])
        rest_array = np.linspace(4., 4., rest)
        
        for idx, node_type in enumerate((champion.get_network_data()[1])[1:]):
        	if (idx == len((champion.get_network_data()[1])[1:]) - 2):
				nodes_types_indexes.extend(rest_array)
        	try:
				nodes_types_indexes.append(float(['sin', 'bound', 'linear', 'gauss', 'sigmoid', 'abs'].index(node_type)))
        	except NameError:
				print "not defined!"
        
        fifo.write('fitness: '+str(champion.stats['fitness'])+' || ' + ' '.join(map(str, node_types)) + ' '+' '.join(map(str, phenotype.get_connectivity_matrix()))+'\n')
    fifo.close()



    # Visualize evolution in a graph
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    plt.figure()
    x = range(len(results['champions']))
    y = np.asarray(fitnesses)
    xa = plt.gca().get_xaxis()
    xa.set_major_locator(MaxNLocator(integer=True))
    plt.plot(x, y)
    plt.axis('on')
    plt.savefig(os.path.join(os.getcwd(), '../../fitness_evolution_N'), bbox_inches='tight', pad_inches=0)
    plt.close()

    return results

if __name__ == '__main__':
    print 'running peas detector 'N' evolutionary process'
    parent = sys.argv[1] #parent process id
    resnhn = run('nhn', 'hard') #starting evolutionary process
    print 'Done'
    os.kill(parent, signal.SIGKILL)