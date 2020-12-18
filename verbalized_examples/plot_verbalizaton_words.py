#!/usr/bin/env python3
# coding=utf-8
#######################################################################################
# Copyright (c) 2020, Gerard Canal, Senka Krivić, Andrew Coles - King's College London
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#######################################################################################

# Author: Gerard Canal <gerard.canal@kcl.ac.uk>, King's College London
import sys

COUNT_WORDS = True  # If false, it will count number of sentences (aka \n)

sys.path.insert(0, '../src/')  # Dirty trick to import a script from a different folder
from VerbalizationSpace import *
import numpy as np
import matplotlib.pyplot as plt
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

if len(sys.argv) > 1:
    true = True
    false = False
    SAVE_FIGURE = eval(sys.argv[1])
else:
    SAVE_FIGURE = False

LIST_OF_FOLDERS = [
    'warehouse_domain_named_wp_problem', 'warehouse_domain_named_wp_problem',
    'warehouse_domain_named_wp_problem_twobots',
    'warehouse_domain_problem',
    'warehouse_domain_problem_twobots',
    #'warehouse_domain_survey_problem'
]
LIST_OF_FOLDERS.extend(['Rover_instance-' + str(i) for i in range(1, 20)])
LIST_OF_FOLDERS.extend(['CrewPlanning_instance-' + str(i) for i in range(1, 31)])
#LIST_OF_FOLDERS.extend(['CrewPlanning_instance-2'])

# RDDL
LIST_OF_FOLDERS.extend(['triangle_tireworld_mdp_triangle_tireworld_inst_mdp__' + str(i) for i in range(1, 11)])
#LIST_OF_FOLDERS = ['triangle_tireworld_mdp_triangle_tireworld_inst_mdp__' + str(i) for i in range(1, 11)]

LIST_OF_FOLDERS.extend(['jacket_dressing_jacket_dressing_instance'])
#LIST_OF_FOLDERS = ['jacket_dressing_jacket_dressing_instance']
LIST_OF_FOLDERS.extend(['jacket_dressing_jacket_dressing_instance'])
LIST_OF_FOLDERS.extend(['shoe_fitting_shoe_fitting_instance'])
LIST_OF_FOLDERS.extend(['two_arms_jacket_dressing_jacket_dressing_instance'])
LIST_OF_FOLDERS.extend(['feeding_feeding_instance'])


def avg(l):
    return sum(l) / float(len(l))


# Compute word numbers
AS = [[[] for _ in range(len(Specificity))] for _ in range(len(Abstraction))]
AE = [[[] for _ in range(len(Explanation))] for _ in range(len(Abstraction))]
ES = [[[] for _ in range(len(Specificity))] for _ in range(len(Explanation))]
for a in Abstraction:
    for s in Specificity:
        for e in Explanation:
            filename = 'narration_' + str(a) + '_' + str(s) + '_' + str(e)
            filename = filename.replace('.', '-') + '.txt'
            for p in LIST_OF_FOLDERS:
                with open(p + '/' + filename, 'r') as f:
                    text = f.read()
                n_words = len(text.split()) if COUNT_WORDS else len(text.split('\n'))
                AS[a - 1][s.value - 1].append(n_words)
                AE[a - 1][e - 1].append(n_words)
                ES[e - 1][s.value - 1].append(n_words)

# Plot word numbers
# PLOT AS
for a in Abstraction:
    for s in Specificity:
        AS[a - 1][s.value - 1] = avg(AS[a - 1][s.value - 1])

# setup the figure and axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

_x = np.arange(len(Abstraction))
_y = np.arange(len(Specificity))
_xx, _yy = np.meshgrid(_x, _y)
x, y = _xx.ravel(), _yy.ravel()

top = np.array([AS[i[0]][i[1]] for i in zip(x, y)]).flatten()
bottom = np.zeros_like(top)
width = depth = 1

ax.bar3d(x, y, bottom, width, depth, top, shade=True, edgecolor=(0, 0, 0, 0.75))
# ax.set_title('AS')

xticks = ['Level ' + str(a.value) for a in Abstraction]
plt.xticks(np.arange(0.5, len(Abstraction), 1), xticks)
ax.set_xticklabels(xticks, rotation=0, ha='right')
yticks = [str(s).replace('Specificity.', '').replace('_', ' ').title() for s in Specificity]
plt.yticks(np.arange(0.5, len(Specificity), 1), yticks)
ax.set_yticklabels(yticks, rotation=0, ha='left')
ax.tick_params(axis='x', which='major', pad=-5)
ax.tick_params(axis='y', which='major', pad=-5)
plt.xlabel('Abstraction')
plt.ylabel('Specificity', labelpad=25)
ax.set_zlabel('Number of words' if COUNT_WORDS else 'Number of sentences')

if not SAVE_FIGURE:
    plt.show()
else:
    plt.savefig('AS.svg', bbox_inches='tight')

# PLOT AE
for a in Abstraction:
    for e in Explanation:
        AE[a - 1][e - 1] = avg(AE[a - 1][e - 1])

# setup the figure and axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

_x = np.arange(len(Abstraction))
_y = np.arange(len(Explanation))
_xx, _yy = np.meshgrid(_x, _y)
x, y = _xx.ravel(), _yy.ravel()

top = np.array([AE[i[0]][i[1]] for i in zip(x, y)]).flatten()
bottom = np.zeros_like(top)
width = depth = 1

ax.bar3d(x, y, bottom, width, depth, top, shade=True, edgecolor=(0, 0, 0, 0.75))
# ax.set_title('AE')

xticks = ['Level ' + str(a.value) for a in Abstraction]
plt.xticks(np.arange(0.5, len(Abstraction), 1), xticks)
ax.set_xticklabels(xticks, rotation=0, ha='right')
yticks = ['Level ' + str(e.value) for e in Explanation]
plt.yticks(np.arange(0.5, len(Explanation), 1), yticks)
ax.set_yticklabels(yticks, rotation=0, ha='left')
ax.tick_params(axis='x', which='major', pad=-5)
ax.tick_params(axis='y', which='major', pad=-5)
plt.xlabel('Abstraction')
plt.ylabel('Explanation', labelpad=15)
ax.set_zlabel('Number of words' if COUNT_WORDS else 'Number of sentences')

if not SAVE_FIGURE:
    plt.show()
else:
    plt.savefig('AE.svg', bbox_inches='tight')

# PLOT ES
for e in Explanation:
    for s in Specificity:
        ES[e - 1][s.value - 1] = avg(ES[e - 1][s.value - 1])

# setup the figure and axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

_x = np.arange(len(Explanation))
_y = np.arange(len(Specificity))
_xx, _yy = np.meshgrid(_x, _y)
x, y = _xx.ravel(), _yy.ravel()

top = np.array([ES[i[0]][i[1]] for i in zip(np.flip(x), y)]).flatten()
bottom = np.zeros_like(top)
width = depth = 1

ax.bar3d(x, y, bottom, width, depth, top, shade=True, edgecolor=(0, 0, 0, 0.75))
# ax.set_title('ES')

xticks = ['Level ' + str(e.value) for e in reversed(Explanation)]
plt.xticks(np.arange(0.5, len(Explanation), 1), xticks)
ax.set_xticklabels(xticks, rotation=0, ha='right')
yticks = [str(s).replace('Specificity.', '').replace('_', ' ').title() for s in Specificity]
plt.yticks(np.arange(0.5, len(Specificity), 1), yticks)
ax.set_yticklabels(yticks, rotation=0, ha='left')
ax.tick_params(axis='x', which='major', pad=-5)
ax.tick_params(axis='y', which='major', pad=-5)
plt.xlabel('Explanation')
plt.ylabel('Specificity', labelpad=25, loc='center')
ax.yaxis.set_label_coords(50.5, 50.5)
ax.set_zlabel('Number of words' if COUNT_WORDS else 'Number of sentences')

if not SAVE_FIGURE:
    plt.show()
else:
    plt.savefig('ES.svg', bbox_inches='tight')


###########
print('Data:')
print('Abstraction-Specificity:')
for s in range(len(Specificity)-1, -1, -1):
    for a in range(0, len(Abstraction)):
        print("%.2f" % AS[a][s] + '\t', end = '')
    print()

print('\nAbstraction-Explanation:')
for e in range(len(Explanation)-1, -1, -1):
    for a in range(0, len(Abstraction)):
        print("%.2f" % AE[a][e] + '\t', end = '')
    print()

print('\nExplanation-Specificity:')
for s in range(len(Specificity)-1, -1, -1):
    for e in range(len(Explanation)-1, -1, -1):
        print("%.2f" % ES[e][s] + '\t', end = '')
    print()


