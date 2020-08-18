#!/usr/env/python3
#######################################################################################
# Copyright (c) 2020, Gerard Canal, Senka Krivić, Andrew Coles, King's College London
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
import random

ACCEPTED_TYPES = [
    'verb',
    'subject',
    'prep',
    'direct-object',
    'indirect-object'
]


class DomainSemantics:
    def __init__(self, name):
        self._actions = {}
        self._name = name

    def add_action(self, a):
        self._actions[a.get_action_name()] = a

    def get_action(self, action_name):
        return self._actions[action_name]

    def __str__(self):
        s = 'Domain ' + self._name + "\n"
        for n, a in self._actions.items():
            s += str(a) + "---\n"
        return s


class ActionSemantics:
    def __init__(self, action_name, parameters=None, semantic=None):
        self._action_name = action_name
        self._params = parameters or []
        self._semantic = semantic or {}

    def get_verbs(self):
        return self._semantic['verb']

    def get_rnd_verb(self):
        return random.choice(self._semantic['verb'])

    def get_semantics(self, stype):
        return self._semantic[stype]

    def get_rnd_verb(self, stype):
        return random.choice(self._semantic[stype])

    def set_action_name(self, action_name):
        self._action_name = action_name

    def get_action_name(self):
        return self._action_name

    def add_param(self, var, param_type):
        self._params.append((var, param_type))

    def add_semantics(self, sem_type, value, prep_needed=False):
        if sem_type not in ACCEPTED_TYPES:
            raise ValueError("Type " + sem_type + " is not recognized.")
        if sem_type is 'prep':  # Only check for prep clauses. subjects, verbs and (in)direct objects are always needed
            value = (value, prep_needed)
        if sem_type in self._semantic:
            self._semantic[sem_type].append(value)
        else:
            self._semantic[sem_type] = [value]

    def __str__(self):
        s = "Action: " + self._action_name + "\n"
        s += "  Parameters:\n"
        for v, t in self._params:
            s += "  - " + v + " of type " + t + "\n"
        s += "  Semantics:\n"
        for t, v in self._semantic.items():
            s += "  - " + t + " is " + str(v) + "\n"
        return s

    __repr__ = __str__
