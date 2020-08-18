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

import mlconjug3
import random
import re
from DomainParser import DomainParser
from ActionSemantics import ActionSemantics

# TODO:
# - MakeSentence(), using mlconjug
# - NarratePlan() - stub, inputs a plan returns narration -- need to check format
CORRECT_PERSONS = ['1s', '1p', '2s', '2p', '3s', '3p']


class PlanNarrator:
    def __init__(self, language='en'):
        self._conjugator = mlconjug3.Conjugator(language=language)

    # Assumes IPC format
    def make_action_sentence(self, ground_action, action_semantics):
        ground_action = re.sub(r'\(|\)', '', ground_action)
        ground_params = ground_action.split(' ')[1:]
        # Todo: find person
        # todo: find tense
        # Create sentence with semantics
        # Default format: subject verb indirect-object direct-object preps
        sentence = action_semantics.get_rnd_semantics('subject') + ' ' + \
            self.conjugate_verb(action_semantics.get_rnd_verb(), 'present', '3s')
        if action_semantics.has_semantics('indirect-object'):
            sentence += ' ' + action_semantics.get_rnd_semantics('indirect_object')
        if action_semantics.has_semantics('direct-object'):
            sentence += ' ' + action_semantics.get_rnd_semantics('direct_object')
        if action_semantics.has_semantics('prep'):
            prep = action_semantics.get_semantics('prep')
            for p in prep:  # Todo: use importance
                sentence += ' ' + random.choice(p[0])

        # Substitute parameters
        for i, p in enumerate(action_semantics.get_params()):
            sentence = sentence.replace(p[0], ground_params[i])
        return sentence.capitalize() + '.'

    def conjugate_verb(self, verb, tense, person) -> str:
        if person not in CORRECT_PERSONS:
            raise ValueError('Unknown person type ' + person)
        conjugated = self._conjugator.conjugate(verb)
        if tense == 'present':
            # NB: MLConjug has the continuous person key as "1p 1p": https://github.com/SekouDiaoNlp/mlconjug3/issues/79
            be = self._conjugator.conjugate('be').conjug_info['indicative']['indicative present'][person]
            return be + " " + conjugated.conjug_info['indicative']['indicative present continuous'][
                person + ' ' + person]
        elif tense == 'past':
            return conjugated.conjug_info['indicative']['indicative past tense'][person]
        elif tense == 'future':
            if random.randint(0, 1):  # will
                return 'will ' + verb
            else:  # Go to
                be = self._conjugator.conjugate('be').conjug_info['indicative']['indicative present'][person]
                return be + ' going to ' + verb
        else:
            raise ValueError("Unknown tense " + tense)