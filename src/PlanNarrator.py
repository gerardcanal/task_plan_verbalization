#!/usr/bin/env python3
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

import mlconjug3
import random
import re
from DomainParser import DomainParser, RegularExpressions
from ActionSemantics import ActionSemantics

# TODO:
# - MakeSentence(), using mlconjug
# - NarratePlan() - stub, inputs a plan returns narration -- need to check format
CORRECT_PERSONS = ['1s', '1p', '2s', '2p', '3s', '3p']


class PlanNarrator:
    def __init__(self, narrator_name=None, language='en'):
        self._conjugator = mlconjug3.Conjugator(language=language)
        self._narrator_name = narrator_name

    # Assumes IPC format
    def make_action_sentence(self, ground_action, ground_params, action_semantics, tense='future'):
        # Find person of the verb
        subject = action_semantics.get_rnd_semantics('subject')
        subj_params = re.findall(RegularExpressions.PARAM, subject)
        person = '3p' if len(subj_params) > 1 else '3s'
        action_params = action_semantics.get_params()
        if self._narrator_name:
            for i, (v, _) in enumerate(action_params):
                if self._narrator_name.lower() == ground_params[i].lower() and v in subj_params:
                    subject = subject.replace(v, 'I')
                    person = '1p' if len(subj_params) > 1 else '1s'
                    break

        # Create sentence with semantics
        # Default format: subject verb indirect-object direct-object preps
        sentence = subject + ' ' + \
            self.conjugate_verb(action_semantics.get_rnd_verb(), tense, person)
        if action_semantics.has_semantics('indirect-object'):
            sentence += ' ' + action_semantics.get_rnd_semantics('indirect-object')
        if action_semantics.has_semantics('direct-object'):
            sentence += ' ' + action_semantics.get_rnd_semantics('direct-object')
        if action_semantics.has_semantics('prep'):
            prep = action_semantics.get_semantics('prep')
            for p in prep:  # Todo: use importance
                sentence += ' ' + random.choice(p[0])

        # Substitute parameters
        for i, p in enumerate(action_params):
            sentence = re.sub('([ \t]?)\\' + p[0] + r'([ \t.:-\?]|$)', '\\1' + ground_params[i] + '\\2', sentence)

        return sentence.capitalize() + '.'

    def conjugate_verb(self, verb, tense, person) -> str:
        if person not in CORRECT_PERSONS:
            raise ValueError('Unknown person type ' + person)

        m = re.search(RegularExpressions.VERB_PREPOST, verb)
        pre = m.group(1) + " " if m.group(1) else ''
        verb = m.group(2)
        post = " " + m.group(3) if m.group(3) else ''
        conjugated = self._conjugator.conjugate(verb)
        if tense == 'present':
            # NB: MLConjug has the continuous person key as "1p 1p": https://github.com/SekouDiaoNlp/mlconjug3/issues/79
            be = self._conjugator.conjugate('be').conjug_info['indicative']['indicative present'][person]
            return be + " " + pre + \
                   conjugated.conjug_info['indicative']['indicative present continuous'][person + ' ' + person] + post
        elif tense == 'past':
            return pre + conjugated.conjug_info['indicative']['indicative past tense'][person] + post
        elif tense == 'future':
            if random.randint(0, 1):  # will
                return 'will ' + pre + verb + post
            else:  # Go to
                be = self._conjugator.conjugate('be').conjug_info['indicative']['indicative present'][person]
                return be + ' going to ' + pre + verb + post
        else:
            raise ValueError("Unknown tense " + tense)


if __name__ == "__main__":
    pn = PlanNarrator()
    DOMAIN_TEST = "/home/gerard/code/ROSPlan_ws/src/pddl_verbalization/domains/office_robot/domain.pddl"
    a = DomainParser(DOMAIN_TEST)
    dsemantics = a.parse()
    print(pn.make_action_sentence("goto_waypoint", "kenny kitchen living-room".split(' '), dsemantics['goto_waypoint']))
    print(pn.make_action_sentence("goto_waypoint", "kenny bathroom bedroom".split(' '), dsemantics['goto_waypoint']))
    pn = PlanNarrator('kenny')
    print(pn.make_action_sentence("goto_waypoint", "kenny kitchen living-room".split(' '), dsemantics['goto_waypoint'], 'past'))
    print(pn.make_action_sentence("goto_waypoint", "kenny bathroom bedroom".split(' '), dsemantics['goto_waypoint'], 'past'))

