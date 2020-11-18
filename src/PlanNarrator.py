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

import mlconjug3
import random
import re
from DomainParser import DomainParser, RegularExpressions
from EsterelProcessing import EsterelProcessing

# TODO:
# - MakeSentence(), using mlconjug
# - NarratePlan() - stub, inputs a plan returns narration -- need to check format
from PatternMatcher import PatternMatcher

CORRECT_PERSONS = ['1s', '1p', '2s', '2p', '3s', '3p']


class PlanNarrator:
    def __init__(self, narrator_name=None, language='en'):
        self._conjugator = mlconjug3.Conjugator(language=language)
        self._narrator_name = narrator_name

    # Assumes IPC format
    def make_action_sentence(self, ground_action, ground_params, action_semantics, compressions=[], tense='future'):
        # Find person of the verb
        if not action_semantics.has_semantics('subject'):
            raise KeyError("Action " + action_semantics.get_action_name() + " has no subject defined")
        subject = action_semantics.get_rnd_semantics('subject')
        subj_params = re.findall(RegularExpressions.PARAM, subject)
        person = '3p' if len(subj_params) > 1 else '3s'
        action_params = action_semantics.get_params()
        if self._narrator_name:
            for i, (v, _) in enumerate(action_params):
                found_narrator = any([self._narrator_name.lower() == x for x in ground_params[i]]) \
                        if type(ground_params[i]) is list else self._narrator_name.lower() == ground_params[i].lower()
                if found_narrator and v in subj_params:  # FIXME check instead of I me when I am the object?
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
            if type(ground_params[i]) is list:
                ground_params[i] = self.make_list_str(ground_params[i])
            sentence = re.sub('([ \t]?)\\' + p[0] + r'([ \t.:-\?]|$)', '\\1' + ground_params[i] + '\\2', sentence)

        # Add narrator in the non-subject parameters (subjects have already been dealt with)
        if self._narrator_name:
            sentence = re.sub(self._narrator_name, 'me', sentence, flags=re.IGNORECASE)

        if compressions:
            sentence += " (via " + self.make_list_str(compressions) + ')'

        return sentence.capitalize() + '.'

    @staticmethod
    def make_list_str(l):
        n = len(l)
        f = "{}{}" if n < 2 else "{}, and {}" if n > 2 else "{} and {}"
        return f.format(', '.join(l[:-1]), l[-1])

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
                   conjugated.conjug_info['indicative']['indicative present continuous'][person] + post
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

    # Compresses two actions if the action is the same, there's only one free parameter, and they comply with some
    # patterns
    def compress_actions(self, action_a, action_b):
        if action_a[0] == action_b[0]:  # same action name
            pattern = PatternMatcher.get_param_pattern(action_a[1:], action_b[1:])
            intermediate = []  # Keeps the intermediate parameters
            if len(pattern) >= len(action_a)-2:  # If only one free variable
                params = [None]*(len(action_a)-1)
                for p in pattern:
                    if p[0] == p[1]:
                        params[p[0]] = action_a[p[0]+1]  # Same parameter
                    else:  # if p[0] > p[1]:  # Parameter moves back, is reused and not needed
                        params[p[1]] = action_a[p[1]+1]
                        params[p[0]] = action_b[p[0]+1]
                        intermediate.append(action_a[p[0]+1])
                # Case where all the params are different, then it's an action applied to a list of parameters
                for i, p in enumerate(params):
                    if not p:
                        if type(action_a[i+1]) is list:
                            params[i] = action_a[i + 1] + [action_b[i + 1]]
                        elif type(action_b[i+1]) is list:
                            params[i] = [action_a[i + 1]] + action_b[i + 1]
                        else:
                            params[i] = [action_a[i+1], action_b[i+1]]
                action_c = [action_a[0]] + params
                # TODO failure case?
                return action_c, intermediate
        return [], []

    def create_script(self, plan, operators, causal_chains, compressions=None):
        verbalization_script = [ ActionScript(n) for n in range(len(plan))]

        # #########33 FIXME test
        # c = causal_chains[0]
        # c._print()
        # action_id = c.achieving_action.action_id
        # goal_params = c.goal.split(' ')[1:]  # TODO remove subjects from the list
        # self.add_causal_action_scripts_rec(c.achieving_action, goal_params, operators, plan, verbalization_script,
        #                                    used_actions, check_jusifies=True)
        # verbalization_script[action_id].goal = c.goal, c.goal_value
        # #########333 end test

        # Traverse causal chains to start to write the script.
        for c in causal_chains:
            # Add achieving action + goal
            action_id = c.achieving_action.action_id
            goal_params = c.goal.split(' ')[1:]  # TODO remove subjects from the list
            self.add_causal_action_scripts_rec(c.achieving_action, goal_params, operators, plan, verbalization_script,
                                               check_jusifies=True)
            verbalization_script[action_id].goal = c.goal, c.goal_value
        return verbalization_script

    def add_causal_action_scripts_rec(self, node, goal_params, operators, plan, verbalization_script, check_jusifies=False):
        consecutive_id = node.action_id
        sorted_children = sorted(node.children, key=lambda x: x.action_id, reverse=True)
        for n in sorted_children:
            if consecutive_id - n.action_id == 1:
                # Actions are consecutive in the plan, add them as justifying the parent
                verbalization_script[node.action_id].justifications.add(n.action_id)
                consecutive_id = n.action_id
                # Add symmetrical justification, and a flag to skip this action as it's already used to justify another
                # one. This action will be verbalized nonetheless to justify a different action.
                verbalization_script[n.action_id].justifies.add(node.action_id)
                verbalization_script[n.action_id].skip = True
            else:
                # In this case, the action is not consecutive so it will be used to justify a later action (so it'll be
                # written two times)
                verbalization_script[n.action_id].justifies.add(node.action_id)
                # In order to avoid extremely cluttering the sentences, we will skip the action that justify other
                # actions when they are not causing a goal-achieving action (thus, check_justifies is only true in the
                # first call
                verbalization_script[n.action_id].skip = check_jusifies
            # Recursive call with this child
            self.add_causal_action_scripts_rec(n, goal_params, operators, plan, verbalization_script, False)
            # TODO should skip become False at any time?


# Helper class to store information on an action used in an plan script
class ActionScript:
    def __init__(self, main_action_id, goal=None):
        self.action = main_action_id
        self.justifications = set()  # This action is justified by the actions in the list
        self.justifies = set()  # This action justifies the actions in the list
        self.goal = goal
        self.skip = False

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

