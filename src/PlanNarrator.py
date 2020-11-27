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
from collections import deque
from PatternMatcher import PatternMatcher
from VerbalizationSpace import VerbalizationSpace, Abstraction, Locality, Specificity, Explanation

CORRECT_PERSONS = ['1s', '1p', '2s', '2p', '3s', '3p']
JUSTIFICATION_TEMPLATES = ['<MAIN_ACTION> because <SUBJECT> <JUSTIFICATION>', '<JUSTIFICATION> to <MAIN_ACTION>',
                           '<JUSTIFICATION> to be able to <MAIN_ACTION>',
                           '<JUSTIFICATION>, which <VERB=allow> <SUBJECT-OBJ> to <MAIN_ACTION>',
                           '<JUSTIFICATION> to then <MAIN_ACTION>',
                           '<JUSTIFICATION> so <SUBJECT> <VERB=can> <MAIN_ACTION>']

JUSTIFIES_LINKERS = ['to later', 'to later be able to', ', which <VERB=allow> <SUBJECT-OBJ> to later',
                     'so <SUBJECT> <VERB=can> later']
GOAL_LINKERS = ['to achieve the goal of', 'to reach the goal of', 'to fulfill the goal of']


class PlanNarrator:
    def __init__(self, narrator_name=None, language='en'):
        self._conjugator = mlconjug3.Conjugator(language=language)
        self._narrator_name = narrator_name
        self._current_step = -1
        self._verbalization_space_params = VerbalizationSpace(3, 1, 2, 4)  # Default parameters

    # Assumes IPC format
    def make_action_sentence_IPC(self, ground_action, ground_params, action_semantics, compressions=[], duration=0,
                                 tense='future'):
        # Find person of the verb
        if not action_semantics.has_semantics('subject'):
            raise KeyError("Action " + action_semantics.get_action_name() + " has no subject defined")
        subject = action_semantics.get_rnd_semantics('subject')
        subj_params = re.findall(RegularExpressions.PARAM, subject)
        person = '3p' if len(subj_params) > 1 or tense == 'indicative' else '3s'
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
        # sentence = subject + ' ' + self.conjugate_verb(action_semantics.get_rnd_verb(), tense, person)
        sentence = self.conjugate_verb(action_semantics.get_rnd_verb(), tense, person)
        if action_semantics.has_semantics('indirect-object'):
            sentence += ' ' + action_semantics.get_rnd_semantics('indirect-object')
        if action_semantics.has_semantics('direct-object'):
            sentence += ' ' + action_semantics.get_rnd_semantics('direct-object')
        if action_semantics.has_semantics('prep'):
            prep = action_semantics.get_semantics('prep')
            for p in prep:  # Todo: use importance!
                sentence += ' ' + random.choice(p[0])

        # Substitute parameters
        for i, p in enumerate(action_params):
            if type(ground_params[i]) is list:
                ground_params[i] = self.make_list_str(ground_params[i])
            sentence = re.sub('([ \t]?)\\' + p[0] + r'([ \t.:-\?]|$)', '\\1' + ground_params[i] + '\\2', sentence)
            subject = re.sub('([ \t]?)\\' + p[0] + r'([ \t.:-\?]|$)', '\\1' + ground_params[i] + '\\2', subject)

        # Add narrator in the non-subject parameters (subjects have already been dealt with)
        if self._narrator_name:
            sentence = re.sub(self._narrator_name, 'me', sentence, flags=re.IGNORECASE)

        duration = float(duration)
        duration_s = 'taking {} second'.format('{0:.2f}'.format(duration).rstrip('0').rstrip('.'))
        if duration > 1:
            duration_s += 's'
        if compressions:
            sentence += " (via " + self.make_list_str(compressions)
            if duration:
                sentence += ', ' + duration_s.format('{0:.2f}'.format(duration).rstrip('0').rstrip('.'))
            sentence += ')'
        elif duration:
            sentence += ' (' + duration_s + ')'

        return person, subject, sentence

    # Tense is the tense for the main action
    def make_action_sentence_from_script(self, ac_script, domain_semantics, compressions, tense="future"):
        get_verb = re.compile(r'<VERB=(\w+)>')  # To get verbs from linkers

        # Later future justifications in the script
        if ac_script.justifies:
            justifies = [(i, compressions.id_to_action_str(i)) for i in sorted(ac_script.justifies)]
            justifies_verb = [self.make_action_sentence_IPC(ja[1][0], ja[1][1:], domain_semantics[ja[1][0]],
                                                            compressions.get_compressed_params(i), ja[2], 'indicative')
                              for i, ja in sorted(justifies, key=lambda x: x[0])]
            justifies_subjects = {s[1] for s in justifies_verb}
            justifies_sentence = self.make_list_str([s[2] for s in justifies_verb]) if justifies_verb else ''
            justifies_linker = random.choice(JUSTIFIES_LINKERS)
            if ',' != justifies_linker[0]:
                justifies_linker = ' ' + justifies_linker
            person = justifies_verb[0][0]
            verb = get_verb.findall(justifies_linker)
            if verb:
                first_justifies = justifies[0][0]
                if compressions.is_compressed(justifies[0][0]):
                    first_justifies = sorted(compressions.get_ids_compressed_action(justifies[0][0]))[0]
                if self._current_step > first_justifies:
                    person = justifies_verb[0][0]
                    verb = self.conjugate_verb(verb[0], 'past', '1s')
                else:
                    verb = ('will ' if verb[0] != 'can' else '') + verb[0]
                justifies_linker = get_verb.sub(verb, justifies_linker)
            justifies_linker = justifies_linker.replace('<SUBJECT-OBJ>',
                                                        'me' if '1' in person else justifies_verb[0][1])
            justifies_linker = justifies_linker.replace('<SUBJECT>', justifies_verb[0][1])
            justifies_linker = justifies_linker.replace('canned',
                                                        'could')  # As mlconjug conjugates past of can as canned

        # Justifications in the script
        if ac_script.justifications:
            template_choice = random.randint(0, len(JUSTIFICATION_TEMPLATES) - 1) if tense != 'present' else 0
            justification_template = JUSTIFICATION_TEMPLATES[template_choice]
            jtense = 'past'
            if tense != 'present':
                jtense = tense
                tense = 'indicative' if template_choice != 0 else tense  # Template 0 must be conjugated as main action goes first
            justifications = [(i, compressions.id_to_action_str(i)) for i in ac_script.justifications]
            justifications_verb = [self.make_action_sentence_IPC(ja[1][0], ja[1][1:], domain_semantics[ja[1][0]],
                                                                 compressions.get_compressed_params(i), ja[2], jtense)
                                   for i, ja in sorted(justifications, key=lambda x: x[0])]
            if jtense == 'future':  # Keep subject
                justifications_sentence = self.make_list_str([justifications_verb[0][2]] +  # First without subject
                                                             [j[1] + ' ' + j[2] for j in justifications_verb[1:]])
            else:
                justifications_sentence = self.make_list_str([s[2] for s in justifications_verb])
            justifications_subjects = {s[1] for s in justifications_verb}
            # if ',' != justification_template[0]:
            #     justification_template = ' ' + justification_template
            verb = get_verb.findall(justification_template)
            person = justifications_verb[0][0]
            if verb:
                verb = self.conjugate_verb(verb[0], jtense, person) if jtense != 'future' else 'will ' + verb[0]
                justification_template = get_verb.sub(verb, justification_template)
            justification_template = justification_template.replace('<SUBJECT-OBJ>',
                                                                    'me' if '1' in person else justifications_verb[0][
                                                                        1])
            justification_template = justification_template.replace('<SUBJECT>', justifications_verb[0][1])
            justification_template = justification_template.replace('canned',
                                                                    'could')  # As mlconjug conjugates past of can as canned
            justification_template = justification_template.replace('will can', 'can')  # Workaround for mlconjug

        ## Main action in the script
        main_action = compressions.id_to_action_str(ac_script.action)
        main_action_verb = self.make_action_sentence_IPC(main_action[1][0], main_action[1][1:],
                                                         domain_semantics[main_action[1][0]],
                                                         compressions.get_compressed_params(ac_script.action),
                                                         main_action[2], tense)
        if ac_script.goal:
            if type(ac_script.goal) is not list:
                ac_script.goal = [ac_script.goal]
            goal_sentence = []
            for g in ac_script.goal:
                goal = g[0].split(' ')
                goal_sentence.append(self.make_predicate_sentence(goal[0], goal[1:], domain_semantics))
            goal = random.choice(GOAL_LINKERS) + ' ' + self.make_list_str(goal_sentence)

        # Sentence will be: justifications + main action + goals + justifies
        s = main_action_verb[1] + ' '  # Subject
        if ac_script.justifications:
            s += justification_template.replace('<JUSTIFICATION>', justifications_sentence).replace('<MAIN_ACTION>',
                                                                                                    main_action_verb[2])
        else:
            s += main_action_verb[2]

        # Add rest of sentence
        s += (' ' + goal if ac_script.goal else '') + \
             (justifies_linker + ' ' + justifies_sentence if ac_script.justifies else '')

        return self.capitalize_first(s) + '.'

    def make_predicate_sentence(self, predicate_name, predicate_ground_params, domain_semantics):
        try:
            predicate_semantics = domain_semantics.get_predicate(predicate_name)
        except KeyError:
            raise KeyError("Predicate " + predicate_name + " has no semantics defined")

        sentence = ''
        predicate_params = predicate_semantics.get_params()
        if predicate_semantics.has_semantics('subject'):
            subject = predicate_semantics.get_rnd_semantics('subject')
            subj_params = re.findall(RegularExpressions.PARAM, subject)
            if self._narrator_name:
                for i, (v, _) in enumerate(predicate_params):
                    found_narrator = any([self._narrator_name.lower() == x for x in predicate_ground_params[i]]) \
                        if type(predicate_ground_params[i]) is list else self._narrator_name.lower() == \
                                                                         predicate_ground_params[i].lower()
                    if found_narrator and v in subj_params:
                        subject = subject.replace(v, 'me')
                        break
            sentence = subject + ' '

        sentence += self.conjugate_verb(predicate_semantics.get_rnd_verb(), 'continuous',
                                        '1s')  # Person is not important here
        if predicate_semantics.has_semantics('direct-object'):
            sentence += ' ' + predicate_semantics.get_rnd_semantics('direct-object')
        if predicate_semantics.has_semantics('indirect-object'):
            sentence += ' ' + predicate_semantics.get_rnd_semantics('indirect-object')
        if predicate_semantics.has_semantics('prep'):
            prep = predicate_semantics.get_semantics('prep')
            for p in prep:  # Todo: use importance!
                sentence += ' ' + random.choice(p[0])

        # Substitute parameters
        predicate_params = predicate_semantics.get_params()
        for i, p in enumerate(predicate_params):
            if type(predicate_ground_params[i]) is list:
                predicate_ground_params[i] = self.make_list_str(predicate_ground_params[i])
            sentence = re.sub('([ \t]?)\\' + p[0] + r'([ \t.:-\?]|$)', '\\1' + predicate_ground_params[i] + '\\2',
                              sentence)

        return sentence

    @staticmethod
    def make_list_str(l):
        if not l:
            return ''
        n = len(l)
        f = "{}{}" if n < 2 else "{}, and {}" if n > 2 else "{} and {}"
        return f.format(', '.join(l[:-1]), l[-1])

    @staticmethod
    def capitalize_first(sentence):
        return sentence[0].upper() + sentence[1:]

    def conjugate_verb(self, verb, tense, person) -> str:
        if person not in CORRECT_PERSONS:
            raise ValueError('Unknown person type ' + person)

        m = re.search(RegularExpressions.VERB_PREPOST, verb)
        pre = m.group(1) + " " if m.group(1) else ''
        verb = m.group(2)
        post = " " + m.group(3) if m.group(3) else ''
        conjugated = self._conjugator.conjugate(verb)
        if tense == 'present':
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
        elif tense == 'indicative':
            return pre + conjugated.conjug_info['indicative']['indicative present'][person] + post
        elif tense == 'continuous':  # Present continuous without be in front
            return pre + conjugated.conjug_info['indicative']['indicative present continuous'][person] + post
        else:
            raise ValueError("Unknown tense " + tense)

    def create_verbalization_script(self, plan, operators, causal_chains, compressions):
        # Compress actions if needed:
        if self._verbalization_space_params.specificity == Specificity.SUMMARY:
            compressions.compress_plan()
        # Compute causality scripts
        causality_script = self.compute_causality_scripts(plan, operators, causal_chains)

        # Join scripts
        verbalization_script = deque()
        compress = True  # FIXME parametrize
        skipped_actions = [False] * len(plan)
        for i in range(len(causality_script) - 1, -1, -1):
            if skipped_actions[i]:
                continue
            s = causality_script[i]

            if self._verbalization_space_params.specificity == Specificity.GENERAL_PICTURE and not s.goal:
                continue

            # It may happen that after compression two actions that were not consecutive become consecutive (all
            # intermediate actions were compressed). If that's the case, A) skip it and add it as a justification to the
            # previous action. B) Remove the previous action from the justifies and go on.
            if verbalization_script and verbalization_script[0].action in s.justifies:
                # verbalization_script[0].justifications.add(s.action)
                # continue
                s.justifies.remove(verbalization_script[0].action)

            if s.goal:  # Goal achieving actions are always kept
                s.justifies.clear()  # Remove justifies for this action as it achieves a goal to avoid overcluttering
            else:
                if compress:
                    s.justifies = {compressions.get_compressed_id(j) for j in s.justifies if causality_script[j].goal
                                   and not skipped_actions[j]}
                else:
                    s.justifies = {j for j in s.justifies if causality_script[j].goal and not skipped_actions[j]}
            keep_justifications = []  # Justifications to keep
            for j in s.justifications:
                if not causality_script[j].goal:  # If justification achieves a goal, we'll not use it here
                    skipped_actions[j] = True
                    if compress:
                        j = compressions.get_compressed_id(j)
                        # If j has been compressed, we need to skip the original compressed actions as those will be
                        # a justification for i
                        if compressions.is_compressed(j):
                            for jj in compressions.get_ids_compressed_action(j):
                                skipped_actions[jj] = True if not causality_script[jj].goal else False
                    keep_justifications.append(j)
            s.justifications = set(keep_justifications)  # TODO hash compressions here?
            if compress and compressions.is_compressed(i):
                cid = compressions.get_compressed_id(i)
                s.action = cid
                if verbalization_script and verbalization_script[0].action in s.justifies:
                    # verbalization_script[0].justifications.add(s.action)  # See above for explanation
                    # continue
                    s.justifies.remove(verbalization_script[0].action)
                for k in compressions.get_ids_compressed_action(cid):
                    if k != i or skipped_actions[k]:
                        skipped_actions[k] = True
                        j = {compressions.get_compressed_id(j) for j in causality_script[k].justifications
                             if not compressions.is_compressed(j) and not skipped_actions[j]}
                        s.justifications = s.justifications.union(j)
                        j = {compressions.get_compressed_id(j) for j in causality_script[k].justifies
                             if not compressions.is_compressed(j) and not skipped_actions[j]}
                        s.justifies = s.justifies.union(j)
                        # TODO join goals in compressions?
                # Clear justifications (remove itself)
                s.justifications.discard(cid)
                s.justifies.discard(cid)
            verbalization_script.appendleft(s)
        return verbalization_script

    def compute_causality_scripts(self, plan, operators, causal_chains):
        # Traverse causal chains to start to write the script.
        causality_script = [ActionScript(n) for n in range(len(plan))]
        for c in causal_chains:
            # Add achieving action + goal
            action_id = c.achieving_action.action_id
            goal_params = c.goal.split(' ')[1:]  # TODO remove subjects from the list?
            causality_script[action_id].goal = c.goal, c.goal_value
            self.add_action_scripts_rec(c.achieving_action, goal_params, operators, plan, causality_script)
        return causality_script

    def add_action_scripts_rec(self, node, goal_params, operators, plan, verbalization_script):
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
            else:
                # In this case, the action is not consecutive so it will be used to justify a later action (so it'll be
                # written two times)
                verbalization_script[n.action_id].justifies.add(node.action_id)
            # Recursive call with this child. We will skip the justifies of the following actions in the chain
            self.add_action_scripts_rec(n, goal_params, operators, plan, verbalization_script)

    def set_current_step(self, current_step):
        self._current_step = current_step

    def set_verbalization_space(self, verbalization_space_params):
        self._verbalization_space_params = verbalization_space_params


# Helper class to store information on an action used in an plan script
class ActionScript:
    def __init__(self, main_action_id, goal=None):
        self.action = main_action_id
        self.justifications = set()  # This action is justified by the actions in the list
        self.justifies = set()  # This action justifies the actions in the list
        self.goal = goal


# Helper class to compute, store, and manage action compressions
class PlanCompressions:
    def __init__(self, plan, goal_achieving_actions, compress=False):
        self._compression_dic = {}  # Dictionary of action in plan -> compressed action id. This Id is len(plan)+index
        self._compressed_actions = []  # List of compressed action strings
        self._compressed_parameters = []  # List of parameters of the compressed actions (i.e. via points)
        self._compressed_ids = []  # Ids that generated the compressed action
        self._plan = plan
        self._goal_achieving_actions = goal_achieving_actions
        if compress:
            self.compress_plan()

    # Check if an action_id has been compressed into another action, or is a compressed action id
    def is_compressed(self, action_id):
        if action_id >= len(self._plan):
            return True
        return action_id in self._compression_dic

    # Returns an action string from an id. If the action_id is a codified compression action or an action that was
    # compressed, it returns the compressed action
    def compressed_id_to_action_str(self, action_id):
        try:
            if action_id < len(self._plan):
                action_id = self._compression_dic[action_id]
            return self._compressed_actions[action_id - len(self._plan)]
        except KeyError:
            return self._plan[action_id]

    # Return an action string from an id. If the action_id is a codified compression action, returns the compressed
    # action otherwise returns the original action in the plan (regardless of if it was compressed).
    def id_to_action_str(self, action_id):
        if action_id < len(self._plan):
            return self._plan[action_id]
        return self._compressed_actions[action_id - len(self._plan)]

    def get_compressed_id(self, action_id):
        try:
            return self._compression_dic[action_id]
        except KeyError:
            return action_id

    def get_ids_compressed_action(self, compressed_id):
        assert compressed_id >= len(self._plan)
        return self._compressed_ids[compressed_id - len(self._plan)]

    def get_compressed_params(self, action_id):
        if action_id < len(self._plan):
            return []
        return self._compressed_parameters[action_id - len(self._plan)]

    # Adds an action compression of action_ids
    def add_compression(self, compressed_actions_ids, action_compression, compressed_params):
        i = len(self._compressed_actions)  # Id of the new action
        self._compressed_actions.append(action_compression)
        self._compressed_parameters.append(compressed_params)
        self._compressed_ids.append(compressed_actions_ids)
        for a in compressed_actions_ids:
            self._compression_dic[a] = len(self._plan) + i
            # This way we have two spaces of ids. This class will handle this translations

    # Compresses two actions if the action is the same, there's only one free parameter, and they comply with some
    # patterns
    def compress_actions(self, action_a, action_b):
        if action_a[0] == action_b[0]:  # same action name
            pattern = PatternMatcher.get_param_pattern(action_a[1:], action_b[1:])
            intermediate = []  # Keeps the intermediate parameters
            if len(pattern) >= len(action_a) - 2:  # If only one free variable
                params = [None] * (len(action_a) - 1)
                for p in pattern:
                    if p[0] == p[1]:
                        params[p[0]] = action_a[p[0] + 1]  # Same parameter
                    else:  # if p[0] > p[1]:  # Parameter moves back, is reused and not needed
                        params[p[1]] = action_a[p[1] + 1]
                        params[p[0]] = action_b[p[0] + 1]
                        intermediate.append(action_a[p[0] + 1])
                # Case where all the params are different, then it's an action applied to a list of parameters
                for i, p in enumerate(params):
                    if not p:
                        if type(action_a[i + 1]) is list:
                            params[i] = action_a[i + 1] + [action_b[i + 1]]
                        elif type(action_b[i + 1]) is list:
                            params[i] = [action_a[i + 1]] + action_b[i + 1]
                        else:
                            params[i] = [action_a[i + 1], action_b[i + 1]]
                action_c = [action_a[0]] + params
                # TODO failure case?
                return action_c, intermediate
        return [], []

    # Computes the compressions of the whole plan
    def compress_plan(self):
        # Plan is (time, action, duration)
        curr_ids = [0]
        curr_action = self._plan[0]  # This will always be i-1 or the compressed action
        curr_intmd = []
        for i in range(1, len(self._plan)):
            # Time will be the one from the start action, duration the sum
            result, intmd = self.compress_actions(curr_action[1], self._plan[i][1])
            if i - 1 in self._goal_achieving_actions:
                result = False  # Force avoid compression in case of goal achieving action
            if result:  # Result stores the compressed action (if compression was made)
                curr_ids.append(i)
                curr_intmd.extend(intmd)
                # Format: ('0.000', ['goto_waypoint', 'robot_assistant', 'wp3', 'wp15'], '13.000')
                # Compressed action start time, duration of the two actions are added
                curr_action = curr_action[0], result, str(float(curr_action[2]) + float(self._plan[i][2]))
            else:  # No compression found
                if len(curr_ids) > 1:  # We have compressed some actions
                    self.add_compression(curr_ids, curr_action, curr_intmd)
                curr_ids = [i]
                curr_action = self._plan[i]  # This will always be i-1 or the compressed action
                curr_intmd = []
        # Check if last action was to be compressed outside the loop
        if len(curr_ids) > 1:  # We have compressed some actions
            self.add_compression(curr_ids, curr_action, curr_intmd)
