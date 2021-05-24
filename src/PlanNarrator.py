#!/usr/bin/env python3
# coding=utf-8
#######################################################################################
# Copyright (c) 2021, Gerard Canal, Senka Krivić, Andrew Coles - King's College London
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
import copy
from DomainParser import DomainParser, RegularExpressions
from collections import deque
from PatternMatcher import PatternMatcher
from VerbalizationSpace import VerbalizationSpace, Abstraction, Locality, Specificity, Explanation

CORRECT_PERSONS = ['1s', '1p', '2s', '2p', '3s', '3p']
IMMEDIATE_JUSTIFICATION_TEMPLATES = ['<MAIN_SUBJECT> <MAIN_ACTION> because <JUSTIFICATION>',
                                     '<JUSTIFICATION> to <MAIN_ACTION>', '<JUSTIFICATION> to be able to <MAIN_ACTION>',
                                     '<JUSTIFICATION>, which <VERB=allow> <MAIN_SUBJECT-OBJ> to <MAIN_ACTION>',
                                     '<JUSTIFICATION> to then <MAIN_ACTION>',
                                     '<JUSTIFICATION> so <MAIN_SUBJECT> <VERB=can> <MAIN_ACTION>']

DEFERRED_JUSTIFICATION_LINKERS = ['to later', 'to later be able to', ', which <VERB=allow> <SUBJECT-OBJ> to later',
                                   'so <SUBJECT> <VERB=can> later']
GOAL_LINKERS = ['to achieve the goal of', 'to reach the goal of', 'to fulfill the goal of']


class PlanNarrator:
    def __init__(self, narrator_name=None, language='en'):
        self._conjugator = mlconjug3.Conjugator(language=language)
        self._narrator_name = narrator_name.replace('_', ' ').title() if narrator_name else ''
        self._current_step = -1
        self._verbalization_space_params = VerbalizationSpace(3, 1, 2, 4)  # Default parameters
        self._subjects = []
        self._subj_plans = SubjectPlans()

    # Assumes IPC format. If ignore_importance is True, all prepositional clauses will be set. Used for deferred justifications
    def make_action_sentence_IPC(self, ground_action, ground_params, domain_semantics, compressions=[], duration=0,
                                 tense='future', ignore_importance=False):
        # Find person of the verb
        action_semantics = domain_semantics[ground_action]
        if not action_semantics.has_semantics('subject'):
            raise KeyError("Action " + action_semantics.get_action_name() + " has no subject defined")
        subject = action_semantics.get_rnd_semantics('subject')
        subj_params = re.findall(RegularExpressions.PARAM, subject)
        person = '3p' if len(subj_params) > 1 or tense == 'infinitive' else '3s'
        action_params = action_semantics.get_params()
        if self._narrator_name:
            for i, (v, _) in enumerate(action_params):
                found_narrator = any([self._narrator_name.lower() == x.replace('_', ' ').lower() for x in ground_params[i]]) \
                    if type(ground_params[i]) is list else self._narrator_name.lower() == ground_params[i].replace('_', ' ').lower()
                if v in subj_params:
                    if found_narrator:
                        if type(ground_params[i]) is list:
                            s_i = ground_params[i].index(self._narrator_name.lower())
                            ground_params_t = copy.deepcopy(ground_params[i])
                            ground_params_t.pop(s_i)
                            ground_params_t = [s.title() for s in ground_params_t]
                            ground_params_t.append('me')
                            subject = self.make_list_str(ground_params_t)
                            person = '1p'
                        else:
                            subject = subject.replace(v, 'I')
                            person = '1p' if len(subj_params) > 1 else '1s'
                        break
                    else:  # If multi subject and not narrator, set person accordingly
                        person = '3p' if type(ground_params[i]) is list else '3s'
            if subject == 'I':
                person = '1p' if len(subj_params) > 1 else '1s'

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
            for p in prep:
                if ignore_importance or (self._verbalization_space_params.abstraction < Abstraction.LEV4 or \
                                         (self._verbalization_space_params.abstraction == Abstraction.LEV4 and p[1])):
                    # If p is important
                    sentence += ' ' + random.choice(p[0])

        # Substitute parameters
        for i, p in enumerate(action_params):
            ground_params[i] = self.replace_object_data(ground_params[i], domain_semantics)
            if type(ground_params[i]) is list:
                ground_params[i] = [s.title() if s.title() in self._subjects else s for s in ground_params[i]]
                ground_params[i] = self.make_list_str(ground_params[i])
            elif p[0] == subject or ground_params[i].title() in self._subjects:
                ground_params[i] = ground_params[i].title()

            sentence = re.sub('([ \t]?)\\' + p[0] + r'([ \t.:-\?]|$)', '\\1' + ground_params[i] + '\\2', sentence)
            subject = re.sub('([ \t]?)\\' + p[0] + r'([ \t.:-\?]|$)', '\\1' + ground_params[i] + '\\2', subject)

        # Add narrator in the non-subject parameters (subjects have already been dealt with)
        if self._narrator_name:
            if self._narrator_name == 'I':
                sentence = re.sub('I ', 'me', sentence)
            else:
                sentence = re.sub(self._narrator_name, 'me', sentence, flags=re.IGNORECASE)

        duration = float(duration) if self._verbalization_space_params.abstraction < Abstraction.LEV3 else 0
        duration_s = 'taking {} second'.format('{0:.2f}'.format(duration).rstrip('0').rstrip('.'))
        if duration != 1:
            duration_s += 's'
        if compressions and self._verbalization_space_params.abstraction < Abstraction.LEV4:
            sentence += " (via " + self.make_list_str(self.replace_object_data(compressions, domain_semantics))
            if duration:
                sentence += ', ' + duration_s.format('{0:.2f}'.format(duration).rstrip('0').rstrip('.'))
            sentence += ')'
        elif duration:
            sentence += ' (' + duration_s + ')'

        sentence = sentence.replace('_', ' ')
        subject = subject.replace('_', ' ')

        return person, subject, sentence

    # Tense is the tense for the main action. If set to None, tense will be taken from the current_step variable
    def make_action_sentence_from_script(self, ac_script, domain_semantics, compressions, tense=None):
        get_verb = re.compile(r'<VERB=(\w+)>')  # To get verbs from linkers

        # Check tense for main action
        compressed_ids = compressions.get_ids_compressed_action(ac_script.action) \
                         if compressions.is_compressed(ac_script.action) else None
        if not tense:
            if self._current_step == ac_script.action or (compressed_ids and self._current_step in compressed_ids):
                tense = 'present'
            elif (compressed_ids and self._current_step < compressed_ids[0]) or \
                 (not compressed_ids and self._current_step < ac_script.action):
                tense = 'future'
            else:
                tense = 'past'
        else:
            self._current_step = compressed_ids[0] if compressed_ids else ac_script.action

        if compressions.is_compressed(ac_script.action):
            ids = compressions.get_ids_compressed_action(ac_script.action)
            main_subj = set()
            for aid in ids:
                ms = self._subj_plans.id_to_subj(aid)
                if ms.lower() == self._narrator_name.lower():
                    ms = 'I'
                main_subj.add(ms)
        else:
            ms = self._subj_plans.id_to_subj(ac_script.action)
            main_subj = {'I' if ms.lower() == self._narrator_name.lower() else ms}


        # Later future (deferred) justifications in the script
        if ac_script.deferred_justifications:
            deferred_justifications = [(i, compressions.id_to_action_str(i)) for i in sorted(ac_script.deferred_justifications)]
            deferred_just_verb = [self.make_action_sentence_IPC(ja[1][0], ja[1][1:], domain_semantics,
                                                            compressions.get_compressed_params(i), ja[2], 'infinitive',
                                                            True)  # Ignore importance flag for later actions.
                              for i, ja in sorted(deferred_justifications, key=lambda x: float(x[1][0]))]
            deferred_justifications_subjects = {s[1] for s in deferred_just_verb}
            deferred_justifications_subjects = deferred_justifications_subjects.union(main_subj)
            if len(deferred_justifications_subjects) > 1:
                # If we have more than one subject, we need to adapt the linker based on the selected one, ensuring the subject is there
                deferred_justif_linker = random.choice([j for j in DEFERRED_JUSTIFICATION_LINKERS if 'SUBJECT' in j])
                subj_aux = re.search(r'<SUBJECT.*?> (?:<VERB=.*?>|to)', deferred_justif_linker).group(0)
                deferred_just_verb = [deferred_just_verb[0]] + [(p, subj, subj_aux + ' ' + v) for p, subj, v in deferred_just_verb[1:]]
            else:
                deferred_justif_linker = random.choice(DEFERRED_JUSTIFICATION_LINKERS)
            if ',' != deferred_justif_linker[0]:
                deferred_justif_linker = ' ' + deferred_justif_linker

            deferred_justif_sentence = deferred_justif_linker + ' ' + self.make_list_str([s[2] for s in deferred_just_verb]) if deferred_just_verb else ''
            person = deferred_just_verb[0][0]
            verb = get_verb.findall(deferred_justif_linker)
            if verb:
                first_deferred_just = deferred_justifications[0][0]
                if compressions.is_compressed(deferred_justifications[0][0]):
                    first_deferred_just = sorted(compressions.get_ids_compressed_action(deferred_justifications[0][0]))[0]
                if self._current_step > first_deferred_just:
                    person = deferred_just_verb[0][0]
                    verb = self.conjugate_verb(verb[0], 'past', '1s')
                else:
                    verb = ('will ' if verb[0] != 'can' else '') + verb[0]
                deferred_justif_sentence = get_verb.sub(verb, deferred_justif_sentence)
            for p, s, _ in deferred_just_verb:
                deferred_justif_sentence = deferred_justif_sentence.replace('<SUBJECT-OBJ>',
                                                                'me' if '1' in p else s, 1)
                deferred_justif_sentence = deferred_justif_sentence.replace('<SUBJECT>', s, 1)
            deferred_justif_sentence = deferred_justif_sentence.replace('canned',
                                                            'could')  # As mlconjug conjugates past of can as canned

        # Justifications in the script
        if ac_script.immediate_justifications:
            template_choice = random.randint(0, len(IMMEDIATE_JUSTIFICATION_TEMPLATES) - 1) if tense != 'present' else 0
            immediate_justification_template = IMMEDIATE_JUSTIFICATION_TEMPLATES[template_choice]
            # Check if current step is an immediate justification
            jtense = ['past']*len(ac_script.immediate_justifications)
            auxtense = tense  # Tense for auxiliar structures in the template
            if tense == 'future':
                jtense = []
                tense = 'infinitive' if template_choice != 0 else tense  # Template 0 must be conjugated as main action goes first
                for j in ac_script.immediate_justifications:
                    j_compressed_ids = compressions.get_ids_compressed_action(j) if compressions.is_compressed(j) else None
                    if self._current_step == j or (j_compressed_ids and self._current_step in j_compressed_ids):
                        jtense.append('present')
                        tense = 'infinitive'
                    elif j < self._current_step or (j_compressed_ids and j_compressed_ids[-1] < self._current_step):
                        jtense.append('past')
                    else:
                        jtense.append('future')
            if tense == 'past':
                tense = 'infinitive' if template_choice != 0 else tense  # Template 0 must be conjugated as main action goes first
            immediate_justifications = [(i, compressions.id_to_action_str(i)) for i in ac_script.immediate_justifications]
            immediate_justifications_verb = [self.make_action_sentence_IPC(ja[1][0], ja[1][1:], domain_semantics,
                                                                 compressions.get_compressed_params(i), ja[2], jtense[k])
                                   for k, (i, ja) in sorted(enumerate(immediate_justifications), key=lambda x: float(x[1][1][0]))]

            immediate_justifications_subjects = {s[1] for s in immediate_justifications_verb}
            immediate_justifications_subjects = immediate_justifications_subjects.union(main_subj)
            if 'present' in jtense or (len(immediate_justifications_subjects) > 1 and 'MAIN_SUBJECT' not in immediate_justification_template):
                # We need a template with MAIN_SUBJECT as we have multiple subjects. Skip 0 as it's the weird tense one
                immediate_justification_template = random.choice([j for j in IMMEDIATE_JUSTIFICATION_TEMPLATES[1:] if 'MAIN_SUBJECT' in j])
            if 'present' in jtense or 'future' in jtense or len(immediate_justifications_subjects) > 1:  # Keep subject always for future or multiple subj
                immediate_justifications_sentence = self.make_list_str([j[1] + ' ' + j[2] for j in immediate_justifications_verb])
            else:
                 immediate_justifications_sentence = immediate_justifications_verb[0][1] + ' '  # Subject + rest of sentence without subj
                 immediate_justifications_sentence += self.make_list_str([s[2] for s in immediate_justifications_verb])

            verb = get_verb.findall(immediate_justification_template)
            person = immediate_justifications_verb[0][0]
            if verb:
                verb = self.conjugate_verb(verb[0], auxtense, person) if auxtense != 'future' else 'will ' + verb[0]
                immediate_justification_template = get_verb.sub(verb, immediate_justification_template)
            immediate_justification_template = immediate_justification_template.replace('canned',
                                                                    'could')  # As mlconjug conjugates past of can as canned
            immediate_justification_template = immediate_justification_template.replace('will can', 'can')  # Workaround for mlconjug

        ## Main action in the script
        main_action = compressions.id_to_action_str(ac_script.action)
        main_action_verb = self.make_action_sentence_IPC(main_action[1][0], main_action[1][1:],
                                                         domain_semantics,
                                                         compressions.get_compressed_params(ac_script.action),
                                                         main_action[2], tense)
        if ac_script.goal:
            if type(ac_script.goal) is not list:
                ac_script.goal = [ac_script.goal]
            goal_sentence = []
            for g in ac_script.goal:
                goal = g[0].split(' ')
                goal_sentence.append(self.make_predicate_sentence(goal[0], goal[1:], domain_semantics, g[1]))
            goal = random.choice(GOAL_LINKERS) + ' ' + self.make_list_str(goal_sentence)

        # Sentence will be: immediate_justifications + main action + goals + deferred_justifications
        if ac_script.immediate_justifications:
            s = immediate_justification_template.replace('<JUSTIFICATION>', immediate_justifications_sentence)
            s = s.replace('<MAIN_ACTION>', main_action_verb[2])
            s = s.replace('<MAIN_SUBJECT-OBJ>', 'me' if '1' in main_action_verb[0] else main_action_verb[1])
            s = s.replace('<MAIN_SUBJECT>', main_action_verb[1])
        else:
            s = main_action_verb[1] + ' ' + main_action_verb[2]  # Subject + verb

        # Add rest of sentence
        s += (' ' + goal if ac_script.goal else '') + \
             (deferred_justif_sentence if ac_script.deferred_justifications else '')

        return self.capitalize_first(s) + '.'

    def make_predicate_sentence(self, predicate_name, predicate_ground_params, domain_semantics, sign=True, tense='continuous', replace_narrator=True):
        try:
            predicate_semantics = domain_semantics.get_predicate(predicate_name)
        except KeyError:
            raise KeyError("Predicate " + predicate_name + " has no semantics defined")

        sentence = ''
        predicate_params = predicate_semantics.get_params()
        person = '3s'
        if predicate_semantics.has_semantics('subject'):
            subject = predicate_semantics.get_rnd_semantics('subject')
            subj_params = re.findall(RegularExpressions.PARAM, subject)
            if self._narrator_name and replace_narrator:
                for i, (v, _) in enumerate(predicate_params):
                    found_narrator = any([self._narrator_name.lower() == x for x in predicate_ground_params[i]]) \
                        if type(predicate_ground_params[i]) is list else self._narrator_name.lower() == \
                                                                         predicate_ground_params[i].replace('_', ' ').lower()
                    if found_narrator and type(predicate_ground_params[i]) is list:
                        s_i = predicate_ground_params[i].index(self._narrator_name.lower())
                        predicate_ground_params[i].pop(s_i)
                        predicate_ground_params[i].append('me')
                        subject = self.make_list_str(predicate_ground_params[i])
                        person = '1p'
                    elif found_narrator and v in subj_params:
                        subject = subject.replace(v, 'me') if tense == 'continuous' else subject.replace(v, 'I')
                        person = '1s' if len(subj_params) == 1 else '1p'
                    break
                if subject == 'I' and tense == 'continuous':
                    subject = subject.replace('I', 'me')
                    person = '1s'
            sentence = subject + ' '

        if not sign:
            sentence += 'not '
        sentence += self.conjugate_verb(predicate_semantics.get_rnd_verb(), tense,
                                        person)  # Person is not important here
        if predicate_semantics.has_semantics('direct-object'):
            sentence += ' ' + predicate_semantics.get_rnd_semantics('direct-object')
        if predicate_semantics.has_semantics('indirect-object'):
            sentence += ' ' + predicate_semantics.get_rnd_semantics('indirect-object')
        if predicate_semantics.has_semantics('prep'):
            prep = predicate_semantics.get_semantics('prep')
            for p in prep:
                if self._verbalization_space_params.abstraction < Abstraction.LEV4 or \
                        (self._verbalization_space_params.abstraction == Abstraction.LEV4 and p[1]):  # If p is important
                    sentence += ' ' + random.choice(p[0])

        # Substitute parameters
        predicate_params = predicate_semantics.get_params()
        for i, p in enumerate(predicate_params):
            predicate_ground_params[i] = self.replace_object_data(predicate_ground_params[i], domain_semantics)
            if type(predicate_ground_params[i]) is list:
                predicate_ground_params[i] = [s.title() if s.title() in self._subjects else s for s in predicate_ground_params[i]]
                predicate_ground_params[i] = self.make_list_str(predicate_ground_params[i])
            elif predicate_ground_params[i].title() in self._subjects:
                predicate_ground_params[i] = predicate_ground_params[i].title()
            sentence = re.sub('([ \t]?)\\' + p[0] + r'([ \t.:-\?]|$)', '\\1' + predicate_ground_params[i] + '\\2',
                              sentence)
        sentence = sentence.replace('_', ' ')
        return sentence

    def replace_object_data(self, ground_param, domain_semantics):
        if self._verbalization_space_params.abstraction == Abstraction.LEV1:
            if type(ground_param) is list:
                for j, gp in enumerate(ground_param):
                    od = domain_semantics.get_object_data(ground_param[j])
                    if od:
                        ground_param[j] = od
            else:
                od = domain_semantics.get_object_data(ground_param)
                if od:
                    ground_param = od
        return ground_param

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
        elif tense == 'infinitive':
            return pre + conjugated.conjug_info['indicative']['indicative present']['1s'] + post
        elif tense == 'continuous':  # Present continuous without be in front
            return pre + conjugated.conjug_info['indicative']['indicative present continuous'][person] + post
        else:
            raise ValueError("Unknown tense " + tense)

    def create_verbalization_script(self, plan, operators, domain_semantics, causal_chains, compressions):
        self._subj_plans.split_plan_by_subjects(plan, domain_semantics, operators)
        self._subjects = self._subj_plans.get_subjects()

        # Compress actions if needed based on verbalization space
        compress = self._verbalization_space_params.specificity == Specificity.SUMMARY
        if compress:
            if self._verbalization_space_params.explanation < Explanation.LEV4:
                # If no goal will be verbalized, do not split compressions when goals are present
                compressions.disable_goal_checking()
            if self._subj_plans:
                for subj in self._subj_plans:
                    compressions.compress_plan(self._subj_plans, subj)
            else:
                compressions.compress_plan()

        # Compute causality scripts
        causality_script = self.compute_causality_scripts(plan, operators, causal_chains, self._subj_plans)

        # Join scripts
        verbalization_script = deque()
        skipped_actions = [False] * len(plan)
        for i in range(len(causality_script) - 1, -1, -1):
            locality_range = self._verbalization_space_params.locality.get_range()
            if self._verbalization_space_params.locality == Locality.RANGE and \
                    (i < locality_range.start or i > locality_range.stop):
                continue

            if skipped_actions[i]:
                continue
            s = copy.deepcopy(causality_script[i])  # Make copy as otherwise causality_script will be destroyed

            obj = self._verbalization_space_params.locality.get_object()
            if self._verbalization_space_params.locality == Locality.OBJECT and obj not in plan[s.action][1]:
                continue

            if self._verbalization_space_params.specificity == Specificity.GENERAL_PICTURE and not s.goal:
                continue

            # Check VerbalizationSpace for Explanations
            if self._verbalization_space_params.explanation < Explanation.LEV3:
                s.deferred_justifications.clear()
            if self._verbalization_space_params.explanation < Explanation.LEV2:
                s.immediate_justifications.clear()

            # It may happen that after compression two actions that were not consecutive become consecutive (all
            # intermediate actions were compressed). If that's the case, A) skip it and add it as an immediate
            # justification to the previous action. B) Remove the previous action from the deferred justifications and go on.
            if verbalization_script and verbalization_script[0].action in s.deferred_justifications:
                # verbalization_script[0].immediate_justifications.add(s.action)
                # continue
                s.deferred_justifications.remove(verbalization_script[0].action)

            if s.goal and self._verbalization_space_params.explanation == Explanation.LEV4:
                # Goal achieving actions are always kept. If level is >3 we print goals, otherwise we don't so there's
                # no need to clear deferred justifications. If it is 5 we can't clear justifications. Therefore this
                # only happens if level is 4
                s.deferred_justifications.clear()  # Remove deferred justif. for this action as it achieves a goal to avoid overcluttering
            else:
                if self._verbalization_space_params.explanation < Explanation.LEV5:
                    if compress:
                        s.deferred_justifications = {compressions.get_compressed_id(j) for j in s.deferred_justifications if causality_script[j].goal
                                                     and not skipped_actions[j]}
                    else:
                        s.deferred_justifications = {j for j in s.deferred_justifications if causality_script[j].goal and not skipped_actions[j]}
                else:  # self._verbalization_space_params.explanation == Explanation.LEV5
                    s.deferred_justifications = {compressions.get_compressed_id(j) for j in s.deferred_justifications}

            if self._verbalization_space_params.explanation < Explanation.LEV4:  # Level <4 does not include goals
                s.goal = None

            keep_justifications = []  # Immediate justifications to keep
            for j in s.immediate_justifications:
                if skipped_actions[j]:
                    continue
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
            s.immediate_justifications = set(keep_justifications)

            # If action i is compressed, update action id accordingly to the compression as well as immediate justifications
            # and deferred justifications
            if compress and compressions.is_compressed(i):
                cid = compressions.get_compressed_id(i)
                s.action = cid
                if verbalization_script and verbalization_script[0].action in s.deferred_justifications:
                    # If last added action (next in the plan) is in deferred justifications of this action, remove it
                    # and add it to immediate justifications
                    s.deferred_justifications.remove(verbalization_script[0].action)
                    verbalization_script[0].immediate_justifications.add(cid)
                    skipped_actions[i] = True
                    continue
                # Add the compressed ids accordingly to deferred and immediate justifications of this action
                for k in compressions.get_ids_compressed_action(cid):
                    if k != i or skipped_actions[k]:
                        skipped_actions[k] = True
                        if self._verbalization_space_params.explanation > Explanation.LEV1:
                            j = {compressions.get_compressed_id(j) for j in causality_script[k].immediate_justifications
                                 if not compressions.is_compressed(j) and not skipped_actions[j]}
                            s.immediate_justifications = s.immediate_justifications.union(j)
                        if self._verbalization_space_params.explanation > Explanation.LEV2:
                            j = {compressions.get_compressed_id(j) for j in causality_script[k].deferred_justifications
                                 if not compressions.is_compressed(j) and not skipped_actions[j]}
                            s.deferred_justifications = s.deferred_justifications.union(j)
                # Clear justifications (remove itself)
                s.immediate_justifications.discard(cid)
                s.deferred_justifications.discard(cid)
            verbalization_script.appendleft(s)
        if self._subj_plans:
            verbalization_script = sorted(verbalization_script, key=lambda x: self.script_sort_time_key(x, compressions))

        # Filter script. After sorting, some actions that were not consecutive may appear as consecutive, so we delete
        # the deferred justification from the script
        for i in range(len(verbalization_script)-1):
            a_next = verbalization_script[i+1].action
            a_i_script = verbalization_script[i]
            if a_next in a_i_script.deferred_justifications:
                a_i_script.deferred_justifications.remove(a_next)
        return verbalization_script

    def compute_causality_scripts(self, plan, operators, causal_chains, subj_plans):
        # Traverse causal chains to start to write the script.
        causality_script = [ActionScript(n) for n in range(len(plan))]
        for c in causal_chains:
            # Add achieving action + goal
            action_id = c.achieving_action.action_id
            goal_params = []
            if c.goal:
                goal_params = c.goal.split(' ')[1:]
                causality_script[action_id].goal = c.goal, c.goal_value
            self.add_action_scripts_rec(c.achieving_action, goal_params, operators, plan, causality_script, subj_plans)
        return causality_script

    def add_action_scripts_rec(self, node, goal_params, operators, plan, verbalization_script, subj_plans):
        consecutive_id = node.action_id
        sorted_children = sorted(node.children, key=lambda x: x.action_id, reverse=True)
        for n in sorted_children:
            if abs(consecutive_id - n.action_id) == 1 or subj_plans.are_subj_consecutive(consecutive_id, n.action_id):
                # Actions are consecutive in the plan, add them as justifying the parent
                verbalization_script[node.action_id].immediate_justifications.add(n.action_id)
                consecutive_id = n.action_id
                # Add symmetrical justification, and a flag to skip this action as it's already used to justify another
                # one. This action will be verbalized nonetheless to justify a different action.
                verbalization_script[n.action_id].deferred_justifications.add(node.action_id)
            else:
                # In this case, the action is not consecutive so it will be used to justify a later action (so it'll be
                # written two times)
                verbalization_script[n.action_id].deferred_justifications.add(node.action_id)
            # Recursive call with this child. We will skip the deferred justifications of the following actions in the chain
            self.add_action_scripts_rec(n, goal_params, operators, plan, verbalization_script, subj_plans)

    def set_current_step(self, current_step):
        self._current_step = current_step

    def set_verbalization_space(self, verbalization_space_params):
        self._verbalization_space_params = verbalization_space_params

    # Method to sort a verbalization script by smallest time
    def script_sort_time_key(self, script, compressions):
        return min([float(compressions.compressed_id_to_action_str(script.action)[0])] +
                   [float(compressions.compressed_id_to_action_str(a)[0]) for a in script.immediate_justifications])


# Helper class to store information on an action used in an plan script
class ActionScript:
    def __init__(self, main_action_id, goal=None):
        self.action = main_action_id
        self.immediate_justifications = set()  # This action is justified by the actions in the list - previously called 'justifications'
        self.deferred_justifications = set()  # This action justifies the actions in the list - previously called 'justifies'
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
        assert self.is_compressed(compressed_id)
        if compressed_id < len(self._plan):
            compressed_id = self._compression_dic[compressed_id]
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

    def disable_goal_checking(self):
        self._goal_achieving_actions = []

    @staticmethod
    # Compresses two actions if the action is the same, there's only one free parameter, and they comply with some
    # patterns
    def compress_actions(action_a, action_b):
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
                return action_c, intermediate
        return [], []

    # Computes the compressions of the whole plan
    def compress_plan(self, subj_plans=None, subj=None):
        # Plan is (time, action, duration)
        curr_ids = [0]
        curr_action = self._plan[0]  # This will always be i-1 or the compressed action
        curr_intmd = []
        last_i = 0  # last i value for the same subject
        compressing_subjects = False
        skip_subject_comp = []  # Ids to skip for subject compression
        i = 0
        #The while is equivalent to: for i in range(1, len(self._plan)):
        while i < len(self._plan)-1:
            i = i + 1  # i will start at 1
            # Time will be the one from the start action, duration the sum
            if i in self._compression_dic:
                continue
            result, intmd = self.compress_actions(curr_action[1], self._plan[i][1])
            compressed_subject = subj and i not in skip_subject_comp and \
                                 self.check_compressed_subject(subj, result, intmd) and \
                                 result and float(curr_action[0])-float(self._plan[i][0]) < 0.05
            if compressed_subject:
                compressing_subjects = True
            elif compressing_subjects:
                # If we are compressing subjects, check if there is not a better compression for this subject (i.e.
                # there was a compression that could be going on)
                result_aux, intmd_aux = self.compress_actions(self._plan[last_i][1], self._plan[i][1])
                if result_aux:
                    # If we found a better compression, backtrack to the last state for this subject and ignore subject
                    # compressions until we passed this point.
                    skip_subject_comp = range(last_i, i+1)
                    i = last_i
                    compressing_subjects = False
                    curr_ids = [i]
                    curr_action = self._plan[i]  # This will always be i-1 or the compressed action
                    curr_intmd = []
                    continue  # Go back to the last action from this subject and keep compressing from there
            i_from_subj = subj_plans.is_from_subj(i, subj) if subj_plans else True  # i is from subject subj
            if not compressed_subject and not i_from_subj:
                continue  # If action is not from current subject, and subject is not in the compression, skip
            if (not subj_plans or subj_plans.is_from_subj(last_i, subj)) and \
                    (last_i in self._goal_achieving_actions or i in self._goal_achieving_actions):
                result = False  # Force avoid compression in case of goal achieving action
            last_i = i if i_from_subj else last_i
            if result:  # Result stores the compressed action (if compression was made)
                curr_ids.append(i)
                curr_intmd.extend(intmd)
                # Format: ('0.000', ['goto_waypoint', 'robot_assistant', 'wp3', 'wp15'], '13.000')
                # Compressed action start time, duration of the two actions are added
                curr_action = curr_action[0], result, self.compute_compressed_duration(curr_action, self._plan[i])
            else:  # No compression found
                if len(curr_ids) > 1:  # We have compressed some actions
                    self.add_compression(curr_ids, curr_action, curr_intmd)
                curr_ids = [i]
                curr_action = self._plan[i]  # This will always be i-1 or the compressed action
                curr_intmd = []
                compressing_subjects = False
        # Check if last action was to be compressed outside the loop
        if len(curr_ids) > 1:  # We have compressed some actions
            self.add_compression(curr_ids, curr_action, curr_intmd)

    @staticmethod
    def check_compressed_subject(subj, result, intmd):
        if result:
            for p in result:
                if type(p) is list and subj.lower() in p:
                    return True
            for i in intmd:
                if i == subj:
                    return True
        return False

    @staticmethod
    # Computes the duration after compressing action a and b
    def compute_compressed_duration(a, b):
        a_start = float(a[0])
        b_start = float(b[0])
        assert a_start <= b_start  # Action a should start before action b
        a_duration = float(a[2])
        b_duration = float(b[2])
        if (b_start + b_duration) <= (a_start + a_duration):
            return a[2]  # If b ends before a, duration is that of a
        # Duration overlap is the duration of action a minus the difference of start times
        # If actions are not overlapping, overlap is 0
        overlap = max(a_duration - (b_start - a_start), 0)
        # Final duration is the sum of the durations of both actions minus the overlapped time
        duration = a_duration + b_duration - overlap
        return str(duration)


# Class to store plans by subject
class SubjectPlans:
    def __init__(self):
        self._plans = {}
        self._subj_to_plan = {}  # From subj action_id to plan action_id
        self._plan_to_subj = {}  # From plan action_id to subject action_id

    def split_plan_by_subjects(self, plan, domain_semantics, operators):
        get_var = re.compile(r'(\?[\w-]+|\\\d+)')
        action_subjects = {}  # Keeps the param id of the subject in each action
        for i, a in enumerate(plan):
            action_name = a[1][0]
            if action_name in action_subjects:
                subjects_idx = action_subjects[action_name]
            else:
                subject = ' '.join(domain_semantics.get_action(a[1][0]).get_semantics('subject'))
                subjects = re.findall(get_var, subject)  # List of variables representing subjects
                subjects_idx = []
                for idx, kv in enumerate(operators[a[1][0]].formula.typed_parameters):
                    if ('?'+kv.key in subjects) or ('\\'+str(idx+1) in subjects):
                        subjects_idx.append(idx)
                if not subjects_idx:
                    self.add_subject_action(subject, a, i)
                else:
                    action_subjects[action_name] = subjects_idx

            for idx in subjects_idx:
                subj_param = a[1][idx+1].replace('_', ' ').title()
                self.add_subject_action(subj_param, a, i)
        return self._plans

    def add_subject_action(self, subject, action, action_id):
        if subject not in self._plans:
            self._plans[subject] = []
        j = len(self._plans[subject])  # Action id for current subject
        self._plans[subject].append(action)
        self._plan_to_subj[action_id] = (subject, j)
        self._subj_to_plan[(subject, j)] = action_id

    def subj_to_id(self, aid, subj):
        return self._subj_to_plan[(subj, aid)]

    def id_to_subj(self, aid):
        if len(self._plans) > 0:
            return self._plan_to_subj[aid][0]
        raise KeyError("Empty subjects!")

    def is_from_subj(self, aid, subj):
        return self._plan_to_subj[aid][0] == subj

    def are_subj_consecutive(self, ida, idb):
        a = self._plan_to_subj[ida]
        b = self._plan_to_subj[idb]
        return a[0] == b[0] and abs(a[1]-b[1]) == 1

    def get_plans(self):
        return self._plans

    def get_subjects(self):
        return list(self._plans.keys())

    def __bool__(self):
        return len(self._plans) > 1  # More than 1 subject

    def __iter__(self):
        return self._plans.__iter__()

    def __getitem__(self, key):
        return self._plans[key]