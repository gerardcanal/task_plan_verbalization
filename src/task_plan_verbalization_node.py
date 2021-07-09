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
import rospy
import sys
import re
import random
import os
import string
import rospkg
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from std_msgs.msg import String
from PlanNarrator import PlanNarrator, DomainParser, RegularExpressions, PlanCompressions
from task_plan_verbalization.srv import NarratePlan, NarratePlanResponse, SetVerbalizationParams, \
                                        SetVerbalizationParamsResponse, NarratePredicate, NarratePredicateResponse, \
                                        QuestionAnswer, QuestionAnswerResponse
from rosplan_knowledge_msgs.srv import GetAttributeService, GetDomainOperatorService, GetDomainOperatorDetailsService
from rosplan_dispatch_msgs.msg import EsterelPlan
from EsterelProcessing import EsterelProcessing
from VerbalizationSpace import VerbalizationSpace, Abstraction, Locality, Specificity, Explanation
from NLPTools import NLPTools

ESTEREL_TIMEOUT = 15  # seconds


class ROSPlanNarratorNode:
    def __init__(self):
        rospy.init_node("rosplan_narrator", sys.argv)

        self._get_goals = rospy.ServiceProxy("/rosplan_knowledge_base/state/goals", GetAttributeService)
        self._get_operators = rospy.ServiceProxy("/rosplan_knowledge_base/domain/operators", GetDomainOperatorService)
        self._get_operator_details = rospy.ServiceProxy("/rosplan_knowledge_base/domain/operator_details", GetDomainOperatorDetailsService)
        self._problem_gen = rospy.ServiceProxy("/rosplan_problem_interface/problem_generation_server", Empty)
        self._planner = rospy.ServiceProxy("/rosplan_planner_interface/planning_server", Empty)
        self._parse_plan = rospy.ServiceProxy("/rosplan_parsing_interface/parse_plan", Empty)
        self._verbalization_pub = rospy.Publisher("~plan_narration", String, queue_size=1)
        self._trigger_plan_srv = rospy.Service("~trigger_planning", Empty, self.trigger_plan_srv)
        self._narrate_plan_srv = rospy.Service("~narrate_plan", NarratePlan, self.narrate_plan_srv)
        self._narrate_predicate_srv = rospy.Service("~narrate_predicate", NarratePredicate, self.narrate_predicate_srv)
        self._set_narration_params = rospy.Service("~set_params", SetVerbalizationParams, self.set_params_srv)
        self._update_narration_srv = rospy.Service("~narrate_current_plan", Trigger, self.update_narration_srv)
        self._plan_received = False
        self._plan = None
        self._verbalization_space_params = VerbalizationSpace(3, 1, 2, 4)  # Default parameters

        self._narrator_name = rospy.get_param("~narrator_name", None)
        self._print_actions = rospy.get_param("~print_actions", False)
        self._narrator = PlanNarrator(self._narrator_name)

        self._domain_semantics = None
        self.parse_domain()
        self.operators = {}
        self.get_all_operators_info()
        self.nlptools = NLPTools()

        # Lastly subscribe to the plan after everything is loaded to prevent errors from plans already computed
        self._raw_plan_subs = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, self.raw_plan_cb)
        self._q_and_a_srv = rospy.Service("~question_plan", QuestionAnswer, self.question_plan_srv)


    def raw_plan_cb(self, msg):
        self._process_input_plan(msg.data)
        self._plan_received = True

    def main_loop(self):
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self._plan_received:
                # Todo get current step from dispatcher?
                current_step = 0
                narration = self.narrate_plan(current_step, random_step=True)
                self._verbalization_pub.publish(narration)
            r.sleep()

    def trigger_plan_srv(self, req):
        rospy.loginfo(rospy.get_name() + ": Generating problem and planning")
        try:
            self._problem_gen.call()
            self._planner.call()
            self._parse_plan.call()
        except rospy.ServiceException as e:
            rospy.logerr(rospy.get_name() + ": Service call failed: %s" % e)
        return EmptyResponse()

    def narrate_plan(self, current_step=-1, random_step=False):
        if not self._plan:
            self._plan_received = False
            rospy.loginfo(rospy.get_name() + ": No plan was found")
            return "No plan was found"

        self._narrator.set_verbalization_space(self._verbalization_space_params)

        if random_step:
            current_step = random.randint(0, len(self._plan)+1)
        self._narrator.set_current_step(current_step)

        verbalization_script = self._narrator.create_verbalization_script(self._plan, self.operators, self._domain_semantics,
                                                                          self._causal_chains, self._compressions)
        # Find current step in the verbalization script/sentence (to mark with an asterisk)
        if 0 <= current_step < len(self._plan):
            current_step = self._plan_to_verbalization_id(current_step, verbalization_script, self._compressions)
        narration = "Narrator is: " + self._narrator_name + '\n' if self._narrator_name else ""
        for i, ac_script in enumerate(verbalization_script):
            s = self._narrator.make_action_sentence_from_script(ac_script, self._domain_semantics, self._compressions)

            if self._print_actions:
                s = self.script_debug_str(ac_script, self._compressions) + ':\n ' + s
            if i == current_step:
                s = '* ' + s

            narration += s + "\n"

            if self._print_actions:
                s += '\n'  # Add extra newline

        self._plan_received = False
        rospy.loginfo(rospy.get_name() + ": Plan narration computed: \n\n" + narration + "\n")
        return narration

    def update_narration_srv(self, req):
        res = TriggerResponse()
        if not self._plan:
            rospy.logwarn(rospy.get_name() + ": No plan available.")
            res.success = False
            res.message = "No plan available."
            return res
        n = self.narrate_plan(random_step=True)
        return TriggerResponse(True, n)

    def narrate_plan_srv(self, req):
        if req.input_plan:
            self._process_input_plan(req.input_plan)
        elif not self._plan:
            rospy.logerr(rospy.get_name() + ": narrate_plan service: no plan supplied and no previous plan available to be verbalized")
        else:
            rospy.loginfo(rospy.get_name() + ": narrate_plan service: using previous plan as input_plan was empty")
        if req.seed > 0:
            random.seed(req.seed)
        else:
            random.seed()
        n = self.narrate_plan(current_step=req.current_step)
        return NarratePlanResponse(n)

    def narrate_predicate_srv(self, req):
        pred = req.grounded_predicate.replace(')', '').replace('(', '').split(' ')
        name = pred[0]
        params = pred[1:]
        return self._narrator.make_predicate_sentence(name, params, self._domain_semantics, tense='future', replace_narrator=False).capitalize()

    def set_params_srv(self, req):
        self._verbalization_space_params = VerbalizationSpace.from_params_srv(req)
        if self._plan:
            self.narrate_plan(random_step=True)
        return SetVerbalizationParamsResponse(True)

    def question_plan_srv(self, req):
        question, tense = self.nlptools.parse_question(req.question)
        pddl_action = self.nlptools.match_question_domain(question, self._domain_semantics)
        if not self._plan:
            rospy.loginfo(rospy.get_name() + ": No plan yet received")
            return "No plan yet received"

        rm = str.maketrans(dict.fromkeys(string.punctuation + string.whitespace))
        matches = []
        for ai, a in enumerate(self._plan):
            a = a[1]
            if len(a) != len(pddl_action):
                continue
            for i in range(len(pddl_action)):
                if pddl_action[i][0] != '?' and pddl_action[i].translate(rm) != a[i].translate(rm):
                    break
            else:
                matches.append((ai, a))
        # TODO check if only match, otherwise ask for specifics with potential options! -> HOW?

        # If only match: compute full verbalization script, create sentence with that? -> Take into account space?
        if len(matches) > 3:
            # Too many options to ask for
            return '', 'There were many possible options. Could you ask again providing more information?'
        elif len(matches) > 1:  # Ask providing alternatives
            args = []
            for i, arg in enumerate(pddl_action[1:]):
                if '?' in arg:
                    args.append((arg, [m[1][i+1] for m in matches]))
                # else:
                #     args.append(arg)
            # create question with alternatives
            subj = question['nsubj'] if question['nsubj'] != 'you' else 'I'
            verb = self._narrator.conjugate_verb(question['verb'], tense, '3s' if subj != 'I' else '1s')
            alternative_queston = 'Did you refer to when ' + question['nsubj'] + ' ' + verb
            _, _, sent = self._narrator.make_action_sentence_IPC(pddl_action[0], pddl_action[1:], self._domain_semantics, tense=tense)
            sent = sent[sent.find(' '):]  # remove verb
            for arg, values in args:
                v_str = self._narrator.make_list_str(values, 'or')
                sent = sent.replace(arg, v_str)
            alternative_queston += sent + '?'
            # alternatives = []
            # for a in zip(*args):
            #     alternatives.append('|'.join(a))
            return '', alternative_queston
        elif len(matches) == 1:
            causality_script = self._narrator.compute_causality_scripts(self._plan, self.operators, self._causal_chains)
            matched_script = causality_script[matches[0][0]]
            matched_script.immediate_justifications.clear()
            current_step = self._narrator.get_current_step()
            s = self._narrator.make_action_sentence_from_script(matched_script, self._domain_semantics, self._compressions, tense)
            return s, ''
        return 'I could not understand the referred action', ''  # FIXME return something that makes sense here

    def _process_input_plan(self, raw_plan):
        self._plan = re.findall(RegularExpressions.PLAN_ACTION, raw_plan)
        self._plan = [(p[0], p[1].split(' '), p[2]) for p in self._plan]  # Split actions and parameters

        goals = self.get_goals()
        try:
            plan_topic = '/rosplan_parsing_interface/complete_plan'
            esterel_plan = rospy.wait_for_message(plan_topic, EsterelPlan, ESTEREL_TIMEOUT)
            self._causal_chains = EsterelProcessing.find_causal_chains(self.operators, goals, self._plan, esterel_plan)
        except rospy.ROSException:
            rospy.logwarn(rospy.get_name() + ': Esterel plan not received in ' + str(ESTEREL_TIMEOUT) + ' seconds. ' +
                                             'Causality will not be checked.')
            self._causal_chains = []
        self._goal_achieving_actions = sorted([c.achieving_action.action_id for c in self._causal_chains])
        self._compressions = PlanCompressions(self._plan, self._goal_achieving_actions)  # Compute action compressions

    def parse_domain(self):
        domain_path = rospy.get_param("~domain_path")
        dp = DomainParser(domain_path)
        self._domain_semantics = dp.parse()

        # Load problem data
        problem_file = os.path.splitext(rospy.get_param('~problem_path', ''))
        if problem_file[0]:
            object_file = problem_file[0] + '_objects.yaml'
            self._domain_semantics.load_obj_data(object_file)

    def get_goals(self):
        goals = []
        try:
            rp_goals = self._get_goals.call()
            for ki in rp_goals.attributes:
                assert ki.knowledge_type == ki.FACT
                g = EsterelProcessing.ki_to_str(ki)
                goals.append(g)
        except rospy.ServiceException as e:
            rospy.logerr(rospy.get_name() + ": Service call failed: %s" % e)
        return goals

    def get_all_operators_info(self):
        operator_list = self._get_operators.call()
        for o in operator_list.operators:
            self.operators[o.name] = self._get_operator_details.call(o.name).op

    @staticmethod
    def script_debug_str(ac_script, compressions):
        PDDL_justifications = ''
        for i, x in enumerate(ac_script.immediate_justifications):
            if i > 0:
                PDDL_justifications += ' '
            if compressions.is_compressed(x):
                PDDL_justifications += '['
                for j, y in enumerate(compressions.get_ids_compressed_action(x)):
                    if j > 0:
                        PDDL_justifications += ' '
                    PDDL_justifications += '(' + ' '.join(compressions.id_to_action_str(y)[1]) + ')'
                PDDL_justifications += ']'
            else:
                PDDL_justifications += '(' + ' '.join(compressions.id_to_action_str(x)[1]) + ')'
        if ac_script.immediate_justifications:
            PDDL_justifications += ' -> '

        PDDL_justifies = ' -> ' if ac_script.deferred_justifications else ''
        for i, x in enumerate(ac_script.deferred_justifications):
            if i > 0:
                PDDL_justifies += ' '
            if compressions.is_compressed(x):
                PDDL_justifies += '['
                for j, y in enumerate(compressions.get_ids_compressed_action(x)):
                    if j > 0:
                        PDDL_justifies += ' '
                    PDDL_justifies += '(' + ' '.join(compressions.id_to_action_str(y)[1]) + ')'
                PDDL_justifies += ']'
            else:
                PDDL_justifies += '(' + ' '.join(compressions.id_to_action_str(x)[1]) + ')'

        PDDL_main_action = ''
        if compressions.is_compressed(ac_script.action):
            PDDL_main_action += '['
            for i, x in enumerate(compressions.get_ids_compressed_action(ac_script.action)):
                if i > 0:
                    PDDL_main_action += ' '
                PDDL_main_action += '(' + ' '.join(compressions.id_to_action_str(x)[1]) + ')'
            PDDL_main_action += ']'
        else:
            PDDL_main_action += '(' + ' '.join(compressions.id_to_action_str(ac_script.action)[1]) + ')'

        return '{' + PDDL_justifications + PDDL_main_action + PDDL_justifies + '}'

    def generate_all_verbalizations(self):
        rospy.wait_for_message('/rosplan_planner_interface/planner_output', String)
        problem_name = os.path.splitext(os.path.basename(rospy.get_param('~problem_path')))[0]
        results_path = rospkg.RosPack().get_path(self.get_package_name()) + '/verbalized_examples/' + \
                       self._domain_semantics.get_name() + '_' + problem_name
        if not os.path.exists(results_path):
            os.makedirs(results_path)

        # Print plan
        with open(results_path + '/' + 'plan.txt', 'w') as f:
            f.write(self._plan)

        for a in Abstraction:
            for s in Specificity:
                for e in Explanation:
                    self._verbalization_space_params = VerbalizationSpace(a, Locality.ALL, s, e)
                    narration = self.narrate_plan(random_step=False)
                    filename = 'narration_' + str(a) + '_' + str(s) + '_' + str(e)
                    filename = filename.replace('.', '-') + '.txt'
                    with open(results_path + '/' + filename, 'w') as f:
                        f.write(narration)

    @staticmethod
    def get_package_name():
        pkg_name = os.path.basename(os.path.dirname(os.path.realpath(__file__)))  # should work in every case
        if pkg_name in ['src', 'bin', 'python']:  # typical sub dirs
            pkg_name = os.path.basename(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
        return pkg_name

    def _plan_to_verbalization_id(self, current_step, verbalization_script, compressions):
        for i, vs in enumerate(verbalization_script):
            if vs.action == current_step or \
                    (compressions.is_compressed(vs.action) and
                     current_step in compressions.get_ids_compressed_action(vs.action)):
                return i
            for j in vs.immediate_justifications:
                if j == current_step or (compressions.is_compressed(j) and
                                         current_step in compressions.get_ids_compressed_action(j)):
                    return i
        return current_step


if __name__ == "__main__":
    node = ROSPlanNarratorNode()
    rospy.loginfo(rospy.get_name() + ": Ready.")
    if rospy.get_param("~evaluation", False):
        node.generate_all_verbalizations()
        rospy.signal_shutdown('All verbalizations have been completed')
    else:
        node.main_loop()
    rospy.spin()
