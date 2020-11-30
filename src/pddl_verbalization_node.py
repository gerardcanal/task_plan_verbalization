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
import rospy
import sys
import re
import random
import copy
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from std_msgs.msg import String
from PlanNarrator import PlanNarrator, DomainParser, RegularExpressions, PlanCompressions
from pddl_verbalization.srv import NarratePlan, NarratePlanResponse, SetVerbalizationParams, SetVerbalizationParamsResponse
from rosplan_knowledge_msgs.srv import GetAttributeService, GetDomainOperatorService, GetDomainOperatorDetailsService
from rosplan_dispatch_msgs.msg import EsterelPlan
from EsterelProcessing import EsterelProcessing
from VerbalizationSpace import VerbalizationSpace, Abstraction, Locality, Specificity, Explanation

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
        self._raw_plan_subs = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, self.raw_plan_cb)
        self._verbalization_pub = rospy.Publisher("~plan_narration", String, queue_size=1)
        self._trigger_plan_srv = rospy.Service("~trigger_planning", Empty, self.trigger_plan_srv)
        self._narrate_plan_srv = rospy.Service("~narrate_plan", NarratePlan, self.narrate_plan_srv)
        self._set_narration_params = rospy.Service("~set_params", SetVerbalizationParams, self.set_params_srv)
        self._update_narration_srv = rospy.Service("~narrate_current_plan", Trigger, self.update_narration_srv)
        self._plan_received = False
        self._plan = None
        self._verbalization_space_params = VerbalizationSpace(3, 1, 2, 4)  # Default parameters

        self._narrator_name = rospy.get_param("~narrator_name", None)
        self._narrator = PlanNarrator(self._narrator_name)

        self._domain_semantics = None
        self.parse_domain()
        self.operators = {}
        self.get_all_operators_info()

    def raw_plan_cb(self, msg):
        self._plan_received = True
        self._plan = msg.data

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
        plan = re.findall(RegularExpressions.PLAN_ACTION, self._plan)
        plan = [(p[0], p[1].split(' '), p[2]) for p in plan]  # Split actions and parameters

        self._narrator.set_verbalization_space(self._verbalization_space_params)

        goals = self.get_goals()
        try:
            plan_topic = '/rosplan_parsing_interface/complete_plan'
            esterel_plan = rospy.wait_for_message(plan_topic, EsterelPlan, ESTEREL_TIMEOUT)
        except rospy.ROSException:
            rospy.logwarn(rospy.get_name() + ': Esterel plan not received in ' + str(ESTEREL_TIMEOUT) + 'seconds. ' +
                                             'Causality will not be checked.')
        causal_chains = EsterelProcessing.find_causal_chains(self.operators, goals, plan, esterel_plan)
        goal_achieving_actions = sorted([c.achieving_action.action_id for c in causal_chains])
        compressions = PlanCompressions(plan, goal_achieving_actions)  # Compute action compressions
        verbalization_script = self._narrator.create_verbalization_script(plan, self.operators, causal_chains,
                                                                          compressions)

        if random_step:
            current_step = random.randint(0, len(verbalization_script))
        self._narrator.set_current_step(current_step)
        narration = "Narrator is: " + self._narrator_name + '\n' if self._narrator_name else ""
        for i, ac_script in enumerate(verbalization_script):
            tense = 'present' if i == current_step else 'past' if i < current_step else 'future'
            #s = self._narrator.make_action_sentence(action[0], action[1:], self._domain_semantics[action[0]], compressions[i], tense) # REMOVE
            s = self._narrator.make_action_sentence_from_script(ac_script, self._domain_semantics, compressions, tense)

            #### DEBUG
            s = self.script_debug_str(ac_script, compressions) + ':\n ' + s
            if tense == 'present':
                s = '* ' + s
            ##### DEBUG END

            narration += s + "\n\n"

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
        self._plan = req.input_plan
        n = self.narrate_plan(current_step=req.current_step)
        return NarratePlanResponse(n)

    def set_params_srv(self, req):
        self._verbalization_space_params = VerbalizationSpace.from_params_srv(req)
        if self._plan:
            self.narrate_plan(random_step=True)
        return SetVerbalizationParamsResponse(True)

    def parse_domain(self):
        domain_path = rospy.get_param("~domain_path")
        dp = DomainParser(domain_path)
        self._domain_semantics = dp.parse()

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

    def script_debug_str(self, ac_script, compressions):
        PDDL_justifications = ''
        for i, x in enumerate(ac_script.justifications):
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
        if ac_script.justifications:
            PDDL_justifications += ' -> '

        PDDL_justifies = ' -> ' if ac_script.justifies else ''
        for i, x in enumerate(ac_script.justifies):
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

    def split_plan_by_subjects(self, plan):
        plans = {}
        get_var = re.compile(r'\?([\w-]+)')
        action_subjects = {}
        for i, a in enumerate(plan):
            action_name = a[1][0]
            if action_name in action_subjects:
                subject_idx = action_subjects[action_name]
            else:
                subject = ' '.join(self._domain_semantics.get_action(a[1][0]).get_semantics('subject'))
                subjects = re.findall(get_var, subject) # List of variables representing subjects
                subjects_idx = []
                for idx, kv in enumerate(self.operators[a[1][0]].formula.typed_parameters):
                    if kv.key in subjects:
                        subjects_idx.append(idx)
                action_subjects[action_name] = subjects_idx

            for idx in subjects_idx:
                param = a[1][idx+1]
                if param not in plans:
                    plans[param] = []
                plans[param].append((i, a))
        return plans


if __name__ == "__main__":
    node = ROSPlanNarratorNode()
    rospy.loginfo(rospy.get_name() + ": Ready.")
    node.main_loop()
    rospy.spin()
