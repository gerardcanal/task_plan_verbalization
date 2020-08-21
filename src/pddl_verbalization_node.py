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
import rospy
import sys
import re
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from std_msgs.msg import String
from PlanNarrator import PlanNarrator, DomainParser, RegularExpressions
from pddl_verbalization.srv import NarratePlan, NarratePlanResponse
# TODO:
# - Set_params for narration
# - Monitor dispatched actions, update narration based on that
import random


class ROSPlanNarratorNode:
    def __init__(self):
        rospy.init_node("rosplan_narrator", sys.argv)

        self._problem_gen = rospy.ServiceProxy("/rosplan_problem_interface/problem_generation_server", Empty)
        self._planner = rospy.ServiceProxy("/rosplan_planner_interface/planning_server", Empty)
        self._raw_plan_subs = rospy.Subscriber("/rosplan_planner_interface/planner_output", String, self.raw_plan_cb)
        self._verbalization_pub = rospy.Publisher("~plan_narration", String, queue_size=1)
        self._trigger_plan_srv = rospy.Service("~trigger_planning", Empty, self.trigger_plan_srv)
        self._narrate_plan_srv = rospy.Service("~narrate_plan", NarratePlan, self.narrate_plan_srv)
        self._update_narration_srv = rospy.Service("~narrate_current_plan", Trigger, self.update_narration_srv)
        self._plan_received = False
        self._plan = None

        self._robot_name = rospy.get_param("~robot_name", None)
        self._narrator = PlanNarrator(self._robot_name)

        self._domain_semantics = None
        self.parse_domain()

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
        except rospy.ServiceException as e:
            rospy.logerr(rospy.get_name() + ": Service call failed: %s" % e)
        return EmptyResponse()

    def narrate_plan(self, current_step=-1, random_step=False):
        plan = re.findall(RegularExpressions.PLAN_ACTION, self._plan)
        if random_step:
            current_step = random.randint(0, len(plan))
        narration = "Narrator is: " + self._robot_name + '\n' if self._robot_name else ""
        for i, (time, action, duration) in enumerate(plan):  # TODO use time and duration
            tense = 'present' if i == current_step else 'past' if i < current_step else 'future'
            action_sp = action.split(' ')
            s = self._narrator.make_action_sentence(action_sp[0], action_sp[1:], self._domain_semantics[action_sp[0]], tense)
            s = '(' + action + '): ' + s
            if tense == 'present':  # FIXME check whether this is useful or not
                s = '* ' + s
            narration += s + "\n\n"
        #self._plan = None
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

    def parse_domain(self):
        domain_path = rospy.get_param("~domain_path")
        dp = DomainParser(domain_path)
        self._domain_semantics = dp.parse()


if __name__ == "__main__":
    node = ROSPlanNarratorNode()
    node.main_loop()
    rospy.spin()
