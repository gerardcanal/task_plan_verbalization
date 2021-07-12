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
from task_plan_verbalization.srv import QuestionAnswer


class QuestionPlanNode:
    def __init__(self):
        rospy.init_node("question_plan_node", sys.argv)
        self.question_srv = rospy.ServiceProxy('/rosplan_narrator/question_plan', QuestionAnswer)

    def main_loop(self):
        r = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            try:
                question = input("Question: ")
                answer = self.process_question(question)
                print("  Answer: " + answer)
            except Exception as e:
                rospy.signal_shutdown('EOF question: ' + str(e))

    def process_question(self, question):
        res = self.question_srv(question)
        if res.alternatives:
            if res.alternatives:
                #return 'There were many possible options. Could you ask again providing more information?'
                print("Answer: " + res.alternatives)  # TODO complete this, create sentence, etc
                question = input("Question: ")
                return self.process_question(question)
        return res.verbalization_answer


if __name__ == "__main__":
    node = QuestionPlanNode()
    rospy.loginfo(rospy.get_name() + ": Ready.")
    node.main_loop()
    rospy.spin()
