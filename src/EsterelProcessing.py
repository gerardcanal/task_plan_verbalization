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


class EsterelProcessing:
    @staticmethod
    def find_causal_chains(operators, goals, plan, esterel_plan):
        causal_chains = EsterelProcessing.achieving_actions(goals, plan, operators)

        return None
        # TODO idea: keep also goals per action, so if one action achieves multiple goals we can handle that

    # Returns a dictionary of goals and actions achieving such goals to start the causal chain comptuation
    @staticmethod
    def achieving_actions(goals, plan, operators):
        causal_chains = {}
        for a in reversed(plan):  # Goals are more likely to be achieved in the end, so let's fine them there

            op = operators[a[1][0]]
            grounding = {}
            for i, tp in enumerate(op.formula.typed_parameters):
                grounding[tp.key] = a[1][i+1]
            for i, g in enumerate(goals):
                if g[0]:  # positive goal, check add list
                    for eff in (op.at_end_add_effects + op.at_start_add_effects):
                        if EsterelProcessing.check_effect_goal(g, eff, grounding):  # Action effect achieves goal
                            causal_chains[goals.pop(i)] = [a[1]]
                            break
                else:
                    for eff in (op.at_end_del_effects + op.at_start_del_effects):
                        if EsterelProcessing.check_effect_goal(g, eff, grounding):  # Action effect achieves goal
                            causal_chains[goals.pop(i)] = [a[1]]
                            break
            if not goals:
                break
        return causal_chains

    @staticmethod
    def check_effect_goal(goal, effect, grounding):
        ground_effect = effect.name + ' ' + EsterelProcessing.typed_params_to_str(effect.typed_parameters, grounding)
        is_goal = goal[1] == ground_effect
        return is_goal

    @staticmethod
    def typed_params_to_str(typed_parameters, grounding):
        return ' '.join([grounding[p.key] for p in typed_parameters])

    @staticmethod
    def ki_to_str(ki):
        return not ki.is_negative, ki.attribute_name + ' ' + ' '.join([v.value for v in ki.values])
