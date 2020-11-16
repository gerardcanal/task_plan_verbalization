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
import functools
from rosplan_dispatch_msgs.msg import EsterelPlanEdge, EsterelPlanNode


class EsterelProcessing:
    @staticmethod
    def find_causal_chains(operators, goals, plan, esterel_plan):
        causal_chains = EsterelProcessing.achieving_actions(goals, plan, operators)
        for i, c in enumerate(causal_chains):
            chain = EsterelProcessing.get_causal_chain(plan[c.achieving_action.action_id][1], esterel_plan)
            causal_chains[i].achieving_action.children = chain
        return causal_chains
        # TODO idea: keep also goals per action, so if one action achieves multiple goals we can handle that

    # Returns list of actions that are causal links to the action sent by parameter
    @staticmethod
    def get_causal_chain(action, esterel_plan):
        # Find action in esterel graph
        for n in esterel_plan.nodes:
            if EsterelProcessing.match_action(action, n.action):
                # Recursively trace back the causality chain of the initial nodes
                source_nodes = EsterelProcessing.get_source_nodes(n, esterel_plan)  # Nodes pointing to this action node
                return EsterelProcessing.causal_chain_rec(source_nodes, esterel_plan)
        return []

    # Recursively computes a causal chain starting from the nodes 'nodes' using the esterel_plan
    @staticmethod
    def causal_chain_rec(nodes, esterel_plan):
        causal_chain = []  # Base case will be when nodes is empty
        for n in nodes:
            if n.node_type == EsterelPlanNode.PLAN_START:
                continue
            n_nodes = EsterelProcessing.get_source_nodes(n, esterel_plan)
            node_action = n.action.name + ' ' + ' '.join([p.value for p in n.action.parameters])
            ccn = CausalityChain.Node(node_action, n.action.action_id)
            ccn.children = EsterelProcessing.causal_chain_rec(n_nodes, esterel_plan)
            causal_chain.append(ccn)
        return causal_chain

    @staticmethod
    def get_source_nodes(n, esterel_plan, edge_type=EsterelPlanEdge.CONDITION_EDGE):
        edges = list(n.edges_in)
        if n.node_type == EsterelPlanNode.ACTION_START:
            edge = next(ei for ei in n.edges_out if esterel_plan.edges[ei].edge_type == EsterelPlanEdge.START_END_ACTION_EDGE)
            end_node = esterel_plan.nodes[esterel_plan.edges[edge].sink_ids[0]]
            assert end_node.action.action_id == n.action.action_id and end_node.node_id != n.node_id
            edges += end_node.edges_in
        elif n.node_type == EsterelPlanNode.ACTION_END:
            edge = next(ei for ei in n.edges_in if esterel_plan.edges[ei].edge_type == EsterelPlanEdge.START_END_ACTION_EDGE)
            start_node = esterel_plan.nodes[esterel_plan.edges[edge].source_ids[0]]
            assert start_node.action.action_id == n.action.action_id and start_node.node_id != n.node_id
            edges += start_node.edges_in
        return [esterel_plan.nodes[ni] for ei in edges for ni in esterel_plan.edges[ei].source_ids
                if esterel_plan.edges[ei].edge_type == edge_type]

    @staticmethod
    def match_action(action_list, rp_action):
        return action_list[0] == rp_action.name and \
               functools.reduce(lambda a, b: a and b, [x[0] == x[1].value for x in zip(action_list[1:],
                                                                                       rp_action.parameters)])

    # Returns a list of goals and actions achieving such goals to start the causal chain computation
    @staticmethod
    def achieving_actions(goals, plan, operators):
        causal_chains = []
        for a_id, a in enumerate(reversed(plan)):  # Goals are more likely to be achieved in the end, so let's fine them there
            op = operators[a[1][0]]
            grounding = {}
            for i, tp in enumerate(op.formula.typed_parameters):
                grounding[tp.key] = a[1][i+1]
            for i, g in enumerate(goals):  # Find if action a achieves the goal g
                if g[0]:  # positive goal, check add list
                    effects_list = op.at_end_add_effects + op.at_start_add_effects
                else:  # Negative goal, check del list
                    effects_list = op.at_end_del_effects + op.at_start_del_effects
                for eff in effects_list:
                    if EsterelProcessing.check_effect_goal(g, eff, grounding):  # Action effect achieves goal
                        g = goals.pop(i)
                        cchain = CausalityChain(g[1], g[0], ' '.join(a[1]), len(plan) - a_id - 1)
                        causal_chains.append(cchain)
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

    # Returns a list of the effect predicates that involve any ground parameter in params
    @staticmethod
    def params_in_effects(params, action_info, ground_action):
        action_list = ground_action.split(' ')
        assert action_info.name == action_list[0]
        grounding = {}
        action_params = action_list[1:]
        for i, tp in enumerate(action_info.formula.typed_parameters):
            grounding[tp.key] = action_params[i]

        ret = []
        effects_list = action_info.at_end_add_effects + action_info.at_start_add_effects + \
                       action_info.at_end_del_effects + action_info.at_start_del_effects
        for eff in effects_list:
            eff_param_list = EsterelProcessing.typed_params_to_str(eff.typed_parameters, grounding)
            for p in params:
                if p in eff_param_list:
                    ret.append(eff.name + ' ' + eff_param_list)
                    break
        return ret  # Todo check polarity in add/del lists?


class CausalityChain:
    class Node:
        def __init__(self, action_str, action_id):
            self.action_str = action_str
            self.action_id = action_id
            self.children = []

        def __str__(self):
            return self.action_str + ' (' + str(self.action_id) + ')'

        def __repr__(self):
            return self.__str__()

        def _print(self, acc=''):
            acc += ' ' + self.action_str + ' (' + str(self.action_id) + ')'
            if self.children:
                acc += ' |'
                for n in self.children:
                    n._print(acc)
            else:
                print(acc)

    def __init__(self, goal, goal_value, action_str, action_id):
        self.goal = goal
        self.goal_value = goal_value
        self.achieving_action = self.Node(action_str, action_id)

    def __str__(self):
        return '(' + self.goal + ' [' + str(self.goal_value) + ']): ' + str(self.achieving_action)

    def __repr__(self):
        return self.__str__()

    def _print(self):
        print('Goal: ' + self.goal)
        print('Actions:')
        self.achieving_action._print('   ')
