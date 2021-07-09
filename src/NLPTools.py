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
import spacy
from nltk import Tree
import re
import copy
from DomainParser import DomainParser


class NLPTools:
    def __init__(self):
        self._nlp = spacy.load('en_core_web_sm')
        self._supported_deps = ['nsubj', 'nsubjpass', 'prep', 'dobj', 'iobj']
        self.dep_to_stype = {"verb": "verb", "prep": "prep", "dobj": "direct-object", "nsubj": "subject",
                            "nsubjpass": "subject", "iobj": "indirect-object"}
        self.stag_to_dep = {"verb": "verb", "prep": "prep", "direct-object": "dobj", "subject": "nsubj",
                            "indirect-object": "iobj"}

    def parse_question(self, question):
        doc = self._nlp(question)
        verbs = [(i, v) for i, v in enumerate(doc) if
                 v.dep_ == "ROOT"]  # TODO check how to handle multiple-verb sentences
        sent = []
        tense = 'present'
        for i, v in verbs:
            #s = [(v.lemma_, 'verb')]
            s = {'verb': v.lemma_}
            for c in v.children:
                if c.dep_ in self._supported_deps:
                    span = doc[c.left_edge.i: c.right_edge.i + 1]
                    #s.append((span.text, c.dep_))
                    s[c.dep_] = span.text
                if c.dep_ == "aux":
                    if c.tag_ == "VBD":
                        tense = 'past'
                    elif c.text_ == 'will':
                        tense = 'future'
            sent.append((s, tense))
        return sent if len(sent) > 1 else sent[0]

    def match_question_domain(self, lquestion, domain_semantics):
        if type(lquestion) is not list:
            lquestion = [lquestion]
        matched_actions = []
        for question in lquestion:
            question = list(question.items())
            assert question[0][0] == "verb"
            for a in domain_semantics.actions_iter():
                if question[0][1] in a.get_verbs():
                    groundings = {}
                    semantics = copy.deepcopy(a.get_all_semantics())
                    for t in question[1:]:
                        stype = self.dep_to_stype[t[0]]
                        if stype in semantics:  # If corresponding tag from the question is in the action semantics
                            stag = a.get_semantics(stype)
                            if type(stag) is list:
                                for s in stag:
                                    match = self.match_semantic_tag(s[0] if type(s) is tuple else s, t[1])
                                    if match:
                                        semantics[stype].remove(s)
                                        if not semantics[stype]:
                                            semantics.pop(stype)
                                        groundings[match[2]] = match[1].replace(' ', '-')
                                        break
                            else:
                                match = self.match_semantic_tag(stag, t[1])
                                if match:
                                    semantics.pop(stype)
                                    groundings[match[2]] = match[1].replace(' ', '-')
                        else:  # We didn't find the tag, so actions don't match
                            break
                    else:  # Match found, loop ended nicely
                        # Check if we left something important unmatched
                        for stype, stag in semantics.items():
                            if stype == "direct-object":  # Dobj are needed
                                break  # to next action
                            # elif stype == "prep":  # Check if tagged as needed -- Ignored as it may not be needed in the question to locate the action
                            #     if type(stag) is list:
                            #         if any([prep[1] for prep in stag]):
                            #             break  # to next action
                            #     elif stag[1]:
                            #         break  # to next action
                        else:  # everything matched, loop ends nicely
                            matched_actions.append(self.ground_action(a, groundings))
                            break  # Don't check more potential actions TODO check if can be merged with previous loop

        return matched_actions[0] if len(matched_actions) == 1 else matched_actions

    # Get grounding from parsed question to the semantig tag (which may include a pddl variable)
    def match_semantic_tag(self, semantic_tag, q_chunk):
        pddl_param = re.compile(r"(\?[\w-]+)")  # Matches pddl params starting with ? (i.e. ?robot ?a-place
        pddl_var = ""

        def get_match(match):  # This will reuse the match made by subn to store the pddl variable
            nonlocal pddl_var
            pddl_var = match.group(1)
            return r"([\w -]+)"

        if type(semantic_tag) is not list:
            semantic_tag = [semantic_tag]
        for stag in semantic_tag:
            stag_re = re.compile(pddl_param.subn(get_match, stag)[0])  # Returns a replacement re where the pddl variable is replaced by a regex matching one or more words
            match = stag_re.search(q_chunk)
            if match and pddl_var:
                return True, match.group(1).lower(), pddl_var
            elif match:  # it matched, but there was no pddl_var
                return True, match.group(0), None
        return False

    def ground_action(self, a, groundings):
        action = [a.get_action_name()]
        for p, type in a.get_params():
            if p in groundings:
                action.append(groundings[p])
            else:
                action.append(p)  # leave ungrounded parameter if missing
        return action


def tok_format(tok):
    # return "_".join([tok.orth_, tok.tag_, tok.pos_, tok.dep_])
    # return "_".join([tok.orth_, tok.tag_])
    # return "_".join([tok.orth_, tok.pos_])
    return "_".join([tok.orth_, tok.dep_])


def to_nltk_tree(node):
    if node.n_lefts + node.n_rights > 0:
        return Tree(tok_format(node), [to_nltk_tree(child) for child in node.children])
    else:
        return tok_format(node)


if __name__ == "__main__":
    document_string = "Why did Asro go from the corridor to the kitchen?"
    document_string = "Why was the coke taken by Asro?"
    document_string = "Why is the robot going to find Senka?"
    document_string = "Why will the robot find Senka?"
    #document_string = "Can we find a plan where Asro doesn't go to the living room?"
    # document_string = "Why did Asro take the can at the corridor?"
    # Load a language model and parse a document.
    nlp = spacy.load('en_core_web_sm')
    doc = nlp(document_string)

    # Print the dependency subtree of each token.
    # These are the words operated upon by the token.
    for token in doc:
        print(token, token.orth_, token.tag_, token.pos_, token.dep_)

    [to_nltk_tree(sent.root).pretty_print() for sent in doc.sents]

    for token in doc:
        print(token.text, token.dep_, token.head.text, token.head.pos_, token.head.tag_,
              [child for child in token.children])

    n = NLPTools()
    while True:
        try:
            document_string = input("Question: ")
        except:
            break
        q = n.parse_question(document_string)
        dp = DomainParser("/home/gerard/code/ROSPlan_ws/src/task_plan_verbalization/domains/office_robot/domain.pddl")
        domain_semantics = dp.parse()
        action = n.match_question_domain(q, domain_semantics)
        print("PDDL action: (" + ' '.join(action) + ')\n')
