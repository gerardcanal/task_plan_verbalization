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
import re
from ActionSemantics import ActionSemantics, DomainSemantics, PredicateSemantics


class RegularExpressions:
    # Matches the domain name
    PDDL_DOMAIN_NAME = re.compile(r"(?s)define\s+\(domain\s+(.*?)\s*\)", re.MULTILINE)
    RDDL_DOMAIN_NAME = re.compile(r"(?s)domain\s+(.*?)\s*\{", re.MULTILINE)

    # Matches the whole action (if they start with a comment)
    PDDL_ACTION = re.compile(r"(?s)(?:^[\t ]*;[ \t\S]*[\r\n\f\v])+[ \t\S]+action(?!\w).*?\).*?$(?=\s*(?:;|\(:|\Z))", re.MULTILINE)
    RDDL_ACTION = re.compile(r"(?s)(?:^[\t ]*//[ \t\S]*[\r\n\f\v])+[ \t\S]+action-fluent.*?;", re.MULTILINE)

    # Matches the action name and parameter list
    #  The first group is the action name, second is the param list
    PDDL_SUBACTION = re.compile(r"(?s)\(:.*action(?:[ \t]*;[ \t\S]*$)?\s([\w-]+).*:parameters\s?\((.*?)\)", re.MULTILINE)
    RDDL_SUBACTION = re.compile(r"(?s)^[ \t]*([\w-]+)(?:\s*\((.*?)\))?", re.MULTILINE)

    # Matches each parameter in the list and the type
    # Group 1 is the list of varnames (?from ?to), group 2 is the type
    PDDL_PARAM_LIST = re.compile(r"((?:\?[\w-]+\s?)+)\s-\s([\w-]+)")
    RDDL_PARAM_LIST = re.compile(r"([\w-]+),?")

    # Matches for the semantic attachments lines (i.e. ; verb = XXX)
    SEMANTIC = re.compile(r"(?:;|//)[ \t]*([\w-]+)\s?=\s?(.*)")

    # Matches forward slashes. Used to split synonyms of type go / move / travel
    SYNONYMS = re.compile(r"\s?/\s?")

    # Matches exclamation marks
    NEEDED_ATTRIBUTE = re.compile(r"\s*!\s*$")

    # Matches a single parameter. Can be either ?XX or \[digit]+
    PARAM = re.compile(r"\?[\w-]+|\\\d+")

    # Matches IPC plan actions
    PLAN_ACTION = re.compile(r"(.*):\s*\((.*)\)\s*\[(.*)\]", re.MULTILINE)

    # Pre and post verb
    VERB_PREPOST = re.compile(r"(?:\((\w*)\)[\t ]*)?(\w+)(?:[\t ]*\(([\w ]*)\))?")

    # Predicates section of domain
    # PREDICATES = re.compile(r'(?s)\(:predicates\s*(.*?\))[^\(]*\)(?=[\n \t]*\(:)', re.MULTILINE)
    PDDL_PREDICATES_SECTION = re.compile(r'(?s)\(:predicates\s*(.*?\))[^\(]*\)', re.MULTILINE)
    RDDL_PREDICATES_SECTION = re.compile(r'(?s)pvariables\s*{(.*?)^\s*};', re.MULTILINE)
    PDDL_SINGLE_PREDICATE_DESCRIPTION = re.compile(r'(?s)\s*((?:;.*?$)*\s*\(.*?\))', re.MULTILINE)  # Covers semantic comments and predicates
    RDDL_SINGLE_PREDICATE_DESCRIPTION = re.compile(r"(?s)((?:^[\t ]*//[ \t\S]*[\r\n\f\v])+[ \t\S]+state-fluent.*?;)", re.MULTILINE)
    PDDL_SINGLE_PREDICATE = re.compile(r'\(([\w_]+)\s+(.*)\)')  # Covers only the predicate (predicate ?x - type)
    RDDL_SINGLE_PREDICATE = re.compile(r'([\w-]+)(?:\((.*)\))?\s*:')  # Covers only the predicate predicate(type)

    # Proxy method to get regexs for different languages
    @staticmethod
    def get(language, regex_name):
        return RegularExpressions.__dict__[language + '_' + regex_name]


class DomainParser:
    def __init__(self, domain_file):
        self.domain_file = domain_file
        dot = domain_file.rfind('.')
        self.language = domain_file[dot+1:].upper()
        if dot < 0 or self.language not in ['RDDL', 'PDDL', 'PPDDL']:
            raise ValueError("Incorrect domain extension provided. Accepted extensions are pddl, rddl, ppddl")

    def parse(self):
        f = open(self.domain_file, "r")
        domain = f.read()
        name = re.search(RegularExpressions.get(self.language, "DOMAIN_NAME"), domain).group(1)
        domain_semantics = DomainSemantics(name)
        action_matches = re.finditer(RegularExpressions.get(self.language, "ACTION"), domain)
        for match in action_matches:
            action_definition = match.group(0)
            semantic = re.findall(RegularExpressions.SEMANTIC, action_definition)
            aux = re.search(RegularExpressions.get(self.language, "SUBACTION"), action_definition)
            action_name = aux.group(1)
            params = re.findall(RegularExpressions.get(self.language, "PARAM_LIST"), aux.group(2)) if aux.group(2) else []
            if self.language == "RDDL":
                params = [('\\' + str(i+1), t) for i, t in enumerate(params)]
            action = ActionSemantics(action_name)
            for var, t in params:
                for v in var.split(' '):
                    action.add_param(v, t)
            for t, d in semantic:
                d = d.strip()
                d, n_subs = re.subn(RegularExpressions.NEEDED_ATTRIBUTE, '', d)
                synonyms = re.split(RegularExpressions.SYNONYMS, d)  # Check if there are multiple versions
                action.add_semantics(t, synonyms, n_subs > 0)
            domain_semantics.add_action(action)

        # Parse predicates
        m = re.search(RegularExpressions.get(self.language, "PREDICATES_SECTION"), domain)
        predicate_matches = []
        if m:
            predicate_matches = re.finditer(RegularExpressions.get(self.language, "SINGLE_PREDICATE_DESCRIPTION"), m.group(1))
        for match in predicate_matches:
            predicate_definition = match.group(1)
            semantic = re.findall(RegularExpressions.SEMANTIC, predicate_definition)
            aux = re.search(RegularExpressions.get(self.language, "SINGLE_PREDICATE"), predicate_definition)
            predicate_name = aux.group(1)
            params = re.findall(RegularExpressions.get(self.language, "PARAM_LIST"), aux.group(2)) if aux.group(2) else []
            if self.language == "RDDL":
                params = [('\\' + str(i+1), t) for i, t in enumerate(params)]
            predicate = PredicateSemantics(predicate_name)
            for var, t in params:
                for v in var.split(' '):
                    predicate.add_param(v, t)
            for t, d in semantic:
                d, n_subs = re.subn(RegularExpressions.NEEDED_ATTRIBUTE, '', d)
                synonyms = re.split(RegularExpressions.SYNONYMS, d)  # Check if there are multiple versions
                predicate.add_semantics(t, synonyms, n_subs > 0)
            domain_semantics.add_predicate(predicate)

        f.close()
        return domain_semantics
