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
from enum import Enum, IntEnum


class VerbalizationSpace:
    def __init__(self, abstraction, locality, specificity, explanation, locality_range=None, locality_object=None):
        self.abstraction = Abstraction(abstraction) if type(abstraction) is int else abstraction
        self.locality = Locality(locality) if type(locality) is int else locality
        self.locality.set_range(locality_range)
        self.locality.set_object(locality_object)
        self.specificity = Specificity(specificity) if type(specificity) is int else specificity
        self.explanation = Explanation(explanation) if type(explanation) is int else explanation

    @classmethod
    def from_params_srv(cls, request_msg):
        abstraction = Abstraction(request_msg.abstraction)
        locality = Locality(request_msg.locality)
        if locality == Locality.OBJECT:
            locality.set_object(request_msg.locality_object_name)
        if locality == Locality.RANGE:
            if request_msg.locality_min >= request_msg.locality_max:
                raise ValueError('Locality min range must be smaller than max range')
            locality.set_range((request_msg.locality_min, request_msg.locality_max))
        specificity = Specificity(request_msg.specificity)
        explanation = Explanation(request_msg.explanation)
        return cls(abstraction, locality, specificity, explanation, locality.range, locality.object)


class Abstraction(IntEnum):
    LEV1 = 1
    LEV2 = 2
    LEV3 = 3
    LEV4 = 4


class Locality(Enum):
    ALL = 1
    RANGE = 2
    OBJECT = 3

    def __init__(self, val):
        Enum.__init__(val)
        self.range = None
        self.object = None

    def set_range(self, plan_range):
        if self == self.RANGE:
            self.range = plan_range

    def get_range(self):
        # If All will return a full slice
        if self.range:
            return slice(*self.range)
        return None

    def set_object(self, object):
        if self == self.OBJECT:
            self.object = object

    def get_object(self):
        return self.object


class Specificity(Enum):
    GENERAL_PICTURE = 1
    SUMMARY = 2
    DETAILED_NARRATIVE = 3


class Explanation(IntEnum):
    LEV1 = 1
    LEV2 = 2
    LEV3 = 3
    LEV4 = 4
    LEV5 = 5
    LEV6 = 6
