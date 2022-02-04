# PlanVerb: Domain-Independent Verbalization and Summary of Task Plans
Verbalization and summarization of task plans. Examples can be found in the branch [`with_examples`](https://github.com/gerardcanal/task_plan_verbalization/tree/with_examples/verbalized_examples). 

**Authors:** Gerard Canal, Senka Krivić, Paul Luff, and Andrew Coles

## Abstract
To increase user trust in planning algorithms, users must be able to understand the output of the planner while getting some notion of the underlying reasons for the action selection. The output of task planners have not been traditionally user-friendly, often consisting of sequences of parametrised actions or task networks, which may not be practical for lay users who may find easier to read natural language descriptions. In this paper, we propose PlanVerb, a domain and planner-independent method for the verbalization of task plans based on semantic tagging of the actions and predicates. Our method can generate natural language descriptions of plans including explanations of causality between actions. The verbalized plans can be summarized by compressing the actions that act on the same parameters. We further extend the concept of verbalization space, previously applied to robot navigation, and apply it to planning to generate different kinds of plan descriptions depending on the needs or preferences of the user. Our method can deal with PDDL and RDDL domains, provided that they are tagged accordingly. We evaluate our results with a user survey that shows that users can read our automatically generated plan descriptions, and are able to successfully answer questions about the plan. We believe methods like the one we propose can be used to foster trust in planning algorithms in a wide range of domains and applications.


## Usage
### Prerequisites
To run the verbalization node, you need to install [ROSPlan](https://github.com/KCL-Planning/ROSPlan#installation), [mlconjug3](https://github.com/SekouDiaoNlp/mlconjug3/blob/master/INSTALL.rst), [nltk](https://www.nltk.org/install.html), and [spaCy](https://spacy.io/usage). PlanVerb is written in Python 3.

For ROS Melodic (Ubuntu 18.04), you may need to install some packages for Python 3:

```
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
```

### PDDL verbalization launch
```
roslaunch task_plan_verbalization rosplan_verbalization.launch domain_path:=PATH/domain.pddl problem_path:=PATH/problem.pddl
```

### RDDL verbalization launch
```
roslaunch task_plan_verbalization rosplan_verbalization_rddl.launch domain_path:=PATH/domain.rddl problem_path:=PATH/inst.rddl
```

### Getting the verbalization
In order to trigger the planning process and get a verbalization of the plan, run:
```
rosservice call /rosplan_narrator/trigger_planning
```

To generate a new verbalization of the same plan with different verbalization space parameters, run:
```
 rosservice call /rosplan_narrator/set_params "{
    abstraction: 3, 
    locality: 1, 
    locality_min: 0, 
    locality_max: 5, 
    locality_object_name: 'post1',
    specificity: 2, 
    explanation: 4
}"
```

### Optional arguments
 - `narrator_name:=NAME` will set the narrator to be "NAME". This means that the actions made by the subject with name = "NAME" will be verbalized in first person.
 - `evaluation:=true` will generate all the combination of verbalizations for all the verbalization space parameters for a single plan and store them in the verbalization_examples folder.
 - `print_actions:=true` will print the PDDL/RDDL actions that generated each sentence, showing the causal relations and compressions.
 
### Example
Example with one of the domains available in the package:
```
roslaunch task_plan_verbalization rosplan_verbalization.launch domain_path:=$(rospack find task_plan_verbalization)/domains/office_robot/domain.pddl problem_path:=$(rospack find task_plan_verbalization)/domains/office_robot/problem_twobots.pddl narrator_name:=robot_assistant
```

And then trigger the planner:
`rosservice call /rosplan_narrator/trigger_planning`

## Citation
If you use our work, please cite us as:

```
@inproceedings{Canal_AAAI2022,
    author = "Gerard Canal and Senka Krivić and Paul Luff and Andrew Coles",
    title = "{PlanVerb: Domain-Independent Verbalization and Summary of Task Plans}",
    booktitle = {Proceedings of the AAAI Conference on Artificial Intelligence},
    volume = {36},
    number = {1},
    pages = {},
    year = {2022}
}
```
