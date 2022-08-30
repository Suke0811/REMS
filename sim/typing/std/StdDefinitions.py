from sim.typing import DefDict
from sim.typing.std import StdUnit as unit
from typing import Any


class SpaceDefinition:
    default_definition = dict()
    default_dtype = Any
    default_name = None
    default_prefixes = None
    default_suffixes = None
    default_format_rule = None
    default_shape = None
    default_rules = None
    default_nested_def = True

    def __int__(self, definition=None, dtype=None, name=None, prefixes=None, suffixes=None, format_rule=None,
                shape=None,
                rules=None, nested_def=None):
        if definition is None: definition = self.default_definition
        if dtype is None: dtype = self.default_dtype
        if name is None: name = self.default_name
        if prefixes is None: name = self.default_name
        if suffixes is None: suffixes = self.default_suffixes
        if format_rule is None: format_rule = self.default_format_rule
        if shape is None: shape = self.default_shape
        if rules is None: rules = self.default_rules
        if nested_def is None: nested_def = self.default_nested_def

        self.definition = definition
        self.dtype = dtype
        self.name = name
        self.prefixes = prefixes
        self.suffixes = suffixes
        self.format_rule = format_rule
        self.shape = shape
        self.rules = rules
        self.nested_def = nested_def

    def to_dict(self):
        return dict(definition=self.definition, dtype=self.dtype, name=self.name, prefixes=self.prefixes,
                    suffixes=self.suffixes, format_rule=self.format_rule, shape=self.shape,
                    rules=self.rules, nested_def=self.nested_def)

    def to_DefDict(self):
        return DefDict(**self.to_dict())



#
# class Pos2D(SpaceDefinition):
#     definition = dict(x=unit.Pos, y=unit.Pos)
#     name = 'pos'
#
# class Euler(SpaceDefinition):
#     definition = dict(th_x=unit.Ang, th_y=unit.Ang, th_z=unit.Ang)
