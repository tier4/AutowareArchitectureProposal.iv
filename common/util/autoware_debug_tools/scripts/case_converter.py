import re


def pascal2snake(s):
    return re.sub(r"([a-z])([A-Z])", r"\1_\2", s).lower()


def snake2pascal(s):
    return "".join([w.capitalize() for w in s.split("_")])
