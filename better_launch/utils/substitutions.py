import re


def substitute_tokens(text: str, substitution_handlers: dict) -> list[str]:
    # Always find the innermost set of brackets with no brackets in between. 
    # TODO This effectively prevents the use of normal brackets
    pattern = re.compile(r'\$\(([^()]+)\)')

    def substitute(m: re.Match):
        cmd, *args = m.group(1).split(" ")

        if cmd not in substitution_handlers:
            raise ValueError(f"Unknown substitution command '{cmd}'")

        return substitution_handlers[cmd](*args)

    while "$(" in text:
        text = pattern.sub(substitute, text)

    return text
