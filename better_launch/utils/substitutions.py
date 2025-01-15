import re


def substitute_tokens_re(text: str, substitution_handlers: dict) -> list[str]:
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

def find_closing_bracket(text: str, start_index: int) -> int:
    stack = 0

    for i in range(start_index, len(text)):
        if text[i] == "$" and i + 1 < len(text) and text[i + 1] == "(":
            stack += 1
        elif text[i] == ")":
            stack -= 1
            if stack == 0:
                return i
    
    raise ValueError("Unmatched token: missing closing ')'")

def substitute_tokens(text: str, substitution_handlers: dict) -> str:
    result = []
    idx = 0

    while idx < len(text):
        token_start = text.find("$(", idx)
        if token_start < 0:
            break
        
        # Add text before the token
        result.append(text[idx:token_start])

        token_end = find_closing_bracket(text, token_start)
        token_content = text[token_start + 2:token_end].strip()
        command, *args = token_content.split()

        if command in substitution_handlers:
            if args:
                args = list(args)
                for i in range(len(args)):
                    args[i] = substitute_tokens(args[i], substitution_handlers)

            resolved = substitution_handlers[command](*args)
            result.append(resolved)
        else:
            raise ValueError(f"Unknown substitution: {command}")
        
        idx = token_end + 1
    
    # Add the remaining text
    result.append(text[idx:])

    return "".join(result)