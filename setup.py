from setuptools import setup, find_packages

setup(
    name='better_launch',
    version='0.9.0',
    packages=find_packages(),
    install_requires=[
        "click",
        "docstring_parser",
        "osrf_pycommon",
        "pyperclip",
        "PyYAML",
        "rich",
        "setproctitle",
        "textual",
    ],
)
