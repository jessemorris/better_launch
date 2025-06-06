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
    maintainer='Nikolas Dahn',
    maintainer_email='nikolas.dahn@gmail.com',
    description='A better replacement for the ROS2 launch system: intuitive, simple, memorable.',
    license='MIT',
)
