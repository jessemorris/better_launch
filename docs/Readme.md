# Install required packages
```bash
sudo apt install sphinx
pip install sphinx-autodoc-typehints myst-parser sphinx-rtd-theme
```

# Build the actual documentation
Source your workspace first, then build the documentation using

```bash
make doc
```

Check the Makefile for other targets, e.g. `help`.
