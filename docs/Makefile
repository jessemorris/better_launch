# Minimal makefile for Sphinx documentation
#

# since we use numpy docstrings we need to run napoleon first so autodoc can understand them. 
# To build the documentation simply run 
# 
#     make doc
# 


# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = source
BUILDDIR      = build

.PHONY: doc
doc: clean napoleon html
	xdg-open build/html/index.html

napoleon:
	sphinx-apidoc -f -o source ..

help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
