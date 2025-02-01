# conf.py

import os
import sys

# -- Path setup --------------------------------------------------------------
# If your Sphinx docs are in a subfolder, adjust path if needed:
# sys.path.insert(0, os.path.abspath('..'))

# -- Project information -----------------------------------------------------
project = 'Unibot'
copyright = '2025, anonymous'
author = 'anonymous'

# The short X.Y version
version = '0.1'
# The full version, including alpha/beta/rc tags
release = '0.1.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# The master toctree document
master_doc = 'index'

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_baseurl = "https://its-a-me-ashwin.github.io/UniBot/"

# Optionally, you can include a custom CSS file in _static if you want:
# html_css_files = [
#     'custom.css',
# ]
