from setuptools import setup, find_packages

setup(
    name="unibot",
    version="0.1.0",
    description="A Python package for controlling ODrive-based(3.4<=) unibot.",
    author="Its-a-me-Ashwin",
    author_email="bharadwaj.ash@northeastern.edu",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "odrive",
        "pygame"
    ],
    python_requires=">=3.7",
)
