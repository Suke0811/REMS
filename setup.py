from setuptools import setup, find_packages
import codecs
import os.path

import sys

def get_extras():
    extras = []
    for arg in sys.argv:
        if '[' in arg and ']' in arg:
            extras.extend(arg[arg.find('[') + 1:arg.find(']')].split(','))
    return extras


def read(rel_path):
    here = os.path.abspath(os.path.dirname(__file__))
    with codecs.open(os.path.join(here, rel_path), 'r') as fp:
        return fp.read()

def get_metadata(field):
    rel_path = "rems/__init__.py"
    for line in read(rel_path).splitlines():
        if line.startswith(f'__{field}__'):
            delim = '"' if '"' in line else "'"
            return line.split(delim)[1]
    else:
        raise RuntimeError(f"Unable to find {field} string.")

# Loads _version.py module without importing the whole package.
def get_version_and_cmdclass(pkg_path):
    from importlib.util import module_from_spec, spec_from_file_location
    spec = spec_from_file_location(
        'version', os.path.join(pkg_path, '_version.py'),
    )
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.__version__, module.get_cmdclass(pkg_path)

version, cmdclass = get_version_and_cmdclass('rems')


with open('requirements.txt') as f:
    requirements = f.read().splitlines()

with open('async_req.txt') as f:
    async_req = f.read().splitlines()

setup(
    name='rems',
    version='0.3.0',
    cmdclass=cmdclass,
    description=get_metadata('description'),
    author=get_metadata('author'),
    license='LGPLv3',
    packages=find_packages(include=['rems', 'rems.*']),
    install_requires=requirements,
    extras_require={
        'core': [],
        'async': async_req,
        'joystick': ['pygame'],
        'keyboard': ['pynput'],
        'full': async_req + ['pygame', 'pynput']
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Framework :: Robot Framework',
        'Intended Audience :: Developers',
        'Intended Audience :: Education',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)',
        'Programming Language :: Python :: 3',
    ],
)

