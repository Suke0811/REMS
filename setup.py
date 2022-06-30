from setuptools import setup, find_packages
setup(
    name='ARS',
    version='0.1.1',
    packages=find_packages(),
    package_dir={},
    install_requires=['pygame',
                        'matplotlib',
                        'pandas',
                        'pynput',
                        'numpy',
                        'pytest',
                        'scipy',
                        'websocket-client',
                        'opencv-contrib-python',
                        'sklearn',
                        'ray',
                        'redis',
                        'dynamixel_sdk',
                        'pybullet',
                        'protobuf~=3.19.0',
                        ]
)
#build wheel
# python setup.py bdist_wheel --universal