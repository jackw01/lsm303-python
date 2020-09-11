#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import find_packages, setup

setup(
    name='lsm303-python',
    version='1.0.0',
    description='Python library for the LSM303 I2C accelerometer/magnetometer',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    author='jackw01',
    python_requires='>=3.7.0',
    url='https://github.com/jackw01/lsm303-python',
    packages=find_packages(),
    zip_safe=False,
    install_requires=[
    ],
    include_package_data=True,
    entry_points={
        'console_scripts': [
            'lsm303_test=lsm303:_test'
        ]
    },
    license='MIT',
    classifiers=[
        # Trove classifiers
        # Full list: https://pypi.python.org/pypi?%3Aaction=list_classifiers
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: Implementation :: CPython',
        'Programming Language :: Python :: Implementation :: PyPy'
    ]
)
