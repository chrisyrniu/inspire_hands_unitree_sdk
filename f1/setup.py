from setuptools import setup, find_packages

setup(
    name='inspire_f1_sdk',
    version='1.0.0',
    description='Python SDK for the Inspire RH56F1 Dexterous Hand (RS485 + EtherCAT)',
    packages=find_packages(),
    python_requires='>=3.8',
    install_requires=[
        'pyserial',
        'pymodbus',
        'pysoem',
        'cyclonedds',
    ],
)
