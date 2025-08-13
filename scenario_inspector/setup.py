from setuptools import setup, find_packages

setup(
    name='scenario_inspector',
    version='1.0.0',
    author='AUAS Inspection Team',
    description='Automated inspection scenario execution system',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        'PyQt5>=5.15.0',
        'psycopg2-binary>=2.9.0',
        'PyYAML>=6.0',
        'bcrypt>=4.0.0',
        'python-dateutil>=2.8.0',
        'typing-extensions>=4.0.0',
        'pyrealsense2>=2.50.0',  # Intel RealSense SDK for Windows
        'pyserial>=3.5',
        'opencv-python>=4.5.0',
        'numpy>=1.21.0',
        'WMI>=1.5.1',  # Windows device management (optional)
    ],
    python_requires='>=3.8',
    entry_points={
        'console_scripts': [
            'scenario_inspector=main:main',
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Manufacturing',
        'Topic :: Scientific/Engineering :: Quality Control',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ],
)