#!/usr/bin/env python3
"""
Constellation Overwatch SDK Setup Script
"""

from setuptools import setup, find_packages
import os

# Read version from SDK_VERSION.md
def get_version():
    version_file = os.path.join(os.path.dirname(__file__), 'SDK_VERSION.md')
    with open(version_file, 'r') as f:
        for line in f:
            if line.startswith('**Version**:'):
                return line.split(':')[1].strip()
    return "1.0.0"

# Read README for long description
def get_long_description():
    readme_file = os.path.join(os.path.dirname(__file__), 'README.md')
    with open(readme_file, 'r', encoding='utf-8') as f:
        return f.read()

setup(
    name="constellation-overwatch-sdk",
    version=get_version(),
    description="Government-owned, community-driven autonomous systems integration platform",
    long_description=get_long_description(),
    long_description_content_type="text/markdown",
    author="Constellation Overwatch Community",
    author_email="community@constellation-overwatch.org",
    url="https://github.com/constellation-overwatch/sdk",
    license="Apache License 2.0",
    
    # Package configuration
    packages=find_packages(where="sdk"),
    package_dir={"": "sdk"},
    python_requires=">=3.8",
    
    # Dependencies
    install_requires=[
        "asyncio-mqtt>=0.11.0",
        "pyyaml>=6.0",
        "numpy>=1.21.0",
        "scipy>=1.7.0",
        "opencv-python>=4.5.0",
        "pillow>=8.3.0",
        "matplotlib>=3.5.0",
        "pandas>=1.3.0",
        "requests>=2.26.0",
        "websockets>=10.0",
        "aiohttp>=3.8.0",
        "cryptography>=3.4.0",
        "pydantic>=1.8.0",
        "fastapi>=0.70.0",
        "uvicorn>=0.15.0",
    ],
    
    # Optional dependencies
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "pytest-asyncio>=0.19.0",
            "black>=22.0.0",
            "flake8>=4.0.0",
            "mypy>=0.910",
            "sphinx>=4.0.0",
            "sphinx-rtd-theme>=1.0.0",
        ],
        "simulation": [
            "gazebo-msgs>=0.1.0",
            "unity-tcp-endpoint>=1.0.0",
            "unreal-engine-python>=4.27.0",
        ],
        "hardware": [
            "pymavlink>=2.4.0",
            "pyserial>=3.5",
            "can-utils>=0.1.0",
        ],
        "ai": [
            "tensorflow>=2.8.0",
            "pytorch>=1.11.0",
            "transformers>=4.15.0",
            "opencv-contrib-python>=4.5.0",
        ]
    },
    
    # Entry points
    entry_points={
        "console_scripts": [
            "constellation-overwatch=sdk.tools.cli:main",
            "co-simulator=sdk.tools.simulator:main",
            "co-monitor=sdk.tools.monitor:main",
            "co-deploy=sdk.tools.deployer:main",
        ],
    },
    
    # Package data
    package_data={
        "sdk": [
            "config/*.yaml",
            "config/*.json",
            "templates/*.yaml",
            "templates/*.json",
            "schemas/*.json",
        ],
    },
    
    # Classifiers
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "Intended Audience :: Other Audience",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: System :: Distributed Computing",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Operating System :: OS Independent",
        "Environment :: Console",
        "Environment :: Web Environment",
    ],
    
    # Keywords
    keywords=[
        "autonomous systems",
        "robotics",
        "drones",
        "uav",
        "simulation",
        "military",
        "defense",
        "mosa",
        "open architecture",
        "sensor fusion",
        "mission planning",
        "tak",
        "mavlink",
        "government",
        "open source"
    ],
    
    # Project URLs
    project_urls={
        "Documentation": "https://docs.constellation-overwatch.org",
        "Source Code": "https://github.com/constellation-overwatch/sdk",
        "Bug Tracker": "https://github.com/constellation-overwatch/sdk/issues",
        "Community": "https://github.com/constellation-overwatch/sdk/discussions",
        "Changelog": "https://github.com/constellation-overwatch/sdk/blob/main/CHANGELOG.md",
    },
    
    # Metadata
    zip_safe=False,
    include_package_data=True,
)
