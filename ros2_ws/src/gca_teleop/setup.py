from glob import glob

from setuptools import find_packages, setup

setup(
    name="gca_teleop",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/gca_teleop"]),
        ("share/gca_teleop", ["package.xml"]),
        ("share/gca_teleop/config", glob("config/*.yaml") + glob("config/*.json")),
        ("share/gca_teleop/scripts", glob("scripts/*.json")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=False,
    maintainer="renjixiong",
    maintainer_email="renjixiong@126.com",
    description="Supervised joystick control of GCA with readonly input and simulation",
    license="LicenseRef-Project-Unspecified",
    entry_points={
        "console_scripts": [
            "gca_teleop_node = gca_teleop.node:main",
            "gca_teleop_probe = gca_teleop.probe:main",
            "gca_teleop_cli = gca_teleop.cli:main",
            "gca_joystick_monitor = gca_teleop.monitor:main",
        ]
    },
)
