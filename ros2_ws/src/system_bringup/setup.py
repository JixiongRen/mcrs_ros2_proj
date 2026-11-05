"""Install launch and configuration resources as well as the package marker."""

from glob import glob

from setuptools import setup

setup(
    name="system_bringup",
    version="0.1.0",
    packages=["system_bringup"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/system_bringup"]),
        ("share/system_bringup", ["package.xml"]),
        ("share/system_bringup/launch", glob("launch/*.launch.py")),
        ("share/system_bringup/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=False,
    maintainer="GCA project contributors",
    maintainer_email="maintainer@example.invalid",
    description="Launch and configuration for GCA simulation and verified hardware.",
    license="LicenseRef-Project-Unspecified",
)
