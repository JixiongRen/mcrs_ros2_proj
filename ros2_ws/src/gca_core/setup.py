"""Install the one ROS-independent ``gca`` Python core through ament."""

from setuptools import find_packages, setup

setup(
    name="gca_core",
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/gca_core"]),
        ("share/gca_core", ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=False,
    maintainer="GCA project contributors",
    maintainer_email="maintainer@example.invalid",
    description="Typed device adapters, dual-axis control runtime and independent simulation.",
    license="LicenseRef-Project-Unspecified",
    tests_require=["pytest"],
)
