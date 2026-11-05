from setuptools import find_packages, setup

setup(
    name="gca_ros",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/gca_ros"]),
        ("share/gca_ros", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="GCA maintainers",
    maintainer_email="maintainers@example.invalid",
    description="ROS host and CLI for supervised dual-axis motion",
    license="LicenseRef-Project-Unspecified",
    entry_points={
        "console_scripts": ["gca_control_node = gca_ros.node:main", "gca_cli = gca_ros.cli:main"]
    },
)
